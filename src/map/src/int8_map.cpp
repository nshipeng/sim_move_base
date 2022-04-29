/*
 * @Description: 
 * @Autor: 
 * @Date: 2021-12-23 14:03:42
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2022-03-10 17:49:12
 */

#include "map/int8_map.h"
#include <nav_msgs/OccupancyGrid.h>

int8_map::int8_map()
{

}

int8_map::~int8_map()
{
}

void int8_map::init(std::string _frame_id, double _robot_radius, double _truncated_distance, double _factor,ros::NodeHandle& _nh){
    inflate_radius = _robot_radius;
    truncated_ =_truncated_distance;
    alpha = _factor;
    nh = _nh;
    t_map_world_.setIdentity();
    frame_id = _frame_id;

    map_sub_ =nh.subscribe("/map",1,&int8_map::map_receive_callback,this);
    click_point_sub_ = nh.subscribe("clicked_point",10,&int8_map::click_point_callback,this);
    global_map_pub_ = std::make_shared<ros::Publisher>(nh.advertise<nav_msgs::OccupancyGrid>("/global_costmap",10));
    visual_help_ptr_ = std::make_shared<PlanningVisualization>(nh,_frame_id);
}

void int8_map::init_grid_map(){
    ROS_WARN("init the grid map.");
    grid_node_map.resize(num_of_colums);
    for(int i = 0;i<num_of_colums; i++){
        grid_node_map[i].resize(num_of_rows);
        for(int j = 0; j<num_of_rows; j++){
            Vector2i index(i,j);
            Vector2d position = grid2world(i,j);
            grid_node_map[i][j] = new GridNode(index, position);
        }
    }
}


void int8_map::map_receive_callback(const nav_msgs::OccupancyGrid& map){
    ROS_INFO("update map..");
    num_of_colums = map.info.width;
    num_of_rows = map.info.height;
    resolution = map.info.resolution;
    inv_resolution = 1/resolution;

    t_map_world_(0,3) = map.info.origin.position.x;
    t_map_world_(1,3) = map.info.origin.position.y;
    Quaterniond q;
    q.x() = map.info.origin.orientation.x;
    q.y() = map.info.origin.orientation.y;
    q.z() = map.info.origin.orientation.z;
    q.w() = map.info.origin.orientation.w;
    t_map_world_.block<3,3>(0,0) = q.toRotationMatrix();
    ROS_WARN_STREAM("t_map_world is \n"<<t_map_world_);
    update(map);
    //init_grid_map();
}

void int8_map::click_point_callback(const geometry_msgs::PointStamped& point){
    //new obstacle point comeing. means delete the old obstacle enviroment.and remove all markers.
    if(point_idx == 0){
        // remove_dynamic_obs();
        // visual_help_ptr_->remove_all_markers(marker_ids);

        click_p1<<point.point.x, point.point.y;
        point_idx = 1;
        ROS_INFO("get 1 obstacle point in world: %f, %f",click_p1(0), click_p1(1));
        {
            int c,r;
            world2grid(click_p1(0),click_p1(1),r,c);
            ROS_INFO("get 1 obstacle point in grid: %d, %d",c, r);
            visual_help_ptr_->drawGoal({click_p1(0), click_p1(1),0.0},0.1,Eigen::Vector4d(1,0,0,1),0);
            marker_ids.push_back(0);
        } 
    }else if(point_idx ==1){
        click_p2<<point.point.x, point.point.y;
        point_idx = 2;
        ROS_INFO("get 2 obstacle point in world: %f, %f",click_p2(0), click_p2(1));
        {
            int c,r;
            world2grid(click_p2(0),click_p2(1),r,c);
            ROS_INFO("get 2 obstacle point in grid: %d, %d",c, r);
            visual_help_ptr_->drawGoal({click_p2(0), click_p2(1),0.0},0.1,Eigen::Vector4d(1,0,0,1),1);
            marker_ids.push_back(1);

            add_obs_info(current_map.data,click_p1,click_p2);
            global_costmap.header.stamp = ros::Time::now();
            inflation_global_map(current_map.data, global_costmap.data);
            ROS_WARN("publish dynamic obstacle map");
            global_map_pub_->publish(global_costmap);
        }
    }else if(point_idx ==2){
        remove_dynamic_obs();
        visual_help_ptr_->remove_all_markers(marker_ids);
        point_idx = 0;
    }
}


void int8_map::update(const nav_msgs::OccupancyGrid &map){
    map_mutex.lock();
    //current_map = std::move(map);
    current_map = map;
    map_mutex.unlock();

    global_costmap.info = current_map.info;
    global_costmap.header.frame_id = "map";
    global_costmap.header.stamp = ros::Time::now();
    inflation_global_map(current_map.data, global_costmap.data);
    ROS_INFO("publish the comeing map.");

    global_map_pub_->publish(global_costmap);

}

void int8_map::inflation_global_map(const nav_msgs::OccupancyGrid::_data_type &map, nav_msgs::OccupancyGrid::_data_type &costmap){
     //膨胀大小
    int inflate_size = truncated_ / resolution;

    //清空costmap
    costmap.clear();
    costmap.resize(num_of_colums * num_of_rows);

    //全地图轮寻
    for (int idx = 0; idx < num_of_colums; idx++)
    {
        for (int idy = 0; idy < num_of_rows; idy++)
        {
            //选取为中心点坐标
            int8_t center_data = map[idx + idy * num_of_colums];
            //如果是未探索，costmap也是未探索
            if (center_data == -1)
            {
                costmap[idx + idy * num_of_colums] = center_data;
            }
            else if (center_data != 0)
            {
                //开始膨胀
                for (int dx = -inflate_size; dx <= inflate_size; dx++)
                {
                    for (int dy = -inflate_size; dy <= inflate_size; dy++)
                    {
                        //超过地图大小
                        if ((idx + dx) >= num_of_colums || (idx + dx) < 0 || (idy + dy) >= num_of_rows || (idy + dy) < 0)
                            continue;

                        //超出膨胀大小
                        if ((dx * dx + dy * dy) > (inflate_size * inflate_size))
                            continue;

                        int8_t val = map[(idx + dx) + (idy + dy) * num_of_colums];
                        //未探索不处理
                        if (val == -1)
                            continue;

                        //距离中心点距离
                        double distance_grid = sqrtf(dx * dx + dy * dy);

                        //计算代价值
                        int8_t data_compare = cal_cost(center_data, distance_grid * resolution);
                        //获取原代价值
                        int8_t data_origin = costmap[(idx + dx) + (idy + dy) * num_of_colums];
                        //取最大值更新costmap
                        if (data_compare > data_origin)
                            costmap[(idx + dx) + (idy + dy) * num_of_colums] = data_compare;
                    }
                }
            }
        }
    }

}

int8_t int8_map::cal_cost(int8_t map_data, double distance){
    int8_t data;
    int8_t threshold = 80;
    if (map_data >= threshold && distance <= inflate_radius)
    {
        data = 100;
    }
    else if (distance <= truncated_ && distance > inflate_radius)
    {
        double val = map_data * exp(-alpha * (distance - inflate_radius));
        if (val > map_data)
            val = map_data;
        if (val < 0)
            val = 0;
        data = val;
    }
    else
    {
        data = 0;
    }
    return data;
}


void int8_map::world2grid(double x, double y, int& row, int& col){
    Vector4d pos_in_world;  pos_in_world<<x,y,0,1.0;
    Vector4d pos_in_map = t_map_world_.inverse() * pos_in_world;

    col = std::min(std::max(int(pos_in_map(0)* inv_resolution),0),int(num_of_colums - 1));
    row = std::min(std::max(int(pos_in_map(1)* inv_resolution),0),int(num_of_rows - 1));
}

Vector2i int8_map::world2grid(double x, double y){
    Vector4d pos_in_world;  pos_in_world<<x,y,0,1.0;
    Vector4d pos_in_map = t_map_world_.inverse() * pos_in_world;

    int col = std::min(std::max(int(pos_in_map(0)* inv_resolution),0),int(num_of_colums - 1));
    int row = std::min(std::max(int(pos_in_map(1)* inv_resolution),0),int(num_of_rows - 1));
    
    Vector2i res; res<<col, row;
    return res;
    
}

void int8_map::grid2world(double& x, double& y, int row, int col){
    double x_in_map = col * resolution;
    double y_in_map = row * resolution;
    Vector4d pos_in_map; pos_in_map<<x_in_map, y_in_map, 0,1.0;

    Vector4d pos_in_world = t_map_world_*pos_in_map;
    x = pos_in_world(0); y = pos_in_world(1);
}

Vector2d int8_map::grid2world(int row, int col){
    double x_in_map = col * resolution;
    double y_in_map = row * resolution;
    Vector4d pos_in_map; pos_in_map<<x_in_map, y_in_map, 0,1.0;
    Vector4d pos_in_world = t_map_world_*pos_in_map;

    Vector2d res; res<<pos_in_world(0),pos_in_world(1);
    return res;
}


int int8_map::grid2index(int row, int col){
    return col + row * num_of_colums;
}


void int8_map::add_obs_info(const nav_msgs::OccupancyGrid::_data_type &map,Vector2d p1, Vector2d p2){
    double y = p2(1) - p1(1);  double x = p2(0) - p1(0);  
    double l = sqrt(pow(x,2)+pow(y,2));
    double k = y/x;

    int pass_grid_size = int(l * inv_resolution);
    double tx,ty,dx;
    int idx, marker_id;
    int col,row;

    //record all marker id and obs id.
    for(int i = 0; i< pass_grid_size; i++){
        dx= x / pass_grid_size;
        tx = p1(0)+i*dx;  ty = p1(1) + k*i*dx;
        marker_id = i+2;
        visual_help_ptr_->drawGoal({tx, ty,0.0},0.1,Eigen::Vector4d(1,0,0,1),marker_id);
        marker_ids.push_back(marker_id);

        world2grid(tx,ty,row,col);
        idx = grid2index(row,col);
        current_map.data[idx] = 100;
        global_costmap.data[idx]=100;
        obs_indexs.push_back(idx);
    }
}

void int8_map::remove_dynamic_obs(){
    if(obs_indexs.size() == 0){
        return;
    }
    for(int i =0;i<obs_indexs.size();i++){
        current_map.data[obs_indexs[i]] = 0;
        global_costmap.data[obs_indexs[i]]=0;
    }
    global_costmap.header.stamp = ros::Time::now();
    inflation_global_map(current_map.data, global_costmap.data);
    ROS_WARN("remove dynamic obstacle from  global cosytmap.");
    global_map_pub_->publish(global_costmap);
}

bool int8_map::is_occupied(int row, int col){
    return (
        col<0 || col >= num_of_colums ||
        row<0 || row >= num_of_rows   ||
        current_map.data[col + row * num_of_colums] >=98
    );
}

bool int8_map::is_free(int row, int col){
    return (
        row >= 0 && row <= num_of_rows   &&
        col >= 0 && col <= num_of_colums &&
        (current_map.data[col + row * num_of_colums] == 0)
    );
}

