/*
 * @Description: 
 * @Autor: 
 * @Date: 2022-01-04 15:15:37
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2022-01-06 17:55:15
 */

#include "path_searching/astar.h"

using namespace Eigen;
// https://github.com/teamo1996/Motion-plan/blob/main/%E7%AC%AC2%E7%AB%A0%EF%BC%9A%E5%9F%BA%E4%BA%8E%E6%90%9C%E7%B4%A2%E7%9A%84%E8%B7%AF%E5%BE%84%E8%A7%84%E5%88%92/hw_2/ros%E7%89%88%E6%9C%AC%E4%BD%9C%E4%B8%9A/src/grid_path_searcher/src/Astar_searcher.cpp

#define INFO ROS_INFO

void Astar::init(const nav_msgs::OccupancyGrid& global_costmap)
{
    num_of_colums = global_costmap.info.width;
    num_of_rows =   global_costmap.info.height;
    resolution = global_costmap.info.resolution;
    inv_resolution = 1.0 / resolution;
    
    t_map_world_.setIdentity();
    t_map_world_(0,3) = global_costmap.info.origin.position.x;
    t_map_world_(1,3) = global_costmap.info.origin.position.y;
    Quaterniond q;
    q.x() = global_costmap.info.origin.orientation.x;
    q.y() = global_costmap.info.origin.orientation.y;
    q.z() = global_costmap.info.origin.orientation.z;
    q.w() = global_costmap.info.origin.orientation.w;
    t_map_world_.block<3,3>(0,0) = q.toRotationMatrix();

    grid_node_map.resize(num_of_colums);
    for (int i = 0; i < num_of_colums; i++)
    {
        grid_node_map[i].resize(num_of_rows);
        for (int j = 0; j < num_of_rows; j++)
        {
            Vector2i index(i, j);
            Vector2d position = grid2world(index);                    //(col, row) in grid map and (x,y) in {world}.
            grid_node_map[i][j] = new GridNode(index, position);
        }
    }
    map_data.assign(global_costmap.data.begin(),global_costmap.data.end());
    //ROS_WARN_STREAM("num_of_colums "<<num_of_colums<<" "<<num_of_rows);
    //ROS_WARN(" astar::init()");

}

void Astar::init(const nav_msgs::OccupancyGrid& global_costmap, ros::NodeHandle& _nh){
    nh = _nh;
    num_of_colums = global_costmap.info.width;
    num_of_rows =   global_costmap.info.height;
    resolution = global_costmap.info.resolution;
    inv_resolution = 1.0 / resolution;
    
    t_map_world_.setIdentity();
    t_map_world_(0,3) = global_costmap.info.origin.position.x;
    t_map_world_(1,3) = global_costmap.info.origin.position.y;
    Quaterniond q;
    q.x() = global_costmap.info.origin.orientation.x;
    q.y() = global_costmap.info.origin.orientation.y;
    q.z() = global_costmap.info.origin.orientation.z;
    q.w() = global_costmap.info.origin.orientation.w;
    t_map_world_.block<3,3>(0,0) = q.toRotationMatrix();

    grid_node_map.resize(num_of_colums);
    for (int i = 0; i < num_of_colums; i++)
    {
        grid_node_map[i].resize(num_of_rows);
        for (int j = 0; j < num_of_rows; j++)
        {
            Vector2i index(i, j);
            Vector2d position = grid2world(index);                    //(col, row) in grid map and (x,y) in {world}.
            grid_node_map[i][j] = new GridNode(index, position);
        }
    }
    map_data.assign(global_costmap.data.begin(),global_costmap.data.end());
    //ROS_WARN(" astar::init()");
}


void Astar::deinit()
{
    for (int i = 0; i < num_of_colums; i++)
    {
        for (int j = 0; j < num_of_rows; j++)
        {
            delete grid_node_map[i][j];
        }
    }

    num_of_colums = 0;
    num_of_rows = 0;

    inv_resolution = 0;
    resolution = 0;
}

void Astar::reset_grid(GridNodePtr ptr)
{
    ptr->set_type = 0;
    ptr->came_from = NULL;
    ptr->g_score = inf;
    ptr->f_score = inf;
}

void Astar::reset_used_grids()
{
    for (int i = 0; i < num_of_colums; i++)
        for (int j = 0; j < num_of_rows; j++)
            reset_grid(grid_node_map[i][j]);
}


// 100 中心点
//  99 内径
//  98 外径
inline double Astar::obstacle_score(const int &idx, const int &idy) const
{
    if (idx < 0 || idx >= num_of_colums ||
        idy < 0 || idy >= num_of_rows)
    {
        ROS_ERROR("wrong at Astar::obstacle_score()");
        exit(0);
        return 1.0;
    }
    int x = map_data[idx + idy * num_of_colums];
    if (x >= 98)
        return 100;
    else
        return x / 98.0 + 1;
}

bool Astar::is_occupied(Eigen::Vector2d pt){
    Vector2i idx = world2grid(pt);
    return is_occupied(idx);
    return false;
}

inline bool Astar::is_occupied(const int &idx, const int &idy) const
{
    return (
        idx < 0 || idx >= num_of_colums ||
        idy < 0 || idy >= num_of_rows ||
        (map_data[idx + idy * num_of_colums] >= 98));
}

inline bool Astar::is_free(const int &idx, const int &idy) const
{
    return (
        idx >= 0 && idx < num_of_colums &&
        idy >= 0 && idy < num_of_rows &&
        (map_data[idx + idy * num_of_colums] == 0));
}
//----------------------------------------------------------------------------------------------------------

inline bool Astar::is_occupied(const Eigen::Vector2i &index) const
{
    return is_occupied(index(0), index(1));
}

inline bool Astar::is_free(const Eigen::Vector2i &index) const
{
    return is_free(index(0), index(1));
}

vector<Vector2d> Astar::get_visited_nodes()
{
    vector<Vector2d> visited_nodes;
    for (int i = 0; i < num_of_colums; i++)
    {
        for (int j = 0; j < num_of_rows; j++)
        {
            if (grid_node_map[i][j]->set_type == -1) // visualize nodes in close list only
                visited_nodes.push_back(grid_node_map[i][j]->coordinate);
        }
    }

    INFO("visited_nodes size : %d", visited_nodes.size());

    return visited_nodes;
}


double Astar::get_heuristic_sorce(GridNodePtr node1, GridNodePtr node2)
{
    Vector2i start_index = node1->index;
    Vector2i end_index = node2->index;

    double h;
    double distance[2];
    distance[0] = abs((double)(start_index(0) - end_index(0)));
    distance[1] = abs((double)(start_index(1) - end_index(1)));
    h = distance[0] + distance[1] + (1.414 - 2.0) * min(distance[0], distance[1]);
    double o1 = obstacle_score(node2->index(0), node2->index(1));
    double o2 = obstacle_score(node1->index(0), node1->index(1));
    h += h * (o1 * 0.7 + o2 * 0.3);

    double tie_breaker = 1 / 25;
    h = h * (1.0 + tie_breaker);
    return h;
}

inline void Astar::get_neighbor(GridNodePtr current_ptr, vector<GridNodePtr> &_neighbor_ptr_set, vector<double> &_edge_cost_set)
{
    _neighbor_ptr_set.clear();
    _edge_cost_set.clear();

    Vector2i current_index = current_ptr->index;
    int current_x = current_index[0];
    int current_y = current_index[1];

    int n_x, n_y, n_z;
    GridNodePtr tmp_ptr = NULL;
    Vector2i tmp_index;

    for (int i = -1; i <= 1; ++i)
    {
        for (int j = -1; j <= 1; ++j)
        {
            if (i == 0 && j == 0)
            {
                continue;
            }

            n_x = current_x + i;
            n_y = current_y + j;

            if ((n_x < 0) || (n_y < 0) || (n_x > num_of_colums - 1) || (n_y > num_of_rows - 1))
            {
                continue;
            }

            if (is_occupied(n_x, n_y))
            {
                continue;
            }

            tmp_ptr = grid_node_map[n_x][n_y];

            double dist = get_heuristic_sorce(current_ptr, tmp_ptr);
            _neighbor_ptr_set.push_back(tmp_ptr);
            _edge_cost_set.push_back(dist);
        }
    }
}


void Astar::search(Vector2d start_pt, Vector2d end_pt)
{
    // 记录路径搜索需要的时间
    struct timespec time_begin, time_end;
    clock_gettime(CLOCK_REALTIME, &time_begin);


    //记录起点和终点对应的栅格坐标
    Vector2i start_idx = world2grid(start_pt);
    Vector2i end_idx =   world2grid(end_pt);

    //目标索引
    goal_idx = end_idx;

    //起点和终点的位置
    Vector2d start_pose = grid2world(start_idx);
    Vector2d end_pose = grid2world(end_idx);

    //初始化起点和终点节点

    GridNodePtr start_node_ptr = grid_node_map[start_idx(0)][start_idx(1)];
    GridNodePtr end_node_ptr = grid_node_map[end_idx(0)][end_idx(1)];


    //待弹出点集
    open_set.clear();

    //计算启发函数
    start_node_ptr->g_score = 0;
    start_node_ptr->f_score = get_heuristic_sorce(start_node_ptr, end_node_ptr);

    //将起点加入开集，自由点为0，闭集为-1，开集为1
    start_node_ptr->set_type = 1;
    start_node_ptr->coordinate = start_pose;
    open_set.insert(make_pair(start_node_ptr->f_score, start_node_ptr));

    //预先定义拓展队列和cost队列
    vector<GridNodePtr> neighbor_ptr_set;
    vector<double> edge_cost_set;

    //note: after A* searching. the map should be reset. since most of gridnode's openset is 0.
    //if the new global map is comeing, the map houle be  update auto. otherwise, remeber reset the grid map. 
    while (!open_set.empty())
    {

        //弹出最小f的节点
        current_node_ptr = open_set.begin()->second;
        current_node_ptr->set_type = -1; // 标记为闭集 表示已经访问

        // 从开集容器中移除   
        open_set.erase(open_set.begin());

        //if reach the goal.
        if (current_node_ptr->index == goal_idx){
            clock_gettime(CLOCK_REALTIME, &time_end);
            terminate_node_ptr = current_node_ptr;
            ROS_WARN("A* sucess find a path in %f ms", get_diff_nsec(time_end, time_begin) * 1000.0);
            return;
        }

        //获取拓展集合
        get_neighbor(current_node_ptr, neighbor_ptr_set, edge_cost_set);
        if (neighbor_ptr_set.size() == 0)
        {
            INFO("index:%d,%d neighbor_ptr_set size 0\n", current_node_ptr->index(0), current_node_ptr->index(1));
        }


        //遍历拓展集合
        for (int i = 0; i < (int)neighbor_ptr_set.size(); i++)
        {
            neighbor_node_ptr = neighbor_ptr_set[i];
            double gh = current_node_ptr->g_score + edge_cost_set[i];
            double fh = gh + get_heuristic_sorce(neighbor_node_ptr, end_node_ptr);
      
            //如果为自由节点 未知节点
            if (neighbor_node_ptr->set_type == 0)
            {
                //计算相应的g和f，并加入opensets
                neighbor_node_ptr->set_type = 1;
                neighbor_node_ptr->g_score = gh;
                neighbor_node_ptr->f_score = fh;
                neighbor_node_ptr->came_from = current_node_ptr;
                open_set.insert(make_pair(neighbor_node_ptr->f_score, neighbor_node_ptr));
                continue;
                
            }
            else if (neighbor_node_ptr->set_type == 1)
            {
                // 如果已经在openlist里面，且最新得分比之前的小要更新
                if (neighbor_node_ptr->g_score > gh)
                {
                    // 更新对应的f值
                    neighbor_node_ptr->g_score = gh;
                    neighbor_node_ptr->f_score = fh;
                    neighbor_node_ptr->came_from = current_node_ptr;
                    open_set.insert(make_pair(neighbor_node_ptr->f_score, neighbor_node_ptr));
                }
            }
            else
            {
                // 如果是closelist里面的则不做处理
                continue;
            }
        }
    }

    // if search fails
    clock_gettime(CLOCK_REALTIME, &time_end);
    if (get_diff_nsec(time_begin, time_end) > 0.1)
        INFO("Astar no result. time elapsed:%f", get_diff_nsec(time_begin, time_end));
}

double Astar::get_diff_nsec(timespec now, timespec last)
{
    double sec = now.tv_sec - last.tv_sec;
    double nsec = (now.tv_nsec - last.tv_nsec) / 1000000000.0;
    return sec + nsec;
}



vector<Vector2d> Astar::get_path()
{
    vector<Vector2d> path;
    vector<GridNodePtr> grid_path;

    GridNodePtr tmp_ptr = terminate_node_ptr;
    if(tmp_ptr == NULL)
        return path;
    while (tmp_ptr->came_from != NULL)
    {
        grid_path.push_back(tmp_ptr);
        tmp_ptr = tmp_ptr->came_from;
    }

    for (auto ptr : grid_path)
        path.push_back(ptr->coordinate);

    //加入第一个；
    path.push_back(tmp_ptr->coordinate);

    reverse(path.begin(), path.end());

    //INFO("pathsize:%d\n", grid_path.size());
    return path;
}

vector<Vector2d> Astar::plan(Vector2d start_pt, Vector2d end_pt){
    search(start_pt, end_pt);
    
    vector<Vector2d> path = get_path();
    reset_used_grids();     //need reset the map here.since most gridnode's status is 1.
    return path ;
}


Vector2i Astar::world2grid(Vector2d pos){
    Vector4d pos_in_world;  pos_in_world<<pos(0),pos(1),0,1.0;
    Vector4d pos_in_map = t_map_world_.inverse() * pos_in_world;

    int col = std::min(std::max(int(pos_in_map(0)* inv_resolution),0),int(num_of_colums - 1));
    int row = std::min(std::max(int(pos_in_map(1)* inv_resolution),0),int(num_of_rows - 1));

    //ROS_WARN_STREAM("t_map_world_ /n"<<t_map_world_);

    Vector2i index; index<<col,row;
    return index;
}


Vector2d Astar::grid2world(Vector2i index){
    double x_in_map = index(0) * resolution;
    double y_in_map = index(1) * resolution;

    Vector4d pos_in_map; pos_in_map<<x_in_map, y_in_map, 0,1.0;
    Vector4d pos_in_world = t_map_world_*pos_in_map;

    Vector2d pos; pos<<pos_in_world(0),pos_in_world(1);
    return pos;
}

int Astar::grid2index(int col, int row){
    return col + row * num_of_colums;
}

int Astar::grid2index(Vector2i index){
    return grid2index(index(0),index(1));
}




