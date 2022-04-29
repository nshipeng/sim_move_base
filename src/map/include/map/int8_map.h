/*
 * @Description: int8_t map. 
 * @Autor: 
 * @Date: 2021-12-23 12:13:28
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2022-01-05 11:25:56
 */


#ifndef _INT8_MAP_H_
#define _INT8_MAP_H_
//--------------------sys----------------------
#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <nav_msgs/OccupancyGrid.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include "ros/ros.h"
#include "utils/timer_utils.h"
#include "utils/planning_visualization.h"
#include <geometry_msgs/PointStamped.h>
#include <algorithm>
#include "grid_node.h"



using namespace Eigen;

class int8_map
{
    public:
        int8_map();
        ~int8_map();
        void init(std::string _frame_id, double _robot_radius, double _truncated_distance, double _factor,ros::NodeHandle& _nh);

        /**
         * @description: init the grid map. used for A*.
         * @param {*}
         * @return {*}
         * @author: 
         */        
        void init_grid_map();

        /**
         * @description: once a new map is comeing. update the map.
         * @param {OccupancyGrid} &map
         */        
        void update(const nav_msgs::OccupancyGrid &map);

        
        /**
         * @description: Add obs info to map.
         * @param {*}
         * @return {*}
         * @author: 
         */        
        void add_obs_info(const nav_msgs::OccupancyGrid::_data_type &map,Vector2d p1, Vector2d p2);

        /**
         * @description: remove the dynamic obs add by click points..
         */        
        void remove_dynamic_obs();

        /**
         * @description: inflation the global map.
         * @param {_data_type} &map
         * @param {_data_type} &costmap
         */        
        void inflation_global_map(const nav_msgs::OccupancyGrid::_data_type &map, nav_msgs::OccupancyGrid::_data_type &costmap);

        /**
         * @description: inflation the local map.
         * @param {_data_type} &map
         * @param {_data_type} &costmap
         */        
        void inflation_local_map(const nav_msgs::OccupancyGrid::_data_type &map, nav_msgs::OccupancyGrid::_data_type &costmap);
        
        /**
         * @description:cal the cost of a cell accroding to the distance  
         * @param {*}
         * @return {*} 
         */        
        int8_t cal_cost(int8_t map_data, double distance);

        /**
         * @description: callback function for receive map.
         * @param {OccupancyGrid&} map
         * @return {*}
         * @author: 
         */        
        void map_receive_callback(const nav_msgs::OccupancyGrid& map);

        /**
         * @description: used for add daynamic obs. 2 points construct a dynamic obs.  
         * @param {*}
         * @return {*}
         * @author: 
         */        
        void click_point_callback(const geometry_msgs::PointStamped& point);

         
        /**
         * @description: convert (x,y) in {world} to (col, row) in grid map.
         * @param {double&} x
         * @param {double&} y
         * @param {double&} row
         * @param {double&} col
         */        
        void world2grid(double x, double y, int& row, int& col);
        Vector2i world2grid(double x, double y);


        /**
         * @description: convert  (col, row) in grid map to (x,y) in {world}.
         * @param {double&} x
         * @param {double&} y
         * @param {double&} row
         * @param {double&} col
         */        
        void grid2world(double& x, double& y, int row, int col);
        Vector2d grid2world(int row, int col);


        /**
         * @description: convert (col,row) in grid map to map's vector index;
         * @param {double&} row
         * @param {double&} col
         * @return {*}
         * @author: 
         */        
        int grid2index(int row, int col);


        /**
         * @description: check weather a grid cell is free or occupied. 
         * @param {int} row
         * @param {int} col
         * @return {*}
         * @author: 
         */
        bool is_occupied(int row, int col);
        bool is_occupied(Vector2i index);
        bool is_free(int row, int col);
        bool is_free(Vector2i index);
        


    //private:

        ros::NodeHandle nh;

        // the resolution of the map
        double resolution;
        double inv_resolution;
        
        //the width and height of the map. this data can be read from nav_msgs/OccupancyGrid msg.
        double width;
        double height;

        std::string frame_id;

        //the rows and colums number of the map.
        double num_of_rows;
        double num_of_colums;

        //approximate robot radius used for calculate map's cost value.
        double inflate_radius;
        
        //truncated distance. here only update the cost of map within truncated distance. this para like inflate_radius. if we increase truncated distance
        //the global map will inflate more.
        double truncated_;

        //the param used for cal the cost of a cell of gird map.  see http://wiki.ros.org/costmap_2d part 6. 
        double alpha;

        nav_msgs::OccupancyGrid local_map_copy;
        nav_msgs::OccupancyGrid global_map_copy;
        nav_msgs::OccupancyGrid global_costmap;
        nav_msgs::OccupancyGrid local_costmap;
        nav_msgs::OccupancyGrid current_map;

        //publisher and subscribers
        std::shared_ptr<ros::Publisher> global_map_pub_;
        std::shared_ptr<ros::Publisher> local_map_pub_;
        ros::Subscriber map_sub_;
        ros::Subscriber click_point_sub_;

        //mutex for update map
        std::mutex map_mutex;

        bool add_obs_flag;

        //The origin of the map [m, m, rad].  This is the real-world pose of the cell (0,0) in the world(the origin of rviz)..
        Matrix4d t_map_world_;

        //used for add dynamic obs
        int point_idx = 0;
        Vector2d click_p1, click_p2;
        std::vector<int> obs_indexs;  //maybe a set data structure can use here.
        std::vector<int> marker_ids;
        

        //visual help
        std::shared_ptr<PlanningVisualization> visual_help_ptr_ ;


        //grid node map data structure. used for A* .
        vector<vector<GridNodePtr>>grid_node_map;
        
        //given row and colum. calculate weather the grid node is occupied or free..
        vector<uint8_t>grid_map_data;
        
};


#endif
