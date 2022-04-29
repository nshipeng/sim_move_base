/*
 * @Description: 
 * @Autor: 
 * @Date: 2022-01-04 15:07:46
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2022-01-07 10:25:47
 */
#ifndef __ASTAR_H_
#define __ASTAR_H_
//--------------------sys----------------------
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <unistd.h>
#include <string>
#include <math.h>
#include <unistd.h>
#include <vector>
#include <thread>
#include <mutex>
#include <fcntl.h>
#include <stdbool.h>
#include <queue>
#include <algorithm> // fill
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <ros/ros.h>
#include "map/grid_node.h"
#include <nav_msgs/OccupancyGrid.h>

using namespace std;
using namespace Eigen;

class Astar
{
public:
    Astar(){};
    ~Astar(){};

    //main API
    void init(const nav_msgs::OccupancyGrid& global_costmap);
    void init(const nav_msgs::OccupancyGrid& global_costmap, ros::NodeHandle& _nh);
    vector<Vector2d> plan(Vector2d start_pt, Vector2d end_pt);

    //used for collision check. pt is (x,y) in {world}.
    bool is_occupied(Eigen::Vector2d pt);
    bool is_safe(vector<Vector2d>& tarj);

private:

   ros::NodeHandle nh;

    //global costmap 
    vector<uint8_t> map_data;
    double resolution, inv_resolution;
    int num_of_colums, num_of_rows; 
    Matrix4d t_map_world_;
    nav_msgs::OccupancyGrid global_costmap_;

    //grid map for A*
    vector<vector<GridNodePtr>> grid_node_map;
    GridNodePtr terminate_node_ptr;
    GridNodePtr current_node_ptr = NULL;
    GridNodePtr neighbor_node_ptr = NULL;
    Vector2i goal_idx;
    std::multimap<double, GridNodePtr> open_set;


    //main A* search function..
    void search(Vector2d start_pt, Vector2d end_pt);
    vector<Vector2d> get_visited_nodes();
    double get_heuristic_sorce(GridNodePtr node1, GridNodePtr node2);
    void get_neighbor(GridNodePtr current_ptr, vector<GridNodePtr> &_neighbor_ptr_set, vector<double> &_edge_cost_set);
    vector<Vector2d> get_path();
    double obstacle_score(const int &idx, const int &idy) const;      //when cal heuristic, we also consider the grid cell's int8 value.


    /**
     * @description: check grid cell is free or occupied
     */    
    bool is_occupied(const int &idx, const int &idy) const;
    bool is_free(const int &idx, const int &idy) const;
    bool is_occupied(const Eigen::Vector2i &index) const;
    bool is_free(const Eigen::Vector2i &index) const;

     /**
     * @description: convert (x,y) position in {world} to (col, row) in grid map.
     */    
    Vector2i world2grid(Vector2d pos);
    
    /**
     * @description: convert (col row) in grid map to (x,y) position in {world}. 
     */    
    Vector2d grid2world(Vector2i index);

    /**
         * @description: convert (col,row) in grid map to map's vector index;
    */        
    int grid2index(int col, int row);
    int grid2index(Vector2i index);



    //reset_grid
    void reset_grid(GridNodePtr ptr);
    void reset_used_grids();
    void deinit();

    double get_diff_nsec(timespec now, timespec last);

    
  
};

#endif