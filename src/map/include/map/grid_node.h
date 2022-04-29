/*
 * @Description: 
 * @Autor: 
 * @Date: 2021-12-23 12:11:35
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2022-01-04 15:56:53
 */
#ifndef _GRID_NODE_H_
#define _GRID_NODE_H_

#include <iostream>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <map>


using namespace Eigen;

#define inf 1 >> 20

struct GridNode;
typedef GridNode *GridNodePtr;

struct GridNode
{
    int set_type; //自由点为0 闭集为-1 开集为1
    Vector2d coordinate;
    Vector2i direction; // direction of expanding
    Vector2i index;

    double g_score, f_score;
    GridNodePtr came_from;
    std::multimap<double, GridNodePtr>::iterator node_iterator;

    GridNode(Vector2i _index, Vector2d _coordinate)
    {
        set_type = 0;
        index = _index;
        coordinate = _coordinate;
        direction = Vector2i::Zero();

        g_score = inf;
        f_score = inf;
        came_from = NULL;
    }

    GridNode(){};
    ~GridNode(){};
};



#endif