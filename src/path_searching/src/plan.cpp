/*
 * @Description: 
 * @Autor: 
 * @Date: 2022-01-04 20:17:09
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2022-01-05 15:18:05
 */

#include "map/int8_map.h"
#include "path_searching/astar.h"
#include "utils/planning_visualization.h"
#include "path_searching/astar_replan.h"

int main(int argc, char** argv){
    ROS_INFO("HELLO A*");

    ros::init(argc, argv, "plan_node");
    ros::NodeHandle nh;

    AstarReplan astar_replan;
    astar_replan.init(nh);

    ros::spin();
    return 0;
}
