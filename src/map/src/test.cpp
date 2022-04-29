/*
 * @Description: map class test.
 * @Autor: 
 * @Date: 2021-12-23 14:51:25
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2022-01-11 20:55:15
 */

#include "map/int8_map.h"

int main(int argc, char** argv){
    ROS_INFO("HELLO MAP");

    ros::init(argc, argv, "map_test_node");
    ros::NodeHandle nh;

    std::shared_ptr<int8_map> map_ptr_ = std::make_shared<int8_map>();
    map_ptr_->init("map",0.5,0.5,10,nh);

    ros::spin();

    ros::Duration(0.1).sleep();
    


    ros::Rate rate(10);
    while(ros::ok()){

       ros::spinOnce();
       rate.sleep();
    }
    
    return 0;
}
