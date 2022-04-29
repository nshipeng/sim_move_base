/*
 * @Description: Sim a mobile robot.
 * @Autor: 
 * @Date: 2021-12-29 16:14:49
 * @LastEditors: Please set LastEditors
 * @LastEditTime: 2021-12-29 16:21:47
 */

#ifndef _MOBILE_ROBOT_H_
#define _MOBILE_ROBOT_H_


class mobot
{
    public:
        mobot(/* args */);
        ~mobot();


        void init();

        void plan();


        void execute();

        void init_pose_estimate_callback();


        

    private:
        //a pointer to status_receive
        //a pointer to mbot_control_pointer

        
};


















#endif




