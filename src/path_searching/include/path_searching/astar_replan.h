/*
 * @Description: 
 * @Autor: 
 */
#ifndef _ASTAR_REPLAN_H_
#define _ASTAR_REPLAN_H_

#include <ros/ros.h>
#include <math.h>
#include <string>
#include <vector>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <functional>  // To use std::bind
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <thread>
#include "path_searching/astar.h"
#include "utils/planning_visualization.h"
#include "traj_opt/non_uniform_bspline.h"
#include "traj_opt/DoubleS.h"

#include "control/rk4.h"
#include "control/mpc.h"
#include <rviz_visual_tools/rviz_visual_tools.h>



using namespace Eigen;


enum STATE{
    INIT,
    WAIT_TRAGET,
    GEN_NEW_TRAJ,
    REPLAN_TRAJ,
    EXEC_TRAJ,
    ROTATE,
    REPLAN
};



class AstarReplan
{
    public:
       
       AstarReplan(){}
       ~AstarReplan(){}

       void init(ros::NodeHandle& nh);

       
        /* helper functions */
        bool callAstarReplan(Vector2d start_pt, Vector2d end_pt); 
        void genTraj(vector<Vector2d>& path);

                                             
        void changeExecState(STATE new_state);
        void printExecState();

        /* ROS functions */
        void execCallback(const ros::TimerEvent& e);
        void checkCollisionCallback(const ros::TimerEvent& e);
        void waypointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
        void initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

        //receive global costmap.
        void updateMapCallback(const nav_msgs::OccupancyGrid& map);

        //function for traj fllowing.
        void ref_Traj(Eigen::Matrix<double, 3, 10> & ref_traj);
        void rviz_show_ref_Traj(const vector<Isometry3d>& axis_points);
        void mpc_control( Eigen::Matrix<double, 3, 1>&x0, Eigen::Matrix<double, 3, 20> & ref_traj,double& u);
        void broadcasterTf(Ref<VectorXd> state);

        void model(const Ref<VectorXd> x0, const Ref<VectorXd> u,  Ref<VectorXd> x_out);
        void nextState(Ref<VectorXd> x0, const Ref<VectorXd> u,Ref<VectorXd> state);

        void planYaw();
        void get_cartograper_pose();

        //diff case. [v,w] is 2d 
        void PID(Vector3d path_point,Vector2d& u);




    private:
       
        Vector2d start_pt;
        Vector2d goal_pt;
        Vector2d current_pt;

        Vector3d pose;
        Vector2d vw;     //represent v and w  

        STATE state;
        bool trigger_, have_target, receive_map;
        bool safe;
        high_resolution_clock::time_point t0;  //the zero time of a execueted traj.


        /* ROS utils */
        ros::NodeHandle nh;
        ros::Timer exec_timer_, map_timer_,safe_timer_;
        Eigen::MatrixXd ctrl_pts;

 
        //map Parameters
        nav_msgs::OccupancyGrid grid_map;    // Initialize grid_map outside
        nav_msgs::OccupancyGrid grid_map_copy;
        std::vector<int8_t> map_data;             // rviz representation of the grid
        std::vector<int8_t> map_data_copy;  


        double resolution;                   // the resolution of the map
        double inv_resolution;
        double num_of_rows;                  //the rows and colums number of the map.
        double num_of_colums;
        Matrix4d t_map_world_;
        std::string frame_id ;               //"map" is the default id


        // A*  Path
        std::vector<Vector2d> path;
        NonUniformBspline nurbs;
        std::vector<Vector3d> points;        //record the path point

        ros::Subscriber goal_sub_;
        ros::Subscriber global_costmap_sub_;
        ros::Subscriber init_pose_sub_;
        ros::Publisher cmd_vel_pub_;

        //A* ptr.
        std::shared_ptr<Astar>astar_ptr_;
        std::shared_ptr<PlanningVisualization> visual_help_ptr_;

        tf::TransformListener p_carto_tf_listener;

        //used for rotate .
        std::shared_ptr<DoubleS> ds_ptr_;
        double rotate_distance;
        double max_rotate_threshold = 0.78;
        double min_rotate_threshold = 0.1;

        //pid parameter
        double kp_v = 0.6;
        double kd_v = 0.1;
        double ki_v = 0.0;

        double distance_error = 0.0;
        double distance_error_dot = 0.0;
        double last_distance_error = 0.0;
        double llast_distance_error = 0.0;   //last last distance error

        double kp_theta = -0.3;
        double kd_theta = -0.15;
        double ki_theta = 0.0;
        
        double theta_error = 0.0;
        double theta_error_dot = 0.0;
        double last_theta_error = 0.0;
        double llast_theta_error = 0.0;

        //for visualization
         rviz_visual_tools::RvizVisualToolsPtr ref_traj_visual_tools;
        

        
};



#endif