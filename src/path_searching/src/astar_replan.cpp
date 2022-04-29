/*
 * @Description: 
 * @Autor: 
 */

#include "path_searching/astar_replan.h"

void AstarReplan::init(ros::NodeHandle& nh_){
    nh = nh_;
    state = STATE::INIT;
    safe = true;
    trigger_ = false;
    have_target = false;
    receive_map = false;
    t_map_world_.setIdentity();

    astar_ptr_ = std::make_shared<Astar>();
    visual_help_ptr_ = std::make_shared<PlanningVisualization>(nh_, "map");
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1,  &AstarReplan::waypointCallback,this);
    global_costmap_sub_ = nh_.subscribe("/global_costmap", 1,  &AstarReplan::updateMapCallback,this);
    init_pose_sub_ = nh_.subscribe("/initialpose", 1,  &AstarReplan::initPoseCallback,this);

    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel",1000);

    safe_timer_ = nh_.createTimer(ros::Duration(0.5), &AstarReplan::checkCollisionCallback, this);
    exec_timer_ = nh_.createTimer(ros::Duration(0.01), &AstarReplan::execCallback, this);

    ds_ptr_ = std::make_shared<DoubleS>(0.5,0.5,0.5);
    
}

void AstarReplan::waypointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){

  // ROS_WARN_STREAM("nav goal pos x is : "<<msg->pose.position.x);
  // ROS_WARN_STREAM("nav goal pos y is : "<<msg->pose.position.y);

  trigger_ = true;
  have_target = true;
  goal_pt(0) = msg->pose.position.x;
  goal_pt(1)= msg->pose.position.y;
  visual_help_ptr_->drawGoal({goal_pt(0), goal_pt(1),0.0},0.1,Eigen::Vector4d(1,0,0,1),1);
  if(state == WAIT_TRAGET){
    changeExecState(STATE::GEN_NEW_TRAJ);
  }
  else if(state == EXEC_TRAJ){
    changeExecState(STATE::REPLAN);
  }
}

void AstarReplan::updateMapCallback(const nav_msgs::OccupancyGrid& map){
      ROS_INFO("update A* map..");
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
      
      grid_map = map;
      astar_ptr_ ->init(map,nh);
      map_data.assign(map.data.begin(),map.data.end());
      receive_map = true;
      trigger_ = true;

}

void AstarReplan::changeExecState(STATE new_state){
  state = new_state;
  if(new_state = STATE::EXEC_TRAJ){
    t0 = std::chrono::high_resolution_clock::now();
  }
}

void AstarReplan::execCallback(const ros::TimerEvent& e){
  switch (state)
  {
    
    case INIT:{
      if(!trigger_ ){
        return;
      }
      changeExecState(STATE::WAIT_TRAGET);
      break;
    }
    
    case WAIT_TRAGET:{
      if(!have_target){
        ROS_INFO("wait traget.");
        sleep(2);
        return;
      }{
        changeExecState(STATE::GEN_NEW_TRAJ);
      }
      break;
    }


    case GEN_NEW_TRAJ:{
      //first of all.. old data must be clear.
      points.clear();

      auto t_s = high_resolution_clock::now();
      double t = 0;

      get_cartograper_pose();
      start_pt(0) = pose(0); start_pt(1) = pose(1);
      path = astar_ptr_->plan(start_pt,goal_pt);
      auto t_end = high_resolution_clock::now();
      auto t_ = std::chrono::duration<double>(t_end - t_s);
      t = t_.count();

      if(path.size() ==0 ){
        ROS_WARN("A* plan fail.");
        changeExecState(STATE::GEN_NEW_TRAJ);
      }else{
        //ROS_WARN("A* find a path. cost time is %fs",t);
        genTraj(path);
        visual_help_ptr_->drawBspline(nurbs,0.05,Eigen::Vector4d(1,1,0,0.4),false,0.02,Eigen::Vector4d(1,0,0,1));
        changeExecState(STATE::EXEC_TRAJ);
      }
      break;
    }

    case EXEC_TRAJ:{
      auto t_now = std::chrono::high_resolution_clock::now();
      auto t_ = std::chrono::duration<double>(t_now - t0);
      double t = t_.count();

      double  tm, tmp;
      nurbs.getTimeSpan(tm, tmp);
      t = std::min(t, tmp);
      Vector3d  pt = nurbs.evaluateDeBoorT(t);
      current_pt(0) = pt(0); current_pt(1) = pt(1);
    

      if(t >= tmp){// && distance_error < 0.2 && theta_error<0.2
        //stop.
        {
          geometry_msgs::Twist cmd_vel;
          cmd_vel.linear.x = 0.0 ;
          cmd_vel.linear.y = 0;
          cmd_vel.linear.z = 0.0;
          cmd_vel.angular.x = 0.0;
          cmd_vel.angular.y = 0.0;
          cmd_vel.angular.z = 0.0;
          cmd_vel_pub_.publish(cmd_vel);
          ros::Rate loop_rate(100);
          loop_rate.sleep();
        }
    
        have_target = false;
        changeExecState(STATE::WAIT_TRAGET);
        return;
      }else{
        visual_help_ptr_->displayCylinderList(pt,0);
        //traj track
        {  
          get_cartograper_pose();
          Eigen::Matrix<double, 3,10> ref_traj;
          ref_Traj(ref_traj);

          rotate_distance = ref_traj.col(0)(2,0) - pose(2);
          ROS_WARN_STREAM(""<<ref_traj.col(0)(2,0)<<" "<<pose(2));
          if(abs(rotate_distance) > max_rotate_threshold){
             //ROS_ERROR("the angle between current theta and ref theta is too big.");
            //  changeExecState(STATE::ROTATE);
            //  return;
          }
          Eigen::Matrix<double, 3, 1>x0; x0<<pose(0),pose(1),pose(2);
          Vector3d path_point;           path_point<<ref_traj.col(0)(0,0),ref_traj.col(0)(1,0),ref_traj.col(0)(2,0);
          Vector2d u;

          //MPC(x0,ref_traj,u);
          //DIFF_MPC(x0,ref_traj,u);
          PID(path_point,u);
          ROS_WARN_STREAM("u is \n"<<u);
          {
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x =u(0) ;
            cmd_vel.linear.y = 0;
            cmd_vel.linear.z = 0.0;
            cmd_vel.angular.x = 0.0;
            cmd_vel.angular.y = 0.0;
            cmd_vel.angular.z = u(1);
            cmd_vel_pub_.publish(cmd_vel);
          }
          //nextState( x0,  u, pose);
          //broadcasterTf(pose);
        }
      }
      break;
    }
    case ROTATE:{
      // get_cartograper_pose();
      // Eigen::Matrix<double, 3,10> ref_traj;
      // ref_Traj(ref_traj);
      // rotate_distance = ref_traj.col(0)(2,0) - pose(2);

      // ds_ptr_->plan(0,rotate_distance,0.0,0.0,0.99);
      // double total_t = ds_ptr_->get_traj_duration();
      // auto t_s = high_resolution_clock::now();
      // double t = 0;

      // while(t<total_t){
      //     auto t_now = high_resolution_clock::now();
      //     auto t_ = std::chrono::duration<double>(t_now - t_s);
      //     t = t_.count();

      //     double w = ds_ptr_->get_traj(t)[1];

      //     Eigen::Matrix<double, 3, 1>x0; x0<<pose(0),pose(1),pose(2);
      //     Vector2d u;
      //     u(0) = 0.0; u(1) = w;
      //     {
      //       geometry_msgs::Twist cmd_vel;
      //       cmd_vel.linear.x = u(0) ;
      //       cmd_vel.linear.y = 0;
      //       cmd_vel.linear.z = 0.0;
      //       cmd_vel.angular.x = 0.0;
      //       cmd_vel.angular.y = 0.0;
      //       cmd_vel.angular.z = u(1);
      //       cmd_vel_pub_.publish(cmd_vel);
      //     }
      //     nextState( x0,  u, pose);
      //     //broadcasterTf(pose);
      // }
      // {
      //   geometry_msgs::Twist cmd_vel;
      //   cmd_vel.linear.x = 00 ;
      //   cmd_vel.linear.y = 0;
      //   cmd_vel.linear.z = 0.0;
      //   cmd_vel.angular.x = 0.0;
      //   cmd_vel.angular.y = 0.0;
      //   cmd_vel.angular.z = 0.0;
      //   cmd_vel_pub_.publish(cmd_vel);
      // }


      // changeExecState(STATE::REPLAN);
      return;
    }

    case REPLAN:{

      points.clear();

      get_cartograper_pose();
      start_pt(0) = pose(0); start_pt(1) = pose(1);
      //start_pt = current_pt;
      path = astar_ptr_->plan(start_pt,goal_pt);
    
      if(path.size()==0){
        changeExecState(STATE::GEN_NEW_TRAJ);
        return;
      }else{
        genTraj(path);
        visual_help_ptr_->drawBspline(nurbs,0.05,Eigen::Vector4d(1,1,0,0.4),false,0.02,Eigen::Vector4d(1,0,0,1));
        changeExecState(STATE::EXEC_TRAJ);
      }
      break;
      
    }
    
  }

}

void AstarReplan::initPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    ROS_WARN("get estimate robot pose.");
    start_pt(0) = msg->pose.pose.position.x;
    start_pt(1) = msg->pose.pose.position.y;
    current_pt = start_pt;
    ROS_WARN_STREAM("start_pt at world is "<<start_pt(0)<<" "<<start_pt(1));
    pose(0) =  msg->pose.pose.position.x;
    pose(1) =  msg->pose.pose.position.y;

    Quaterniond q;
    q.x() = msg->pose.pose.orientation.x; q.y() = msg->pose.pose.orientation.y;
    q.z() = msg->pose.pose.orientation.z; q.w() = msg->pose.pose.orientation.w;
    Eigen::AngleAxisd axis_angle(q);
    pose(2) =  axis_angle.angle();
    //ROS_WARN_STREAM(" "<<axis_angle.angle()<<"\n "<<axis_angle.axis());


    broadcasterTf(pose);
    visual_help_ptr_->drawGoal({start_pt(0), start_pt(1),0.0},0.1,Eigen::Vector4d(1,0,0,1),0);
}


void AstarReplan::genTraj(vector<Vector2d>& path){
    if(path.size() ==0){
      ROS_WARN("empty path.");
      return;
    }
    points.clear();

    //fiter the path to n points
    //https://github.com/ros-industrial/industrial_core/blob/melodic-devel/industrial_trajectory_filters/src/n_point_filter.cpp
    
    int n_points_ =int(path.size() /5) ;
    if(path.size() > n_points_){
        points.push_back({path.front()(0), path.front()(1),0});
        int intermediate_points = n_points_ - 2;

        double int_point_increment = double(path.size()) / double(intermediate_points + 1.0);
        //ROS_INFO_STREAM( "Number of intermediate points: " << intermediate_points << ", increment: " << int_point_increment);

        for (int i = 1; i <= intermediate_points; i++)
          {
            int int_point_index = int(double(i) * int_point_increment);
            //ROS_INFO_STREAM("Intermediate point index: " << int_point_index);
            points.push_back({path[int_point_index](0), path[int_point_index](1),0.0});
          }
        
        points.push_back({path.back()(0), path.back()(1),0});
        ROS_INFO_STREAM("filter path from: " << path.size() << " to: " << points.size());

    }else{
        ROS_INFO_STREAM("Path size less than " << n_points_);
        points.push_back({path.front()(0), path.front()(1),0});
        int idx= path.size() - 2;
        for (int i = 1; i <= idx; i++)
          {
            if(i % 2 == 0){
              points.push_back({path[i](0), path[i](1),0.0});
            }
          }
        
        points.push_back({path.back()(0), path.back()(1),0});
        ROS_INFO_STREAM("Filtered path from: " << path.size() << " to: " << points.size());
    }

    

    // for(auto point:path){
    //   points.push_back({point(0), point(1),0});
    // }

    std::vector<Eigen::Vector3d> start_end_derivatives;
    start_end_derivatives.push_back({0,0,0});
    start_end_derivatives.push_back({0,0,0});
    start_end_derivatives.push_back({0,0,0});
    start_end_derivatives.push_back({0,0,0});

    //ROS_WARN_STREAM("ctrl_pts size : "<<ctrl_pts.size());
    double ts = 1;
    NonUniformBspline::parameterizeToBspline(ts, points, start_end_derivatives, ctrl_pts);
    NonUniformBspline init(ctrl_pts, 3, ts);

    nurbs = NonUniformBspline(ctrl_pts, 3, ts);
}



void AstarReplan::checkCollisionCallback(const ros::TimerEvent& e){

  if(state == STATE::EXEC_TRAJ){

    auto t_now = std::chrono::high_resolution_clock::now();
    auto t_ = std::chrono::duration<double>(t_now - t0);
    double t = t_.count();

    double  tm, tmp;
    nurbs.getTimeSpan(tm, tmp);
    t = std::min(t, tmp);

    //check next 5s' s trajectory is safe.
    vector<Eigen::Vector2d> check_points;
    for(double dt = 0; dt<5.0; dt+=0.1){
       Vector3d  pt = nurbs.evaluateDeBoorT(t + dt);
       Vector2d p;  p(0) =pt(0); p(1) = pt(1);
       check_points.push_back(p);
    }
    
    for(int i = 0;i<check_points.size(); i++){
      if(astar_ptr_->is_occupied(check_points[i])){
        ROS_WARN("Current trajctory not safe. replan...");
        changeExecState(STATE::REPLAN);
      }
    }
  }
}

void AstarReplan::ref_Traj(Eigen::Matrix<double, 3, 10> & ref_traj){
    auto t_now = std::chrono::high_resolution_clock::now();
    auto t_ = std::chrono::duration<double>(t_now - t0);
    double t = t_.count();

    double  tm, tmp;
    nurbs.getTimeSpan(tm, tmp);
    t = std::min(t, tmp);

    Vector2d p_prev; p_prev<<pose(0),pose(1);
    for(double dt = 0.0; dt<1; dt+=0.1){
            Vector3d  pt = nurbs.evaluateDeBoorT(t + dt);
            Vector2d p;  p(0) =pt(0); p(1) = pt(1);
            int i = int(dt/0.1);
            
            double theta = 0.0;
            double dy = p(1) - p_prev(1);
            double dx = p(0) - p_prev(0);
            theta = atan2(dy,dx);
            //theta = 0.0;
            ref_traj.col(i)<<p(0), p(1),theta;

            p_prev = p;

    }
}

void AstarReplan::model(const Ref<VectorXd> x0, const Ref<VectorXd> u,  Ref<VectorXd> x_out){
     Eigen::Matrix<double,3,3> A; A.setZero();

      double theta = x0(2);
      Eigen::Matrix<double,3,2> B; 
      B << std::cos(theta),    0,
          std::sin(theta),    0,
          0,                  1;
      x_out = A * x0 + B*u;
}

void AstarReplan::nextState(Ref<VectorXd> x0, const Ref<VectorXd> u,Ref<VectorXd> state){

  bool huanyu = false;
  if(huanyu ){
    Eigen::Matrix<double,3,3> A; A.setZero();

      double theta = x0(2);
      double dt = 0.1;
      Eigen::Matrix<double,3,3> B; 
      B << dt* std::cos(theta),    -dt* std::sin(theta),   0,
           dt* std::sin(theta),     dt* std::cos(theta),   0,
            0,                      0,                     dt*1;
      state = x0 + B *u;
    
  }else{
     Eigen::Matrix<double,3,3> A; A.setZero();

     double theta = x0(2);
     double dt = 0.1;
     Eigen::Matrix<double,3,2> B; 
     B << dt* std::cos(theta),    0,
          dt* std::sin(theta),    0,
          0,                     dt*1;
    
      state = x0 + B *u;
  }
 
}



void AstarReplan::broadcasterTf(Ref<VectorXd> state){

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;
    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "map";
    transformStamped.child_frame_id = "base_link";
    transformStamped.transform.translation.x = state(0);
    transformStamped.transform.translation.y = state(1);
    transformStamped.transform.translation.z = 0.0;


    Vector3d z = {0,0,1};
    Eigen::AngleAxisd rotz(state(2),z);
    Quaterniond q(rotz.toRotationMatrix());

    transformStamped.transform.rotation.x = q.x();
    transformStamped.transform.rotation.y = q.y();
    transformStamped.transform.rotation.z = q.z();
    transformStamped.transform.rotation.w = q.w();
  
    br.sendTransform(transformStamped);
 }

void AstarReplan::planYaw(){
  
}

void AstarReplan::get_cartograper_pose()
{ 
    tf::StampedTransform carto_tf;
    try
    {
        p_carto_tf_listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(3.0));
        p_carto_tf_listener.lookupTransform("/map", "/base_link", ros::Time(0), carto_tf);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("get_cartograper_pose:%s", ex.what());
    }

    double x = static_cast<double>(carto_tf.getOrigin().x());
    double y = static_cast<double>(carto_tf.getOrigin().y());

    pose(0) = x;
    pose(1) = y;
    pose(2) = tf::getYaw(carto_tf.getRotation());
    ROS_WARN_STREAM("cur pose is "<<pose(0)<<" "<<pose(1)<<" "<<pose(2));
}

void AstarReplan::PID(Vector3d path_point,Vector2d& u){
    double dx = path_point(0) - pose(0);
    double dy = path_point(1) - pose(1);

    //linear velocity .
    distance_error = sqrt(pow(dx,2) + pow(dy,2));
    distance_error_dot = distance_error - 2*last_distance_error + llast_distance_error;
    u(0) = kp_v * distance_error  + kd_v * distance_error_dot;
    // if(u(0) > 0.3){
    //   u(0) = 0.3;
    // }
    // if(u(0) < -0.3){
    //   u(0) = -0.3;
    // }

    //angular velocity
    double target_theta = atan2(dy,dx);  double cur_theta = pose(2);
    double theta_error = target_theta - cur_theta;

    //chech weather the angle sudden change.
    //check which one is short path. Clockwise or counterclockwise.
    if(abs(theta_error) >3.1415926){    //mean's the rotate path is not shortst.
      if(target_theta < 0){
         target_theta += 6.28;
         theta_error = target_theta - cur_theta;
      }
      if(cur_theta < 0 ){
        cur_theta += 6.28;
        theta_error = target_theta - cur_theta;
      }

    }


    double theta_error_dot = theta_error - 2*last_theta_error + llast_theta_error;
    u(1) = kp_theta * theta_error + kv_theta * theta_error_dot;
    double w_max = 0.5;
    // if(u(1)>w_max){
    //   u(1) = w_max;
    // }
    // if(u(1) < -w_max){
    //   u(1) = -w_max;
    // }

    llast_distance_error = last_distance_error;
    last_distance_error = distance_error;

    llast_theta_error = last_theta_error;
    last_theta_error = theta_error;

}

  



