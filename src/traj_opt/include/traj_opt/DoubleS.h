#ifndef DOUBLE_S_H_
#define DOUBLE_S_H_

#include <vector>
#include <cmath>
#include <cstdio>
#include <iostream>

using namespace std;

class DoubleS {
public:
    DoubleS();
    DoubleS(double vmax_magnititude, double amax_magnititude, double jmax_magnititude);
    // DoubleS(double vmax, double vmin, double amax, double amin, double jmax, double jmin) 
    //         :vmax_(vmax), vmin_(vmin), amax_(amax), amin_(amin), jmax_(jmax), jmin_(jmin) {}

    void set_limits(double vmax_magnititude, double amax_magnititude, double jmax_magnititude);
    vector<double> get_traj(double t);
    vector<double> get_traj(int cnt, double dt); 
    double get_traj_duration();
    double get_traj_distance();
    void plan(double q0, double q1, double v0, double v1, double lambda);
    const vector<double> &get_planned_time_point() {return time_point_;}
    void reset_traj();

private:
    double jmax_, jmin_, amax_, amin_, vmax_, vmin_;
    bool if_pos_increase_ = false;
    bool if_no_pos_displace_ = false;
    bool if_traj_feasible = false;
    vector<double> time_point_;
    double alima_, alimd_, vlim_;
    double q0_, q1_, v0_, v1_;

    void adjust_inner_para(double q0, double q1, double v0, double v1);
    void plan_null_end_velocity(double q0, double q1);
};


#endif