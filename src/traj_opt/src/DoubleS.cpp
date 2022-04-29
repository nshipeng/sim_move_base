#include "traj_opt/DoubleS.h"
#include <cassert>

#define EPS 1e-12

DoubleS::DoubleS() : time_point_(vector<double>(8, 0.0)){}

DoubleS::DoubleS(double vmax_magnititude, double amax_magnititude, double jmax_magnititude)
            :vmax_(vmax_magnititude), vmin_(-vmax_magnititude), 
             amax_(amax_magnititude), amin_(-amax_magnititude),
             jmax_(jmax_magnititude), jmin_(-jmax_magnititude), time_point_(vector<double>(8, 0.0)){}

void DoubleS::set_limits(double vmax_magnititude, double amax_magnititude, double jmax_magnititude) {
    vmax_ = vmax_magnititude;
    vmin_ = -vmax_magnititude;
    amax_ = amax_magnititude;
    amin_ = -amax_magnititude;
    jmax_ = jmax_magnititude;
    jmin_ = -jmax_magnititude;
}

void DoubleS::adjust_inner_para(double q0, double q1, double v0, double v1) {
    if (q1 > q0 || fabs(q1 - q0) < EPS) {
        if_pos_increase_ = true;
        q0_ = q0;
        q1_ = q1;
        v0_ = v0;
        v1_ = v1;
    } else {
        if_pos_increase_ = false;
        q0_ = -q0;
        q1_ = -q1;
        v0_ = -v0;
        v1_ = -v1;
        double vmax_hat = -vmin_;
        double vmin_hat = -vmax_;
        double amax_hat = -amin_;
        double amin_hat = -amax_;
        double jmax_hat = -jmin_;
        double jmin_hat = -jmax_;

        vmax_ = vmax_hat;
        vmin_ = vmin_hat;
        amax_ = amax_hat;
        amin_ = amin_hat;
        jmax_ = jmax_hat;
        jmin_ = jmin_hat;
    }
}

void DoubleS::plan(double input_q0, double input_q1, double input_v0, double input_v1, double lambda) {
    adjust_inner_para(input_q0, input_q1, input_v0, input_v1);
    if_traj_feasible = false;

    if (fabs(input_v0) < EPS && fabs(input_v1) < EPS) {
        plan_null_end_velocity(input_q0, input_q1);
        return; 
    }

    double ta = sqrt(abs(v1_ - v0_) / jmax_);
    double tb = amax_ / jmax_;
    double pos_increment = q1_ - q0_;
    if ((ta < tb && ta * (v0_ + v1_) > pos_increment) || (tb < ta && 0.5 * (v0_ + v1_) * (tb + abs(v1_ - v0_) / amax_) > pos_increment)) {
        printf("can not compute the trajectory for the input parameter\n");
        throw 0;
    }

    if_traj_feasible = true;

    if (fabs(input_q0 - input_q1) < EPS) {
        if_no_pos_displace_ = true;
        return;
    } else {
        if_no_pos_displace_ = false;
    }

    double Tj1, Ta, Tv, Tj2, Td;
    if ((vmax_ - v0_) * jmax_ < pow(amax_, 2)) {
        Tj1 = sqrt((vmax_ - v0_) / jmax_);
        Ta = 2 * Tj1;
    } else {
        Tj1 = amax_ / jmax_;
        Ta = Tj1 + (vmax_ - v0_) / amax_;
    }

    if ((vmax_ - v1_) * jmax_ < pow(amax_, 2)) {
        Tj2 = sqrt((vmax_ - v1_) / jmax_);
        Td = 2 * Tj2;
    } else {
        Tj2 = amax_ / jmax_;
        Td = Tj2 + (vmax_ - v1_) / amax_;
    }

    Tv = pos_increment / vmax_ - 0.5 * Ta * (1 + v0_ / vmax_) - 0.5 * Td * (1 + v1_ / vmax_);
    if (Tv >= 0) {
        time_point_[0] = 0.0;
        time_point_[1] = Tj1;
        time_point_[2] = Ta - Tj1;
        time_point_[3] = Ta;
        time_point_[4] = Ta + Tv;
        time_point_[5] = Ta + Tv + Tj2;
        time_point_[6] = Ta + Tv + Td - Tj2;
        time_point_[7] = Ta + Tv + Td;
        vlim_ = vmax_;
        alima_ = jmax_ * Tj1;
        alimd_ = -jmax_ * Tj2;

        printf("Ta = %lf, Tv = %lf, Td = %lf, Tj1 = %lf, Tj2 = %lf\n", Ta, Tv, Td, Tj1, Tj2);

        return;
    }

    
    do {
        Tj1 = Tj2 = amax_ / jmax_;
        double delta = pow(amax_, 4) / pow(jmax_, 2) + 2 * (pow(v0_, 2) + pow(v1_, 2)) + amax_ * (4 * pos_increment - 2 * amax_ / jmax_ * (v0_ + v1_));
        Ta = (pow(amax_, 2) / jmax_ - 2 * v0_ + sqrt(delta)) / (2 * amax_);
        Td = (pow(amax_, 2) / jmax_ - 2 * v1_ + sqrt(delta)) / (2 * amax_);
        Tv = 0.0;

        if (Ta < 0) {
            Ta = Tj1 = 0.0;
            Td = 2 * pos_increment / (v1_ + v0_);
            Tj2 = (jmax_ * pos_increment - sqrt(jmax_ * (jmax_ * pow(pos_increment, 2) + pow(v0_ + v1_, 2) * (v1_ - v0_)))) / (jmax_ * (v0_ + v1_));
            Tv = 0.0;
            break;
        }

        if (Td < 0) {
            Td = Tj2 = 0.0;
            Ta = 2 * pos_increment / (v1_ + v0_);
            Tj1 = (jmax_ * pos_increment - sqrt(jmax_ * (jmax_ * pow(pos_increment, 2) + pow(v0_ + v1_, 2) * (v1_ - v0_)))) / (jmax_ * (v0_ + v1_));
            Tv = 0.0;
            break;
        }
        amax_ = lambda * amax_;
    } while (Ta < 2 * Tj1 || Td < 2 * Tj2);
    alima_ = jmax_ * Tj1;
    alimd_ = -jmax_ * Tj2;
    if (Ta > 0) {
        vlim_ = v0_ + (Ta - Tj1) * alima_;
    } else {
        vlim_ = v1_ - (Td - Tj2) * alimd_;
    }

    time_point_[0] = 0.0;
    time_point_[1] = Tj1;
    time_point_[2] = Ta - Tj1;
    time_point_[3] = Ta;
    time_point_[4] = Ta + Tv;
    time_point_[5] = Ta + Tv + Tj2;
    time_point_[6] = Ta + Tv + Td - Tj2;
    time_point_[7] = Ta + Tv + Td;


    printf("Ta = %lf, Tv = %lf, Td = %lf, Tj1 = %lf, Tj2 = %lf\n", Ta, Tv, Td, Tj1, Tj2);
    printf("alima = %lf, alimd = %lf, vlim_ = %lf\n", alima_, alimd_, vlim_);
    

}

void DoubleS::plan_null_end_velocity(double q0, double q1) {
    // cout <<"in plan_null_end_velocity " << endl;
    if_traj_feasible =  true;

    if (fabs(q0 - q1) < EPS) {
        if_no_pos_displace_ = true;
        return;
    } else {
        if_no_pos_displace_ = false;
    }


    double Tj, Ta, Tv;
    if (vmax_ * jmax_ >= pow(amax_, 2)) {
        Tj = amax_ / jmax_;
        Ta = Tj + vmax_ / amax_;
    } else {
        Tj = sqrt(vmax_ / jmax_);
        Ta = 2 * Tj;
    }

    Tv = (q1_ - q0_) / vmax_ - Ta;
    if (Tv >= 0) {
        time_point_[0] = 0.0;
        time_point_[1] = Tj;
        time_point_[2] = Ta - Tj;
        time_point_[3] = Ta;
        time_point_[4] = Ta + Tv;
        time_point_[5] = Ta + Tv + Tj;
        time_point_[6] = 2 * Ta + Tv - Tj;
        time_point_[7] = 2 * Ta + Tv;
        alima_ = amax_;
        alimd_ = amin_;
        vlim_ = vmax_;

        return;
    }

    double pos_increment = q1_ - q0_;
    if (pos_increment >= 2 * pow(amax_, 3) / pow(jmax_, 2)) {
        Tj  = amax_ / jmax_;
        Ta  = Tj / 2 + sqrt(pow(Tj / 2, 2) + pos_increment / amax_);
    } else {
        Tj = pow(pos_increment / (2 * jmax_), 1.0 / 3.0);
        Ta = 2 * Tj;
    }
    time_point_[0] = 0.0;
    time_point_[1] = Tj;
    time_point_[2] = Ta - Tj;
    time_point_[3] = Ta;
    time_point_[4] = Ta;
    time_point_[5] = Ta + Tj;
    time_point_[6] = 2 * Ta - Tj;
    time_point_[7] = 2 * Ta;
    alima_ = jmax_ * Tj;
    alimd_ = -alima_;
    vlim_ = (Ta - Tj) * alima_;
}

vector<double> DoubleS::get_traj(double t) {

    if (!if_traj_feasible) {
        cout << "no feasible double S trajectory!" << endl;
        throw 0;
    }

    if (if_no_pos_displace_) {
        return vector<double> {q0_, 0.0, 0.0, 0.0};
    }

    if (t > time_point_.back()) {
        t = time_point_.back();
    }

    int index = 0;
    while (time_point_[index + 1] < EPS) index++;
    while(index < time_point_.size() - 1 && t > time_point_[index + 1]) {
        index++;
    } 
    assert(index < time_point_.size() - 1);

    double q, v, a, j;
    double Tj1 = time_point_[1];
    double Ta = time_point_[3];
    double Tj2 = time_point_[5] - time_point_[4];
    double Td = time_point_[7] - time_point_[4];
    double T = time_point_[7];

    switch (index)
    {
    case 0: 
        q = q0_ + v0_ * t + jmax_ * pow(t, 3) / 6.0;
        v = v0_ + jmax_ *  pow(t, 2) / 2.0;
        a = jmax_ * t;
        j = jmax_;
        break;
    case 1:
        q = q0_ + v0_ * t + alima_ / 6.0 * (3 * pow(t, 2) - 3 * Tj1 * t + pow(Tj1, 2));
        v = v0_ + alima_ * (t - Tj1 / 2.0);
        a = alima_;
        j = 0;
        break;
    case 2:
        q = q0_ + (vlim_ + v0_) * Ta / 2.0 - vlim_ * (Ta - t) - jmin_ * pow(Ta - t, 3) / 6.0;
        v = vlim_ + jmin_ * pow(Ta - t, 2) / 2.0;
        a = -jmin_  * (Ta - t);
        j = jmin_;
        break;
    case 3:
        q = q0_ + (vlim_ + v0_) * Ta / 2.0 + vlim_ * (t - Ta);
        v = vlim_;
        a = 0;
        j = 0;
        break;
    case 4:
        q = q1_ - (vlim_ + v1_) * Td / 2.0 + vlim_ * (t - T + Td) - jmax_ * pow(t - T + Td, 3) / 6.0;
        v = vlim_ - jmax_ * pow(t - T + Td, 2) / 2.0;
        a = -jmax_ * (t - T + Td);
        j = jmin_;
        break;
    case 5:
        q = q1_ - (vlim_ + v1_) * Td / 2.0 + vlim_ * (t - T + Td) + alimd_ / 6.0 * (3 * pow(t - T + Td, 2)- 3 * Tj2 * (t - T + Td) + pow(Tj2, 2));
        v = vlim_ + alimd_ * (t - T + Td - Tj2 / 2.0);
        a = alimd_;
        j = 0;
        break;
    case 6:
        q = q1_ - v1_ * (T - t) - jmax_ * pow(T - t, 3) / 6.0;
        v = v1_ + jmax_ * pow(T - t, 2) / 2.0;
        a = -jmax_ * (T - t);
        j = jmax_;
        break;
    default:
        throw 0;
        break;
    }

    vector<double> result = {q, v, a, j};
    if (!if_pos_increase_) {
        for (double &i : result) i *= -1;
    }

    return result;
    
}


vector<double> DoubleS::get_traj(int cnt, double dt) {
    double timing = cnt * dt;
    return get_traj(timing);
}

double DoubleS::get_traj_duration() {
    if (!if_traj_feasible) {
        cout << "no feasible double S trajectory!" << endl;
        throw 0;
    }
    return time_point_.back();
}

double DoubleS::get_traj_distance() {
    return fabs(q1_ - q0_);
}
void DoubleS::reset_traj() {
    time_point_ = vector<double>(8, 0.0);
    if_traj_feasible = false;
}