#ifndef COSTS_H
#define COSTS_H

#include <vector>
#include <algorithm>
#include <cmath>
#include <iterator>
#include <map>
#include "constants.h"
#include "vehicle.h"
using namespace std;

const double collision_weight = pow(10,5);
const double buffer_weight = pow(10,3);
const double efficiency_weight = pow(10,3);
const double off_middle_lane_weight = pow(10,3);
const double keep_lane_weight = pow(10,3);
// const double in_lane_buffer_weight = pow(10,3);
const double total_jerk_weight = pow(10,1);

double logistic(double x){
    /**
     * the function return a value between 0 and 1 for x in the range[0, infinity]
     * or - 1 to 1 for x in the range[-infinity, infinity].
     * used to design cost function.
     */
    return 2.0 / (1 + exp(-x)) - 1.0;
}

vector<double> discrete_differential(vector<double> sequence){
    vector<double> diff;
    for(int i=0; i<sequence.size()-1; ++i){
        diff.push_back((sequence[i+1] - sequence[i])/DELTA_T);
    }
    return diff;
}

double nearest_approach(const vector<vector<double>> &trajectory, const vector<vector<double>> pred_traj){
    /**
     * compute the distance between every dicrete trajectory point and one of other cars to get the closest distance.
     */
    double closest = 99999;
    for(int i=0;i<N_SAMPLES;++i){
        double distance = sqrt(pow((trajectory[0][i]-pred_traj[i][0]),2)+pow((trajectory[1][i]-pred_traj[i][2]),2));
        if (distance<closest){
            closest = distance;
        }
    }
    return closest;
}

double nearest_approach_to_any_vehicle(const vector<vector<double>> &trajectory,const map<int,vector<vector<double>>> &predictions){
    /**
     * @param trajectory is a 2D vector {{s0,s1,...sn},{d0,d1,...dn}}, n = N_SAMPLES-1.
     * @param predictions a map contains all other vehicle's trajectory{s,s_dot,d,d_dot} in each time_step
     * Calculate the closest distance to any other car during a trajectory.
     */
    double closest = 99999;
    for(auto prediction:predictions){
        vector<vector<double>> pred_traj = prediction.second;          //{{s0,s_dot0,d0,d_dot0}...}
        int pred_lane = int(pred_traj[0][2]/4);
        if(pred_lane >=0 && pred_lane <3){
            // cout<<"print pred_traj:"<<endl;
            // printVector2D(pred_traj);
            // cout<<"print trajectory: "<<endl;
            // printVector2D(trajectory);
            double distance = nearest_approach(trajectory,pred_traj);
            if (distance<closest){
                closest = distance;
            }
        }
    }
    return closest;
}

// define cost functions
double get_collision_cost(const vector<vector<double>> &trajectory,const map<int,vector<vector<double>>> &predictions){
    /**
     * penalize the possibe collision with other cars.
     */
    double cost = 0;
    double closest = nearest_approach_to_any_vehicle(trajectory,predictions);
    if(closest < 3 * VEHICLE_CIRCLE){
        cost = 1.0;
        // cout<<"collision!!!,closest distance to other car is "<<closest<<endl;
    }
    // cout<<"collision_cost: "<<cost<<endl;
    return cost;
}

double get_buffer_cost(const vector<vector<double>> &trajectory,const map<int,vector<vector<double>>> &predictions){
    /**
     * penalize the ego_car getting close to any of other cars.
     */
    // double cost = 0;
    double closest = nearest_approach_to_any_vehicle(trajectory,predictions);
    double cost = logistic(6 * VEHICLE_CIRCLE / closest);
    // cout<<"buffer_cost: "<<cost<<endl;
    return cost;
}

double get_in_lane_buffer_cost(const vector<vector<double>> &trajectory,const map<int,vector<vector<double>>> &predictions){
    double current_d = trajectory[1][0];
    int current_lane = int(current_d/4);
    double closest = 99999;
    for(auto prediction:predictions){
        vector<vector<double>> pred_traj = prediction.second;
        int pred_lane = int(pred_traj[0][2]/4);
        if (pred_lane == current_lane){
            double distance = nearest_approach(trajectory,pred_traj);
            if(distance < closest){
                closest = distance;
            }
        }
    } 
    double cost = logistic(2 * VEHICLE_CIRCLE / closest);
    // cout<<"in_lane_buffer_cost: "<<cost<<endl;
    return cost;
}

double get_efficiency_cost(const vector<vector<double>> &trajectory,const map<int,vector<vector<double>>> &predictions){
    /**
     * penalize ego_car drive in low velocity, take more time to final goal_s.
     */
    vector<double> target_s_dot = discrete_differential(trajectory[0]);
    double final_velocity = target_s_dot[target_s_dot.size()-1];
    double cost = logistic(fabs(SPEED_LIMIT-final_velocity)/SPEED_LIMIT);
    // cout<<"efficiency_cost: "<<cost<<endl;
    return cost;
}

double get_off_middle_lane_cost(const vector<vector<double>> &trajectory,const map<int,vector<vector<double>>> &predictions){
    /**
     * penalize ego_car not in middle_lane, push the car to middle_lane.
     */
    double target_d = trajectory[1][N_SAMPLES-1];
    double target_s = trajectory[0][N_SAMPLES-1];
    double current_d = trajectory[1][0];
    int target_lane = int(target_d/4);
    double nearest_to_ahead = 999;
    double nearest_to_behind = 999;

    for(auto prediction : predictions){
        vector<vector<double>> pred_traj = prediction.second;
        int pred_lane = pred_traj[N_SAMPLES-1][2]/4;
        double other_car_s = pred_traj[N_SAMPLES-1][0];
        double distance = target_s - other_car_s;
        if(pred_lane == target_lane){
            if (distance>0 && distance<nearest_to_ahead){
                nearest_to_ahead = distance;
            }
            else if(distance<0 && fabs(distance)<nearest_to_behind){
                nearest_to_behind = -distance;
            }
        }
    }
    
    // double cost = logistic(sqrt(pow((target_d-6),2)));
    // cout<<"off_lane value: "<<fabs(target_d-6)<<endl;
    double lane_cost = (5*(target_d-6)*(target_d-6)+2*(target_d-2)*(target_d-2)+(target_d-10)*(target_d-10))/64;
    cout<<"lane_cost: "<<lane_cost<<endl;
    double distance_cost = 1 + (FOLLOW_DISTANCE+60)/(nearest_to_ahead+60);
    if(nearest_to_ahead <FOLLOW_DISTANCE || nearest_to_behind <FOLLOW_DISTANCE ){
        distance_cost = 2.1;
    }
    cout<<"distance_cost: "<<distance_cost<<endl;
    double cost = logistic(lane_cost + distance_cost);
    // cout<<"off_middle_lane_cost: "<<cost<<endl;
    return cost;
}

double get_keep_lane_cost(const vector<vector<double>> &trajectory,const map<int,vector<vector<double>>> &predictions){
    double target_d = trajectory[1][N_SAMPLES-1];
    int target_lane = int(target_d/4);
    double current_d = trajectory[1][0];
    int current_lane = int(current_d/4);
    double best_d = current_lane * 4.0 + 2;
    double closest = nearest_approach_to_any_vehicle(trajectory,predictions);
    cout<<"closest: "<<closest<<endl;
    closest = min(closest, FOLLOW_DISTANCE);
    double cost = logistic(fabs(2 * best_d -target_d-current_d) /closest);
    // double cost = logistic(fabs(best_d - target_d)/closest);
    return cost;
}

double get_max_accel_cost(const vector<vector<double>> &trajectory,const map<int,vector<vector<double>>> &predictions){
    double cost = 0;
    vector<double> traj_s = trajectory[0];
    vector<double> traj_d = trajectory[1];
    vector<double> traj_s_dot = discrete_differential(traj_s);
    vector<double> traj_s_ddot = discrete_differential(traj_s_dot);
    vector<double> traj_d_dot = discrete_differential(traj_d);
    vector<double> traj_d_ddot = discrete_differential(traj_d_dot);
    for (int i=0;i<traj_s_ddot.size();++i){
        if(traj_s_ddot[i] > MAX_INSTANTANEOUS_ACCEL){
            cost = 1.0;
        }
    }
    return cost;
}

double get_total_jerk_cost(const vector<vector<double>> &trajectory,const map<int,vector<vector<double>>> &predictions){
    /**
     * penalize total_jerk exceeds limit.
     */
    vector<double> traj_s = trajectory[0];
    vector<double> traj_d = trajectory[1];
    vector<double> traj_s_dot = discrete_differential(traj_s);
    vector<double> traj_s_ddot = discrete_differential(traj_s_dot);
    vector<double> traj_s_dddot = discrete_differential(traj_s_ddot);
    vector<double> traj_d_dot = discrete_differential(traj_d);
    vector<double> traj_d_ddot = discrete_differential(traj_d_dot);
    vector<double> traj_d_dddot = discrete_differential(traj_d_ddot);
    double total_jerk_s = 0;
    double total_jerk_d = 0;
    double time_interval = DT/N_SAMPLES;
    for(int i=0;i<traj_s_dddot.size();++i){
        double t = i * time_interval;
        total_jerk_s += fabs(traj_s_dddot[i] * t);
        total_jerk_d += fabs(traj_d_dddot[i] * t);
    }
    double jerk_cost = (total_jerk_s+total_jerk_d)/20/N_SAMPLES;
    double cost = logistic(jerk_cost);
    return cost;
}

double max_jerk_cost(const vector<vector<double>> &trajectory,const map<int,vector<vector<double>>> &predictions){

}

double calculate_cost(const vector<vector<double>> &trajectory,const map<int,vector<vector<double>>> &predictions){
    /**
     * @param trajectory is a 2D vector {{s0,s1,...sn},{d0,d1,...dn}}, n = N_SAMPLES-1.
     * @param predictions is a map of all other cars from sensor_fusion. for each car has
     * {{s0,s_dot0,d0,d_dot0},{s1,s_dot1,d1,d_dot1},...}
     * evaluate total cost to choose best trajectory with the lowest cost.
     */
    // double collision_cost = get_collision_cost(trajectory,predictions) * collision_weight;
    // double buffer_cost = get_buffer_cost(trajectory,predictions) * buffer_weight;
    double efficiency_cost = get_efficiency_cost(trajectory,predictions) * efficiency_weight;
    cout<<"efficiency_cost: "<<efficiency_cost<<endl;
    double off_mid_cost = get_off_middle_lane_cost(trajectory,predictions) * off_middle_lane_weight;
    cout<<"off_mid_cost: "<<off_mid_cost<<endl;
    // double keep_lane_cost = get_keep_lane_cost(trajectory, predictions) * keep_lane_weight;
    // double inlane_buff_cost = in_lane_buffer_cost(trajectory,predictions);
    // double total_jerk_cost = get_total_jerk_cost(trajectory,predictions) * total_jerk_weight;
    double total_cost = off_mid_cost + efficiency_cost;
    cout<<"total_cost: "<<total_cost<<endl;
    return total_cost;
}

#endif