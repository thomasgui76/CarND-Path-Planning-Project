#ifndef VEHICLE_H
#define VEHICLE_H
#include <string>
#include<vector>
#include<map>
#include"constants.h"
#include"jmt.h"
using namespace std;

// enum available_states {"KL", "LCL", "LCR"};          // enumerate all avaialble states for Vechicle

class  Vehicle
{
private:
    /* data */

public:
    int id,lane;
    string state;
    double s,s_dot,s_ddot, d,d_dot,d_ddot;

    Vehicle();       // default constructor
    // Vehicle(Vehicle & vehicle);         // copy constructor
    Vehicle(double s, double s_dot, double s_ddot,double d, double d_dot,double d_ddot);        // constructor
    ~ Vehicle();

    vector<vector<double>> generate_prediction(double start_time, double duration, int sample_points);
    vector<string> get_successor_state();
    vector<vector<double>> generate_target(string state,double duration, map<int,vector<vector<double>>> predictions);
    vector<vector<double>> generate_trajectory(vector<vector<double>> target,double duration);
    vector<double> get_car_ahead(int target_lane,map<int,vector<vector<double>>> predictions);               
    vector<double> get_car_behind(int target_lane,map<int,vector<vector<double>>> predictions);
    double get_nearest_distance(const double &target_velocity, const vector<double> &other_car, double duration);  
    void showVehicle();           
};

 Vehicle:: Vehicle(){}               // initiate ego car    
 Vehicle:: Vehicle(double s, double s_dot, double s_ddot,double d, double d_dot,double d_ddot){
     this->s = s;
     this->s_dot = s_dot;
     this->s_ddot = s_ddot;
     this->d = d;
     this->d_dot = d_dot;
     this->d_ddot = d_ddot;
     this->state = "KL";
 }

 Vehicle::~ Vehicle()
{
}

vector<string> Vehicle::get_successor_state(){
    /** 
     * @param state is current state
     * @output return a series available successor states
     * to simplify, after lane change, the state turn into "KL" immediately
     * you may add other states such as "CS","PLCL","PLCR"
     */
    vector<string> states;
    states.push_back("KL");
    if(this->state.compare("KL")==0){
        states.push_back("LCL");
        states.push_back("LCR");
    }
    return states;
}

vector<vector<double>> Vehicle::generate_prediction(double start, double duration, int sample_points){
    /**
     * @param start is the time the trajectory start from
     * @param duration is time span from the trajectory start to end
     * @param samle_points convert continous trajectory into discrete points
     * @output return a time series {{s0,s_dot0,d0,d_dot0},{s1,s_dot1,d1,d_dot1},...} size equals to sample_points
     * each time step including {s,s_dot,d,d_dot}
     * assume other cars keep constant speed and keep d position in lane center for simplicity.
     * i.e. s_dot keep constant and d_dot = 0, Actually may change and adapt to more complex situation
     * for example, other car can change lane
     */
    double time_interval = duration / sample_points;
    vector<vector<double>> prediction;
    for (int i=0; i<sample_points; ++i ){
        double traj_time = start + i * time_interval;
        double new_s = this->s + this->s_dot * traj_time;
        vector<double> discrete_pred = {new_s,this->s_dot ,this->d,0.0};
        prediction.push_back(discrete_pred);
    }
    // printVector2D(prediction);
    return prediction;
}

vector<vector<double>> Vehicle:: generate_target(string state,double duration, map<int,vector<vector<double>>> predictions){
    /**
     * @param state is next state for ego_car
     * @param duration is time span when the ego_car move from the start position to target position.
     * @param predictions is a map of all other cars from sensor_fusion.
     * @output return the 2D target vector according to state as the format
     * {{target_s, target_s_dot, target_s_ddot},{target_d,target_d_dot,target_d_ddot}}
     */
    // cout<<"enter generate_target function"<<endl;
    int current_lane = -1;
    if(this->d >=0.8 && this->d <=3.2){
        this->lane = 0;
    }
    else if(this->d >=4.8 && this->d <=7.2){
        this->lane = 1;
    }
    else if(this->d >=8.8 && this->d <=11.2)
    {
        this->lane = 2;
    }
    current_lane = this->lane;                  // when lane change, if d not into valid range, vehicle's lane keep previous value.
    

    int target_lane=current_lane;               // initilize target_lane same as current_lane, may change according to different state.
    // longitudinal target
    double target_s_ddot = 0;                     // target accel = 0, i.e. after duration T to constant speed.
    // double avg_accel = this->s_ddot/2;            // assume acceleration is linear.
    // double target_s_dot = min(this->s_dot+ avg_accel * duration,SPEED_LIMIT);         // initilize longitudinal velocity constraint.
    double target_s_dot;
    double target_s;
    // lateral target
    double target_d;                              // just initilize target_d first, the final value may decided by different state.
    double target_d_dot = 0;                      // when arrive in target_lane, keep in the center, no displacement in lateral.
    double target_d_ddot = 0;                     // no lateral acceleration.
        
    if(state.compare("KL")==0){
        target_lane = current_lane;
        target_d = target_lane * 4 + 2.0;
    }
    else if (state.compare("LCL")==0)
    {   
        // judge current_lane to keep ego_car in road.
        // if current_lane is left-most, keep current_lane actually.
        if(current_lane>0){
            target_lane = current_lane -1 ;
            target_d = target_lane * 4 + 2.0;
        }
        else
        {
            return {};
        }
    }
    else if (state.compare("LCR")==0)
    {
        // judge current_lane to keep ego_car in road.
        // if current_lane is right-most, keep current_lane actually.
        if(current_lane<LANE_NUMS-1){
            target_lane = current_lane +1 ;
            target_d = target_lane * 4 + 2.0;
        }

        else
        {
            return {};
        }
        
    }
    // adjust longitudinal velocity according to distance to the leading car in same lane.
    // assume other cars following behind in same lane may take right action,none of my business.
    // also penalize the consequent trajectory in collision cost function.
    // double target_velocity = min(this->s_dot+0.224,SPEED_LIMIT);               // if no car_ahead in same lane, try to drive at the speed_limit
    double target_velocity = SPEED_LIMIT;

    // cout<<"current lane: "<<current_lane<<endl;
    // cout<<"target lane: "<<target_lane<<endl;
    vector<double> car_ahead = get_car_ahead(target_lane,predictions);
    vector<double> car_behind = get_car_behind(target_lane, predictions);

    if(!car_ahead.empty()){
        // double distance_to_ahead = get_nearest_distance(target_velocity,car_ahead,DT);
        double distance_to_ahead = fabs(car_ahead[0] -this->s);
        if(distance_to_ahead > FOLLOW_DISTANCE && distance_to_ahead < 2 * FOLLOW_DISTANCE){
            cout<<"approach to car_ahead"<<endl;
            target_velocity = car_ahead[1];
            if (!car_behind.empty()){
                target_velocity = (target_velocity+car_behind[1])/2+0.224;
                //double distance_to_behind = get_nearest_distance(target_velocity,car_behind,DT);
                double distance_to_behind = fabs(this->s - car_behind[0]);
                if(distance_to_behind < FOLLOW_DISTANCE){
                    
                    if(state == "LCL" || state == "LCR"){
                        cout<<"can't turn left or right,because there is other car behind too closed"<<endl;
                        return {};
                    }
                    
                }
            }
        }
        
        else if(distance_to_ahead < FOLLOW_DISTANCE){
            if(state == "LCL" || state == "LCR"){
                cout<<"can't turn left or right, because there is other car ahead in the lanes"<<endl;
                return {};
            }
            else
            {
                // in "KL" state
                target_velocity = car_ahead[1] * (distance_to_ahead-TOO_CLOSED)/(FOLLOW_DISTANCE-TOO_CLOSED);               // when ego car get closer to car_ahead, decrease velocity
                cout<<"close to FOLLOW_DISTANCE"<<endl;
                if (distance_to_ahead < TOO_CLOSED){
                    target_velocity = 0.224;                              // when too closed to car_ahead, emergency brake
                    cout<<"warning!!! too closed to car_ahead"<<endl;
                }
            }
            
        }
        
    }
    else{
        // no car_ahead, but there is car_behind in target_lane
        if (!car_behind.empty()){
            // double distance_to_behind = get_nearest_distance(target_velocity,car_behind,DT);
            double distance_to_behind = fabs(this->s - car_behind[0]);
            if(distance_to_behind < FOLLOW_DISTANCE){
                if(state == "LCL" || state == "LCR"){
                    cout<<"can't turn left or right,because there is other car behind too closed"<<endl;
                    return {};
                }
                
            }
        }
    }
    target_s_dot = target_velocity;
    target_s = this->s + (this->s_dot + target_velocity) * 0.5 * duration;
    // target_s = this->s + target_velocity * duration;
    // target_s = this->s + 88;
    vector<vector<double>> target = {{target_s,target_s_dot,target_s_ddot},{target_d,target_d_dot,target_d_ddot}};
    // cout<<"print target in generate_target function"<<endl;
    // printVector2D(target);
    return target;
}

vector<vector<double>> Vehicle:: generate_trajectory(vector<vector<double>> target,double duration){
    /**
     * @param target is ego_car target space vector.
     * @param duration is time span for the ego_car move from the start to target.
     * return the discrete trajectory points in 2D vector as
     * {{s0,s1,...sn},{d0,d1,...dn}}, n = N_SAMPLES-1.
     */
    vector<double> start_s = {this->s,this->s_dot,this->s_ddot};
    vector<double> start_d = {this->d,this->d_dot,this->d_ddot};
    // cout<<"print start_s"<<endl;
    // printVector1D(start_s);
    // cout<<"print start_d"<<endl;
    // printVector1D(start_d);

    vector<double> target_s = target[0];
    vector<double> target_d = target[1];
    // cout<<"duration: "<<duration<<endl;
    // cout<<"print target_s"<<endl;
    // printVector1D(target_s);
    // cout<<"print target_d"<<endl;
    // printVector1D(target_d);
    vector<double> traj_s_coef = JMT(start_s,target_s,duration);
    vector<double> traj_d_coef = JMT(start_d,target_d,duration);
    vector<double> discrete_traj_s,discrete_traj_d;
    vector<vector<double>> trajectory;
    double time_interval = duration/(N_SAMPLES-1);
    // double t = 0;
    // generate discrete trajectory points, size = N_SAMPLES.
    for(int i=0;i<N_SAMPLES;++i){
        double next_s =0;
        double next_d =0;
        double t = time_interval * i;
        for(int j=0;j<traj_s_coef.size();++j){
            next_s += traj_s_coef[j] * pow(t,j);
            next_d += traj_d_coef[j] * pow(t,j);
        }
        discrete_traj_s.push_back(next_s);
        discrete_traj_d.push_back(next_d);
    }
    // cout<<"print discrete_traj_s"<<endl;
    // printVector1D(discrete_traj_s);
    // cout<<"print discrete_traj_d"<<endl;
    // printVector1D(discrete_traj_d);
    trajectory.push_back(discrete_traj_s);
    trajectory.push_back(discrete_traj_d);
    return trajectory;
}

vector<double> Vehicle::get_car_ahead(int target_lane,map<int,vector<vector<double>>> predictions){
    /**
     * @param target_lane is the lane that ego_car intend to change to
     * @param predictions a map contains all other vehicle's trajectory{s,s_dot,d,d_dot} in each time_step
     * @output return {s,s_dot} stand for the leading car s and s_dot
     * assume other car keep velocity and lane during the prediction duration
     * we use the longitudinal s and velocity of the leading other car
     * in the target_lane to generate ego_car's trajectory
     * so we just get other car start s and velocity to compute distance to ego_car
     */
    double nearest_distance = 99999;
    bool found = false;
    double car_ahead_s;
    double car_ahead_vel;
    vector<double> car_ahead;
    for(auto prediction : predictions){
        vector<vector<double>> pred_traj = prediction.second;
        // printVector2D(pred_traj);
        int pred_lane = int(pred_traj[0][2]/4);
        if(pred_lane == target_lane){
            double other_car_s = pred_traj[0][0];
            double other_car_vel = pred_traj[0][1];
            double distance_to_ahead = fabs(other_car_s - this->s);
            if((distance_to_ahead < nearest_distance) && (other_car_s >= this->s )){
                nearest_distance = distance_to_ahead;
                car_ahead_s=other_car_s;
                car_ahead_vel=other_car_vel;
                found = true;
            }
        }
    }
    if(found){
    car_ahead.push_back(car_ahead_s);
    car_ahead.push_back(car_ahead_vel);
    // if not find other car in the front, return empty vector.
    return car_ahead;
    }
}

vector<double> Vehicle::get_car_behind(int target_lane,map<int,vector<vector<double>>> predictions){
    /**
     * @param target_lane is the lane that ego_car intend to change to
     * @param predictions a map contains all other vehicle's trajectory{s,s_dot,d,d_dot} in each time_step
     * @output return {s,s_dot} stand for the leading car s and s_dot
     * assume other car keep velocity and lane during the prediction duration
     * we use the longitudinal s and velocity of the following other car
     * in the target_lane to generate ego_car's trajectory
     * so we just get other car start s and velocity to compute distance to ego_car
     */
    double nearest_distance = 99999;
    bool found = false;
    double car_behind_s;
    double car_behind_vel;
    vector<double> car_behind;
    for(auto prediction : predictions){
        vector<vector<double>> pred_traj = prediction.second;
        // printVector2D(pred_traj);
        int pred_lane = int(pred_traj[0][2]/4);
        if(pred_lane == target_lane){
            // double other_car_s = pred_traj[N_SAMPLES-1][0];
            double other_car_s = pred_traj[0][0];
            double other_car_vel = pred_traj[0][1];
            double distance_to_behind = fabs(this->s - other_car_s);
            if((distance_to_behind < nearest_distance) && (other_car_s <= this->s )){
                nearest_distance = distance_to_behind;
                car_behind_s = other_car_s;
                car_behind_vel = other_car_vel;
                found = true;
            }
        }
    }
    if(found){
        car_behind.push_back(car_behind_s);
        car_behind.push_back(car_behind_vel);
        // if not find other car behind, return empty vector.
        return car_behind;
    }
}

double Vehicle::get_nearest_distance(const double &target_velocity, const vector<double> &other_car, double duration){
    /**
     * @param target_car {target_s, target_speed}, assume ego_car move from current position(this->s) at 
     * current_speed(this->s_dot) to target_s at target_speed. the acceleration is constant.
     * @param other_car {other_car_s, other_car_vel}, assume other_car move from current position(other_car_s)
     * at constant speed(other_car_vel) to the end position after time duration.
     * we can infer the discrete position for both target_car and other_car
     * calculate the distance at each time step, to get the closest distance
     * @output return the closest distance
     */
    double time_interval = duration/N_SAMPLES;
    double nearest = 9999;
    for(int i=0;i<N_SAMPLES;++i){
        double target_s = this->s +(this->s_dot+target_velocity)/2 * time_interval *i;
        double other_car_s = other_car[0] + other_car[1] * time_interval *i;
        double distance = fabs(target_s-other_car_s);
        if (distance < nearest){
            nearest = distance;
        }
    }
    return nearest;
}

void Vehicle::showVehicle(){
    // cout<<"Vehicle states"<<endl;
    cout<<"[s: "<<this->s<<", s_dot: "<<this->s_dot<<", s_ddot: "<<this->s_ddot<<"]"<<endl;
    cout<<"[d: "<<this->d<<", d_dot: "<<this->d_dot<<", d_ddot: "<<this->d_ddot<<"]"<<endl;
}

#endif