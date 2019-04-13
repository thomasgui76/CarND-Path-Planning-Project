#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"
#include "costs.h"
#include "constants.h"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using namespace std;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  const double max_s = 6945.554;

  // static int lane = 1;
  static double ref_velocity = 0; // reference velocity used to generate path_points
  Vehicle ego_car = Vehicle();
  cout<<"show ego_car initiate state: "<<endl;
  ego_car.showVehicle();
  ego_car.state = "KL";
  ego_car.lane = 1;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&ego_car,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];            // mph


          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */
          // to get previous path size
          int prev_size = previous_path_x.size();
          cout<<"prev_size: "<<prev_size<<endl;

          if (prev_size>0){
            car_s = end_path_s;
          }
          // int lane = 1;
          // double ref_volecity = 0; //mph

          // to generate other cars discrete predictions
          // sensor_fusion data like : [ id, x, y, vx, vy, s, d]
          // assume other cars drive in constant velocity, keep lane in predict duration time
          // predictions map like v_id : {{s0,s_dot0,d0,d_dot0},{s1,s_dot1,d1,d_dot1}...}
          // each other vehicle generate dicrete points with N_Sample size
          // for ego car use previous_path, other_cars predictions start from the end of previous path
          double start = prev_size * PATH_DT;
          // consider time delay impact
          double duration = DT - start;
          // double duration = DT;
          map<int,vector<vector<double>>> predictions;
          for (auto sf : sensor_fusion){
            // double other_car_d = sf[6];
            // cout<<"other_car_d: "<<sf[6]<<endl;
            double vx = sf[3];
            double vy = sf[4];
            double other_car_s_dot = sqrt(vx * vx + vy * vy);
            Vehicle other_car = Vehicle(sf[5],other_car_s_dot,0,sf[6],0,0);
            // cout<<"show other car state"<<endl;
            // other_car.showVehicle();
            vector<vector<double>> pred = other_car.generate_prediction(start,duration,N_SAMPLES);
            int v_id = sf[0];
            predictions[v_id] = pred;
          }

          // cout<<"ref_volecity: "<<ref_volecity<<endl;

          // create a list of widely spaced(x,y) waypoints, evenly spaced at 30m
          vector<double> ptsx;
          vector<double> ptsy;

          // reference x,y,yaw rates
          // either we will reference the starting points as where the car is or
          // at the previous paths end points
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          // if the previous size is almost empty, we use the car as the starting reference
          // when the car start from the orgin or stop
          // in the situation, the car only has location s, d. while velocity and acceleration is 0.
          if (prev_size <3 ) {
            // use the car_x, car_y as the current position
            // inversely infer the previous one position
            // and the two points consititute a line which is tangent to the car at the moment

            double prev_car_x = car_x - cos(ref_yaw);
            double prev_car_y = car_y - sin(ref_yaw);

            // cout<<"prev_car_x: "<<prev_car_x<<endl;
            // cout<<"car_x: "<<car_x<<endl;

            ptsx.push_back(prev_car_x);
            ptsx.push_back(car_x);

            ptsy.push_back(prev_car_y);
            ptsy.push_back(car_y);

            // initilize ego_car's start state vector
            ego_car.s = car_s;
            ego_car.d = car_d;
            ego_car.s_dot = car_speed;           // mph
            ego_car.d_dot = 0;
            ego_car.s_ddot = 0;
            ego_car.d_ddot = 0;
          }

          else {
            // cout<<"prev_size: "<<prev_size<<endl;
            // use the last point in previous path as reference point
            ref_x = previous_path_x[prev_size-1];                
            ref_y = previous_path_y[prev_size-1];                
            // cout<<"ref_x: "<<ref_x<<", ref_y: "<<ref_y<<endl;
            // compute s, d for the reference point
            vector<double> ref_s_d = getFrenet(ref_x,ref_y,ref_yaw,map_waypoints_x,map_waypoints_y);
            double ref_s = ref_s_d[0];                           
            double ref_d = ref_s_d[1];
            int prev_s_index = -1;                           // get index of previous point to reference s
            while(ref_s >map_waypoints_s[prev_s_index+1]){
              ++prev_s_index;
            }
            // [dx,dy] is nomal vector of map_way_piont_s pointing outward
            double ref_dx = map_waypoints_dx[prev_s_index];         
            double ref_dy = map_waypoints_dy[prev_s_index];         
            // [sx, sy] is tangential vector
            double ref_sx = -ref_dx;                             
            double ref_sy = -ref_dy;

            double prev_ref_x = previous_path_x[prev_size-2];
            double prev_ref_y = previous_path_y[prev_size-2];

            double p_prev_ref_x = previous_path_x[prev_size-3];
            double p_prev_ref_y = previous_path_y[prev_size-3];
            // cout<<"prev_ref_x: "<<prev_ref_x<<", prev_ref_y: "<<prev_ref_y<<endl;
            ref_yaw = atan2((ref_y-prev_ref_y),(ref_x-prev_ref_x));
            // cout<<"ref_yaw: "<<ref_yaw<<endl;

            double ref_vel_x1 = ref_x - prev_ref_x;
            double ref_vel_y1 = ref_y - prev_ref_y;
            double ref_s_dot = ref_vel_x1 * ref_sx + ref_vel_y1 * ref_sy;
            double ref_d_dot = ref_vel_x1 * ref_dx + ref_vel_y1 * ref_dy;

            double ref_vel_x2 = prev_ref_x - p_prev_ref_x;
            double ref_vel_y2 = prev_ref_y - p_prev_ref_x;
            double ref_accel_x = ref_vel_x1 - ref_vel_x2;
            double ref_accel_y = ref_vel_y1 - ref_vel_y2;
            double ref_s_d_dot = ref_accel_x * ref_sx + ref_accel_y * ref_sy;
            double ref_d_d_dot = ref_accel_x * ref_dx + ref_accel_y * ref_dy;

            ptsx.push_back(prev_ref_x);
            ptsx.push_back(ref_x);
            ptsy.push_back(prev_ref_y);
            ptsy.push_back(ref_y);

            // initilize ego_car's start state vector
            ego_car.s = ref_s;
            ego_car.d = ref_d;
            ego_car.s_dot = ref_s_dot;
            ego_car.d_dot = ref_d_dot;
            ego_car.s_ddot = ref_s_d_dot;
            ego_car.d_ddot = ref_d_d_dot;
          }

          cout<<"show ego_car state: "<<endl;
          ego_car.showVehicle();          

          vector<string> states = ego_car.get_successor_state();
          double best_cost = 9999999;
          vector<vector<double>> best_target;
          vector<vector<double>> best_trajectory;

          for (auto state : states){
            cout<<"state: "<<state<<endl;
            vector<vector<double>> target = ego_car.generate_target(state,duration,predictions);
            cout<<"print target:"<<endl;
            printVector2D(target);
            if(!target.empty()){
              vector<vector<double>> trajectory = ego_car.generate_trajectory(target,duration);
              // cout<<"print trajectory: "<<endl;
              // printVector2D(trajectory);
              double cost = calculate_cost(trajectory,predictions);
              if(cost<best_cost){
                best_cost = cost;
                best_target = target;
                best_trajectory = trajectory;
              }
            }
            
          }
          
          cout<<"best_cost: "<<best_cost<<endl;
          cout<<"best_target: "<<endl;
          printVector2D(best_target);
          // cout<<"best_trajectory: "<<endl;
          // printVector2D(best_trajectory);
          double target_d = best_target[1][0];
          // cout<<"target_d: "<<target_d<<endl;
          int target_lane = int(target_d/4);
          cout<<"target_lane: "<<target_lane<<endl;
          // double ref_velocity = (best_trajectory[0][N_SAMPLES-1] - best_trajectory[0][0])/DT;
          double target_s = best_target[0][0];
          double target_speed = best_target[0][1];
          cout<<"target_speed: "<<target_speed<<endl;
          
          // set accelerate_coef;
          double accelerate_coef = (target_speed-ref_velocity)/SPEED_LIMIT;
          
          if (target_speed < 0.224){
            ref_velocity = 0.224;
          }
          else
          {
            ref_velocity += accelerate_coef * 0.224;
          }
          
          /*
          else if(ref_velocity < target_speed-0.224){
            // ref_velocity += 0.224;
            ref_velocity += accelerate_coef * 0.224;
          }
          else if ( ref_velocity>= target_speed )
          {
            ref_velocity -= 0.112;
          }
          */
          
          cout<<"ref_velocity: "<<ref_velocity<<endl;

          // add three points evenly spaced 30m along Frenet s
          vector<double> next_wp0 = getXY(car_s+30,(2+4*target_lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60,(2+4*target_lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90,(2+4*target_lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);
          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          for (int i=0; i<ptsx.size(); ++i) {
            double shift_x = ptsx[i] - ref_x;
            double shift_y = ptsy[i] - ref_y;

            ptsx[i] = shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw);
            ptsy[i] = shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw);
          }

          // create a spline
          tk::spline s;
          // set x,y points to spline

          /*
          cout<<"print ptsx value: "<<endl;
          for(int i =0;i<ptsx.size();i++){
            cout<<ptsx[i]<<endl;
          }
          */

          s.set_points(ptsx,ptsy);
          // define the actual (x,y) points we will use for the planner

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // start with all of previous path points from last time
          for (int i=0; i<previous_path_x.size(); i++){
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);

          }
          
          double target_x = 30;
          double target_y = s(target_x);
          double target_dist = sqrt(target_x * target_x + target_y * target_y);
          // vector<double> target_XY = getXY(target_position,target_d,map_waypoints_s,map_waypoints_x,map_waypoints_y);
          // double target_x = target_XY[0];
          // double target_dist = target_position - car_s;

          double x_add_on = 0;

          // after filled with previous path points, then we fill the rest points with 
          for (int i=0; i<(50-previous_path_x.size()); i++){
            double N = target_dist/(0.02*ref_velocity); //1 mile=1609 meters, 1 hour=3600 second
            double x_point = x_add_on + target_x/N;
            double y_point = s(x_point);
            x_add_on = x_point;

            double x_ref = x_point;     // temp var for rotation computation
            double y_ref = y_point;

            // rotate back to normal after rotating it earlier
            x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
            y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

            x_point += ref_x;   // add the starting reference point
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);

          }

          /*
          cout<<"print next_x_vals"<<endl;
          cout<<"size of next_x_vals: "<<next_x_vals.size()<<endl;
          for(int i=0;i<next_x_vals.size();i++){
            cout<<next_x_vals[i]<<endl;
          }
          */

          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}