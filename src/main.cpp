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
  double max_s = 6945.554;

  static int lane = 1;
  static double ref_volecity = 0; //mph

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

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
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
          double car_speed = j[1]["speed"];

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
          // int lane = 1;
          // double ref_volecity = 0; //mph

          
          if (prev_size >0){
            car_s = end_path_s;
          }

          bool too_close = false;

          // to find ref_v to use
          for(int i=0;i<sensor_fusion.size();i++){
            // check the car in the same lane
            float d = sensor_fusion[i][6];
            if ( d < (2 + 4 * lane + 1.8) && d > (2 + 4 * lane - 1.8)) {
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double check_speed = sqrt(vx*vx+vy*vy);
              double check_car_s = sensor_fusion[i][5];

              check_car_s += prev_size*0.02 * check_speed;  // if use previous paths points, need to project out to future
              // check_car in front of me and gap>30
              if(check_car_s>car_s && (check_car_s-car_s)<30){
                // we can take different actions
                // 1. decrease speed to avoid collision
                // 2. change lane to others
                // ref_volecity = 22.4;    //mph
                too_close = true;
                // cout<<"detect too close"<<endl;
              }
            }
          }

          
          if (too_close){
            ref_volecity -= 0.224;
          }
          else if (ref_volecity<44.8){
            ref_volecity += 0.224;
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
          if (prev_size <2 ) {
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
          }

          else {
            // cout<<"prev_size: "<<prev_size<<endl;
            ref_x = previous_path_x[prev_size-1];
            ref_y = previous_path_y[prev_size-1];
            // cout<<"ref_x: "<<ref_x<<", ref_y: "<<ref_y<<endl;

            double prev_ref_x = previous_path_x[prev_size-2];
            double prev_ref_y = previous_path_y[prev_size-2];
            // cout<<"prev_ref_x: "<<prev_ref_x<<", prev_ref_y: "<<prev_ref_y<<endl;
            ref_yaw = atan2((ref_y-prev_ref_y),(ref_x-prev_ref_x));
            // cout<<"ref_yaw: "<<ref_yaw<<endl;
            ptsx.push_back(prev_ref_x);
            ptsx.push_back(ref_x);
            ptsy.push_back(prev_ref_y);
            ptsy.push_back(ref_y);

          }          

          // add three points evenly spaced 30m along Frenet s
          vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

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

          double x_add_on = 0;

          // after filled with previous path points, then we fill the rest points with 
          for (int i=0; i<(50-previous_path_x.size()); i++){
            double N = target_dist/(0.02*ref_volecity/2.24); //1 mile=1609 meters, 1 hour=3600 second
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