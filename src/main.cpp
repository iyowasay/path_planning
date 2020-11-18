#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "spline.h"
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

const float lane_width = 4.0;
const float SPEED_LIMIT = 49.5f; // MPH, a bit lower than speed limit.
const float MPH2MPS = 0.44704; // 1 MPH = 0.44704 m/s 
const int TOTAL_POINTS = 50; 
const int NUM_LANES = 3; 
const float UPDATE_RATE = 0.02; // the car will visit sequentially every 0.02 seconds
const int WAIT_AFTER_CHANGE = 50;

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

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while(getline(in_map_, line)) {
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
  
  int lane = 1; // start in the middle lane(lane 1)
  double cur_speed = 0.0; // MPH 
  int iter_count=0;
  
  h.onMessage([&iter_count, &cur_speed, &lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx, &map_waypoints_dy]
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
          // // Ego car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"]; // in degree
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          // of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          // Note that the previous_path_x contains points that are generated and sent in last iteration 
          // but not executed or used by the simulator(left-out points)
          int unused_n = previous_path_x.size();

          std::cout << lane << std::endl;
          // sensor fusion
          if(unused_n > 0)
            car_s = end_path_s;

          bool collision_warn = false;
          double safe_gap = 27.0;
          double cur_gap = 0.0;
          double target_d = get_d_value(lane);
          double max_speed = SPEED_LIMIT;
          // avoid collision 
          for(int i=0;i < sensor_fusion.size();++i)
          {
            double sense_d = sensor_fusion[i][6];
            if(sense_d < target_d+2 && sense_d > target_d-2)
            {
              double sense_s = sensor_fusion[i][5]; 
              double vx = sensor_fusion[i][3];
              double vy = sensor_fusion[i][4];
              double v = sqrt(vx*vx+vy*vy);
              double predict_sense_s; // the s value of surrounding vehicle after a short period(prediction of future position)
              predict_sense_s = sense_s + static_cast<double>(unused_n)*UPDATE_RATE*v; // fuzzy logic
              cur_gap = predict_sense_s - car_s;
              if((car_s < predict_sense_s) && cur_gap <= safe_gap)
              {
                collision_warn = true;
                max_speed = v/MPH2MPS;
                // std::cout << "Front car speed:" << v << std::endl;
              }
            }
          }

          iter_count++;
          if(collision_warn)
          {
            if(iter_count > WAIT_AFTER_CHANGE)
            {
              int curLane = lane;
              lane = decide_better_lane(curLane, car_s, NUM_LANES, cur_gap, sensor_fusion);
              if(lane != curLane) // lane changed
                iter_count=0;
            }

            if(cur_speed >= max_speed)
            {
              cur_speed -= 0.2*(1+0.28*(safe_gap-cur_gap)/safe_gap);
              if(cur_speed < 0)
                cur_speed = 0.0;
            }
            else if(cur_speed < max_speed)
              cur_speed += 0.224;
          }
          else if(cur_speed < max_speed)
            cur_speed += 0.224;


          // x and y points for curve fitting(spline)
          vector<double> X; 
          vector<double> Y;

          //reference of x,y,angle for transformation of coordinate
          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw); 

          if(unused_n < 2)
          {
            // use starting position
            double prev_car_x = car_x - cos(car_yaw);
            double prev_car_y = car_y - sin(car_yaw);

            X.push_back(prev_car_x);
            Y.push_back(prev_car_y);
            X.push_back(car_x);
            Y.push_back(car_y);
          }
          else
          {
            ref_x = previous_path_x[unused_n-1];
            ref_y = previous_path_y[unused_n-1];

            double ref_x_prev = previous_path_x[unused_n-2];
            double ref_y_prev = previous_path_y[unused_n-2];
            ref_yaw = atan2((ref_y-ref_y_prev), (ref_x-ref_x_prev));

            X.push_back(ref_x_prev);
            Y.push_back(ref_y_prev);
            X.push_back(ref_x);
            Y.push_back(ref_y);
          }

          // add 3 more sparse(spaced) points using Fernet system
          // since d unit vector is calculated from the middle(double-yellow driving line)
          // d coordinate in the middle of targeting lane = lane_number*4 + 4/2
          double target_x_ = 30.0;
          double spacing = 30.0; // this spacing value is to avoid aggressive lane change(make it smoother) 
          for(unsigned int i = 1;i < 3;++i)
          {
            vector<double> pt1 = getXY(car_s+i*spacing, target_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            X.push_back(pt1[0]);
            Y.push_back(pt1[1]);
          }

          // shift and rotation from global coordiante to local coordinate of eagle vehicle
          // shift car reference angle to 0 degree, mathematically easier, local transformation
          // this transformation also prevents us from getting multiple y values with one x value(vertical line) via the spline function.
          transform(X, Y, ref_x, ref_y, ref_yaw, "forward");
          // spline --- why use spline instead of polynomial fit?
          tk::spline s;
          s.set_points(X,Y); // add points into spline
          // path planning points that will be sent to the simulator.
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          // use the left-out points from previous
          for(int j=0;j < unused_n;++j)
          {
            next_x_vals.push_back(previous_path_x[j]);
            next_y_vals.push_back(previous_path_y[j]);
          }

          // in vehicle coordinate
          double target_y = s(target_x_);
          double target_distance = distance(target_x_, target_y, 0,0);

          double x_add_on = 0; // start from the origin of vehicle coordinate

          for(int i=1;i < TOTAL_POINTS - unused_n ;++i)
          {
            // think of other ways of getting these points(instead of linear approximation)
            double N = target_distance/(UPDATE_RATE*cur_speed*MPH2MPS); 
            double x_point = x_add_on + target_x_/N;
            double y_point = s(x_point);
            x_add_on = x_point;
            double x_ref = x_point;
            double y_ref = y_point;
            // inverse transformation(car coordinates to global coordinates)
            x_point = x_ref*cos(ref_yaw)-y_ref*sin(ref_yaw);
            y_point = x_ref*sin(ref_yaw)+y_ref*cos(ref_yaw);
            x_point += ref_x;
            y_point += ref_y;

            next_x_vals.push_back(x_point);
            next_y_vals.push_back(y_point);
          }

          assert(next_x_vals.size() == next_y_vals.size());
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