#ifndef HELPERS_H
#define HELPERS_H

#include <math.h>
#include <string>
#include <vector>

// for convenience
using std::string;
using std::vector;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
//   else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

//
// Helper functions related to waypoints and converting from XY to Frenet
//   or vice versa
//

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Calculate distance between two points
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

// Calculate closest waypoint to current x, y position
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, 
                    const vector<double> &maps_y) {
  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for (int i = 0; i < maps_x.size(); ++i) {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if (dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

// Returns next waypoint of the closest waypoint
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, 
                 const vector<double> &maps_y) {
  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = std::min(2*pi() - angle, angle);

  if (angle > pi()/2) {
    ++closestWaypoint;
    if (closestWaypoint == maps_x.size()) {
      closestWaypoint = 0;
    }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, 
                         const vector<double> &maps_x, 
                         const vector<double> &maps_y) {
  int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

  int prev_wp;
  prev_wp = next_wp-1;
  if (next_wp == 0) {
    prev_wp  = maps_x.size()-1;
  }

  double n_x = maps_x[next_wp]-maps_x[prev_wp];
  double n_y = maps_y[next_wp]-maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
  double proj_x = proj_norm*n_x;
  double proj_y = proj_norm*n_y;

  double frenet_d = distance(x_x,x_y,proj_x,proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000-maps_x[prev_wp];
  double center_y = 2000-maps_y[prev_wp];
  double centerToPos = distance(center_x,center_y,x_x,x_y);
  double centerToRef = distance(center_x,center_y,proj_x,proj_y);

  if (centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for (int i = 0; i < prev_wp; ++i) {
    frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
  }

  frenet_s += distance(0,0,proj_x,proj_y);

  return {frenet_s,frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, 
                     const vector<double> &maps_x, 
                     const vector<double> &maps_y) {
  int prev_wp = -1;

  while (s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1))) {
    ++prev_wp;
  }

  int wp2 = (prev_wp+1)%maps_x.size();

  double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),
                         (maps_x[wp2]-maps_x[prev_wp]));
  // the x,y,s along the segment
  double seg_s = (s-maps_s[prev_wp]);

  double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
  double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

  double perp_heading = heading-pi()/2;

  double x = seg_x + d*cos(perp_heading);
  double y = seg_y + d*sin(perp_heading);

  return {x,y};
}

void transform(vector<double> &vec_x, vector<double> &vec_y, double global_x, double global_y, double global_yaw, string indicate)
{
  assert(vec_x.size()==vec_y.size());
  // vector<double> trans_x(vec_x.size(), 0.0);
  // vector<double> trans_y(vec_y.size(), 0.0);
  double shift_x, shift_y;
  if(indicate=="inverse")
  {
    for(int i=0;i < vec_x.size();++i)
    {

    }

  }
  else if(indicate=="forward")
  {
    for(int i=0;i < vec_x.size();++i)
    {
      shift_x = vec_x[i]-global_x;
      shift_y = vec_y[i]-global_y;
      // rotation matrix
      vec_x[i] = shift_x*cos(0-global_yaw)-shift_y*sin(0-global_yaw);
      vec_y[i] = shift_x*sin(0-global_yaw)+shift_y*cos(0-global_yaw);
    }
  }

}

double get_d_value(int lane_number)
{
  double lane_width = 4;
  return lane_number*lane_width+lane_width/2;
}


int decide_better_lane(int cur_lane, double car_s, int num_all_lanes, double gap_ahead,const vector<vector<double>>& sensor_data)
{
  int better_lane = cur_lane;
  bool DEBUG = true;
  double true_car_s = car_s-5.0;
  vector<int> safe_change_lanes;
  double sensor_s;
  double sensor_d;
  double front_gap = 15.0;
  double back_gap = 15.0;
  double front_car_s=99999999; // closest car in the lane we want to change to
  double back_car_s=0;
  vector<double> front_car_s1{99999999, 99999999};
  vector<double> back_car_s1{0, 0};
  bool not_safe = false;
  vector<bool> not_safe1{false, false};
  if(cur_lane-1 >= 0) // left lane change
    safe_change_lanes.push_back(cur_lane-1);
  if(cur_lane+1 < num_all_lanes) // right lane change
    safe_change_lanes.push_back(cur_lane+1);

  if(safe_change_lanes.size()==2)
  {
    for(int i=0;i<sensor_data.size();++i)
    {
      sensor_d = sensor_data[i][6];
      for(int j=0;j<safe_change_lanes.size();++j)
      {
        if((sensor_d > get_d_value(safe_change_lanes[j])-2) && (sensor_d <= get_d_value(safe_change_lanes[j])+2))
        {
          sensor_s = sensor_data[i][5];
          if(sensor_s < true_car_s+front_gap && sensor_s > true_car_s-back_gap)
            not_safe1[j] = true;
          if(sensor_s >= true_car_s && sensor_s - true_car_s < front_car_s1[j]-car_s)
            front_car_s1[j] = sensor_s;
          else if(sensor_s < true_car_s && true_car_s - sensor_s < true_car_s-back_car_s1[j])
            back_car_s1[j] = sensor_s;
        }
      }
    }

    if(!not_safe1[0] && !not_safe1[1] && front_car_s1[0] > gap_ahead && front_car_s1[1] > gap_ahead)
    {
      if(front_car_s1[0] > front_car_s1[1])
        better_lane = safe_change_lanes[0];
      else
        better_lane = safe_change_lanes[1];
    }
    else if(!not_safe1[0] && front_car_s1[0] > gap_ahead)
      better_lane = safe_change_lanes[0];
    else if(!not_safe1[1] && front_car_s1[1] > gap_ahead)
      better_lane = safe_change_lanes[1];

    if(DEBUG)
    {
      std::cout << "Gap ahead:" << gap_ahead << std::endl;
      std::cout << "Front car gap: ";
      for(int n=0;n < 2;++n)
        std::cout << front_car_s1[n] - true_car_s << "  ";
      std::cout << std::endl;
      std::cout << "Back car gap: ";
      for(int m=0;m < 2;++m)
        std::cout << true_car_s - back_car_s1[m] << "  ";
      std::cout << std::endl;
    }
  }
  else if(safe_change_lanes.size()==1)
  {
    for(int i=0;i<sensor_data.size();++i)
    {
      sensor_d = sensor_data[i][6];
      if((sensor_d > get_d_value(safe_change_lanes[0])-2) && (sensor_d <= get_d_value(safe_change_lanes[0])+2))
      {
        sensor_s = sensor_data[i][5];
        if(sensor_s < true_car_s+front_gap && sensor_s > true_car_s-back_gap)
          not_safe = true;

        if(sensor_s >= true_car_s && sensor_s - true_car_s < front_car_s-car_s)
          front_car_s = sensor_s;
        else if(sensor_s < true_car_s && true_car_s - sensor_s < true_car_s-back_car_s)
          back_car_s = sensor_s;
      }
    }

    if(!not_safe && front_car_s > gap_ahead)
      better_lane = safe_change_lanes[0]; // change to the side lane(either right or left)
    if(DEBUG)
    {
      std::cout << "Gap ahead:" << gap_ahead << std::endl;
      std::cout << "Front car gap: " << front_car_s- true_car_s << std::endl;
      std::cout << "Back car gap: " << true_car_s-back_car_s << std::endl;
    }

  }
  
  return better_lane;
}

vector<vector<double>> drive_in_circle(double car_yaw, double car_x, double car_y)
{
  // drive in a circle
  vector<double> x_vec;
  vector<double> y_vec;
  double dist_inc = 0.35;
  double cur_yaw;
  for (int i = 0; i < 50; ++i) {
    cur_yaw = (car_yaw+i*(0.8));//%(2*pi());
    x_vec.push_back(car_x+(dist_inc*i)*cos(deg2rad(cur_yaw)));
    y_vec.push_back(car_y+(dist_inc*i)*sin(deg2rad(cur_yaw)));
  }
  return {x_vec, y_vec};
}


double goal_distance_cost(int goal_lane, int intended_lane, int final_lane, double distance_to_goal) {
  // The cost increases with both the distance of intended lane from the goal
  //   and the distance of the final lane from the goal. The cost of being out 
  //   of the goal lane also becomes larger as the vehicle approaches the goal.
  int delta_d = 2.0 * goal_lane - intended_lane - final_lane;
  double cost = 1 - exp(-(std::abs(delta_d) / distance_to_goal));

  return cost;
}

double inefficiency_cost(int target_speed, int intended_lane, int final_lane, const std::vector<int> &lane_speeds) {
  // Cost becomes higher for trajectories with intended lane and final lane 
  //   that have traffic slower than target_speed.
  double speed_intended = lane_speeds[intended_lane];
  double speed_final = lane_speeds[final_lane];
  double cost = (2.0*target_speed - speed_intended - speed_final)/target_speed;

  return cost;
}

#endif  // HELPERS_H