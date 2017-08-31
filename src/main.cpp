#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include <assert.h>
#include <typeinfo>
#include <map>
#include <string>
#include "spline.h"

#define bignum 100000.0

using namespace std;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
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

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

int find_lane_number(double car_lane)
{
    // find the lane number from the d value
    // using lane width = 4 m
    if (car_lane > 8)
        return (3);
    else if (car_lane > 4)
        return (2);
    else
        return (1);
}

std::map<std::string, vector<double>> get_nearby_car_info(int my_lane, double car_s, vector<vector<double>> sensor_fusion)
{
    // find position, velocity and distance to the cars in neighboring lanes

    std::map<std::string, vector<double>> nearby_cars;
    std::map<std::string, double> max_dist_to_car;

    max_dist_to_car["left_front"] = bignum;
    max_dist_to_car["left_behind"] = bignum;
    max_dist_to_car["right_front"] = bignum;
    max_dist_to_car["right_behind"] = bignum;
    max_dist_to_car["same_front"] = bignum;

    nearby_cars["left_front"] = {-1, -1, bignum};
    nearby_cars["left_behind"] = {-1, -1, bignum};
    nearby_cars["right_front"] = {-1, -1, bignum};
    nearby_cars["right_behind"] = {-1, -1, bignum};
    nearby_cars["same_front"] = {-1, -1, bignum};

    for (int i=0; i<sensor_fusion.size(); i++)
    {
        vector<double> other_car = sensor_fusion[i];
        // sensor fusion array: id, x, y, vx, vy, s, d
        int lane_num = find_lane_number(other_car[6]);
        double ox = other_car[1];
        double oy = other_car[2];
        double ovx = other_car[3];
        double ovy = other_car[4];
        double ov = sqrt(ovx*ovx + ovy*ovy);
        double os = other_car[5];
        double dist_ahead = os - car_s;
        double dist_behind = car_s - os;

        if (lane_num - my_lane == 1)  // car in right lane
        {
            if ((os > car_s) && (dist_ahead < max_dist_to_car["right_front"]))
            {
                max_dist_to_car["right_front"] = dist_ahead;
                nearby_cars["right_front"] = {os, ov, dist_ahead};
            }
            if ((os < car_s) && (dist_behind < max_dist_to_car["right_behind"]))
            {
                max_dist_to_car["right_behind"] = dist_behind;
                nearby_cars["right_behind"] = {os, ov, dist_behind};
            }
        }
        else if (my_lane - lane_num == 1)  // car in left lane
        {
            if ((os > car_s) && (dist_ahead < max_dist_to_car["left_front"]))
            {
                max_dist_to_car["left_front"] = dist_ahead;
                nearby_cars["left_front"] = {os, ov, dist_ahead};
            }
            if ((os < car_s) && (dist_behind < max_dist_to_car["left_behind"]))
            {
                max_dist_to_car["left_behind"] = dist_behind;
                nearby_cars["left_behind"] = {os, ov, dist_behind};
            }
        }
        else if (my_lane == lane_num)  // car in same lane
        {
            if ((os > car_s) && (dist_ahead < max_dist_to_car["same_front"]))
            {
                max_dist_to_car["same_front"] = dist_ahead;
                nearby_cars["same_front"] = {os, ov, dist_ahead};
            }
        }

    }

    return (nearby_cars);
}

vector<double> get_car_in_front(int my_lane, double car_s, vector<vector<double>> sensor_fusion)
{
    // find position, velocity and distance to the car in front

    double max_dist_to_car_ahead = 10000;
    int car_in_front_id = -1;
    double car_in_front_v = -100;
    double car_in_front_s = -100;
    for (int i=0; i<sensor_fusion.size(); i++)
    {
        vector<double> other_car = sensor_fusion[i];
        // sensor fusion array: id, x, y, vx, vy, s, d
        int lane_num = find_lane_number(other_car[6]);
        if ((lane_num == my_lane) && ((double) other_car[5] > car_s))
        {
            double front_x = other_car[1];
            double front_y = other_car[2];
            double dist_ahead = other_car[5] - car_s;
            if (dist_ahead < max_dist_to_car_ahead)
            {
                max_dist_to_car_ahead = dist_ahead;
                car_in_front_id = other_car[0];
                double vx = other_car[3];
                double vy = other_car[4];
                car_in_front_v = sqrt(vx*vx + vy*vy);
                car_in_front_s = other_car[5];
            }
        }
    }

    vector<double> info = {car_in_front_s, car_in_front_v, max_dist_to_car_ahead};
    return (info);
}

int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2( (map_y-y),(map_x-x) );

	double angle = abs(theta-heading);

	if(angle > pi()/4)
	{
		closestWaypoint++;
	}

	return closestWaypoint;

}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
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

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}

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

  // speed limit 
  double max_speed = 21.0; // in m/s, somewhat less that 50 mph
  double target_speed = 1.0; // starting speed
  double target_lane = 1; // middle lane

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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

  h.onMessage([&target_lane,&target_speed,&max_speed,&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
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

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

          	json msgJson;

          	vector<double> next_x_vals;
          	vector<double> next_y_vals;

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
            //
            std::cout << "size of previous path = " << previous_path_x.size() << std::endl;
            // find other cars in the target lane

            double delta_t = 0.02; // 20 ms
            double dist_inc = 0.5; // meters
            double speed_limit = 50; // miles/hr
            speed_limit *= (1.6e3 / 3600); // m/s
            double alpha = 10.0;

            // find the closest car in front of my car
            int my_lane = find_lane_number(car_d);
            //vector<double> car_in_front = get_car_in_front(my_lane, car_s, sensor_fusion);

            std::map<std::string, vector<double>> nearby_cars = get_nearby_car_info(my_lane, car_s, sensor_fusion);

            // find closest cars in neighboring lanes
            //vector<double> cars_in_next_lane = get_nearby_car_info(my_lane, car_s, sensor_fusion);

            // alert when collision with car in front in imminent
            double time_to_collision = 0.5; // seconds
            double keep_distance = 10.0; // meters
            //if (car_in_front[1] > 0)
            int collision_ahead = 0;
            int collision_left = 0;
            int collision_right = 0;
            if (nearby_cars["same_front"][2] < bignum)
            {
                double car_in_front_s = nearby_cars["same_front"][0];
                double car_in_front_v = nearby_cars["same_front"][1];
                double max_dist_to_car_ahead = nearby_cars["same_front"][2];

                std::cout << " car in front (s, v) = " << car_in_front_s << ", " << car_in_front_v << " , my (s, v) = " << car_s << ", " << car_speed << "\n";
                double dis = car_in_front_s - car_s + (car_in_front_v - car_speed) * time_to_collision;
                std::cout << "dist now = " <<  max_dist_to_car_ahead << " dist proj " << dis << "\n";
                if (dis < keep_distance)
                {
                    std::cout << "!!!!!!!!!!!!!!!!!!!!!!!collision within 5 seconds, dis = " << dis << " \n";
                    collision_ahead = 1;
                }
            }

            // check if lane change is possible
            if (collision_ahead == 1)
            {
                if (target_lane > 0) // check left lane
                {
                    double car_in_front_s = nearby_cars["left_front"][0];
                    double car_in_front_v = nearby_cars["left_front"][1];
                    double max_dist_to_car_ahead = nearby_cars["left_front"][2];
                    double dis = car_in_front_s - car_s + (car_in_front_v - car_speed) * time_to_collision;
                    if (dis < keep_distance)
                        collision_left = 1;
                }
                if (target_lane < 2) // check right lane
                {
                    double car_in_front_s = nearby_cars["right_front"][0];
                    double car_in_front_v = nearby_cars["right_front"][1];
                    double max_dist_to_car_ahead = nearby_cars["right_front"][2];
                    double dis = car_in_front_s - car_s + (car_in_front_v - car_speed) * time_to_collision;
                    if (dis < keep_distance)
                        collision_right = 1;
                }

            }

            // create a spline from a set of points
            vector<double> s_spline;
            vector<double> d_spline;
            vector<double> anchor_pts_x, anchor_pts_y; // points to create the spline

            // start with a couple of points from the previous path
            int prev_path_size = previous_path_x.size();
            double last_x, last_y, second_last_x, second_last_y, heading_angle;
            if (prev_path_size > 1) // path history available
            {
                last_x = previous_path_x[prev_path_size-1];
                last_y = previous_path_y[prev_path_size-1];
                second_last_x = previous_path_x[prev_path_size-2];
                second_last_y = previous_path_y[prev_path_size-2];
                heading_angle = atan2(last_y - second_last_y, last_x - second_last_x);
            }
            else // just starting, use the current position
            {
                last_x = car_x;
                last_y = car_y;
                heading_angle = car_yaw,
                second_last_x = car_x - 5;
                second_last_y = car_y + 5 * tan(heading_angle);
            }

            // add the first two points in the anchor points list
            anchor_pts_x.push_back(second_last_x);
            anchor_pts_x.push_back(last_x);
            anchor_pts_y.push_back(second_last_y);
            anchor_pts_y.push_back(last_y);

            // add a few equally separated points ahead
            int slow_down = 0;
            if (collision_ahead == 1)
            {
                std::cout << "c_left, c_right, target_lane: " << collision_left << ", " << collision_right << ", " << target_lane << std::endl;
                if ((collision_left == 0) && (target_lane > 0))
                {
                    std::cout << "move left \n";
                    target_lane -= 1;
                }
                else if ((collision_right == 0) && (target_lane < 2))
                {
                    std::cout << "move right \n";
                    target_lane += 1;
                }
                else
                {
                    std::cout << "reduce speed \n";
                    slow_down = 1;
                }
            }
            //target_lane = 0;
            double ds = 30.0;
            for (int i=0; i<3; i++)
            {
                double temp_s = car_s + (i + 1) * ds;
                double temp_d = target_lane * 4 + 2;
                vector<double> xy = getXY(temp_s, temp_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
                anchor_pts_x.push_back(xy[0]);
                anchor_pts_y.push_back(xy[1]);
            }
            /*
            std::cout << "number of anchor points = " << anchor_pts_x.size() << std::endl;
            std::cout << "anchor points before coord trans \n";
            for (int i=0; i<anchor_pts_x.size(); i++)
                std::cout << "(x, y) = " << anchor_pts_x[i] << ", " << anchor_pts_y[i] << std::endl;
            std::cout << "last point: " << last_x << ", " << last_y << std::endl;
            std::cout << "heading angle = " << heading_angle << std::endl;
            */

            // change the points to the car's coordinate system
            for (int i=0; i<anchor_pts_x.size(); i++)
            {
                //shift origin
                double shift_x = anchor_pts_x[i] - last_x;
                double shift_y = anchor_pts_y[i] - last_y;
                // rotate
                anchor_pts_x[i] = (shift_x * cos(0 - heading_angle) - shift_y * sin(0 - heading_angle));
                anchor_pts_y[i] = (shift_x * sin(0 - heading_angle) + shift_y * cos(0 - heading_angle));
            }

            // create the spline
            tk::spline s;
            //std::cout << "trying to create spline from points \n";
            //for (int i=0; i<anchor_pts_x.size(); i++)
            //    std::cout << "(x, y) = " << anchor_pts_x[i] << ", " << anchor_pts_y[i] << std::endl;
            s.set_points(anchor_pts_x, anchor_pts_y);
            //std::cout << "created spline\n";

            // create the additional path that we want to append to the existing path
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_d = sqrt(target_x*target_x + target_y*target_y);

            // include all the unused previous points
            for (int i=0; i<prev_path_size; i++)
            {
                next_x_vals.push_back(previous_path_x[i]);
                next_y_vals.push_back(previous_path_y[i]);
            }

            // include the new points
            int n_rem_pts = 50 - prev_path_size;
            double inc_x = 0.0;
            for (int i=0; i<n_rem_pts; i++)
            {
                double n_points = target_d / target_speed / delta_t;
                double new_x = inc_x + target_x / n_points;
                double new_y = s(new_x);

                inc_x += target_x / n_points;

                // shift back to global coordinate system
                double store_x = new_x;
                double store_y = new_y;

                new_x = store_x * cos(heading_angle) - store_y * sin(heading_angle);
                new_y = store_x * sin(heading_angle) + store_y * cos(heading_angle);

                new_x += last_x;
                new_y += last_y;

                // push the new point to the path array
                next_x_vals.push_back(new_x);
                next_y_vals.push_back(new_y);

                // check speed and change it gradually if required
                if ((target_speed > 0.95 * max_speed) || (slow_down == 1))
                    target_speed -= 0.224;
                else if (target_speed < max_speed)
                    target_speed += 0.224;
                std::cout << "speed = " << target_speed << std::endl;
            }


          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

            // print the last point of the projected path
            std::cout << "last points in projected path: ";
            std::cout << next_x_vals[next_x_vals.size()-1] << ", ";
            std::cout << next_y_vals[next_y_vals.size()-1] << "\n";

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

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
