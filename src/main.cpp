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
#include "spline.h"

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
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
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

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
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
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
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

  // Start in lane 1: mjd
  int lane = 1; // middle lane

  // have a reference velocity to target: mjd
  double ref_vel = 0.0; //49.5; // mph
  
  
  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane, &ref_vel](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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


		// mjd last path from simulator
		int prev_size = previous_path_x.size();


		// Sensor Fusion and avoiding other vehicles
		if(prev_size > 0) {
		  car_s = end_path_s;
		}

		// Checking other cars positions
		// bool too_close = false;
		bool car_infront = false;
		bool car_left = false;
		bool car_right = false;

		// Check all targets
		for(int i=0; i<sensor_fusion.size(); i++) {
		  //cout << "contacs" << sensor_fusion.size() << endl;
		  float d = sensor_fusion[i][6]; // car d value
		  int contact_lane = -1;
		  // check if the vehicle is in left lane
		  if(d < 4 && d > 0) { 
		    contact_lane = 0;
		  }
		  // check if the vehicle is in center lane
		  else if(d < 8 && d > 4 ){
		    contact_lane = 1;
		  }
		  // check if the vehicle is in right lane
		  else if(d < 12 && d > 8){
		    contact_lane = 2;
		  }
		  if(contact_lane < 0) {
		    continue;
		  }
		  // find the cars speed
		  double vx = sensor_fusion[i][3];
		  double vy = sensor_fusion[i][4];
		  double check_speed = sqrt(vx*vx+vy*vy);
		  double check_car_s = sensor_fusion[i][5];

		  // Estimate the cars s-position after previous traj
		  check_car_s += ((double)prev_size*0.02*check_speed); // if using prev points can project s value outwards in time

		  //
		  if (contact_lane == lane) {
		    car_infront |= check_car_s > car_s && check_car_s - car_s < 30;
		    //cout << "Car in front" << endl;
		  }
		  else if (contact_lane == lane-1) {
		    car_left |= car_s - 30 < check_car_s && car_s + 30 > check_car_s;
		    //cout << "car to the left" << endl;
		  }
		  else if (contact_lane == lane+1) {
		    car_right |= car_s - 30 < check_car_s && car_s + 30 > check_car_s;
		    //cout << "car to the right" << endl;
		  }
		  
		  //if((check_car_s > car_s) && ((check_car_s-car_s) <30)){
		    // Do some logic here, lower ref velocity so we don't crash into car or flag to change lanes
		    //ref_vel = 29.5; // mph
		    // too_close = true;


		}
		  // Behavior

		  /***************
Drive in center lane unless there is a vehicle in front, try to move left, then try to move right. go back to center when possible. 
		   *************/
		  if (car_infront) {
		    cout << "car in front" << endl;
		    if(!car_left && lane > 0) {
		      lane--; // no car left and left lane
		    }
		    else if(!car_right && lane != 2) {
		      lane++; // change right
		    }
		    else {
		      ref_vel -= 0.224; // subtracts about 5 m/s^2 change
		    }
		  }
		  else {
		    if (lane != 1) { // not in center
		      if( (lane == 0 && !car_right) ||
			  (lane == 2 && !car_left) ) {
			lane = 1; // go to center
		      }
		    }
		    if(ref_vel < 48.5) { // 49.5 goes over speed still
		      ref_vel += 0.324; //0.224 works but slow
		    }
		  }
		  

		      /*
		      if (lane == 0) { // Left lane
			if (car_right) {
			  continue;  // if car is right of us do nothing
			}
			else {
			  cout << "Change to center" << endl;
			  lane = 1; // Go to center lane (right)
			}
		      }
		      else if (lane == 1) { // Center lane
			if (car_right && car_left) {
			  continue;
			}
			else if (car_left && !car_right) {
			  cout << "car to the left" << endl;
			  cout << "cheange to right" << endl;
			  lane = 2; //continue;
			}
			else if (car_right && !car_left) {
			  cout << "car to the right" << endl;
			  cout << "change to left" << endl;
			  lane = 0; // go to left lane
			}
			else if (!car_right && !car_left) {
			  cout << "change left" << endl;
			  lane = 0; // go to left lane
			}
			else {
			  continue;
			}
		      }
		      else if (lane == 2) { // Right lane
			if (car_left) {
			  cout << "car to the left" << endl;
			  continue;
			}
			else {
			  lane = 1; // Go to Center lane
			  cout << "change to the left" << endl;
			}
		      }
		    }
		    
		    //}
		    */
		      
		
		  

	
		    //cout << "current lane" << lane << endl;
		  //if(car_infront) {
		  //ref_vel -= 0.224; // subtracts about 5 m/s^2 change
		  //}
		//else if(ref_vel < 48.5) { // 49.5 goes over speed still
		//  ref_vel += 0.324; //0.224 works but slow

		//}
		

          	// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
		// use spline.h instead of polinomial fitting or smothing out the path

		// Start: Path Smoothing

		// Start in lane 1: mjd
		//int lane = 1; // middle lane

		// have a reference velocity to target: mjd
		//double ref_vel = 49.5; // mph
		// create a path based on its angle tangent to the car

		// Create a list of widely spaced x,y waypoints, evenly spaced at 30m. Later we will interperlate these waypoints with a spline and fill it in with more points that control speed
		vector<double> ptsx;
		vector<double> ptsy;

		// reference x,y, yaw, states
		// either we will reference the starting point as were the car is or at the previous paths end point
		double ref_x = car_x;
		double ref_y = car_y;
		double ref_yaw = deg2rad(car_yaw);

		// if previous size is almost empty, use the car as starting reference
		if(prev_size < 2){
		  // Use two points that make the path tangent to the car
		  double prev_car_x = car_x - cos(car_yaw);
		  double prev_car_y = car_y - sin(car_yaw);

		  ptsx.push_back(prev_car_x);
		  ptsx.push_back(car_x);

		  ptsy.push_back(prev_car_y);
		  ptsy.push_back(car_y);
		}

		// use the previous path's end point as starting ref
		else{
		  // Redefine reference state as previous path end points
		  ref_x = previous_path_x[prev_size-1];
		  ref_y = previous_path_y[prev_size-1];

		  double ref_x_prev = previous_path_x[prev_size-2];
		  double ref_y_prev = previous_path_y[prev_size-2];
		  ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

		  // Use two points that make the path tangent to the previouse path's end point
		  ptsx.push_back(ref_x_prev);
		  ptsx.push_back(ref_x);

		  ptsy.push_back(ref_y_prev);
		  ptsy.push_back(ref_y);
		}

		// in Frenet add evenly 30m spaced points ahead of the starting reference
		vector<double> next_wp0 = getXY(car_s+30, 2+4*lane, map_waypoints_s,map_waypoints_x,map_waypoints_y);
		vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
		vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

		ptsx.push_back(next_wp0[0]);
		ptsx.push_back(next_wp1[0]);
		ptsx.push_back(next_wp2[0]);

		ptsy.push_back(next_wp0[1]);
		ptsy.push_back(next_wp1[1]);
		ptsy.push_back(next_wp2[1]);

		for (int i = 0; i < ptsx.size(); i++){
		  // shift car reference angle to 0 degrees
		  double shift_x = ptsx[i]-ref_x;
		  double shift_y = ptsy[i]-ref_y;

		  ptsx[i] = (shift_x *cos(0-ref_yaw)-shift_y*sin(0-ref_yaw));
		  ptsy[i] = (shift_x *sin(0-ref_yaw)+shift_y*cos(0-ref_yaw));
		}

		// create a spline
		tk::spline s;

		// set (x,y) points to the spline
		s.set_points(ptsx,ptsy);

		// Define the actual (x,y) points we will use for the planner
		vector<double> next_x_vals;
		vector<double> next_y_vals;

		// Start with all of the previous path points from last time
		for(int i=0; i<previous_path_x.size(); i++){
		  next_x_vals.push_back(previous_path_x[i]);
		  next_y_vals.push_back(previous_path_y[i]);
		}

		// Calculate the how to break up splne points so that we travel at our desired reference velocity
		double target_x = 30.0;
		double target_y = s(target_x); // ask spline for the y from the given x
		double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y));
		double x_add_on = 0;

		// Fill up the rest of our path planner after filling it with previous points, here we will always output 50 points
		for (int i=1; i<=50-previous_path_x.size(); i++) {
		  double N = (target_dist/(0.02*ref_vel/2.24)); // divide by 2.24 to put it in meters per sec
		  double x_point = x_add_on+(target_x)/N;
		  double y_point = s(x_point);

		  x_add_on = x_point;

		  double x_ref = x_point;
		  double y_ref = y_point;

		  // rotate back to normal after rotating it earlier
		  x_point = (x_ref *cos(ref_yaw)-y_ref*sin(ref_yaw));
		  y_point = (x_ref *sin(ref_yaw)+y_ref*cos(ref_yaw));

		  x_point += ref_x;
		  y_point += ref_y;

		  next_x_vals.push_back(x_point);
		  next_y_vals.push_back(y_point);
		  
		}
		
		// End: Path Smoothing
		

		// Start: Basic driving in lane
		/*
		double dist_inc = 0.5;
		for(int i = 0; i < 50; i++)
		  {
		    double next_s = car_s+(i+1)*dist_inc;
		    double next_d = 6;  // set Fernet d
		    vector<double> xy = getXY(next_s, next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
		    
		    next_x_vals.push_back(xy[0]);
		    next_y_vals.push_back(xy[1]);
		  }
		*/
		// End: Basic driving in lane

          	json msgJson;
		
          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

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
