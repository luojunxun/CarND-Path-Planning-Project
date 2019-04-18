#include <fstream>
#include <uWS/uWS.h>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <math.h>
#include <chrono>
#include "helpers.h"
#include "json.hpp"
#include "vehicle.h"
#include "trajectory_generator.h"

// for convenience
using namespace std;
using json = nlohmann::json;

int main() {
	uWS::Hub h;
	// Ego vehicle object
	Vehicle ego = Vehicle();
	Trajectory_Generator tg = Trajectory_Generator();
	
	h.onMessage([&ego, &tg](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
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
			 
			// Ego car's localization Data
			ego.x = j[1]["x"];
			ego.y = j[1]["y"];
			ego.s = j[1]["s"];
			ego.d = j[1]["d"];
			ego.yaw = j[1]["yaw"];
			ego.speed = j[1]["speed"];

			// Previous path data given to the Planner
			auto previous_path_x = j[1]["previous_path_x"];
			auto previous_path_y = j[1]["previous_path_y"];
			// Previous path's end s and d values 
			double end_path_s = j[1]["end_path_s"];
			double end_path_d = j[1]["end_path_d"];

			// Sensor Fusion Data, a list of all other cars on the same side of the road.
			auto sensor_fusion = j[1]["sensor_fusion"];

			// List of actual (x,y) waypoints used for trajectory generation
			vector<double> next_x_vals;
			vector<double> next_y_vals;
			
			// Get environment around ego vehicle
			ego.prediction(sensor_fusion, previous_path_x.size(), end_path_s);
			// Decide next state, new lane, and new ref_vel for ego vehicle
			ego.behavior_planning();
			// Generate reference trajectory for ego vehicle to follow
			tg.generate_trajectory(ego, next_x_vals, next_y_vals, previous_path_x, previous_path_y, end_path_s);
			
			// Send reference trajectory to simulator to execute
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