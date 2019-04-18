#ifndef TRAJECTORY_GENERATOR_H
#define TRAJECTORY_GENERATOR_H

#include <vector>
#include <fstream>
#include <math.h>
#include <iostream>
#include "helpers.h"
#include "vehicle.h"
#include "spline.h"
#include "json.hpp"

using namespace std;

class Trajectory_Generator
{
	public:
	// Load up map values for waypoints x,y,s and d normalized normal vectors
	vector<double> map_waypoints_x;
	vector<double> map_waypoints_y;
	vector<double> map_waypoints_s;
	vector<double> map_waypoints_dx;
	vector<double> map_waypoints_dy;
	
	Trajectory_Generator(void);
	virtual ~Trajectory_Generator(void);
	
	void generate_trajectory(Vehicle car, vector<double>& next_x_vals, vector<double>& next_y_vals, vector<double> previous_path_x, vector<double> previous_path_y, double end_path_s);
};
#endif