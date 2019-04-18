#include "trajectory_generator.h"

Trajectory_Generator::Trajectory_Generator(void)
{
	// Waypoint map to read from
	string map_file_ = "../data/highway_map.csv";
	ifstream in_map_(map_file_.c_str(), ifstream::in);
	string line;
	// Read map file into waypoints vectors
	while (getline(in_map_, line)) 
	{
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
}

Trajectory_Generator::~Trajectory_Generator(void){}

void Trajectory_Generator::generate_trajectory (Vehicle car, 
	vector<double>& next_x_vals, vector<double>& next_y_vals, 
	vector<double> previous_path_x, vector<double> previous_path_y, 
	double end_path_s)
{
	vector<double> control_points_x; // Control points that are spaced car.cp_inc apart
	vector<double> control_points_y; // These will be used for spline fitting later
	
	int prev_path_size = previous_path_x.size(); // Length of previous path [number of points]

	double ref_x;  // Reference starting point for interpolation
	double ref_y;  // Could be either ego vehicle state or end point of previous path 
	double ref_yaw; 
	double prev_ref_x;
	double prev_ref_y;
	double ref_s;
	
	if(prev_path_size < 2)
	{   // If previous path is too small, make path locally tangent to car heading
		ref_x = car.x;
		ref_y = car.y;
		ref_yaw = deg2rad(car.yaw);
		prev_ref_x = car.x - cos(car.yaw);
		prev_ref_y = car.y - sin(car.yaw);
		ref_s = car.s;
	}
	else
	{	// Use previous path's endpoint as reference
		ref_x = previous_path_x[prev_path_size-1];
		ref_y = previous_path_y[prev_path_size-1];
		prev_ref_x = previous_path_x[prev_path_size-2];
		prev_ref_y = previous_path_y[prev_path_size-2];
		ref_yaw = atan2(ref_y-prev_ref_y, ref_x-prev_ref_x);
		ref_s = end_path_s;
	}

	control_points_x.push_back(prev_ref_x);
	control_points_x.push_back(ref_x);
	control_points_y.push_back(prev_ref_y);
	control_points_y.push_back(ref_y);
	
	// Add evenly spaced points car.cp_inc apart in Frenet coordinates ahead of starting reference 
	vector<double> next_cp0 = getXY(ref_s +     car.cp_inc, 0.5*LANE_WIDTH + LANE_WIDTH * car.lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_cp1 = getXY(ref_s + 2.0*car.cp_inc, 0.5*LANE_WIDTH + LANE_WIDTH * car.lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	vector<double> next_cp2 = getXY(ref_s + 3.0*car.cp_inc, 0.5*LANE_WIDTH + LANE_WIDTH * car.lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
	
	control_points_x.push_back(next_cp0[0]);
	control_points_x.push_back(next_cp1[0]);
	control_points_x.push_back(next_cp2[0]);
	
	control_points_y.push_back(next_cp0[1]);
	control_points_y.push_back(next_cp1[1]);
	control_points_y.push_back(next_cp2[1]);
	
	// Make vehicle frame as local reference frame for control points
	for (int i = 0; i < control_points_x.size(); ++i)
	{
		double shift_x = control_points_x[i] - ref_x;
		double shift_y = control_points_y[i] - ref_y;
		control_points_x[i] = shift_x * cos(-ref_yaw) - shift_y * sin(-ref_yaw);
		control_points_y[i] = shift_x * sin(-ref_yaw) + shift_y * cos(-ref_yaw);
	}
	
	// Create a spline from the control points defined
	tk::spline control_spline;
	control_spline.set_points(control_points_x, control_points_y);
	
	// Define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
	for(int i = 0; i < prev_path_size; ++i)
	{   // First add unprocessed previous path points from last time step
		next_x_vals.push_back(previous_path_x[i]);
		next_y_vals.push_back(previous_path_y[i]);
	}
	
	double target_x = car.cp_inc;
	double target_y = control_spline(target_x);
	double target_dist = sqrt(pow(target_x,2) + pow(target_y,2));
	double N = target_dist / (car.Ts * car.ref_vel * MPH_TO_MPS);
	double x_inc = target_x / N;
	double x_offset = 0;
	
	// Complete trajectory generation with spline points
	for(int i=0; i < car.path_size - prev_path_size; ++i)
	{
		double x_spline = x_offset + x_inc;
		double y_spline = control_spline(x_spline);
		x_offset = x_spline;
		
		// Transform back to global frame from vehicle frame
		double x_global = x_spline*cos(ref_yaw) - y_spline*sin(ref_yaw);
		double y_global = x_spline*sin(ref_yaw) + y_spline*cos(ref_yaw);
		x_global += ref_x;
		y_global += ref_y;
		
		// Add waypoint to path lists
		next_x_vals.push_back(x_global);
		next_y_vals.push_back(y_global);
	}	
}