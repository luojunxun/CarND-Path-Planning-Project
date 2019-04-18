#ifndef VEHICLE_H
#define VEHICLE_H

#include <math.h>
#include <iostream>
#include <vector>
#include "helpers.h"

#define STATE_KL		0
#define STATE_PLCL		1
#define STATE_PLCR		2
#define STATE_LCL		3
#define STATE_LCR		4
#define STATE_LC		5

#define LANE_LEFT		0
#define LANE_CENTER		1
#define LANE_RIGHT		2

using namespace std;

class Vehicle{
	public:
		int state; // States: KL=0, PLCL=1, PLCR=2, LCL=3, LCR=4, LC=5
		int lane;  // Lane: Left=0, Middle=1, Right=2
		double x; // Global coordinates [m]
		double y;
		double s; // Frenet coordinates [m]
		double d;
		double yaw;		// [rad]
		double speed;	// [mph]
		double ref_vel; // Reference speed of ego vehicle [mph]
		double max_ref_vel; // Maximum reference speed of ego vehicle [mph]
		double ref_vel_delta; // Change in ref_vel in mph to achieve target average acceleration
		double lane_offset; // Lateral offset from lane centre in [m]. To check when lane changing is complete.
		double Ts; // Sample time of simulator [sec]
		int path_size; // Length of current path [number of points]
		double cp_inc; // Distance between successive control points in Frenet coordinates [m]

		bool obstacle_cur_front; // indications of surrounding cars
		bool obstacle_cur_rear;
		bool obstacle_left_front;
		bool obstacle_left_rear;
		bool obstacle_right_front;
		bool obstacle_right_rear;

		double obstacle_cur_front_v; // Speed of obstacle front vehicle [mph]
		double min_change_speed; // Minimum speed above which car should change lanes [mph]
		
		int PLCL_count; // Timer to stay in STATE_PLCL
		int PLCR_count; // Timer to stay in STATE_PLCR
		int PLC_count_threshold; // Timer threshold to switch back to STATE_KL from STATE_PLCx
		double lane_ctr_threshold; // Distance of ego car from lane centre to check end of STATE_LCx
		int KL_count; // Timer to bias vehicle to stay in centre lane
		int KL_count_threshold; // Timer threshold to bias vehicle to stay in centre lane

		Vehicle();
		virtual ~Vehicle(void);
		void prediction(vector<vector<double> > sensor_fusion, int prev_path_size, double end_path_s);
		void behavior_planning(void);
		void state_KL(void);
		void state_PLCL(void);
		void state_PLCR(void);
		void state_LC(void);
};
#endif