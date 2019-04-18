#include "vehicle.h"

Vehicle::Vehicle(void)
{
	state 			= STATE_KL;
	lane  			= LANE_CENTER;
	ref_vel			= 0.0;
	max_ref_vel 	= 49.0;
	ref_vel_delta 	= 0.3;
	lane_offset 	= 0.05;
	Ts 				= 0.02;
	path_size		= 50;
	cp_inc			= 35;
	obstacle_cur_front		= false;
	obstacle_cur_rear		= false;
	obstacle_left_front		= false;
	obstacle_left_rear		= false;
	obstacle_right_front	= false;
	obstacle_right_rear		= false;
	
	PLCL_count	= 0;
	PLCR_count	= 0;
	PLC_count_threshold = 3;
	KL_count = 0;
	KL_count_threshold = 300;
	min_change_speed = 34;
}

Vehicle::~Vehicle() {}

void Vehicle::prediction(vector< vector<double> > sensor_fusion, int prev_path_size, double end_path_s)
{
	double obstacle_d = 0;
	double obstacle_s = 0;
	
	// Future Frenet path position of car
	double car_next_s;
	
	// Assume no obstacles at the start
	obstacle_cur_front		= false;
	obstacle_cur_rear		= false;
	obstacle_left_front		= false;
	obstacle_left_rear		= false;
	obstacle_right_front	= false;
	obstacle_right_rear		= false;

	// If previous path points exist, project car path position at end of previous path
	if(prev_path_size > 0)
		car_next_s = end_path_s;
	else // Otherwise car future path position is current s
		car_next_s = s;

	// Scan inputs from sensor_fusion
	// Find closest surrounding vehicles in left, current and right lanes
	double obstacle_cur_min_front_s = ROAD_LENGTH; // max and min distances in [m] for obstacle cars in all lanes
	double obstacle_cur_max_rear_s = -1;
	double obstacle_left_min_front_s = ROAD_LENGTH;
	double obstacle_left_max_rear_s = -1;
	double obstacle_right_min_front_s = ROAD_LENGTH;
	double obstacle_right_max_rear_s = -1;

	int obstacle_cur_front_id = -1; // Vehicle IDs of closest surrounding cars
	int obstacle_cur_rear_id = -1;
	int obstacle_left_front_id = -1;
	int obstacle_left_rear_id = -1;
	int obstacle_right_front_id = -1;
	int obstacle_right_rear_id = -1;

	for (int i = 0; i < sensor_fusion.size(); ++i)
	{
		obstacle_d = sensor_fusion[i][6]; //Frenet coordinates of surrounding cars
		obstacle_s = sensor_fusion[i][5];
		
		// #1 Find front and rear cars in current lane
		if ((obstacle_d > LANE_WIDTH*lane) && (obstacle_d < LANE_WIDTH*(lane+1)))
		{   // Check if this is closest front car
			if((obstacle_s > s) && (obstacle_s < obstacle_cur_min_front_s))
			{
				obstacle_cur_min_front_s = obstacle_s;
				obstacle_cur_front_id    = i;
			} // Check if this is closest rear car
			else if((obstacle_s < s) && (obstacle_s > obstacle_cur_max_rear_s ))
			{
				obstacle_cur_max_rear_s = obstacle_s;
				obstacle_cur_rear_id 	= i;
			}
			continue;
		}
		
		// #2 Find front and rear cars in left lane
		if(lane != LANE_LEFT) 
		{
			if((obstacle_d > LANE_WIDTH*(lane-1)) && (obstacle_d < LANE_WIDTH*lane))
			{   // Check if this is closest left front car
				if((obstacle_s > s) && (obstacle_s < obstacle_left_min_front_s))
				{
					obstacle_left_min_front_s = obstacle_s;
					obstacle_left_front_id    = i;
				}
				// Check if this is closest left rear car
				else if((obstacle_s < s) && (obstacle_s > obstacle_left_max_rear_s))
				{
					obstacle_left_max_rear_s = obstacle_s;
					obstacle_left_rear_id 	= i;
				}
				continue;
			}	
		}
		
		// #3 Find front and rear vehicle in right lane
		if(lane != LANE_RIGHT)
		{
			if((obstacle_d > LANE_WIDTH*(lane+1)) && (obstacle_d < LANE_WIDTH*(lane+2)))
			{	// Check if this is closest right front car
				if((obstacle_s > s) && (obstacle_s < obstacle_right_min_front_s))
				{
					obstacle_right_min_front_s = obstacle_s;
					obstacle_right_front_id    = i;
				}// Check if this is closest right rear car
				else if((obstacle_s < s) && (obstacle_s > obstacle_right_max_rear_s))
				{
					obstacle_right_max_rear_s = obstacle_s;
					obstacle_right_rear_id 	= i;
				}
				continue;
			}
		}
	} //for
	
	// Check if gap between obstacle vehicles and ego vehicle is within certain ranges
	// If yes, set up a flag indicating an obstacle
	// Predict behaviors of surrounding cars
	double obstacle_vx = 0; // speeds of surrounding cars
	double obstacle_vy = 0;
	double obstacle_v = 0;

	if(obstacle_cur_front_id != -1)
	{   // Obstacle speed
		obstacle_vx = sensor_fusion[obstacle_cur_front_id][3];
		obstacle_vy = sensor_fusion[obstacle_cur_front_id][4];
		obstacle_v = sqrt(pow(obstacle_vx,2) + pow(obstacle_vy,2));
		// Obstacle position
		obstacle_s = sensor_fusion[obstacle_cur_front_id][5];
		obstacle_s += (double)prev_path_size * Ts * obstacle_v;
		if((obstacle_s - car_next_s) < cp_inc)
		{   
			obstacle_cur_front = true; // Flag for front car in current lane
			obstacle_cur_front_v = obstacle_v / MPH_TO_MPS; // [mph]
		}
	}
	
	if(obstacle_cur_rear_id != -1)
	{   // Obstacle speed
		obstacle_vx = sensor_fusion[obstacle_cur_rear_id][3];
		obstacle_vy = sensor_fusion[obstacle_cur_rear_id][4];
		obstacle_v = sqrt(pow(obstacle_vx, 2) + pow(obstacle_vy, 2));
		// Obstacle position
		obstacle_s = sensor_fusion[obstacle_cur_rear_id][5];
		obstacle_s += (double)prev_path_size * Ts * obstacle_v;
		if((car_next_s - obstacle_s) < 0.5*cp_inc)
			obstacle_cur_rear = true; // Flag for rear car in current lane
	}
	
	if(obstacle_left_front_id != -1)
	{   // Obstacle speed
		obstacle_vx = sensor_fusion[obstacle_left_front_id][3];
		obstacle_vy = sensor_fusion[obstacle_left_front_id][4];
		obstacle_v = sqrt(pow(obstacle_vx, 2) + pow(obstacle_vy, 2));
		// Obstacle position
		obstacle_s = sensor_fusion[obstacle_left_front_id][5];
		obstacle_s += (double)prev_path_size * Ts * obstacle_v;
		if((obstacle_s - car_next_s) < cp_inc)
			obstacle_left_front = true; // Flag for front car in left lane
	}
	
	if(obstacle_left_rear_id != -1)
	{   // Obstacle speed
		obstacle_vx = sensor_fusion[obstacle_left_rear_id][3];
		obstacle_vy = sensor_fusion[obstacle_left_rear_id][4];
		obstacle_v = sqrt(pow(obstacle_vx, 2) + pow(obstacle_vy, 2));
		// Obstacle position
		obstacle_s = sensor_fusion[obstacle_left_rear_id][5];
		obstacle_s += (double)prev_path_size * Ts * obstacle_v;
		if((car_next_s - obstacle_s) < 0.5*cp_inc)
			obstacle_left_rear = true; // Flag for rear car in left lane
	}
	
	if(obstacle_right_front_id != -1)
	{   // Obstacle speed
		obstacle_vx = sensor_fusion[obstacle_right_front_id][3];
		obstacle_vy = sensor_fusion[obstacle_right_front_id][4];
		obstacle_v = sqrt(pow(obstacle_vx, 2) + pow(obstacle_vy, 2));
		// Obstacle position
		obstacle_s = sensor_fusion[obstacle_right_front_id][5];
		obstacle_s += (double)prev_path_size * Ts * obstacle_v;
		if((obstacle_s - car_next_s) < cp_inc)
			obstacle_right_front = true; // Flag for front car in right lane
	}
	
	if(obstacle_right_rear_id != -1)
	{   // Obstacle speed
		obstacle_vx = sensor_fusion[obstacle_right_rear_id][3];
		obstacle_vy = sensor_fusion[obstacle_right_rear_id][4];
		obstacle_v = sqrt(pow(obstacle_vx, 2) + pow(obstacle_vy, 2));
		// Obstacle position
		obstacle_s = sensor_fusion[obstacle_right_rear_id][5];
		obstacle_s += (double)prev_path_size * Ts * obstacle_v;
		if((car_next_s - obstacle_s) < 0.5*cp_inc)
			obstacle_right_rear = true; // Flag for rear car in right lane
	}
}

void Vehicle::behavior_planning(void)
{
	switch(state)
	{
		case STATE_KL: 
			state_KL();	
			break;
		case STATE_PLCL: 
			state_PLCL(); 
			break;
		case STATE_PLCR: 
			state_PLCR(); 
			break;
		case STATE_LC: 
			state_LC();	
			break;
		default: 
			state = STATE_KL;
	}
}

void Vehicle::state_KL(void)
{
	if(obstacle_cur_front)
	{	// If there is an obstacle in front, either lower down speed or prepare lane change
		printf("KL, front car detected\n");
		if(obstacle_cur_front_v < speed)
			ref_vel -= ref_vel_delta; // Reduce current speed
		
		// Check current lane to decide whether to change left or right
		if(lane != LANE_LEFT && PLCL_count == 0)
		{
			printf("KL to PLCL\n");
			state = STATE_PLCL; 
		}
		else if (lane != LANE_RIGHT && PLCR_count == 0)
		{
			printf("KL to PLCR\n");
			state = STATE_PLCR;
		}
		else
		{   // Set both PLCx state counts to zero and try changing lanes again in the next step
			PLCL_count = 0;
			PLCR_count = 0;
		}
	}	
	else
	{   // Check if gap between front vehicle and ego vehicle is large enough to accelerate
		if(ref_vel < max_ref_vel)
			ref_vel += ref_vel_delta;
		
		if (lane != LANE_CENTER)
		{   // trying to run on the center lane
			KL_count = KL_count + 1;
			if(KL_count > KL_count_threshold)
			{
				if(lane != LANE_LEFT)
				{
					printf("KL to PLCL\n");
					state = STATE_PLCL; 
				}
				else if (lane != LANE_RIGHT)
				{
					printf("KL to PLCR\n");
					state = STATE_PLCR;
				}
			}
		}
		else
			KL_count = 0;		
	}
}

void Vehicle::state_PLCL(void)
{
	bool change_lane = true; // Flag to change lanes if safe
	
	// Car in front in left lane
	if(obstacle_left_front)
	{	// Check if gap between front left vehicle and ego vehicle is too small
		printf("PLCL, left front car detected\n");
		change_lane = false; // NOT safe to change lane
		if(obstacle_cur_front && obstacle_cur_front_v < speed)
			ref_vel -= ref_vel_delta; // Reduce current speed
		PLCL_count = PLCL_count + 1;
		if(PLCL_count > PLC_count_threshold)
		{
			state = STATE_KL;
			printf("PLCR to KL\n");
		}
		return;
	}
	
	// Car in rear in left lane
	if(obstacle_left_rear)
	{   // Check if gap between rear left vehicle and ego vehicle is too small
		printf("PLCL, left rear car detected\n");
		change_lane = false; // NOT safe to change lane
		if(obstacle_cur_front && obstacle_cur_front_v < speed)
			ref_vel -= ref_vel_delta; // Reduce current speed
		PLCL_count = PLCL_count + 1;
		if(PLCL_count > PLC_count_threshold)
		{
			state = STATE_KL;
			printf("PLCL to KL\n");
		}
		return;
	}
	
	if(change_lane && (speed > min_change_speed))
	{
		state = STATE_LC;
		printf("PLCL to LC\n");
		lane = lane - 1;
		PLCL_count = 0;
	}
}

void Vehicle::state_PLCR(void)
{
	bool change_lane = true; // Flag to change lanes if safe

	// Car in front in right lane
	if(obstacle_right_front)
	{   // Check if gap between front right vehicle and ego vehicle is too small
		printf("PLCR, right front car detected\n");
		change_lane = false; // NOT safe to change lane
		if(obstacle_cur_front && obstacle_cur_front_v < speed)
			ref_vel -= ref_vel_delta; // Reduce current speed
		PLCR_count = PLCR_count + 1;
		if(PLCR_count > PLC_count_threshold)
		{
			state = STATE_KL;
			printf("PLCR to KL\n");
		}
		return;
	}

	// Car in rear in right lane
	if(obstacle_right_rear)
	{   // Check if gap between rear right vehicle and ego vehicle is too small
		printf("PLCR, right rear car detected\n");
		change_lane = false; // NOT safe to change lane
		if(obstacle_cur_front && obstacle_cur_front_v < speed)
			ref_vel -= ref_vel_delta; // Reduce current speed
		PLCR_count = PLCR_count + 1;
		if(PLCR_count > PLC_count_threshold)
		{
			state = STATE_KL;
			printf("PLCR to KL\n");
		}
		return;
	}
	
	if(change_lane && (speed > min_change_speed))
	{
		state = STATE_LC;
		printf("PLCR to LC\n");
		lane = lane + 1;
		PLCR_count = 0;
	}
}

void Vehicle::state_LC(void)
{
	double lane_centre = LANE_WIDTH*0.5 + LANE_WIDTH*lane;
	if(fabs(d - lane_centre)  < lane_offset)
	{
		state = STATE_KL;
		printf("LC to KL\n");
		// Set PLCx and KL_count timers to zero since lane change occurred
		PLCL_count = 0;
		PLCR_count = 0;
		KL_count   = 0;
	}	
}