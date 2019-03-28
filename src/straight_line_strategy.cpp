#include <vector>

#include "logger.h"
#include "map.h"
#include "point.h"
#include "straight_line_strategy.h"
#include "trajectory.h"
#include "utils.h"

using namespace std;

// ----------------------------------------------------------------------------
// PUBLIC
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
void StraightLineStrategy::GenerateTrajectory()
{
  // GenerateXYTrajectory();
  GenerateSDTrajectory();
}

// ----------------------------------------------------------------------------
StraightLineStrategy::StraightLineStrategy()
{
  LOG(logDEBUG3) << "StraightLineStrategy::StraightLineStrategy()";
}

// ----------------------------------------------------------------------------
// PRIVATE
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
void StraightLineStrategy::GenerateXYTrajectory()
{
  // Clear previous trajectory
  trajectory.clear();
  
  double car_x = start_point.GetX();
  double car_y = start_point.GetY();
  double car_yaw = start_yaw;
  
  // Straight line test
  double dist_inc = 0.5;
  for(int i = 0; i < 50; i++)
  {
    Point p;
    double x = car_x + (dist_inc*i) * cos(deg2rad(car_yaw));
    double y = car_y + (dist_inc*i) * sin(deg2rad(car_yaw));
    p.SetXY(x, y);
    trajectory.push_back(p);
  }
}

// ----------------------------------------------------------------------------
void StraightLineStrategy::GenerateSDTrajectory()
{
  // Clear previous trajectory
  trajectory.clear();
  
  double car_s = start_point.GetS();
  LOG(logDEBUG3) << "StraightLineStrategy::GenerateSDTrajectory() - car_s = " << car_s;
  
  // Straight line test
  double dist_inc = 0.2;
  for(int i = 0; i < 50; i++)
  {
    double s = car_s + double(i+1) * dist_inc;
    double d = 2 + 4;
    Point p;
    p.SetFrenet(s, d);
    trajectory.push_back(p);
    LOG(logDEBUG4) << "StraightLineStrategy::GenerateSDTrajectory() - p : " << p;
  }
}

// ----------------------------------------------------------------------------
void StraightLineStrategy::GenerateSDTrajectory2()
{

}
  
  