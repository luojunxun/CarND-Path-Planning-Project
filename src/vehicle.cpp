#include "point.h"
#include "road.h"
#include "vehicle.h"

#include "logger.h"
#include "utils.h"

#include <iostream>
#include <math.h>
#include <map>
#include <string>
#include <iterator>

// ----------------------------------------------------------------------------
// PUBLIC
// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Initializes Vehicle
Vehicle::Vehicle()
{
  LOG(logDEBUG4) << "Vehicle::Vehicle()";
  speed = 0;
  yaw = 0;
  
  road_ptr = NULL;
}

// ----------------------------------------------------------------------------
Vehicle::Vehicle(const EnvironmentSensorData::SensedVehicleData& data, Road* road)
{
  this->road_ptr = road;
  UpdateSensorData(data);
}

// ----------------------------------------------------------------------------
Vehicle::~Vehicle() {}

// ----------------------------------------------------------------------------
void Vehicle::Move(double delta_t)
{
  position = PredictPosition(delta_t);
}

// ----------------------------------------------------------------------------
Point Vehicle::PredictPosition(double delta_t) const
{
  // The idea here is to assume constant velocity and constant yaw
  // However, the yaw must be transformed into Frenet coordinate system
  // so that the vehicle follow a line in that system.
  
  // Algorithm: Assume yaw_frenet = 0, so the vehicle will stay in the same lane (constant d-coordinate).
  const double s = this->position.GetS();
  const double d = this->position.GetD();
  const double future_s = s + speed * delta_t;
  
  return PointFrenet(future_s, d);
}

// ----------------------------------------------------------------------------
void Vehicle::Translate(const Point& new_pos)
{
  double dy = new_pos.GetY() - position.GetY();
  double dx = new_pos.GetX() - position.GetX();
  yaw = atan2(dy, dx);
  position = new_pos;
}

// ----------------------------------------------------------------------------
void Vehicle::UpdateSensorData(const EnvironmentSensorData::SensedVehicleData& data)
{
  PointCartesian pc(data.x, data.y);

  Point p(pc);
  position = p;
  speed = Magnitude(data.vx, data.vy); // [m/s]
  
  yaw = atan3(data.vy, data.vx);
  
  // Update current Lane
  lane = int(p.GetD() / road_ptr->LANE_WIDTH);
}
