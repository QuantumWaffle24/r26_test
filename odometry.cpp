#include "odometry.h"
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;

Odometry::Odometry(double wheel_radius, double rpm)
    : radius(wheel_radius), rpm(rpm) {
  // Linear velocity (m/s) = wheel circumference * revolutions per second
  double rps = rpm / 60.0;
  linear_vel = 2.0 * M_PI * radius * rps;
}

double Odometry::distance(int x1, int y1, int x2, int y2) {
  double dx = static_cast<double>(x2 - x1);
  double dy = static_cast<double>(y2 - y1);
  return std::sqrt(dx * dx + dy * dy);
}

double Odometry::angle(int x1, int y1, int x2, int y2) {
  // atan2 returns radians; convert to degrees
  return std::atan2(static_cast<double>(y2 - y1),
                    static_cast<double>(x2 - x1)) * 180.0 / M_PI;
}

static double normalize180(double a) {
  // Normalize to (-180, 180]
  while (a <= -180.0) a += 360.0;
  while (a >   180.0) a -= 360.0;
  return a;
}

MotionCommand Odometry::computeCommands(vector<pair<int, int>> &path) {
  MotionCommand res{0.0, 0.0};
  if (path.size() < 2 || linear_vel <= 0.0) return res;

  // 1) Total linear distance (in "grid units")
  // NOTE: This treats one grid step as 1 meter. If your grid cell size != 1 m,
  // you can multiply total_dist by cell_size before dividing by linear_vel.
  double total_dist = 0.0;
  for (size_t i = 1; i < path.size(); ++i) {
    total_dist += distance(path[i - 1].first, path[i - 1].second,
                           path[i].first,     path[i].second);
  }

  // 2) Total rotation: sum of absolute heading changes between segments
  double total_turn = 0.0;
  double prev_heading = angle(path[0].first, path[0].second,
                              path[1].first, path[1].second);
  for (size_t i = 2; i < path.size(); ++i) {
    double h = angle(path[i - 1].first, path[i - 1].second,
                     path[i].first,     path[i].second);
    double d = normalize180(h - prev_heading);
    total_turn += std::fabs(d);
    prev_heading = h;
  }

  // 3) Time = distance / speed (turn time not modeled here)
  res.time_sec = total_dist / linear_vel;
  res.angle_deg = total_turn;
  return res;
}
