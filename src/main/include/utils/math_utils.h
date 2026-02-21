#ifndef MATHUTILS_H
#define MATHUTILS_H
#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/geometry/Rotation2d.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include "units/angular_velocity.h"

using namespace frc;
using TPS = units::turns_per_second_t;

Rotation2d getAngleFromRobotToTarget(Translation2d target, Translation2d reference, Rotation2d direction);
double getDistanceFromRobotToTarget(Translation2d target, Translation2d reference);
// TPS getTPSFromDistance(double distance, double maxSpeed, double slope, double y_intercept);

#endif 