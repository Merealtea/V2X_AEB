#pragma once

#include <cmath>
#include <deque>
#include <set>
#include <unordered_map>
#include <vector>

#include <ros/ros.h>

#include <opencv2/opencv.hpp>

// // #include <mrpt/bayes/CParticleFilterData.h>
// // #include <mrpt/config.h>
// // #include <mrpt/math/distributions.h>
// // #include <mrpt/math/matrix_serialization.h>
// // #include <mrpt/math/wrap2pi.h>
// // #include <mrpt/obs/CObservation.h>
// // // #include <mrpt/obs/CObservationBearingRange.h>
// // #include <mrpt/obs/CSensoryFrame.h>
// // #include <mrpt/poses/CPose3D.h>
// // #include <mrpt/random.h>
// // #include <mrpt/serialization/CArchive.h>
// // #include <mrpt/serialization/CSerializable.h>
// // #include <mrpt/system/os.h>

const double EPS = 1e-6;

namespace tracking {
namespace part_based_tracking {
enum class VehiclePartType {
  OVERALL = 0,
  REAR, // for radar
  PLATE,
  LEFTREARLIGHT,
  RIGHTREARLIGHT,
  LEFTBACKWHEEL,
  RIGHTBACKWHEEL,
  LEFTFRONTWHEEL,
  RIGHTFRONTWHEEL,
  REARCENTER
};

struct PointRangeYaw {
  double range, yaw;
  bool is_valid;

  PointRangeYaw() : range(0.0), yaw(0.0), is_valid(false) {}
  PointRangeYaw(float range, float yaw, bool is_valid)
      : range(range), yaw(yaw), is_valid(is_valid) {}
};

struct SensorPose {
  double x, y, z;          // in meter
  double pitch, roll, yaw; // in rad

  SensorPose() {}
  SensorPose(const double x, const double y, const double z, const double pitch,
             const double roll, const double yaw)
      : x(x), y(y), z(z), pitch(pitch), roll(roll), yaw(yaw) {}
  SensorPose &operator=(const SensorPose &other) {
    x = other.x;
    y = other.y;
    z = other.z;
    pitch = other.pitch;
    roll = other.roll;
    yaw = other.yaw;
  }
};

struct VehiclePartData {
  double x, y;
  VehiclePartType part_type;
};

struct ParticleVehicleData {
  double x, y, theta; // in global coordinates, needs double precision
  std::vector<VehiclePartData> parts;
};
} // namespace part_based_tracking
} // namespace tracking
