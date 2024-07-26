/**
	*brief: Colored point cloud define
	*author: copy form velodyne ros driver
	*/
#ifndef VELODYNE_H
#define VELODYNE_H

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

namespace pcl
{
/** Euclidean Velodyne coordinate, including intensity and ring number. */
struct PointXYZIR
{
	PCL_ADD_POINT4D;								// quad-word XYZ
	float intensity;								///< laser intensity reading
	uint16_t ring;									///< laser ring number
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
} EIGEN_ALIGN16;

}; // namespace pcl

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZIR,(float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring))
typedef pcl::PointXYZIR VPoint; //added by Zhang Jianlin
typedef pcl::PointCloud<VPoint> VPointCloud; //added by Zhang Jianlin

#endif
