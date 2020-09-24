#pragma once
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <ctime>
#include <vector>
#include <algorithm>


///////////////////////////////////////////////////////////////////////////////////////
// 						Strategy Design Pattern										//
//				This is an interface for Sampling method							//
//////////////////////////////////////////////////////////////////////////////////////

class SamplingMethod
{
public:
	/*  \brief Sampling method for point cloud. 
		\param[in]  cloud : the pointer of pcl::PointCloud and point type is pcl::PointXYZL
		\param[in]  kd_tree : kd_tree which contains input loud
		\param[out] sampling_result : store the indices of sampled points
	*/
	virtual void exec_method(pcl::PointCloud<pcl::PointXYZL>::Ptr cloud, pcl::KdTreeFLANN<pcl::PointXYZL> kd_tree, std::vector<std::size_t>& sampling_result) {
		std::cerr << "This method is only for PDS or DPDS" << std::endl;
		return;
	};


	/*  \brief Curvature dependented method for point cloud. The points are sorted by features.
		\param[in]  cloud : the pointer of pcl::PointCloud and point type is pcl::PointXYZL. The points is sorted by features.
		\param[in]  kd_tree : kd_tree which contains input loud
		\param[in]  features : points features
		\param[out] sampling_result : store the indices of sampled points
	*/
	virtual void exec_method(pcl::PointCloud<pcl::PointXYZL>::Ptr cloud, pcl::KdTreeFLANN<pcl::PointXYZL> kd_tree, std::vector<float>& features, std::vector<std::size_t>& sampling_results) {
		std::cerr << "This method is only for curvatrure dependent PDS" << std::endl;
		std::cerr << "Need points' features!" << std::endl;
		return;
	};

};

