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
	/*Input:
		cloud : the pointer of pcl::PointCloud and point type is pcl::PointXYZL
		kd_tree : kd_tree which contains input loud
		sampling_result : store the indices of sampled points
	*/
	virtual void exec_method(pcl::PointCloud<pcl::PointXYZL>::Ptr cloud,pcl::KdTreeFLANN<pcl::PointXYZL> kd_tree,std::vector<std::size_t>& sampling_result)=0;

};

