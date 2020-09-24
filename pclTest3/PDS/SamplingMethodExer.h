#pragma once
#include "PDS/SamplingMethod.h"
class SamplingMethodExer
{
public:
	void setSamplingMethod(SamplingMethod* method) {
		this->method = method;
	}

	void execSamplingMethod(pcl::PointCloud<pcl::PointXYZL>::Ptr cloud, pcl::KdTreeFLANN<pcl::PointXYZL>::Ptr kd_tree, std::vector<std::size_t>& re) {
		method->exec_method(cloud,*kd_tree,re);
	}

private:
	SamplingMethod* method;
};
