#pragma once
#include "PDS/SamplingMethod.h"

//////////////////////////////////////////////////////////////////////////////////////
//				  Curvature Dependent Poisson Disk Sampling							//
//				A curvature dependented down-sampling method						//
//			  Improve the feature points ratio of sampled points					//
//////////////////////////////////////////////////////////////////////////////////////

class CurvatureDependentPDS :
	public SamplingMethod
{
public:

	void exec_method(pcl::PointCloud<pcl::PointXYZL>::Ptr cloud, pcl::KdTreeFLANN<pcl::PointXYZL> kd_tree, std::vector<float>& features, std::vector<std::size_t>& sampling_result) {

		std::cout
			<< "////////////////////////////////////////////////////////////////////" << std::endl
			<< std::endl
			<< "Execute Poisson Disk Sampling methed." << std::endl
			<< std::endl;

		float radius,threshold;
		do {
			std::cout << "Set the sample radius r(r>0):";
			std::cin >> radius;
		} while (radius <= 0);

		do {
			std::cout << "Set the threshold t of feature points (t>0):";
			std::cin >> threshold;
		} while (threshold <= 0 ||threshold >= 1);

		std::cout << "Sampling radius r:" << radius << std::endl;
		std::cout << "Threshold of feature points t:" << threshold << std::endl;
		std::cout << "Curvature Dependent PDS started" << std::endl;
		std::clock_t time_start = clock();

		//shuffle the indices of points
		int size = cloud->width*cloud->height;
		std::vector<int> index(size);
		for (int i = 0; i < size; i++) {
			index[i] = i;
		}

		int separation = 0;
		for (int i = 0; i < features.size(); i++) {
			if (features[i] < threshold) {
				separation = i;
			}
		}

		std::random_shuffle(index.begin(), index.begin()+separation);
		std::random_shuffle(index.begin()+separation, index.end());

		//random PDS method 
		std::vector<int> neighbors;
		std::vector<float> distances;
		for (std::size_t i = 0; i < size; i++) {
			int k = index[i];
			pcl::PointXYZL p = cloud->points[k];
			if (p.label == 1) continue;

			p.label = 1;
			sampling_result.push_back(k);

			if (kd_tree.radiusSearch(p, radius, neighbors, distances) > 0) {
				for (std::size_t i = 0; i < neighbors.size(); i++) {
					cloud->points[neighbors[i]].label = 1;
				}
				neighbors.clear();
				distances.clear();
			}

			if (!(i % 10000)) {
				std::cout << "*** i = " << i << std::endl;
			}

		}

		clock_t time_end = clock();
		std::cout << "Sampling finished." << std::endl;
		std::cout << "Time use:" << 1000 * (time_end - time_start) / (double)CLOCKS_PER_SEC << "ms" << std::endl;
		std::cout << "Data points number:" << size << std::endl;
		std::cout << "Sampled points number:" << sampling_result.size() << std::endl;
	}
};

