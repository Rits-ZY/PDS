#pragma once
#include <cstdlib>
#include "SamplingMethod.h"


///////////////////////////////////////////////////////////////////////////////////////
// 					Dual-Shell Poisson Disk Sampling								//
//				An improved down-sampling method based on PDS   					//
//       	Improve the uniformity of Point Cloud with two concentric circle		//
//////////////////////////////////////////////////////////////////////////////////////

class cmp {
public:
	bool operator()(const std::pair<int,int>& a,const std::pair<int, int>& b) {
		return a.second > b.second;
	}
};

class DPDS :
	public SamplingMethod
{
public:
	void exec_method(pcl::PointCloud<pcl::PointXYZL>::Ptr cloud, pcl::KdTreeFLANN<pcl::PointXYZL> kd_tree, std::vector<std::size_t>& sampling_result) {
		
		std::cout 
			<< "////////////////////////////////////////////////////////////////////" << std::endl
			<< std::endl
			<< "Execute Dual-Shell Poisson Disk Sampling methed." << std::endl
			<< std::endl;


		float radius_r, radius_R;
		
		do {
			std::cout << "Set the minor sample radius r(r>0):";
			std::cin >> radius_r;
		} while (radius_r <= 0);

		do {
			std::cout << "Set the major sample radius R(R>r):";
			std::cin >> radius_R;
		} while (radius_R <= radius_r);

		std::cout << "R = " << static_cast<float>(radius_R / radius_r) << "*r" << std::endl;
		std::cout << "Sampling radius (r,R):" << "(" << radius_r << "," << radius_R << ")" << std::endl;
		std::cout << "DPDS started" << std::endl;
		std::clock_t time_start = clock();

		//initialize the indices of points
		int size = cloud->width*cloud->height;
		std::vector<int> index(size);
		for (int i = 0; i < size; i++) {
			index[i] = i;
		}
		//std::random_shuffle(index.begin(), index.end());

		//initialize the socre of points
		std::vector<int> score(size);	
		std::vector<int> score_index;
		int max_score = -1;
		for (int i = 0; i < size; i++) {
			score[i] = 0;
		}
			
		//dpds method
		std::vector<int> neighbors;
		std::vector<float> distances;
		int k = std::rand() % size;
		for (int i = 0; i < size; i++) {
			pcl::PointXYZL p = cloud->points[k];
			if (p.label == 1) continue;

			p.label = 1;
			sampling_result.push_back(k);

			if (kd_tree.radiusSearch(p, radius_R, neighbors, distances) > 0) {
				for (std::size_t i = 0; i < neighbors.size(); i++) {
					if (distances[i] < radius_r*radius_r) {
						cloud->points[neighbors[i]].label = 1;
						score[neighbors[i]] = -1;
					}
					else {
						if (cloud->points[neighbors[i]].label != 1) {
							score[neighbors[i]] += 1;
						}
					}
				}
				neighbors.clear();
				distances.clear();
			}

			max_score = -1;
			for (int i = 0; i < size; i++) {
				if (cloud->points[i].label != 1 && score[i] >= max_score) {
					max_score = score[i];
				}
			}

			if (max_score == -1) { break; }

			score_index.clear();
			for (int i = 0; i < size; i++) {
				if (cloud->points[i].label != 1 && score[i] == max_score) {
					score_index.push_back(i);
				}
			}

			if (score_index.size() == 0) {
				std::cerr << "Error:max_size=0  sampling finished" << std::endl;
				break;
			}

			std::srand((int)std::time(0));
			k = score_index[std::rand() % score_index.size()];

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

