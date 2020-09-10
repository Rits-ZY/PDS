#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include "PDS/PDS.h"
#include "PDS/DPDS.h"
#include "PDS/SamplingMethodExer.h"


int
main(int argc, char** argv)
{
	std::string inputfile;
	std::string outputfile="out.ply";
/*
	if (argc <= 1 || argc >= 4) {
		std::cerr << "Please set input PLY file exec command line:pclTest3.exe input.ply" << std::endl;
		return (-1);
	}

	if (argc == 2) {
		inputfile = argv[1];
	}

	if (argc == 3) {
		inputfile = argv[1];
		inputfile = argv[2];
	}*/
	
	pcl::PointCloud<pcl::PointXYZL>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZL>);

	if (pcl::io::loadPLYFile<pcl::PointXYZL>("bunny.ply", *cloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file test.ply \n");
		return (-1);
	}
	std::cout << "Loaded "
		<< cloud->width * cloud->height
		<< " data points from "
		<< "bunny.ply"
		<< std::endl;
	
	pcl::KdTreeFLANN<pcl::PointXYZL> kdtree;
	kdtree.setInputCloud(cloud);
	SamplingMethodExer sme;
	PDS pds;
	DPDS dpds;
	sme.setSamplingMethod(dpds);
	std::vector<std::size_t> sampling_result;


	sme.execSamplingMethod(cloud,kdtree.makeShared(), sampling_result);
	
	pcl::PointCloud<pcl::PointXYZ> result_pc;
	result_pc.resize(sampling_result.size());

	for (std::size_t i = 0; i < result_pc.size(); i++) {
		result_pc.points[i].x = cloud->points[sampling_result[i]].x;
		result_pc.points[i].y = cloud->points[sampling_result[i]].y;
		result_pc.points[i].z = cloud->points[sampling_result[i]].z;
	}
	
	
	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	viewer.showCloud(result_pc.makeShared());
	while (!viewer.wasStopped())
	{
	}
	
	
	return (0);
}