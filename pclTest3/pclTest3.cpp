#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include "PDS/PDS.h"
#include "PDS/DPDS.h"
#include "PDS/CurvatureDependentPDS.h"
#include "PDS/ImprovedDPDS.h"
#include "PDS/SamplingMethodExer.h"

void
printUsage(const char* progName)
{
	std::cout << "\n\nUsage: " << progName << " [options]\n\n"
		<< "Options:\n"
		<< "-------------------------------------------\n"
		<< "-h           this help\n"
		<< "-pds         Poisson Disk Sampling\n"
		<< "-dpds        Dual-Shell Poisson Disk Sampling\n"
		<< "-idpds       Improved Dual-Shell Poisson Disk Sampling\n"
		<< "-cpds        Curvature Dependent Poisson Disk Sampling\n"
		<< "\n\n";
}


int
main(int argc, char** argv)
{
	if (pcl::console::find_argument(argc, argv, "-h") >= 0)
	{
		printUsage(argv[0]);
		return 0;
	}

	bool pds(false), dpds(false), idpds(false), cpds(false);

	if (pcl::console::find_argument(argc, argv, "-pds") >= 0)
	{
		pds = true;
		std::cout << "Simple Poisson Disk Sampling example\n";
	}
	else if (pcl::console::find_argument(argc, argv, "-dpds") >= 0)
	{
		dpds = true;
		std::cout << "Simple Dual-Shell Poisson Disk Sampling example\n";
	}
	else if (pcl::console::find_argument(argc, argv, "-idpds") >= 0)
	{
		idpds = true;
		std::cout << "Simple Improved Dual-Shell Poisson Disk Sampling\n";
	}
	else if (pcl::console::find_argument(argc, argv, "-cpds") >= 0)
	{
		cpds = true;
		std::cout << "Simple Curvature Dependent Poisson Disk Sampling example\n";
	}
	else
	{
		printUsage(argv[0]);
		return 0;
	}
	
	//load point cloud data
	pcl::PointCloud<pcl::PointXYZL>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZL>);

	std::string inputfile;
	do {
		std::cout << "Set input PLY file:";
		std::cin >> inputfile;
	} while (inputfile.length()==0);

	if (pcl::io::loadPLYFile<pcl::PointXYZL>(inputfile, *cloud) == -1) //* load the file
	{
		PCL_ERROR("Couldn't read file bunny.ply \n");
		return (-1);
	}
	std::cout << "Loaded "
		<< cloud->width * cloud->height
		<< " data points from "
		<< "bunny.ply"
		<< std::endl;
	
	pcl::KdTreeFLANN<pcl::PointXYZL> kdtree;
	kdtree.setInputCloud(cloud);
	
	
	std::vector<std::size_t> sampling_result;
	std::vector<float> features;
	SamplingMethodExer sme;

	if (pds)
	{   
		sme.setSamplingMethod(new PDS());
	}
	else if (dpds)
	{
		sme.setSamplingMethod(new DPDS());
	}
	else if (idpds)
	{
		sme.setSamplingMethod(new ImprovedDPDS());
	}
	else if (cpds)
	{
		sme.setSamplingMethod(new DPDS());
	}

	sme.execSamplingMethod(cloud, kdtree.makeShared(), sampling_result);
	pcl::PointCloud<pcl::PointXYZ> result_pc;
	result_pc.resize(sampling_result.size());

	for (std::size_t i = 0; i < result_pc.size(); i++) {
		result_pc.points[i].x = cloud->points[sampling_result[i]].x;
		result_pc.points[i].y = cloud->points[sampling_result[i]].y;
		result_pc.points[i].z = cloud->points[sampling_result[i]].z;
	}
	
	

	//Main loop
	pcl::visualization::CloudViewer viewer("Simple Cloud Viewer");
	viewer.showCloud(result_pc.makeShared());
	while (!viewer.wasStopped())
	{
	}
	
	
	return (0);
}