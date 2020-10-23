// Transform, Union, Registration (TUR)
// Chris Collander
// christopher.collander@mavs.uta.edu
// Licensed under GNU GPLv3
// Copyright 2019, Chris Collander

#include <iostream>
#include <string>
#include <chrono>
#include <thread>

#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/centroid.h>
#include <pcl/registration/icp.h>

const double pi = 3.14159265358979323846;

using namespace std;

// Convert a motor position index into degrees
// Tested and verified
float index2angle(int index) {
    const float index_min = 0;
    const float index_max = 4095;
    const float angle_min = -pi;
    const float angle_max = pi;
    return (index - index_min) * (angle_max - angle_min) / (index_max - index_min) + angle_min;
}

// Visualize a point cloud and wait for user to close the window
/// Tested and verified
void visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl_cloud) {
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (pcl_cloud, "cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    viewer->addCoordinateSystem (0.1);
    viewer->initCameraParameters ();
    while (!viewer->wasStopped ()) {
        viewer->spinOnce (100);
        this_thread::sleep_for(chrono::milliseconds(100));
    }
}

// From a filename, determine what angles to rotate the object (radians)
// !!! Currently works only with RELATIVE filenames !!!
void extract_transformation_coordinates(string filename, float& x_rotate, float& y_rotate, float& z_rotate) {
	// Parse filename to extract x and y
	int underscore = filename.find('_');
	int period = filename.find('.');
	int yaw = stoi(filename.substr(0,underscore));
	int pitch = stoi(filename.substr(underscore+1, period-underscore-1));
	//cout<<"Yaw   : "<<yaw<<endl;
	//cout<<"Pitch : "<<pitch<<endl;

	x_rotate = 1*index2angle(pitch);
	y_rotate = -1*index2angle(yaw);  // Verified
	z_rotate = 0;
	//cout<<"x_rotate : "<<index2angle(pitch)<<endl;
	//cout<<"y_rotate : "<<-1*index2angle(yaw)<<endl;
}

// Given a filename, load a PCD file, perform the rotation, and return it
pcl::PointCloud<pcl::PointXYZ>::Ptr loadAndRotate(string filename) {
	pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr centered_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>);

	if (pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *input_cloud) == -1) {
	    cerr<<"Couldn't read file "<<filename<<endl;
	    return input_cloud;
  	}

  	// Extract the centroid and center the pcl
  	Eigen::Vector4f centroid;
  	Eigen::Affine3f transformer1;
  	compute3DCentroid(*input_cloud, centroid);
  	// cout<<"Centroid: "<<centroid[0]<<","<<centroid[1]<<","<<centroid[2]<<endl;
  	transformer1 = Eigen::Affine3f::Identity();
  	transformer1.translate(Eigen::Vector3f(-centroid[0],-centroid[1],0));
  	transformPointCloud(*input_cloud, *centered_cloud, transformer1);

  	// Extract transformation coordinates
  	float x_rotate = 0;
  	float y_rotate = 0;
  	float z_rotate = 0;
  	extract_transformation_coordinates(filename, x_rotate, y_rotate, z_rotate);

  	// Transform Cloud
  	Eigen::Affine3f transformer;
  	transformer = Eigen::Affine3f::Identity();
  	transformer.rotate(Eigen::AngleAxisf(y_rotate, Eigen::Vector3f::UnitY()));
        transformer.rotate(Eigen::AngleAxisf(x_rotate, Eigen::Vector3f::UnitX()));
  	transformPointCloud(*centered_cloud, *transformed_cloud, transformer);

  	// Visualize the point cloud after Z and Y (yaw)
  	//visualize(transformed_cloud);

  	// Return our transformed cloud
  	return transformed_cloud;
}

// Union two point clouds together
pcl::PointCloud<pcl::PointXYZ>::Ptr add_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2) {
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(cloud2);
	icp.setInputTarget(cloud1);
	pcl::PointCloud<pcl::PointXYZ>::Ptr Final(new pcl::PointCloud<pcl::PointXYZ>);
	icp.align(*Final);
	if(icp.hasConverged()) {
		// cout <<"ICP Converged... "<<icp.getFitnessScore()<<endl;
		*cloud1 += *Final;
		return cloud1;
	}
	else {
		// cout <<"ICP Failed... "<<endl;
		*cloud1 += *cloud2;
		return cloud1;
	}
}

int main(int argc, char** argv) {

	if(argc<3) {
		cerr<<"Incorrect argument count"<<endl;
		cerr<<"Example : ";
		cerr<<"./transform_and_union out.pcd file1.pcd [file2.pcd] [file3.pcd] ..."<<endl;
		return -1;
	}

	int numfiles = argc - 2;

	string output_file = argv[1];
	string file1 = argv[2];

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = loadAndRotate(file1);

	// For every cloud after the initial, load, rotate, add
	for(int lcv=3;lcv<numfiles+2;lcv++) {
		pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud = loadAndRotate(argv[lcv]);
		cloud = add_clouds(cloud, temp_cloud);
	}

	// Save our new file
	// visualize(cloud);
	pcl::io::savePCDFileASCII (output_file, *cloud);

	return 0;
}
