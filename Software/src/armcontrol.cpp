// Armcontrol
// Chris Collander
// christopher.collander@mavs.uta.edu
// Licensed under GNU GPLv3
// Copyright 2019, Chris Collander

#include <iostream>
#include <string>
#include <sstream>
#include <cmath>
#include <chrono>
#include <thread>
#include <random>
#include <unistd.h>

#include "dynamixel_sdk.h"

#include <librealsense2/rs.hpp>

#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

// Set to true to visualize the point cloud after every frame
#define VISUALIZE false

// Dynamixel communication parameters
#define PROTOCOL_VERSION 2.0
#define YAW_ID 2
#define PITCH_ID 1
#define BAUDRATE 57600
#define DEVICENAME "/dev/ttyUSB0"

// Dynamixel command parameters
#define TORQUE_ENABLE 1
#define TORQUE_DISABLE 0
#define LED_ENABLE 1
#define LED_DISABLE 0
#define DXL_MOVING_STATUS_THRESHOLD 20
#define DXL_MOVING_STATUS_TIME_THRESHOLD_SECONDS 12
#define ESC_ASCII_VALUE 0x1b

// Predefined positional motor values
#define YAW_MINIMUM_POSITION_VALUE 0
#define YAW_MAXIMUM_POSITION_VALUE 4095
#define PITCH_MINIMUM_POSITION_VALUE 1024
#define PITCH_MAXIMUM_POSITION_VALUE 3072
#define CENTER_POSITION_VALUE 2048

// Register addresses for Dynamixel commands
#define ADDR_GOAL_POSITION 116
#define ADDR_MAX_POSIT_LIM 48
#define ADDR_MIN_POSIT_LIM 52
#define ADDR_TORQUE_ENABLE 64
#define ADDR_PROFILE_ACCEL 108
#define ADDR_PROFILE_VELOC 112
#define ADDR_PRESENT_POSIT 132
#define ADDR_OPERATING_MOD 11
#define ADDR_HOMING_OFFSET 20
#define ADDR_DRIVE_MODE 10
#define ADDR_LED 65
#define ADDR_HARDWARE_ERROR 70

// Transform and Filter dimensions
#define OBJECT_RADIUS 0.1
#define X_ORIGIN 0.011
#define Y_ORIGIN 0.0
#define Z_ORIGIN 0.58

using namespace std;
using namespace rs2;

// Global variables! I know, I hate them too....
dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
uint8_t dxl_error = 0;
int32_t yaw_present_position = 0;
int32_t pitch_present_position = 0;

// Randomness, Uniform, Unbiased
random_device seeder;
mt19937 rng(seeder());
uniform_int_distribution<int> yaw_gen(YAW_MINIMUM_POSITION_VALUE, YAW_MAXIMUM_POSITION_VALUE);
uniform_int_distribution<int> pitch_gen(PITCH_MINIMUM_POSITION_VALUE, PITCH_MAXIMUM_POSITION_VALUE);

// Global PCL Variables for reduced calculations
pcl::CropBox<pcl::PointXYZ> boxFilter;
Eigen::Affine3f transformer;

// Easy function to see if a file already exists
bool fileexists(const string &filename) {
    return access(filename.c_str(), 0)==0;
}

// Determine if yaw and pitch are within minimum and maximum values
bool withinBounds(int yaw, int pitch) {
    bool test1 = yaw >= YAW_MINIMUM_POSITION_VALUE;
    bool test2 = yaw <= YAW_MAXIMUM_POSITION_VALUE;
    bool test3 = pitch >= PITCH_MINIMUM_POSITION_VALUE;
    bool test4 = pitch <= PITCH_MAXIMUM_POSITION_VALUE;
    return test1 && test2 && test3 && test4;
}

// Convert an angle in degrees into a motor position index
int angle2index(float angle) {
    const float index_min = 1024;
    const float index_max = 3072;
    const float angle_min = -180;
    const float angle_max = 180;
    const float index = (angle - angle_min) * (index_max - index_min) / (angle_max - angle_min) + index_min;
    return int(index+0.5);
}

// Convert a motor position index into degrees
float index2angle(int index) {
    const float index_min = 1024;
    const float index_max = 3072;
    const float angle_min = -180;
    const float angle_max = 180;
    return (index - index_min) * (angle_max - angle_min) / (index_max - index_min) + angle_min;
}

// Move the manipulator to a specific yaw and pitch (by positional index).
// Will stop when either position delta is less than DXL_MOVING_STATUS_THRESHOLD
// Or (assuming it can't get close enough to the new position)
// Will time out after DXL_MOVING_STATUS_TIME_THRESHOLD_SECONDS
// Either way, will save actual positions to global variables ???_present_position
void moveTo(int yaw, int pitch) {
    cout<<"Move: {"<<yaw<<","<<pitch<<"}"<<flush;

    packetHandler->write4ByteTxRx(portHandler, YAW_ID, ADDR_GOAL_POSITION, yaw, &dxl_error);
    packetHandler->write4ByteTxRx(portHandler, PITCH_ID, ADDR_GOAL_POSITION, pitch, &dxl_error);
    auto start = chrono::steady_clock::now();
    do { // Wait until both motors are at the correct locations (based on DXL_MOVING_STATUS_THRESHOLD). Possible timeout
      packetHandler->read4ByteTxRx(portHandler, YAW_ID, ADDR_PRESENT_POSIT, (uint32_t*)&yaw_present_position, &dxl_error);
      packetHandler->read4ByteTxRx(portHandler, PITCH_ID, ADDR_PRESENT_POSIT, (uint32_t*)&pitch_present_position, &dxl_error);
    } while(chrono::duration_cast<chrono::seconds>(chrono::steady_clock::now() - start).count()<DXL_MOVING_STATUS_TIME_THRESHOLD_SECONDS && ((abs(yaw-yaw_present_position)>DXL_MOVING_STATUS_THRESHOLD) || (abs(pitch-pitch_present_position)>DXL_MOVING_STATUS_THRESHOLD)));
    cout<<" ---> {"<<yaw_present_position<<","<<pitch_present_position<<"}"<<flush;
}

// Check the hardware error status registers of the motors for any problems
void checkHardwareStatus() {
    int yaw_status = -1;
    int pitch_status = -1;
    packetHandler->read4ByteTxRx(portHandler, YAW_ID, ADDR_HARDWARE_ERROR, (uint32_t*)&yaw_status, &dxl_error);
    packetHandler->read4ByteTxRx(portHandler, PITCH_ID, ADDR_HARDWARE_ERROR, (uint32_t*)&pitch_status, &dxl_error);
    yaw_status   &= 0xF;
    pitch_status &= 0xF;
    cout<<"Hardware Status:\tYAW="<<yaw_status<<"\tPITCH="<<pitch_status<<endl;
}

// Will convert realsense points to a PCL ptr
pcl::PointCloud<pcl::PointXYZ>::Ptr points_to_pcl(const points& points) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    auto sp = points.get_profile().as<video_stream_profile>();
    cloud->width = sp.width();
    cloud->height = sp.height();
    cloud->is_dense = false;
    cloud->points.resize(points.size());
    auto ptr = points.get_vertices();
    for (auto& p : cloud->points) {
        p.x = ptr->x;
        p.y = ptr->y;
        p.z = ptr->z;
        ptr++;
    }
    return cloud;
}

// Will allow visualization of a cloud, returning from the function when the window is closed
void visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr &pcl_cloud) {
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (pcl_cloud, "cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    while (!viewer->wasStopped ()) {
        viewer->spinOnce (100);
        this_thread::sleep_for(chrono::milliseconds(100));
    }
}

// Obtain a point cloud from the realsense
// Handles translation and rotation to center the object,
// And filtering to remove things that are not the object.
// Saves to a file based on the name parameter
void getPointCloudWithName(pipeline &pipe, points &pts, pointcloud &pc, string name) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ> ());

    // Get our point cloud
    auto frames = pipe.wait_for_frames();
    auto color = frames.get_color_frame();
    auto depth = frames.get_depth_frame();
    pts = pc.calculate(depth);
    pc.map_to(color);
    input_cloud = points_to_pcl(pts);

    // Translate and filter
    transformPointCloud(*input_cloud, *transformed_cloud, transformer);
    boxFilter.setInputCloud(transformed_cloud);
    boxFilter.filter(*filtered_cloud);

    // Save and visualize
    pcl::io::savePCDFileASCII(name.c_str(), *filtered_cloud);
    if(VISUALIZE)
        visualize(filtered_cloud);
}

// Reboot the motors and establish correct parameters
void reboot() {
    cout<<"Pre-Reboot"<<endl;
    checkHardwareStatus();

    // Reboot both motors
    packetHandler->reboot(portHandler, YAW_ID, &dxl_error);
    packetHandler->reboot(portHandler, PITCH_ID, &dxl_error);

    // Wait 5 seconds for reboot
    this_thread::sleep_until(chrono::system_clock::now() + chrono::seconds(5));
    cout<<"Post-Reboot"<<endl;
    checkHardwareStatus();

    // Set positional offsets
    packetHandler->write4ByteTxRx(portHandler, YAW_ID, ADDR_HOMING_OFFSET, 0, &dxl_error);
    packetHandler->write4ByteTxRx(portHandler, PITCH_ID, ADDR_HOMING_OFFSET, 0, &dxl_error);

    // Set drive and operating modes
    packetHandler->write1ByteTxRx(portHandler, YAW_ID, ADDR_DRIVE_MODE, 0, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, PITCH_ID, ADDR_DRIVE_MODE, 0, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, YAW_ID, ADDR_OPERATING_MOD, 3, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, PITCH_ID, ADDR_OPERATING_MOD, 3, &dxl_error);

    // Set Yaw Profile
    packetHandler->write4ByteTxRx(portHandler, YAW_ID, ADDR_PROFILE_ACCEL, 25, &dxl_error);
    packetHandler->write4ByteTxRx(portHandler, YAW_ID, ADDR_PROFILE_VELOC, 25, &dxl_error);

    // Set Pitch Profile
    packetHandler->write4ByteTxRx(portHandler, PITCH_ID, ADDR_PROFILE_ACCEL, 25, &dxl_error);
    packetHandler->write4ByteTxRx(portHandler, PITCH_ID, ADDR_PROFILE_VELOC, 25, &dxl_error);

    // Set Minimum and Maximum Yaw Values
    packetHandler->write4ByteTxRx(portHandler, YAW_ID, ADDR_MAX_POSIT_LIM, YAW_MAXIMUM_POSITION_VALUE, &dxl_error);
    packetHandler->write4ByteTxRx(portHandler, YAW_ID, ADDR_MIN_POSIT_LIM, YAW_MINIMUM_POSITION_VALUE, &dxl_error);

    // Set Minimum and Maximum Pitch Values
    packetHandler->write4ByteTxRx(portHandler, PITCH_ID, ADDR_MAX_POSIT_LIM, PITCH_MAXIMUM_POSITION_VALUE, &dxl_error);
    packetHandler->write4ByteTxRx(portHandler, PITCH_ID, ADDR_MIN_POSIT_LIM, PITCH_MINIMUM_POSITION_VALUE, &dxl_error);

    // Enable Torque
    packetHandler->write1ByteTxRx(portHandler, YAW_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    packetHandler->write1ByteTxRx(portHandler, PITCH_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);

    // Center both motors
    moveTo(CENTER_POSITION_VALUE, CENTER_POSITION_VALUE);
    cout<<endl;
}

// bundles together the getPointCloudWithName and moveTo functions for abstraction
// Given a yaw and pitch (indexes) go to that position and obtain, transform, filter, and save the pointcluod.
// Will NOT overwrite or even try to obtain PC for a position that it already has
void moveAndSave(int yaw, int pitch, pipeline &pipe, points &pts, pointcloud &pc) {
    stringstream ss;
    moveTo(yaw, pitch);
    ss << "./" << yaw_present_position << "_" << pitch_present_position <<".pcd";
    if(!fileexists(ss.str())) {
        cout<<"\tFile: NEW"<<endl;
        getPointCloudWithName(pipe, pts, pc, ss.str());
    } else if(!withinBounds(yaw_present_position,pitch_present_position)) {
        cout<<"\tERROR: OUT OF BOUNDS! Rebooting motors..."<<endl;
        reboot();
    } else
        cout<<"\tFile: EXISTS"<<endl;
}

// Move and save a random point cloud within valid values (uniform, unbiased, discrete)
void getRandomPC(pipeline &pipe, points &pts, pointcloud &pc) {
    int yaw = yaw_gen(rng);
    int pitch = pitch_gen(rng);
    moveAndSave(yaw, pitch, pipe, pts, pc);
}

int main(int argc, char** argv) {
    // Initialize Camera
    pointcloud pc;
    points pts;
    pipeline pipe;
    pipe.start();

    // Set up our PCL filter and transform (Doing this here, so we only need to establish this once.)
    transformer = Eigen::Affine3f::Identity();
    transformer.rotate(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitY()));
    transformer.translation() << X_ORIGIN, Y_ORIGIN, Z_ORIGIN;
    boxFilter.setMin(Eigen::Vector4f(-OBJECT_RADIUS, -OBJECT_RADIUS, -OBJECT_RADIUS, 1.0));
    boxFilter.setMax(Eigen::Vector4f(OBJECT_RADIUS, OBJECT_RADIUS, OBJECT_RADIUS, 1.0));

    // Open serial port with correct baudrate
    portHandler->openPort();
    portHandler->setBaudRate(BAUDRATE);

    // Reboot the motors and establish correct parameters
    reboot();
    cout<<"Centered"<<endl;
    this_thread::sleep_until(chrono::system_clock::now() + chrono::seconds(5));

    // Get data!
    cout<<"Starting Data Collection"<<endl;

    // Obtain random point clouds
    while(true)
        getRandomPC(pipe,pts,pc);
    return 0;

    /*
    // Uncomment this block if you want 20 yaw and 10 pitch values
    int actions_list[][2] = {
        {0,1024},   {0,1252},   {0,1479},   {0,1707},   {0,1934},
        {0,2162},   {0,2389},   {0,2617},   {0,2844},   {0,3072},
        {216,1024}, {216,1252}, {216,1479}, {216,1707}, {216,1934},
        {216,2162}, {216,2389}, {216,2617}, {216,2844}, {216,3072},
        {431,1024}, {431,1252}, {431,1479}, {431,1707}, {431,1934},
        {431,2162}, {431,2389}, {431,2617}, {431,2844}, {431,3072},
        {647,1024}, {647,1252}, {647,1479}, {647,1707}, {647,1934},
        {647,2162}, {647,2389}, {647,2617}, {647,2844}, {647,3072},
        {862,1024}, {862,1252}, {862,1479}, {862,1707}, {862,1934},
        {862,2162}, {862,2389}, {862,2617}, {862,2844}, {862,3072},
        {1078,1024},{1078,1252},{1078,1479},{1078,1707},{1078,1934},
        {1078,2162},{1078,2389},{1078,2617},{1078,2844},{1078,3072},
        {1293,1024},{1293,1252},{1293,1479},{1293,1707},{1293,1934},
        {1293,2162},{1293,2389},{1293,2617},{1293,2844},{1293,3072},
        {1509,1024},{1509,1252},{1509,1479},{1509,1707},{1509,1934},
        {1509,2162},{1509,2389},{1509,2617},{1509,2844},{1509,3072},
        {1725,1024},{1725,1252},{1725,1479},{1725,1707},{1725,1934},
        {1725,2162},{1725,2389},{1725,2617},{1725,2844},{1725,3072},
        {1940,1024},{1940,1252},{1940,1479},{1940,1707},{1940,1934},
        {1940,2162},{1940,2389},{1940,2617},{1940,2844},{1940,3072},
        {2156,1024},{2156,1252},{2156,1479},{2156,1707},{2156,1934},
        {2156,2162},{2156,2389},{2156,2617},{2156,2844},{2156,3072},
        {2371,1024},{2371,1252},{2371,1479},{2371,1707},{2371,1934},
        {2371,2162},{2371,2389},{2371,2617},{2371,2844},{2371,3072},
        {2587,1024},{2587,1252},{2587,1479},{2587,1707},{2587,1934},
        {2587,2162},{2587,2389},{2587,2617},{2587,2844},{2587,3072},
        {2803,1024},{2803,1252},{2803,1479},{2803,1707},{2803,1934},
        {2803,2162},{2803,2389},{2803,2617},{2803,2844},{2803,3072},
        {3018,1024},{3018,1252},{3018,1479},{3018,1707},{3018,1934},
        {3018,2162},{3018,2389},{3018,2617},{3018,2844},{3018,3072},
        {3234,1024},{3234,1252},{3234,1479},{3234,1707},{3234,1934},
        {3234,2162},{3234,2389},{3234,2617},{3234,2844},{3234,3072},
        {3449,1024},{3449,1252},{3449,1479},{3449,1707},{3449,1934},
        {3449,2162},{3449,2389},{3449,2617},{3449,2844},{3449,3072},
        {3665,1024},{3665,1252},{3665,1479},{3665,1707},{3665,1934},
        {3665,2162},{3665,2389},{3665,2617},{3665,2844},{3665,3072},
        {3880,1024},{3880,1252},{3880,1479},{3880,1707},{3880,1934},
        {3880,2162},{3880,2389},{3880,2617},{3880,2844},{3880,3072},
        {4096,1024},{4096,1252},{4096,1479},{4096,1707},{4096,1934},
        {4096,2162},{4096,2389},{4096,2617},{4096,2844},{4096,3072}};
    int yaw, pitch;
    for(int i=0;i<200;i++) {
        yaw = actions_list[i][0];
        pitch = actions_list[i][1];
        moveAndSave(yaw, pitch, pipe, pts, pc);
    }
    */
}
