## Next Best View via Topological Features

This repository holds software, data, and CAD files for the Robotic Vision Lab's research into the next best view using computational topology.

### Getting Started

These instructions will get you a copy of the project up and running on your local machine.

### Prerequisites

* PCL
* Boost-Filesystem
* Intel RealSense SDK

### Installation

Download the [Dynamixel SDK](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/master/c%2B%2B/include/dynamixel_sdk), read the included license, and copy all files in the SDK's include folder to the include folder for this software.

From the primary folder run

```
cmake .
```

And compile with

```
make
```

### Programs

#### Armcontrol

Armcontrol is meant to be used with the corresponding [2-DOF manipulator] to obtain 3D point clouds from an Intel RealSense D415 in various positions. The filenames are encoded with the motor positions as {yaw}_{pitch}.pcd, ex: 1024_2386.pcd. Files are saved in the current folder. The default configuration is to randomly sample the action space of the motors, however functions in the code are abstracted to allow changes to be easily made if another method is preferred.

Run the program as 

```
./armcontrol
```

#### Transform, Union, Registration (TUR)

TUR is used to combine multiple 3D point clouds from the Armcontrol software. It does the following:

* Translates each point cloud to the world coordinate frame based on the coordinates in the filename formatted as {yaw}_{pitch}.pcd, ex: 1024_2386.pcd

* Unions each of the point clouds together into a single file

* Uses iterative closest point (ICP) registration to match the point clouds

Run the program as

```
./transform_and_union out_file.pcd {yaw1}_{pitch1}.pcd [{yaw2}_{pitch2}.pcd] [{yaw3}_{pitch3}.pcd] ...
```

### Authors

* Chris Collander &lt;christopher.collander@mavs.uta.edu&gt;

### License and Copyright

This project is licensed under GNU GPLv3 - see the [LICENSE.md](LICENSE.md) file for details

Copyright 2019-2020, Chris Collander
