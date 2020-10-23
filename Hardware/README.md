# NBV_Hardware

This repository holds an open design for a 2DoF manipulator for 3D point cloud scanning

![PCLManipulator](/arm.png)

This design was created for research in 3D Point Cloud Analysis for Next Best View. Fusion 360 was used to create a parameterized model capable of changes to the design. This manipulator can be fully constructed of 3D printed pieces, lasercut wood, a carbon fiber tube (found at a local hobby shop), 2 Dynamixel XL430 motors, a Realsense D415 camera, and simple mechanical screws.

## Printing/Cutting

For 3D Printing, a Prusa i3 mk3s was used for the pieces, with some pieces being printed in PETG and others in PLA. As relatively little stress will be placed on the pieces, the actual plastic doesn't matter. The only piece that should be printed with high detail is the carbon fiber mount. Ensure the carbon fiber rod does not move in the mount and the mount is secure on the motor with no wiggle room. As the standoffs needed to be specific heights to allow the camera to be facing directly at the origin, these were 3D printed in high quality. If you can find another way to securely ensure the height of each layer, feel free to go that route. This just worked well for us. We even 3D printed the nuts for the standoffs, but we do admit this is unnecessary considering we could have just made a trip to the hardware store. We've included the STL file for them anyways, in case anybody wants to take the same route.

For laser cutting, a Full Spectrum Laser 20x12 Hobby lasercutter was used with RetinaEngrave and CorelDraw. The pieces were all constructed to fit within the bounds of the lasercutter. 3mm baltic birch was used, but if the parameter in Fusion360 for material thickness is changed, there is no reason that a different size of material wouldn't work. For the manipulator itself, simple wood glue and clamps were used for adhesion.

### Building

To build the manipulator you need to lasercut the following files in 3mm wood
* 2x BasePlateVector.dxf
* 2x BaseSupportVector.dxf
* 2x PitchPlateVector.dxf
* 2x PitchSupportVector.dxf
* 2x YawPlateVector.dxf
* 2x YawSupportVector.dxf
* 2x Platform1Vector.dxf
* 2x Platform2Vector.dxf

The following pieces should be 3D printed
* 1x CameraSupport.stl
* 1x CarbonFiberMotorMount.stl
* 14x Standoff.stl
* 7x Nut.stl (optional)

The following pieces should be purchased
* 1x Realsense D4xx Camera
* 2x Dynamixel XL430 motors (and any necessary cables, boards, power supplies, and USB adapters to get these working for your system)
* 1x Carbon Fiber 5mm-OD square tube: [here](https://www.hobbytown.com/midwest-carbon-fiber-square-tube-.196-od-x-.118-id-x-40-mid5854/p28823)
* Any screws necessary for the build, a mixture of M3 and M5 metal screws.

## Authors

* **Chris Collander**

## License and Copyright
This project is licensed under GNU GPLv3 - see the [LICENSE.md](LICENSE.md) file for details

Copyright 2019, Chris Collander
