# vortex-m3-sonar-driver
A driver for the Kongsberg M3 Sonar

## Specifications
Requires an onboard CPU running Ubunto 22.04 and a CPU running Windows for the M3 API.

1. Start the M3 API on Windows, and connect to the sonar head.
2. Edit the parameter file to match the Windows API's ip address and port (defaults to 20001 / 21001). Also choose what ROS2 topic to publish the pointcloud to. The callback_time is given in milliseconds, and decides how often the software will try to get a new image from the API.
3. Run the M3Publisher file on Ubuntu 22.04 using ROS2.

