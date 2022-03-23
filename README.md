# RAPTOR PX4 Motion Capture Data Publisher

This repository contains an application that will run on an onboard computer onboard a drone and publish MAVLINK messages for the local pose to a PX4 flight controller. 

## Installation

This repository will be included in all of our other repositories wherever it is needed. However, for development, you may want to have this repository cloned separately. 

You will need the following dependencies installed on your system:

- [MAVSDK](https://mavsdk.mavlink.io/main/en/cpp/guide/installation.html)
  
After MAVSDK has been installed, clone this repository using: 

```bash
git clone --recursive https://github.com/raptor-ethz/mocap_px4_publisher
```

## Usage

Then, since this application also taps into the ViCon datastream on the local network, you will need to supply the IP address as command line argument when launching the application. From the root folder, this would look something like this: 

```bash
./build/mocap_px4_apps/mocap_px4_pub_app/mocap_px4_pub [IP address]
```