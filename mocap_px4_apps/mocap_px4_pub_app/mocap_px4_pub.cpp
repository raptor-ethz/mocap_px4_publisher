///////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) OMG Plc 2009.
// All rights reserved.  This software is protected by copyright
// law and international treaties.  No part of this software / document
// may be reproduced or distributed in any form or by any means,
// whether transiently or incidentally to some other use of this software,
// without the written permission of the copyright owner.
//
///////////////////////////////////////////////////////////////////////////////

#include "DataStreamClient.h"

#include <cassert>
#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#ifdef WIN32
#include <conio.h>   // For _kbhit()
#include <cstdio>    // For getchar()
#include <windows.h> // For Sleep()
#else
#include <unistd.h> // For sleep()
#endif              // WIN32

#include <string.h>
#include <time.h>

using namespace ViconDataStreamSDK::CPP;

////////////////////////////////////////////////////////////////////////////////////////////////
// Mavsdk Headers and namespaces

#include <cmath>
#include <future>
#include <thread>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mocap/mocap.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mavlink_passthrough/mavlink_passthrough.h>

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

#include "matrix/math.hpp"
////////////////////////////////////////////////////////////////////////////////////////////////

// mocap quality check

// global variable

unsigned int old_frame_number{0};
unsigned int missed_frames{0};

bool checkMocapData(unsigned int frame_number) {
  // bad data if 0 or unchanged
  if (frame_number == 0 || frame_number == old_frame_number) {
    ++ missed_frames; // increment if bad data
  } else {
    missed_frames = 0; // reset if good data
  }
  // update old frame number
  old_frame_number = frame_number;
  // check for x consecutive missed frames
  if (missed_frames >= 5) {
    std::cout << "Bad motion capture data detected \n";
    return false;
  }
  // return true otherwise
  return true;
}

bool enable_log = true; // set to true to log data

/* RPM, VOLTAGE, CURRENT */
int32_t rpm[4] = {0, 0, 0, 0};   // RPM [rotations per minute]
float voltage[4] = {0, 0, 0, 0}; // Voltage [Volts]
float current[4] = {0, 0, 0, 0}; // Current [Ampere]

// motor speed callback function
std::function<void(const mavlink_message_t &)> MotorSpeedCallback =
    [](const mavlink_message_t &raw_msg)
{
  mavlink_msg_esc_status_get_rpm(&raw_msg, &rpm[0]);
  mavlink_msg_esc_status_get_voltage(&raw_msg, &voltage[0]);
  mavlink_msg_esc_status_get_current(&raw_msg, &current[0]);

  // for (int i = 0; i < 4; i++)
  // {
  //   std::cout << "m " << i + 1 << ": rpm: \t" << rpm[i] << "\t volt: \t" << voltage[i] << "\t amps: \t" << current[i] << std::endl;
  // }
};

namespace
{
  std::string Adapt(const bool i_Value) { return i_Value ? "True" : "False"; }

  // Set time standard
  std::string Adapt(const TimecodeStandard::Enum i_Standard)
  {
    switch (i_Standard)
    {
    default:
    case TimecodeStandard::None:
      return "0";
    case TimecodeStandard::PAL:
      return "1";
    case TimecodeStandard::NTSC:
      return "2";
    case TimecodeStandard::NTSCDrop:
      return "3";
    case TimecodeStandard::Film:
      return "4";
    case TimecodeStandard::NTSCFilm:
      return "5";
    case TimecodeStandard::ATSC:
      return "6";
    }
  }

  // Doubt - Set the direction for i axis
  std::string Adapt(const Direction::Enum i_Direction)
  {
    switch (i_Direction)
    {
    case Direction::Forward:
      return "Forward";
    case Direction::Backward:
      return "Backward";
    case Direction::Left:
      return "Left";
    case Direction::Right:
      return "Right";
    case Direction::Up:
      return "Up";
    case Direction::Down:
      return "Down";
    default:
      return "Unknown";
    }
  }

  // Enable forceplate input device (not relevant)
  std::string Adapt(const DeviceType::Enum i_DeviceType)
  {
    switch (i_DeviceType)
    {
    case DeviceType::ForcePlate:
      return "ForcePlate";
    case DeviceType::Unknown:
    default:
      return "Unknown";
    }
  }

  // Set the unit for state variable
  std::string Adapt(const Unit::Enum i_Unit)
  {
    switch (i_Unit)
    {
    case Unit::Meter:
      return "Meter";
    case Unit::Volt:
      return "Volt";
    case Unit::NewtonMeter:
      return "NewtonMeter";
    case Unit::Newton:
      return "Newton";
    case Unit::Kilogram:
      return "Kilogram";
    case Unit::Second:
      return "Second";
    case Unit::Ampere:
      return "Ampere";
    case Unit::Kelvin:
      return "Kelvin";
    case Unit::Mole:
      return "Mole";
    case Unit::Candela:
      return "Candela";
    case Unit::Radian:
      return "Radian";
    case Unit::Steradian:
      return "Steradian";
    case Unit::MeterSquared:
      return "MeterSquared";
    case Unit::MeterCubed:
      return "MeterCubed";
    case Unit::MeterPerSecond:
      return "MeterPerSecond";
    case Unit::MeterPerSecondSquared:
      return "MeterPerSecondSquared";
    case Unit::RadianPerSecond:
      return "RadianPerSecond";
    case Unit::RadianPerSecondSquared:
      return "RadianPerSecondSquared";
    case Unit::Hertz:
      return "Hertz";
    case Unit::Joule:
      return "Joule";
    case Unit::Watt:
      return "Watt";
    case Unit::Pascal:
      return "Pascal";
    case Unit::Lumen:
      return "Lumen";
    case Unit::Lux:
      return "Lux";
    case Unit::Coulomb:
      return "Coulomb";
    case Unit::Ohm:
      return "Ohm";
    case Unit::Farad:
      return "Farad";
    case Unit::Weber:
      return "Weber";
    case Unit::Tesla:
      return "Tesla";
    case Unit::Henry:
      return "Henry";
    case Unit::Siemens:
      return "Siemens";
    case Unit::Becquerel:
      return "Becquerel";
    case Unit::Gray:
      return "Gray";
    case Unit::Sievert:
      return "Sievert";
    case Unit::Katal:
      return "Katal";

    case Unit::Unknown:
    default:
      return "Unknown";
    }
  }
#ifdef WIN32
  bool Hit()
  {
    bool hit = false;
    while (_kbhit())
    {
      getchar();
      hit = true;
    }
    return hit;
  }
#endif

  class NullBuffer : public std::streambuf
  {
  public:
    int overflow(int c) { return c; }
  };

  NullBuffer Null;
  std::ostream NullStream(&Null);

} // namespace

////////////////////////////////////////////////////////////////////////////

// Mavsdk functions

void usage(const std::string &bin_name)
{
  std::cerr
      << "Usage : " << bin_name << " <connection_url>\n"
      << "Connection URL format should be :\n"
      << " For TCP : tcp://[server_host][:server_port]\n"
      << " For UDP : udp://[bind_host][:bind_port]\n"
      << " For Serial : serial:///path/to/serial/dev[:baudrate]\n"
      << "For example, to connect to the simulator use URL: udp://:14540\n";
}

std::shared_ptr<System> get_system(Mavsdk &mavsdk)
{
  std::cout << "Waiting to discover system...\n";
  auto prom = std::promise<std::shared_ptr<System>>{};
  auto fut = prom.get_future();

  // We wait for new systems to be discovered, once we find one that has an
  // autopilot, we decide to use it.
  mavsdk.subscribe_on_new_system([&mavsdk, &prom]()
                                 {
    auto system = mavsdk.systems().back();

    if (system->has_autopilot()) {
      std::cout << "Discovered autopilot\n";

      // Unsubscribe again as we only want to find one system.
      mavsdk.subscribe_on_new_system(nullptr);
      prom.set_value(system);
    } });

  // We usually receive heartbeats at 1Hz, therefore we should find a
  // system after around 3 seconds max, surely.
  if (fut.wait_for(seconds(3)) == std::future_status::timeout)
  {
    std::cerr << "No autopilot found.\n";
    return {};
  }

  // Get discovered system now.
  return fut.get();
}

//////////////////////////////////////////////////////////////////////////////////////////

int main(int argc, char *argv[])
{

  ////////////////////////////////////////////////////////////////////////////

  // Yaw offset in local frame

  constexpr static float yaw_offset_degrees = 0;
  constexpr static float yaw_offset_radians = yaw_offset_degrees * M_PI / 180;
  ////////////////////////////////////////////////////////////////////////////

  // Mavsdk variables initialize
  if (argc != 2)
  {
    usage("error");
  }
  Mavsdk mavsdk;
  ConnectionResult connection_result =
      // Max Xbee
      // mavsdk.add_any_connection("serial:///dev/tty.usbserial-D309S1F2");
      // Mac usb port
      // mavsdk.add_any_connection("serial:///dev/tty.usbmodem01");
      // Raspberry pi usb port
      // mavsdk.add_any_connection("serial:///dev/ttyACM0");
      // Raspberry pi FTDI
      mavsdk.add_any_connection(argv[1]);

  if (connection_result != ConnectionResult::Success)
  {
    std::cerr << "Connection failed: " << connection_result << '\n';
    return 1;
  }

  if (connection_result != ConnectionResult::Success)
  {
    std::cerr << "Connection failed: " << connection_result << '\n';
    return 1;
  }

  auto system = get_system(mavsdk);
  if (!system)
  {
    return 1;
  }

  // Instantiate plugins.
  auto telemetry = Telemetry{system};
  auto vision = Mocap{system};
  auto mavlink = MavlinkPassthrough{system}; // for mavlink passtrough

  // set telemetry subscription rate POI
  const int TELEMETRY_RATE_HZ = 200;
  bool success = true;
  auto sub_rate_result = telemetry.set_rate_position_velocity_ned(TELEMETRY_RATE_HZ);
  if (sub_rate_result != mavsdk::Telemetry::Result::Success) {
   std::cout << "Failed to set subscription rate for pos_vel: " << (int)sub_rate_result << std::endl;
    success = false;
  }
  sub_rate_result = telemetry.set_rate_attitude(TELEMETRY_RATE_HZ);
  if (sub_rate_result != mavsdk::Telemetry::Result::Success) {
   std::cout << "Failed to set subscription rate for attitude: " << (int)sub_rate_result << std::endl;
    success = false;
  }
  sub_rate_result = telemetry.set_rate_imu(TELEMETRY_RATE_HZ);
  if (sub_rate_result != mavsdk::Telemetry::Result::Success) {
   std::cout << "Failed to set subscription rate for imu: " << (int)sub_rate_result << std::endl;
    success = false;
  }
  if (success) {
    std::cout << "Successfully set subscription rates for pos_vel, attitude, and imu." << std::endl;
  }

  std::cout << "System is ready\n";
  sleep_for(seconds(1));

  /* GET CURRENT DATE + TIME */
  std::time_t ct = std::time(0);
  std::string date = std::string(ctime(&ct));

  /* INITIALIZE LOGGING */
  // std::ofstream myLog;
  // std::string Name = date;
  // myLog.open("log/" + Name + ".csv");
  // std::cout << "Started logging to log/" << Name << ".csv\n";

  /* MAVLINK MOTOR SPEED MESSAGES */
 // if (enable_log == true)
  //{
  //  std::cout << "subscribe to motor speeds" << std::endl;
  //  mavlink.subscribe_message_async(291, MotorSpeedCallback);
 // }
  // // Send mocap command to Mavsdk
  Mocap::VisionPositionEstimate vision_msg;

  vision_msg.time_usec = 0;

  std::vector<float> v = {NAN};
  vision_msg.pose_covariance.covariance_matrix = v;

  //////////////////////////////////////////////////////////////////////////////////////////

  // Program options
  std::vector<std::string> Hosts;
  Hosts.push_back("10.10.10.5");

  if (Hosts.empty())
  {
    Hosts.push_back("localhost:801");
  }

  bool bOptimizeWireless = false;
  bool bQuiet = false;

  std::vector<std::string> HapticOnList(0);
  unsigned int ClientBufferSize = 0;
  std::string AxisMapping = "ZUp";
  std::vector<std::string> FilteredSubjects;
  std::vector<std::string> LocalAdapters;

  std::ostream &OutputStream(bQuiet ? NullStream : std::cout);

  bool First = true;
  std::string HostName;
  for (const auto &rHost : Hosts)
  {
    if (!First)
    {
      HostName += ";";
    }
    HostName += rHost;
    First = false;
  }

  // Make a new client
  ViconDataStreamSDK::CPP::Client DirectClient;

  if (bOptimizeWireless)
  {
    const Output_ConfigureWireless ConfigureWirelessResult =
        DirectClient.ConfigureWireless();

    if (ConfigureWirelessResult.Result != Result::Success)
    {
      std::cout << "Wireless Config: " << ConfigureWirelessResult.Error
                << std::endl;
    }
  }

  std::cout << "Connecting to " << HostName << " ..." << std::flush;
  while (!DirectClient.IsConnected().Connected)
  {

    // Direct connection
    const Output_Connect ConnectResult = DirectClient.Connect(HostName);
    const bool ok = (ConnectResult.Result == Result::Success);

    if (!ok)
    {
      std::cout << "Warning - connect failed... ";
      switch (ConnectResult.Result)
      {
      case Result::ClientAlreadyConnected:
        std::cout << "Client Already Connected" << std::endl;
        break;
      case Result::InvalidHostName:
        std::cout << "Invalid Host Name" << std::endl;
        break;
      case Result::ClientConnectionFailed:
        std::cout << "Client Connection Failed" << std::endl;
        break;
      default:
        std::cout << "Unrecognized Error: " << ConnectResult.Result
                  << std::endl;
        break;
      }
    }

    std::cout << ".";
#ifdef WIN32
    Sleep(1000);
#else
    sleep(1);
#endif
    // }

    std::cout << std::endl;
    // Enable some different data types
    DirectClient.EnableSegmentData();

    DirectClient.SetStreamMode(ViconDataStreamSDK::CPP::StreamMode::ServerPush);
    // }

    // Set the global up axis
    DirectClient.SetAxisMapping(Direction::Forward, Direction::Left,
                                Direction::Up); // Z-up

    if (AxisMapping == "YUp")
    {
      DirectClient.SetAxisMapping(Direction::Forward, Direction::Up,
                                  Direction::Right); // Y-up
    }
    else if (AxisMapping == "XUp")
    {
      DirectClient.SetAxisMapping(Direction::Up, Direction::Forward,
                                  Direction::Left); // Y-up
    }

    Output_GetAxisMapping _Output_GetAxisMapping =
        DirectClient.GetAxisMapping();
    // std::cout << "Axis Mapping: X-" << Adapt(_Output_GetAxisMapping.XAxis)
    //           << " Y-" << Adapt(_Output_GetAxisMapping.YAxis) << " Z-"
    //           << Adapt(_Output_GetAxisMapping.ZAxis) << std::endl;

    // Discover the version number
    Output_GetVersion _Output_GetVersion = DirectClient.GetVersion();
    // std::cout << "Version: " << _Output_GetVersion.Major << "."
    //           << _Output_GetVersion.Minor << "." << _Output_GetVersion.Point
    //           << "." << _Output_GetVersion.Revision << std::endl;

    if (ClientBufferSize > 0)
    {
      DirectClient.SetBufferSize(ClientBufferSize);
      std::cout << "Setting client buffer size to " << ClientBufferSize
                << std::endl;
    }

    bool bSubjectFilterApplied = false;

    ViconDataStreamSDK::CPP::Client &MyClient(DirectClient);

    size_t Counter = 0;
    const std::chrono::high_resolution_clock::time_point StartTime =
        std::chrono::high_resolution_clock::now();
    auto prev_sent_time = StartTime;
    // Loop until a key is pressed
#ifdef WIN32
    while (!Hit())
#else
    while (true)
#endif
    {
      // Get a frame
      // OutputStream << "Waiting for new frame...";
      while (MyClient.GetFrame().Result != Result::Success)
      {
// Sleep a little so that we don't lumber the CPU with a busy poll
#ifdef WIN32
        Sleep(200);
#else
        sleep(1);
#endif

        OutputStream << ".";
      }
      OutputStream << std::endl;

      // We have to call this after the call to get frame, otherwise we don't
      // have any subject info to map the name to ids
      if (!bSubjectFilterApplied)
      {
        for (const auto &rSubject : FilteredSubjects)
        {
          Output_AddToSubjectFilter SubjectFilterResult =
              MyClient.AddToSubjectFilter(rSubject);
          bSubjectFilterApplied = bSubjectFilterApplied ||
                                  SubjectFilterResult.Result == Result::Success;
        }
      }

      const std::chrono::high_resolution_clock::time_point Now =
          std::chrono::high_resolution_clock::now();

      // Get the frame number
      Output_GetFrameNumber _Output_GetFrameNumber = MyClient.GetFrameNumber();
      // OutputStream << "Frame Number: " << _Output_GetFrameNumber.FrameNumber
      //            << std::endl;

// check frame number for data quality
      if (!checkMocapData(_Output_GetFrameNumber.FrameNumber)) {
        // TODO action upon bad data
        continue;
      }

      // ///////////////////////////////////////////////////////////
      // Set frame number

      // ///////////////////////////////////////////////////////////

      Output_GetFrameRate Rate = MyClient.GetFrameRate();
      // OutputStream << "Frame rate: " << Rate.FrameRateHz << std::endl;

      // Show frame rates
      for (unsigned int FramerateIndex = 0;
           FramerateIndex < MyClient.GetFrameRateCount().Count;
           ++FramerateIndex)
      {
        std::string FramerateName =
            MyClient.GetFrameRateName(FramerateIndex).Name;
        double FramerateValue = MyClient.GetFrameRateValue(FramerateName).Value;

        // OutputStream << FramerateName << ": " << FramerateValue << "Hz"
        //              << std::endl;
      }
      // OutputStream << std::endl;

      // Get the timecode
      Output_GetTimecode _Output_GetTimecode = MyClient.GetTimecode();

      // OutputStream << "Timecode: " << _Output_GetTimecode.Hours << "h "
      //              << _Output_GetTimecode.Minutes << "m "
      //              << _Output_GetTimecode.Seconds << "s "
      //              << _Output_GetTimecode.Frames << "f "
      //              << _Output_GetTimecode.SubFrame << "sf "
      //              << Adapt(_Output_GetTimecode.FieldFlag) << " "
      //              << Adapt(_Output_GetTimecode.Standard) << " "
      //              << _Output_GetTimecode.SubFramesPerFrame << " "
      //              << _Output_GetTimecode.UserBits << std::endl
      //              << std::endl;

      // Get the latency
      // OutputStream << "Latency: " << (float)MyClient.GetLatencyTotal().Total
      //              << "s" << std::endl;
      float latency = MyClient.GetLatencyTotal().Total;

      ///////////////////////////////////////////////////////////
      // Set latency

      ///////////////////////////////////////////////////////////

      // for (unsigned int LatencySampleIndex = 0;
      //      LatencySampleIndex < MyClient.GetLatencySampleCount().Count;
      //      ++LatencySampleIndex) {
      //   std::string SampleName =
      //       MyClient.GetLatencySampleName(LatencySampleIndex).Name;
      //   double SampleValue =
      //   MyClient.GetLatencySampleValue(SampleName).Value;

      //   OutputStream << "  " << SampleName << " " << SampleValue << "s"
      //                << std::endl;
      // }

      // OutputStream << std::endl;

      // Output_GetHardwareFrameNumber _Output_GetHardwareFrameNumber =
      //     MyClient.GetHardwareFrameNumber();
      // OutputStream << "Hardware Frame Number: "
      //              << _Output_GetHardwareFrameNumber.HardwareFrameNumber
      //              << std::endl;

      // Count the number of subjects
      unsigned int SubjectCount = MyClient.GetSubjectCount().SubjectCount;
      // OutputStream << "Subjects (" << SubjectCount << "):" << std::endl;
      for (unsigned int SubjectIndex = 0; SubjectIndex < SubjectCount;
           ++SubjectIndex)
      {
        // OutputStream << "  Subject #" << SubjectIndex << std::endl;

        // Get the subject name
        std::string SubjectName =
            MyClient.GetSubjectName(SubjectIndex).SubjectName;
        // OutputStream << "    Name: " << SubjectName << std::endl;

        ///////////////////////////////////////////////////////////
        // Set object name

        ///////////////////////////////////////////////////////////

        // Get the root segment
        // std::string RootSegment =
        //     MyClient.GetSubjectRootSegmentName(SubjectName).SegmentName;
        // OutputStream << "    Root Segment: " << RootSegment << std::endl;

        // Count the number of segments
        unsigned int SegmentCount =
            MyClient.GetSegmentCount(SubjectName).SegmentCount;
        // OutputStream << "    Segments (" << SegmentCount << "):" <<
        // std::endl;
        for (unsigned int SegmentIndex = 0; SegmentIndex < SegmentCount;
             ++SegmentIndex)
        {
          // OutputStream << "      Segment #" << SegmentIndex << std::endl;

          // Get the segment name
          std::string SegmentName =
              MyClient.GetSegmentName(SubjectName, SegmentIndex).SegmentName;
          // OutputStream << "        Name: " << SegmentName << std::endl;

          // Get the segment parent
          // std::string SegmentParentName =
          //     MyClient.GetSegmentParentName(SubjectName, SegmentName)
          //         .SegmentName;
          // OutputStream << "        Parent: " << SegmentParentName
          //              << std::endl;

          // Get the segment's children
          // unsigned int ChildCount =
          //     MyClient.GetSegmentChildCount(SubjectName, SegmentName)
          //         .SegmentCount;
          // OutputStream << "     Children (" << ChildCount
          //              << "):" << std::endl;
          // for (unsigned int ChildIndex = 0; ChildIndex < ChildCount;
          //      ++ChildIndex) {
          //   // std::string ChildName =
          //   //     MyClient
          //   //         .GetSegmentChildName(SubjectName, SegmentName,
          //   //         ChildIndex) .SegmentName;
          //   // OutputStream << "       " << ChildName << std::endl;
          // }

          // // Get the static segment scale
          // Output_GetSegmentStaticScale _Output_GetSegmentStaticScale =
          //     MyClient.GetSegmentStaticScale(SubjectName, SegmentName);
          // if (_Output_GetSegmentStaticScale.Result == Result::Success) {
          //   OutputStream << "        Static Scale: ("
          //                << _Output_GetSegmentStaticScale.Scale[0] << ", "
          //                << _Output_GetSegmentStaticScale.Scale[1] << ", "
          //                << _Output_GetSegmentStaticScale.Scale[2] << ")"
          //                << std::endl;
          // }

          // // Get the static segment translation
          // Output_GetSegmentStaticTranslation
          //     _Output_GetSegmentStaticTranslation =
          //         MyClient.GetSegmentStaticTranslation(SubjectName,
          //                                              SegmentName);
          // OutputStream << "        Static Translation: ("
          //              <<
          //              _Output_GetSegmentStaticTranslation.Translation[0]
          //              << ", "
          //              <<
          //              _Output_GetSegmentStaticTranslation.Translation[1]
          //              << ", "
          //              <<
          //              _Output_GetSegmentStaticTranslation.Translation[2]
          //              << ")" << std::endl;

          // // Get the static segment rotation in helical co-ordinates
          // Output_GetSegmentStaticRotationHelical
          //     _Output_GetSegmentStaticRotationHelical =
          //         MyClient.GetSegmentStaticRotationHelical(SubjectName,
          //                                                  SegmentName);
          // OutputStream << "        Static Rotation Helical: ("
          //              <<
          //              _Output_GetSegmentStaticRotationHelical.Rotation[0]
          //              << ", "
          //              <<
          //              _Output_GetSegmentStaticRotationHelical.Rotation[1]
          //              << ", "
          //              <<
          //              _Output_GetSegmentStaticRotationHelical.Rotation[2]
          //              << ")" << std::endl;

          // // Get the static segment rotation as a matrix
          // Output_GetSegmentStaticRotationMatrix
          //     _Output_GetSegmentStaticRotationMatrix =
          //         MyClient.GetSegmentStaticRotationMatrix(SubjectName,
          //                                                 SegmentName);
          // OutputStream
          //     << "        Static Rotation Matrix: ("
          //     << _Output_GetSegmentStaticRotationMatrix.Rotation[0] << ", "
          //     << _Output_GetSegmentStaticRotationMatrix.Rotation[1] << ", "
          //     << _Output_GetSegmentStaticRotationMatrix.Rotation[2] << ", "
          //     << _Output_GetSegmentStaticRotationMatrix.Rotation[3] << ", "
          //     << _Output_GetSegmentStaticRotationMatrix.Rotation[4] << ", "
          //     << _Output_GetSegmentStaticRotationMatrix.Rotation[5] << ", "
          //     << _Output_GetSegmentStaticRotationMatrix.Rotation[6] << ", "
          //     << _Output_GetSegmentStaticRotationMatrix.Rotation[7] << ", "
          //     << _Output_GetSegmentStaticRotationMatrix.Rotation[8] << ")"
          //     << std::endl;

          // // Get the static segment rotation in quaternion co-ordinates
          // Output_GetSegmentStaticRotationQuaternion
          //     _Output_GetSegmentStaticRotationQuaternion =
          //         MyClient.GetSegmentStaticRotationQuaternion(SubjectName,
          //                                                     SegmentName);
          // OutputStream
          //     << "        Static Rotation Quaternion: ("
          //     << _Output_GetSegmentStaticRotationQuaternion.Rotation[0]
          //     << ", "
          //     << _Output_GetSegmentStaticRotationQuaternion.Rotation[1]
          //     << ", "
          //     << _Output_GetSegmentStaticRotationQuaternion.Rotation[2]
          //     << ", "
          //     << _Output_GetSegmentStaticRotationQuaternion.Rotation[3] <<
          //     ")"
          //     << std::endl;

          // // Get the static segment rotation in EulerXYZ co-ordinates
          // Output_GetSegmentStaticRotationEulerXYZ
          //     _Output_GetSegmentStaticRotationEulerXYZ =
          //         MyClient.GetSegmentStaticRotationEulerXYZ(SubjectName,
          //                                                   SegmentName);
          // OutputStream << "        Static Rotation EulerXYZ: ("
          //              <<
          //              _Output_GetSegmentStaticRotationEulerXYZ.Rotation[0]
          //              << ", "
          //              <<
          //              _Output_GetSegmentStaticRotationEulerXYZ.Rotation[1]
          //              << ", "
          //              <<
          //              _Output_GetSegmentStaticRotationEulerXYZ.Rotation[2]
          //              << ")" << std::endl;

          // Get the global segment translation
          Output_GetSegmentGlobalTranslation
              _Output_GetSegmentGlobalTranslation =
                  MyClient.GetSegmentGlobalTranslation(SubjectName,
                                                       SegmentName);
          // OutputStream << "        Global Translation: ("
          //              << _Output_GetSegmentGlobalTranslation.Translation[0]
          //              << ", "
          //              << _Output_GetSegmentGlobalTranslation.Translation[1]
          //              << ", "
          //              << _Output_GetSegmentGlobalTranslation.Translation[2]
          //              << ") "
          //              << Adapt(_Output_GetSegmentGlobalTranslation.Occluded)
          //              << std::endl;

          ///////////////////////////////////////////////////////////
          // Set global translation

          vision_msg.position_body.x_m =
              _Output_GetSegmentGlobalTranslation.Translation[0] / 1000.0;
          vision_msg.position_body.y_m =
              -_Output_GetSegmentGlobalTranslation.Translation[1] / 1000.0;
          vision_msg.position_body.z_m =
              -_Output_GetSegmentGlobalTranslation.Translation[2] / 1000.0;

          ///////////////////////////////////////////////////////////

          // Get the global segment rotation in quaternion co-ordinates
          Output_GetSegmentGlobalRotationQuaternion
              _Output_GetSegmentGlobalRotationQuaternion =
                  MyClient.GetSegmentGlobalRotationQuaternion(SubjectName,
                                                              SegmentName);
          // OutputStream
          //     << "        Global Rotation Quaternion: ("
          //     << _Output_GetSegmentGlobalRotationQuaternion.Rotation[0] << ", "
          //     << _Output_GetSegmentGlobalRotationQuaternion.Rotation[1] << ", "
          //     << _Output_GetSegmentGlobalRotationQuaternion.Rotation[2] << ", "
          //     << _Output_GetSegmentGlobalRotationQuaternion.Rotation[3] << ") "
          //     << Adapt(_Output_GetSegmentGlobalRotationQuaternion.Occluded)
          //     << std::endl;

          ///////////////////////////////////////////////////////////
          // Set global rotation quaternion

          matrix::Quatf quat_orientation(
              _Output_GetSegmentGlobalRotationQuaternion.Rotation[1],
              _Output_GetSegmentGlobalRotationQuaternion.Rotation[2],
              _Output_GetSegmentGlobalRotationQuaternion.Rotation[3],
              _Output_GetSegmentGlobalRotationQuaternion.Rotation[0]);

          matrix::Eulerf euler_orientation(quat_orientation);

          // Roll transformation
          if (euler_orientation(0) > 0)
            vision_msg.angle_body.roll_rad = M_PI - euler_orientation(0);

          else if (euler_orientation(0) < 0)
            vision_msg.angle_body.roll_rad = -M_PI - euler_orientation(0);

          // Pitch transformation
          vision_msg.angle_body.pitch_rad = -euler_orientation(1);

          if (euler_orientation(2) > 0)
            vision_msg.angle_body.yaw_rad = M_PI - euler_orientation(2);

          else if (euler_orientation(2) < 0)
            vision_msg.angle_body.yaw_rad = -M_PI - euler_orientation(2);

          // Invert Sign to match px4 convention
          vision_msg.angle_body.yaw_rad = -vision_msg.angle_body.yaw_rad;
          // std::cout << '\t' << "Euler Orientation: "
          //           << "roll " << vision_msg.angle_body.roll_rad *(180 / M_PI)<< '\t' << "pitch "
          //           << vision_msg.angle_body.pitch_rad *(180 / M_PI)<< '\t' << "yaw "
          //           << vision_msg.angle_body.yaw_rad *(180 / M_PI)<< '\n';
          ///////////////////////////////////////////////////////////

          //   // Get the global segment rotation in EulerXYZ co-ordinates
          //   Output_GetSegmentGlobalRotationEulerXYZ
          //       _Output_GetSegmentGlobalRotationEulerXYZ =
          //           MyClient.GetSegmentGlobalRotationEulerXYZ(SubjectName,
          //                                                     SegmentName);
          //   OutputStream << "        Global Rotation EulerXYZ: ("
          //                << _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[0]
          //                << ", "
          //                << _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[1]
          //                << ", "
          //                << _Output_GetSegmentGlobalRotationEulerXYZ.Rotation[2]
          //                << ") "
          //                << Adapt(
          //                       _Output_GetSegmentGlobalRotationEulerXYZ.Occluded)
          //                << std::endl
          //                << std::endl;
        }

        // // Get the quality of the subject (object) if supported
        // Output_GetObjectQuality _Output_GetObjectQuality =
        //     MyClient.GetObjectQuality(SubjectName);
        // if (_Output_GetObjectQuality.Result == Result::Success) {
        //   double Quality = _Output_GetObjectQuality.Quality;
        //   OutputStream << "    Quality: " << Quality << std::endl;
        // }

        ///////////////////////////////////////////////////////////
        // Publish data POI

        if (SubjectName.compare("srl_quad") == 0)
        {
          Mocap::Result result =
              vision.set_vision_position_estimate(vision_msg);
          // std::cout << "(srl_quad) object detected" << result << std::endl;
        

        // std::cout << "pos: " << vision_msg.position_body.x_m  << "\t" << -vision_msg.position_body.y_m << "\t" << -vision_msg.position_body.z_m  << std::endl;
	}
        // LOGGING//
        /*if (enable_log == true)
        {
         auto t_end = std::chrono::high_resolution_clock::now();
          int t = std::chrono::duration<double, std::milli>(t_end - StartTime).count();
          if (telemetry.actuator_control_target().controls.size() != 0)
          {
            Telemetry::Imu imu_now = telemetry.scaled_imu();
            myLog << t << ","
                  << telemetry.position_velocity_ned().position.north_m << ","
                  << telemetry.position_velocity_ned().position.east_m << ","
                  << -telemetry.position_velocity_ned().position.down_m << ","
                  << telemetry.position_velocity_ned().velocity.north_m_s << ","
                  << telemetry.position_velocity_ned().velocity.east_m_s << ","
                  << -telemetry.position_velocity_ned().velocity.down_m_s << ","
                  << telemetry.attitude_euler().roll_deg << ","
                  << telemetry.attitude_euler().pitch_deg << ","
                  << telemetry.attitude_euler().yaw_deg << ","
                  << telemetry.attitude_angular_velocity_body().roll_rad_s << ","
                  << telemetry.attitude_angular_velocity_body().pitch_rad_s << ","
                  << telemetry.attitude_angular_velocity_body().yaw_rad_s << ","
                  << telemetry.actuator_control_target().controls.at(0) << ","
                  << telemetry.actuator_control_target().controls.at(1) << ","
                  << telemetry.actuator_control_target().controls.at(2) << ","
                  << telemetry.actuator_control_target().controls.at(3) << ","
                  << imu_now.acceleration_frd.forward_m_s2 << ","
                  << imu_now.acceleration_frd.down_m_s2 << ","
                  << imu_now.acceleration_frd.right_m_s2 << ","
                  << rpm[0] << ","
                  << rpm[1] << ","
                  << rpm[2] << ","
                  << rpm[3] << ","
                  << voltage[0] << ","
                  << voltage[1] << ","
                  << voltage[2] << ","
                  << voltage[3] << ","
                  << current[0] << ","
                  << current[1] << ","
                  << current[2] << ","
                  << current[3] << "\n";
          }
        }*/

        // telemetry
        const auto& pos_vel = telemetry.position_velocity_ned();
        std::cout << pos_vel.position.north_m << '\t' << pos_vel.position.east_m << '\t' <<  pos_vel.position.down_m << '\n';

        // Only 100 Hz required, let CPU rest
        std::this_thread::sleep_for(std::chrono::milliseconds(1));

        ///////////////////////////////////////////////////////////
      }
    }

    ++Counter;
  }
}
