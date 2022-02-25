
add_executable(mocap_px4_pub  mocap_px4_pub.cpp)


target_link_libraries(mocap_px4_pub
ViconDataStreamSDK_lib
MAVSDK::mavsdk_action
MAVSDK::mavsdk_offboard
MAVSDK::mavsdk_telemetry
MAVSDK::mavsdk_mocap
MAVSDK::mavsdk
)

