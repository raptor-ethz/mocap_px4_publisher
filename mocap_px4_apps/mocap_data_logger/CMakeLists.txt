

add_executable(mocap_data_logger mocap_data_logger.cpp)

target_link_libraries(mocap_data_logger
ViconDataStreamSDK_lib
)

