cmake_minimum_required(VERSION 3.0.2)
project(scara_arm_hw_interface)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS 
    message_generation
    cmake_modules 
    hardware_interface 
    controller_manager 
    roscpp 
    control_msgs 
    urdf 
    std_msgs 
    ros_control_boilerplate
    )

# add_message_files(FILES
#     armCmd.msg
#     armTelemetry.msg
# )

# generate_messages(DEPENDENCIES 
#     std_msgs
# )

catkin_package(
    CATKIN_DEPENDS 
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

add_executable(scara_arm_hw_main 
    include/scara_arm_hw_interface/scara_arm_hw_interface.h
    src/scara_arm_hw_interface.cpp
    src/scara_arm_hw_main.cpp    
)
add_dependencies(scara_arm_hw_main ${${PROJECT_NAME}_EXPORTED_TARGETS}
    ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(scara_arm_hw_main
    ${catkin_LIBRARIES}
)