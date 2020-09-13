cmake_minimum_required(VERSION 2.8.3)
project(catix_messages)

find_package(catkin REQUIRED genmsg std_msgs)

add_message_files(FILES
    TwoDofLegState.msg
    EightDofPlatformState.msg
    ServoState.msg
    PwmState.msg)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(DEPENDS std_msgs)
