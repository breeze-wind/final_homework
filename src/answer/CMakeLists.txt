cmake_minimum_required(VERSION 3.5)
project(answer)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(OpenCV REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

add_executable(answer src/answer.cpp)
target_include_directories(answer PRIVATE
        ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(answer
        ${OpenCV_LIBS}
        rclcpp
        sensor_msgs::sensor_msgs
        cv_bridge
)
