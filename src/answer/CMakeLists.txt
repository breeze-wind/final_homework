cmake_minimum_required(VERSION 3.8)
project(map_processing)

# 设置C++标准
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV  EXACT REQUIRED COMPONENTS core imgproc highgui)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED) # 如果是用了自定义消息类型

# 如果使用了自定义消息类型（需先在package.xml中指定）
# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/MapData.msg"
#   DEPENDENCIES
#   nav_msgs
#   geometry_msgs
# )
# 构建可执行文件
add_executable(map_parser
        ${CMAKE_SOURCE_DIR}/src/${PROJECT_NAME}/src/main.cpp
        # 如果实现拆分了头文件
        ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}/map_structure.hpp
        src/navigation/navigation_node.cpp
)

# 链接依赖项
target_include_directories(map_parser PRIVATE
        include
        ${OpenCV_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}
)

# 仅保留其中一种方式（建议使用ament方式）
ament_target_dependencies(map_parser
        rclcpp
        cv_bridge
        nav_msgs
        sensor_msgs
        geometry_msgs
        tf2_ros
)

arget_include_directories(map_parser PRIVATE
        include
        ${OpenCV_INCLUDE_DIRS}
        ${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME}
)

# 仅保留其中一种方式（建议使用ament方式）
ament_target_dependencies(map_parser
        rclcpp
        cv_bridge
        nav_msgs
        sensor_msgs
        geometry_msgs
        tf2_ros
)
target_link_libraries(map_parser ${OpenCV_LIBS})  # OpenCV需要特殊处理



# 安装目标文件
install(TARGETS
        map_parser
        DESTINATION lib/${PROJECT_NAME}
)

# 导出依赖项
ament_export_dependencies(ament_cmake)
ament_export_dependencies(rclcpp)
ament_export_dependencies(cv_bridge)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(geometry_msgs)

# 结束配置
ament_package()
