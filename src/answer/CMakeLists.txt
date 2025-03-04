cmake_minimum_required(VERSION 3.14)
project(map_processor)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 查找 OpenCV 库
find_package(OpenCV REQUIRED)

# 包含 OpenCV 头文件
include_directories(${OpenCV_INCLUDE_DIRS})


# 包含 serialPro 头文件路径（相对路径）
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)  # answer/include
include_directories(${CMAKE_SOURCE_DIR}/src/serialPro/include)  # serialPro/include

# 包含 serialPro 库路径（相对路径）
link_directories(${CMAKE_SOURCE_DIR}/src/serialPro/lib)

# 添加可执行文件
add_executable(map_processor src/map_processor.cpp)

# 链接 OpenCV 和 serialPro 库
target_link_libraries(map_processor ${OpenCV_LIBS} serialPro)