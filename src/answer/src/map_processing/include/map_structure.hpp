#pragma once
#include <opencv2/core/types.hpp>
struct NavigationMap {
    cv::Mat occupancy_grid;      // 可通行区域二值化地图
    std::vector<cv::Point> portals;        // 传送点坐标集合
    cv::Rect supply_area;                  // 补给区边界框
    cv::Point base_center;                 // 敌方基地中心坐标
    cv::Point self_position;               // 己方哨兵坐标
    std::vector<cv::Point> enemy_positions; // 敌方步兵坐标集合
};