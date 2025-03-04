//
// Created by sfx233 on 25-3-4.
//
#include <iostream>
#include "serialPro.h"
#include <opencv2/opencv.hpp>  // OpenCV 用于处理和保存图像
#include <fstream>             // 用于保存地图信息到文件

// 全局变量：用于保存接收到的地图数据
cv::Mat mapImage;
using sp::serialPro;
// 回调函数：处理接收到的串口消息
void onMessageReceived(const std::vector<uint8_t>& data) {
    // 假设接收到的数据是二进制图像数据
    if (!data.empty()) {
        // 将二进制数据解码为 OpenCV 图像
        mapImage = cv::imdecode(data, cv::IMREAD_COLOR);
        if (mapImage.empty()) {
            std::cerr << "Error: Failed to decode image data!" << std::endl;
            return;
        }

        // 保存图像为地图文件
        std::string mapFilePath = "output_map.png";
        if (cv::imwrite(mapFilePath, mapImage)) {
            std::cout << "Map saved to: " << mapFilePath << std::endl;
        } else {
            std::cerr << "Error: Failed to save map image!" << std::endl;
        }
    }
}

int main() {
    // 实例化 serialPro 类

    serialPro <uint64_t,uint64_t> serial;

    // 注册回调函数
    serial.registerCallback(1,onMessageReceived);

    // 启动串口通信（假设串口已正确配置）
    if (!serial.open("/dev/ttyUSB0", 115200)) {
        std::cerr << "Error: Failed to open serial port!" << std::endl;
        return -1;
    }

    std::cout << "Serial port opened successfully. Waiting for map data..." << std::endl;

    // 主循环：持续接收和处理消息
    while (true) {
        // 更新串口数据接收
        serial.update();

        // 简单退出条件（可根据需要修改）
        if (!mapImage.empty()) {
            std::cout << "Map processing completed. Exiting..." << std::endl;
            break;
        }
    }

    return 0;
}
