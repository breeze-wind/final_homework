#ifndef NAVIGATION_NODE_HPP
#define NAVIGATION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <example_interfaces/msg/bool.hpp>
#include <example_interfaces/msg/int64.hpp>
#include <info_interfaces/msg/map.hpp>
#include <info_interfaces/msg/area.hpp>
#include <info_interfaces/msg/robot.hpp>
#include <my_serial/serial_handler.hpp>
#include <algorithm>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <unordered_map>
#include <tuple>

// 前置声明
namespace my_serial {
    class SerialHandler;
}

namespace navigation {

class Node : public rclcpp::Node {
public:
    explicit Node(const std::string& name);

private:
    // 状态机前向声明
    class StateMachine;

    // 内部类型声明
    enum class MapType { GRID, REAL };
    using Position = std::tuple<int32_t, int32_t>;

    // 组件初始化
    void initialize_components();
    void setup_serial();
    void setup_publishers();
    void setup_subscriptions();


    template<typename T>
    void create_generic_sub(
        const std::string& topic,
        std::function<void(typename T::SharedPtr)> callback)
    {
        m_subs.push_back(
            this->create_subscription<T>(
                topic,
                1,
                [cb = std::move(callback)](typename T::SharedPtr msg) {
                    cb(std::move(msg));
                }
            )
        );
    }

    // 状态机相关
    std::unique_ptr<StateMachine> m_fsm;

    // 硬件接口
    my_serial::SerialHandler m_serial;

    // ROS接口存储
    std::unordered_map<std::string, rclcpp::PublisherBase::SharedPtr> m_pubs;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> m_subs;

    // 供状态机访问的接口
    friend class StateMachine;
    void publish_pose(double x, double y, double theta);
    void publish_stop();
};

// 状态机类声明
class Node::StateMachine {
public:
    // 状态枚举
    enum class State {
        INIT,
        COMBAT,
        RECOVERY,
        PASSWORD_READY,
        COMPLETED
    };

    explicit StateMachine(Node& ctx);

    // 事件处理接口
    void handle_password(const my_serial::password_receive_t& pass);
    void update_map(const info_interfaces::msg::Map::SharedPtr map, MapType type);

private:
    // 状态转换控制
    Node& m_ctx;
    std::mutex m_mutex;
    State m_current = State::INIT;

    // 地图数据
    info_interfaces::msg::Map::SharedPtr m_grid_map;
    info_interfaces::msg::Map::SharedPtr m_real_map;

    // 密码相关
    int64_t m_password = 0;

    // 状态迁移方法
    void transit_to(State new_state);
    void execute_entry_action(State s);
    void execute_exit_action(State s);

    // 路径规划方法
    void path_planning(const Position& start, const Position& target);
    void execute_path(const algorithm::Path& path);
};

} // namespace navigation

#endif // NAVIGATION_NODE_HPP
