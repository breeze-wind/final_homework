#include "navigation_node.hpp"

namespace navigation {

class StateMachine; // 前置声明

class Node : public rclcpp::Node {
public:
    explicit Node(const std::string& name)
        : rclcpp::Node(name),
          m_serial(constant::serial_path, constant::baud_rate),
          m_fsm(std::make_unique<StateMachine>(*this)) {
        initialize_components();
    }

private:
    // 组件初始化（解耦硬件和逻辑）
    void initialize_components() {
        setup_serial();
        setup_publishers();
        setup_subscriptions();
    }

    // 串口配置（单独提取方法）
    void setup_serial() {
        m_serial.spin(true);
        m_serial.registerCallback(my_serial::CMD_READ,
            [this](const auto& msg) { m_fsm->handle_password(msg); });
    }

    // 发布者统一配置
    void setup_publishers() {
        m_pubs = {
            {"pose", create_publisher<geometry_msgs::msg::Pose2D>(topic_name::pose, 1)},
            {"shoot", create_publisher<example_interfaces::msg::Bool>(topic_name::shoot, 1)},
            {"password", create_publisher<example_interfaces::msg::Int64>(topic_name::password, 1)}
        };
    }

    // 订阅者泛型模板
    template<typename T>
    void create_generic_sub(const std::string& topic,
                          std::function<void(typename T::SharedPtr)> callback) {
        m_subs.push_back(
            this->create_subscription<T>(topic, 1,
                [cb=std::move(callback)](typename T::SharedPtr msg) {
                    cb(std::move(msg));
                })
        );
    }

    // 订阅配置（类型擦除实现）
    void setup_subscriptions() {
        using namespace std::placeholders;

        create_generic_sub<info_interfaces::msg::Map>(
            topic_name::grid_map,
            [this](auto msg) { m_fsm->update_map(msg, MapType::GRID); });

        create_generic_sub<info_interfaces::msg::Map>(
            topic_name::real_map,
            [this](auto msg) { m_fsm->update_map(msg, MapType::REAL); });

        // 其他订阅项类似配置...
    }

    // 状态机核心（实现细节隐藏）
    class StateMachine {
    public:
        explicit StateMachine(Node& ctx) : m_ctx(ctx) {}

        void handle_password(const my_serial::password_receive_t& pass) {
            std::lock_guard<std::mutex> lock(m_mutex);
            m_password = pass.password;
            transit_to(State::PASSWORD_READY);
        }

        void update_map(const info_interfaces::msg::Map::SharedPtr map, MapType type) {
            std::lock_guard<std::mutex> lock(m_mutex);
            (type == MapType::GRID) ? m_grid_map = map : m_real_map = map;
            check_initialization();
        }

        // 其他事件处理函数(妹写完）

    private:
        enum class State { INIT, COMBAT, RECOVERY, PASSWORD_READY, COMPLETED };//四种状态
        enum class MapType { GRID, REAL };

        Node& m_ctx;
        std::mutex m_mutex;
        State m_current = State::INIT;
        info_interfaces::msg::Map::SharedPtr m_grid_map, m_real_map;
        int64_t m_password = 0;

        void transit_to(State new_state) {
            if (m_current == new_state) return;
            execute_exit_action(m_current);
            execute_entry_action(new_state);
            m_current = new_state;
        }

        void execute_entry_action(State s) {
            switch(s) {
                case State::COMBAT:
                    m_ctx.RCLCPP_INFO("Enter combat mode");
                    break;
                case State::RECOVERY:
                    m_ctx.RCLCPP_INFO("Enter recovery mode");
                    break;
                // 其他状态处理...
            }
        }

        void path_planning(const Position& start, const Position& target) {
            // 使用A*算法但带缓存机制
            static std::tuple<Position, Position, algorithm::Path> cache;
            auto& [cached_start, cached_target, cached_path] = cache;

            if (start == cached_start && target == cached_target) {
                use_cached_path(cached_path);
                return;
            }

            cached_path = algorithm::a_star(m_grid_map, start.x, start.y, target.x, target.y);
            cached_start = start;
            cached_target = target;
            execute_path(cached_path);
        }

        void execute_path(const algorithm::Path& path) {
            if (path.empty()) {
                m_ctx.publish_stop();
                return;
            }

            auto [dx, dy] = calculate_movement(path[0], path[1]);
            m_ctx.publish_pose(dx * constant::speed_scale,
                              dy * constant::speed_scale,
                              calculate_orientation());
        }

        // 其他状态相关方法...
    };

    std::unique_ptr<StateMachine> m_fsm;
    my_serial::SerialHandler m_serial;
    std::unordered_map<std::string, rclcpp::PublisherBase::SharedPtr> m_pubs;
    std::vector<rclcpp::SubscriptionBase::SharedPtr> m_subs;

public:
    // 供状态机调用的统一接口
    void publish_pose(double x, double y, double theta) {
        auto msg = geometry_msgs::msg::Pose2D();
        msg.x = x; msg.y = y; msg.theta = theta;
        std::dynamic_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::Pose2D>>(
            m_pubs["pose"])->publish(msg);
    }

    void publish_stop() {
        publish_pose(0, 0, 0);
    }
};

} // namespace navigation

// main函数保持相似
