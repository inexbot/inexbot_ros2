#include <atomic>
#include <thread>
#include <chrono>
#include <vector>
#include <string>
#include <memory>

#include <nlohmann/json.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "cpp/interface/nrc_interface.h"
#include "cpp/interface/nrc_interface_servo.h"
#include "nrc_msgs/msg/robot_status.hpp"

using namespace std::chrono_literals;
using json = nlohmann::json;

class NrcNode;
static NrcNode* g_node = nullptr;   // 裸回调没有 this，用全局指针转发

class NrcNode : public rclcpp::Node
{
public:
    NrcNode() : Node("nrc_node")
    {
        g_node = this;

        // ---------- 参数 ----------
        ip_ = this->declare_parameter("robot_ip", std::string("127.0.0.1"));
        int ctrl_port  = this->declare_parameter("robot_port", 6001);
        int state_port = this->declare_parameter("state_port", 7000);
        state_interval_ = this->declare_parameter("state_interval_ms", 100);
        ctrl_port_  = std::to_string(ctrl_port);
        state_port_ = std::to_string(state_port);
        this->declare_parameter("library_version", get_library_version());

        RCLCPP_INFO(this->get_logger(), "SDK 版本: %s", get_library_version().c_str());
        RCLCPP_INFO(this->get_logger(), "控制器 %s  控制端口=%s  状态端口=%s  推送周期=%dms",
                    ip_.c_str(), ctrl_port_.c_str(), state_port_.c_str(), state_interval_);

        // ---------- 发布者 ----------
        connected_pub_ = this->create_publisher<std_msgs::msg::Bool>("~/connected", 10);
        joint_pub_     = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);

        status_timer_ = this->create_wall_timer(
            500ms, std::bind(&NrcNode::publish_connected, this));

        robot_status_pub_ = this->create_publisher<nrc_msgs::msg::RobotStatus>("~/robot_status", 10);
        status_query_timer_ = this->create_wall_timer(
            200ms, std::bind(&NrcNode::poll_and_publish_status, this));

        // ---------- 服务 ----------
        power_srv_ = this->create_service<std_srvs::srv::SetBool>(
            "~/set_servo_power",
            std::bind(&NrcNode::handle_servo_power, this,
                      std::placeholders::_1, std::placeholders::_2));

        // ---------- 连接线程 ----------
        running_ = true;
        connect_thread_ = std::thread(&NrcNode::connect_loop, this);
    }

    ~NrcNode()
    {
        running_ = false;
        if (connect_thread_.joinable()) {
            connect_thread_.join();
        }
        if (state_id_ >= 0) disconnect_robot(state_id_);
        if (ctrl_id_  >= 0) disconnect_robot(ctrl_id_);
        g_node = nullptr;
    }

    // 供静态回调转发进来
    void on_state_message(const char* message)
    {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
                         "原始推送: %s", message);
        try {
            json j = json::parse(message);
            if (!j.contains("replyData")) return;
            const auto& d = j["replyData"];

            // 关节位置 -> /joint_states
            if (d.contains("realPosACS")) {
                sensor_msgs::msg::JointState msg;

                if (d.contains("timestamp") && d["timestamp"].size() >= 2) {
                    msg.header.stamp.sec     = d["timestamp"][0].get<int32_t>();
                    msg.header.stamp.nanosec = d["timestamp"][1].get<uint32_t>() * 1000u;
                } else {
                    msg.header.stamp = this->now();
                }

                auto pos = d["realPosACS"].get<std::vector<double>>();
                msg.name.reserve(pos.size());
                msg.position.reserve(pos.size());
                for (size_t i = 0; i < pos.size(); ++i) {
                    msg.name.push_back("joint" + std::to_string(i + 1));
                    msg.position.push_back(pos[i]);   // 单位待确认：度 or 弧度
                }
                joint_pub_->publish(msg);
            }

            // 伺服状态
            if (d.contains("servoStatus")) {
                int s = d["servoStatus"].get<int>();
                if (s != servo_status_) {
                    servo_status_ = s;
                    RCLCPP_INFO(this->get_logger(), "伺服状态: %d (%s)",
                                s, servo_state_name(s));
                }
            }

            // 程序运行状态
            if (d.contains("programRunStatus")) {
                int s = d["programRunStatus"].get<int>();
                if (s != program_state_) {
                    program_state_ = s;
                    RCLCPP_INFO(this->get_logger(), "程序运行状态: %d", s);
                }
            }

            //操作模式
            if (d.contains("operationMode")) {
                int m = d["operationMode"].get<int>();
                if (m != operation_mode_) {
                    operation_mode_ = m;
                    const char* name = (m == 0) ? "示教" : (m == 1) ? "远程" : (m == 2) ? "运行" : "未知";
                    RCLCPP_INFO(this->get_logger(), "操作模式: %d (%s)", m, name);
                }
            }

        } catch (const std::exception& e) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "状态解析失败: %s", e.what());
        }
    }

private:
    static const char* servo_state_name(int s)
    {
        switch (s) {
            case 0: return "停止";
            case 1: return "就绪";
            case 2: return "报警";
            case 3: return "运行";
            default: return "未知";
        }
    }

    // 裸函数指针入口
    static void state_callback_trampoline(const char* message)
    {
        if (g_node && message) {
            g_node->on_state_message(message);
        }
    }

    // ==================== 连接管理 ====================

    void connect_loop()
    {
        while (running_) {
            maintain_ctrl_connection();
            maintain_state_connection();

            for (int i = 0; i < 10 && running_; ++i) {
                std::this_thread::sleep_for(100ms);
            }
        }
    }

    // 6001：常规控制端口
    void maintain_ctrl_connection()
    {
        if (ctrl_id_ < 0) {
            SOCKETFD id = connect_robot(ip_, ctrl_port_);   // 阻塞
            if (id >= 0) {
                ctrl_id_ = id;
                RCLCPP_INFO(this->get_logger(), "控制端口已连接, id=%d", id);
            }
            return;
        }

        bool now = (get_connection_status(ctrl_id_) == SUCCESS);
        if (now != connected_) {
            connected_ = now;
            RCLCPP_INFO(this->get_logger(), "控制连接: %s", now ? "已连接" : "未连接");
        }
        if (!now) {
            disconnect_robot(ctrl_id_);
            ctrl_id_ = -1;
        }
    }

    // 7000：状态推送端口
    void maintain_state_connection()
    {
        if (state_id_ < 0) {
            SOCKETFD id = connect_robot(ip_, state_port_);  // 阻塞
            if (id >= 0) {
                state_id_ = id;
                RCLCPP_INFO(this->get_logger(), "状态端口已连接, id=%d", id);
                setup_state_subscription();
            }
            return;
        }

        if (get_connection_status(state_id_) != SUCCESS) {
            RCLCPP_WARN(this->get_logger(), "状态端口已断开");
            disconnect_robot(state_id_);
            state_id_ = -1;
        }
    }

    void setup_state_subscription()
    {
        Result r1 = robot_state_callback(state_id_, &NrcNode::state_callback_trampoline);
        if (r1 != SUCCESS) {
            RCLCPP_WARN(this->get_logger(), "注册状态回调失败: %d", static_cast<int>(r1));
        }

        RobotState param;
        param.channel          = 1;
        param.mode             = 1;                 // 持续推送
        param.interval         = state_interval_;   // 推送周期 ms
        param.position         = 0;                 // 0-关节坐标 1-直角坐标
        param.servoStatus      = true;
        param.programRunStatus = true;
        param.operationMode    = true;   

        Result r2 = get_robot_state(state_id_, param);
        if (r2 == SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "状态订阅已建立");
        } else {
            RCLCPP_WARN(this->get_logger(), "状态订阅失败: %d", static_cast<int>(r2));
        }
    }

    // ==================== 伺服使能 ====================

    void handle_servo_power(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
        std::shared_ptr<std_srvs::srv::SetBool::Response> res)
    {
        if (!connected_ || ctrl_id_ < 0) {
            res->success = false;
            res->message = "控制器未连接";
            return;
        }
        int state = -1;
        Result qr = get_servo_state(ctrl_id_, state);
        RCLCPP_INFO(this->get_logger(), "主动查询伺服状态: Result=%d, state=%d",
                    static_cast<int>(qr), state);
        if (qr != SUCCESS) {
            res->success = false;
            res->message = "无法查询伺服状态, Result=" + std::to_string(static_cast<int>(qr));
            return;
        }

        if (req->data) {
            // ---------- 上电 ----------
            RCLCPP_INFO(this->get_logger(), "收到上电请求，当前伺服状态=%d", state);

            if (state == 3) {
                res->success = true;
                res->message = "伺服已处于运行状态";
                return;
            }
            if (state == 2) {
                res->success = false;
                res->message = "伺服处于报警状态，请先清除报警";
                return;
            }
             // ---- 新增：切到远程模式 ----
            int mode = -1;
            get_current_mode(ctrl_id_, mode);
            RCLCPP_INFO(this->get_logger(), "当前操作模式: %d", mode);

            if (mode != 1) {
            Result rm = set_current_mode(ctrl_id_, 1);   // 1 = 远程
            RCLCPP_INFO(this->get_logger(), "切换到远程模式, Result=%d", static_cast<int>(rm));
            std::this_thread::sleep_for(500ms);
            }
            // ---- 新增结束 ----
            // 先切到就绪态
            if (state != 1) {
                Result r = set_servo_state(ctrl_id_, 1);
                if (r != SUCCESS && r != OPERATION_NOT_ALLOWED) {   // -4 = 已在该状态，放行
                    res->success = false;
                    res->message = "设置就绪状态失败, Result=" + std::to_string(static_cast<int>(r));
                    return;
                }
                if (!wait_servo_state(1, 3000)) {
                    res->success = false;
                    res->message = "等待就绪状态超时，当前状态=" + std::to_string(servo_status_.load());
                    return;
                }
            }

            Result r = set_servo_poweron(ctrl_id_);	   
            if (r != SUCCESS) {
                res->success = false;
                res->message = "上电失败, Result=" + std::to_string(static_cast<int>(r));
                return;
            }
            if (!wait_servo_state(3, 5000)) {
                res->success = false;
                res->message = "上电后等待运行状态超时，当前状态=" + std::to_string(servo_status_.load());
                return;
            }
            res->success = true;
            res->message = "上电成功";
            RCLCPP_INFO(this->get_logger(), "上电成功");

        } else {
            // ---------- 下电 ----------
            RCLCPP_INFO(this->get_logger(), "收到下电请求，当前伺服状态=%d", state);

            if (state != 3) {
                res->success = true;
                res->message = "伺服未处于运行状态，无需下电";
                return;
            }

            Result r = set_servo_poweroff(ctrl_id_);
            if (r != SUCCESS) {
                res->success = false;
                res->message = "下电失败, Result=" + std::to_string(static_cast<int>(r));
                return;
            }
            if (!wait_servo_state(1, 5000)) {
                res->success = false;
                res->message = "下电后等待就绪状态超时，当前状态=" + std::to_string(servo_status_.load());
                return;
            }
            res->success = true;
            res->message = "下电成功";
            RCLCPP_INFO(this->get_logger(), "下电成功");
        }
    }

    // 等待伺服状态变成 target，超时返回 false
        bool wait_servo_state(int target, int timeout_ms)
    {
        auto deadline = std::chrono::steady_clock::now()
                        + std::chrono::milliseconds(timeout_ms);
        while (std::chrono::steady_clock::now() < deadline) {
            int s = -1;
            if (get_servo_state(ctrl_id_, s) == SUCCESS) {
                servo_status_ = s;          // 顺便刷新缓存
                if (s == target) return true;
            }
            std::this_thread::sleep_for(100ms);
        }
        return false;
    }

    // ==================== 其它 ====================

    void publish_connected()
    {
        std_msgs::msg::Bool msg;
        msg.data = connected_;
        connected_pub_->publish(msg);
    }


    void poll_and_publish_status()
    {
        nrc_msgs::msg::RobotStatus msg;
        msg.header.stamp = this->now();
        msg.connected = connected_;
        msg.servo_state = -1;
        msg.operation_mode = -1;
        msg.program_state = -1;

        if (connected_ && ctrl_id_ >= 0) {
            int v = -1;
            if (get_servo_state(ctrl_id_, v) == SUCCESS)         msg.servo_state = v;
            if (get_current_mode(ctrl_id_, v) == SUCCESS)        msg.operation_mode = v;
            if (get_robot_running_state(ctrl_id_, v) == SUCCESS) msg.program_state = v;
        }

        if (msg.servo_state != last_servo_) {
            last_servo_ = msg.servo_state;
            RCLCPP_INFO(this->get_logger(), "伺服状态: %d (%s)",
                        msg.servo_state, servo_state_name(msg.servo_state));
        }

        robot_status_pub_->publish(msg);
    }

    // ---------- 成员 ----------
    std::string ip_, ctrl_port_, state_port_;
    int state_interval_{100};

    std::atomic<SOCKETFD> ctrl_id_{-1};
    std::atomic<SOCKETFD> state_id_{-1};
    std::atomic<bool> connected_{false};
    std::atomic<bool> running_{false};
    std::atomic<int>  servo_status_{-1};
    std::atomic<int>  program_state_{-1};
    std::atomic<int> operation_mode_{-1};

    std::thread connect_thread_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr power_srv_;
    rclcpp::Publisher<nrc_msgs::msg::RobotStatus>::SharedPtr robot_status_pub_;
    rclcpp::TimerBase::SharedPtr status_query_timer_;
    int last_servo_{-99};
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NrcNode>();

    // Service 回调里有阻塞等待，必须用多线程执行器，否则会卡住其它回调
    rclcpp::executors::MultiThreadedExecutor exec;
    exec.add_node(node);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
