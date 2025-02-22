#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <cmath>
#include <vector>

class Stacker : public rclcpp::Node
{
public:
    Stacker() : Node("stacker_node")
    {
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/command_joint_trajectory", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&Stacker::joint_state_callback, this, std::placeholders::_1));
        client_ = this->create_client<std_srvs::srv::Trigger>("/spawn_object");

        joint_order_ = {"joint1", "joint2", "joint3", "joint4", "gripper"};
        joint_states_ = {0.0, 0.0, 0.0, 0.0, 0.0};
        home_point_ = {0.270, 0.0, 0.205, 0.0};
        pick_point_1_ = {0.21, 0.21, 0.05, 0.0};
        pick_point_2_ = {0.21, 0.21, 0.025, 0.0};
        place_point_1_ = {0.2, -0.2, 0.05, 0.0};
        place_point_2_ = {0.2, -0.2, 0.02, 0.0};
        place_point_3_ = {0.2, -0.2, 0.10, 0.0};
        place_point_4_ = {0.2, -0.2, 0.076, 0.0};

        GRIPPER_OPEN = 1.2;
        GRIPPER_CLOSE = 0.0;
        gripper_state_ = GRIPPER_CLOSE;

        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
    }

    void spawn_object()
    {
        auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
        auto future = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Service called successfully");
            RCLCPP_INFO(this->get_logger(), "Response: %s", response->message.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed or timed out");
        }
    }

    std::vector<double> inverse_kinematics(double x, double y, double z, double phi)
    {
        const double a0 = 0.012, a1 = 0.077, a2 = 0.130, a3 = 0.124, a4 = 0.126, delta = 1.385;

        double x_dash = x - a0;
        double z_dash = z - a1;
        double r = std::sqrt(y * y + x_dash * x_dash);
        double wr = r - a4 * std::cos(-phi);
        double wz = z_dash - a4 * std::sin(-phi);

        double D = (-(a2 * a2) - (a3 * a3) + (wr * wr) + (wz * wz)) / (2 * a2 * a3);

        if (std::abs(D) > 1) {
            RCLCPP_ERROR(this->get_logger(), "D is out of range: %f", D);
            return {0.0, NAN, NAN, NAN};  // Return grip state for consistency
        }

        double theta1 = std::atan2(y, x_dash);
        double theta3_dash = std::atan2(std::sqrt(1 - D * D), D);
        double theta2_dash = -std::atan2(wz, wr) - std::atan2(a3 * std::sin(theta3_dash), a2 + a3 * std::cos(theta3_dash));
        double theta4 = phi - (theta2_dash + theta3_dash);

        double theta2 = theta2_dash + delta;
        double theta3 = theta3_dash - delta;

        return {theta1, theta2, theta3, theta4};
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        for (size_t i = 0; i < joint_order_.size(); ++i) {
            auto it = std::find(msg->name.begin(), msg->name.end(), joint_order_[i]);
            if (it != msg->name.end()) {
                joint_states_[i] = msg->position[std::distance(msg->name.begin(), it)];
            }
        }
    }

    void publish_trajectory(const std::vector<double>& joint_positions)
    {
        RCLCPP_INFO(this->get_logger(), "Publishing trajectory: [%f, %f, %f, %f, %f]", joint_positions[0], joint_positions[1],
                    joint_positions[2], joint_positions[3], gripper_state_);
        trajectory_msgs::msg::JointTrajectory trajectory_msg;
        trajectory_msg.joint_names = joint_order_;

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions = joint_positions;
        point.positions.push_back(gripper_state_);
        point.velocities = {0.0, 0.0, 0.0, 0.0, 0.0};
        point.time_from_start.sec = 1;
        point.time_from_start.nanosec = 0;

        trajectory_msg.points.push_back(point);
        publisher_->publish(trajectory_msg);
        RCLCPP_INFO(this->get_logger(), "Published JointTrajectory message");
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    void move(const std::vector<double>& position)
    {
        RCLCPP_INFO(this->get_logger(), "Moving to position: [%f, %f, %f, %f]", position[0], position[1], position[2], position[3]);
        auto joint_target = inverse_kinematics(position[0], position[1], position[2], position[3]);
        publish_trajectory(joint_target);
    }

    void open_gripper()
    {
        RCLCPP_INFO(this->get_logger(), "Opening gripper");
        gripper_state_ = GRIPPER_OPEN;
        publish_trajectory(std::vector<double>(joint_states_.begin(), joint_states_.begin() + 4));
    }

    void close_gripper()
    {
        RCLCPP_INFO(this->get_logger(), "Closing gripper");
        gripper_state_ = GRIPPER_CLOSE;
        publish_trajectory(std::vector<double>(joint_states_.begin(), joint_states_.begin() + 4));
    }

    void start_stacking()
    {
        move(home_point_);
        std::vector<std::pair<std::string, std::function<void()>>> steps = {
            {"Spawn Object", [this]() { spawn_object(); }},
            {"Open gripper", [this]() { open_gripper(); }},
            {"pick_1", [this]() { move(pick_point_1_); }},
            {"pick_2", [this]() { move(pick_point_2_); }},
            {"Close gripper", [this]() { close_gripper(); }},
            {"home", [this]() { move(home_point_); }},
            {"place_1", [this]() { move(place_point_1_); }},
            {"place_2", [this]() { move(place_point_2_); }},
            {"Open gripper", [this]() { open_gripper(); }},
            {"place_1", [this]() { move(place_point_1_); }},
            {"Go to home position", [this]() { move(home_point_); }},
            {"Spawn Object", [this]() { spawn_object(); }},
            {"Open gripper", [this]() { open_gripper(); }},
            {"pick_1", [this]() { move(pick_point_1_); }},
            {"pick_2", [this]() { move(pick_point_2_); }},
            {"Close gripper", [this]() { close_gripper(); }},
            {"home", [this]() { move(home_point_); }},
            {"place_1", [this]() { move(place_point_3_); }},
            {"place_2", [this]() { move(place_point_4_); }},
            {"Open gripper", [this]() { open_gripper(); }},
            {"place_1", [this]() { move(place_point_3_); }},
            {"Go to home position", [this]() { move(home_point_); }}};

        for (auto& step : steps) {
            RCLCPP_INFO(this->get_logger(), "Step: %s", step.first.c_str());
            step.second();
            rclcpp::spin_some(this->get_node_base_interface());
        }
    }

private:
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client_;
    std::vector<std::string> joint_order_;
    std::vector<double> joint_states_;
    std::vector<double> home_point_;
    std::vector<double> pick_point_1_;
    std::vector<double> pick_point_2_;
    std::vector<double> place_point_1_;
    std::vector<double> place_point_2_;
    std::vector<double> place_point_3_;
    std::vector<double> place_point_4_;
    double GRIPPER_OPEN;
    double GRIPPER_CLOSE;
    double gripper_state_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto stacker = std::make_shared<Stacker>();
    stacker->start_stacking();
    rclcpp::shutdown();
    return 0;
}