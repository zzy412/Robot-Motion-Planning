
#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "VehicleModel.h"

using namespace std::chrono_literals;

class VehicleSimNode : public rclcpp::Node
{
public:
    VehicleSimNode()
        : Node("vehicle_sim_node"),
          vehicle_(1.2)
    {
        // 与当前 Hybrid A* 的起点一致
        state_ = robot::State{2.0, 2.0, 0.0};

        // 仿真参数
        dt_ = 0.05;
        max_steering_ = robot::PI / 6.0;
        max_steering_rate_ = robot::PI / 6.0;

        // 如果 0.5 秒没有收到指令，车辆停车
        command_timeout_ = 0.5;

        drive_sub_ = create_subscription<
            ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive",
            rclcpp::QoS(10),
            [this](
                const ackermann_msgs::msg::
                    AckermannDriveStamped::SharedPtr msg)
            {
                target_speed_ =
                    std::clamp(
                        static_cast<double>(msg->drive.speed),
                        -1.0, 1.0);

                target_steering_ =
                    std::clamp(
                        static_cast<double>(
                            msg->drive.steering_angle),
                        -max_steering_,
                        max_steering_);

                last_command_time_ = now();
                received_command_ = true;
            });

        odom_pub_ = create_publisher<
            nav_msgs::msg::Odometry>("/odom", 10);

        auto path_qos =
            rclcpp::QoS(1).reliable().transient_local();

        path_pub_ = create_publisher<
            nav_msgs::msg::Path>(
            "/actual_path", path_qos);

        marker_pub_ = create_publisher<
            visualization_msgs::msg::Marker>(
            "/vehicle_marker", 10);

        tf_broadcaster_ =
            std::make_unique<
                tf2_ros::TransformBroadcaster>(*this);

        actual_path_.header.frame_id = "map";

        last_command_time_ = now();

        timer_ = create_wall_timer(
            50ms,
            [this]() { updateVehicle(); });

        RCLCPP_INFO(
            get_logger(),
            "Vehicle simulator started at (2, 2, 0).");
    }

private:
    geometry_msgs::msg::Quaternion
    makeQuaternion(double yaw) const
    {
        geometry_msgs::msg::Quaternion q;

        q.x = 0.0;
        q.y = 0.0;
        q.z = std::sin(yaw / 2.0);
        q.w = std::cos(yaw / 2.0);

        return q;
    }

    void updateVehicle()
    {
        const auto stamp = now();

        double speed = 0.0;

        // 只执行最近 0.5 秒内收到的指令
        if (received_command_ &&
            (stamp - last_command_time_).seconds()
                <= command_timeout_)
        {
            speed = target_speed_;
        }

        // 模拟有限转向速率
        const double max_change =
            max_steering_rate_ * dt_;

        const double steering_error =
            target_steering_ - actual_steering_;

        actual_steering_ += std::clamp(
            steering_error,
            -max_change,
            max_change);

        // 调用原有 C++ 车辆运动学模型
        state_ = vehicle_.step(
            state_,
            speed,
            actual_steering_,
            dt_);

        publishOdometry(stamp, speed);
        publishTF(stamp);
        publishMarker(stamp);
        publishActualPath(stamp);
    }

    void publishOdometry(
        const rclcpp::Time & stamp,
        double speed)
    {
        nav_msgs::msg::Odometry msg;

        msg.header.stamp = stamp;
        msg.header.frame_id = "map";
        msg.child_frame_id = "base_link";

        msg.pose.pose.position.x = state_.x;
        msg.pose.pose.position.y = state_.y;
        msg.pose.pose.position.z = 0.0;

        msg.pose.pose.orientation =
            makeQuaternion(state_.theta);

        msg.twist.twist.linear.x = speed;

        msg.twist.twist.angular.z =
            speed * std::tan(actual_steering_)
            / vehicle_.wheelbase();

        odom_pub_->publish(msg);
    }

    void publishTF(const rclcpp::Time & stamp)
    {
        geometry_msgs::msg::TransformStamped tf;

        tf.header.stamp = stamp;
        tf.header.frame_id = "map";
        tf.child_frame_id = "base_link";

        tf.transform.translation.x = state_.x;
        tf.transform.translation.y = state_.y;
        tf.transform.translation.z = 0.0;

        tf.transform.rotation =
            makeQuaternion(state_.theta);

        tf_broadcaster_->sendTransform(tf);
    }

    void publishMarker(const rclcpp::Time & stamp)
    {
        visualization_msgs::msg::Marker msg;

        msg.header.frame_id = "base_link";
        msg.header.stamp = stamp;

        msg.ns = "ackermann_vehicle";
        msg.id = 0;

        msg.type =
            visualization_msgs::msg::Marker::CUBE;

        msg.action =
            visualization_msgs::msg::Marker::ADD;

        msg.pose.orientation.w = 1.0;

        msg.scale.x = 1.8;
        msg.scale.y = 0.9;
        msg.scale.z = 0.35;

        msg.color.r = 0.1f;
        msg.color.g = 0.4f;
        msg.color.b = 1.0f;
        msg.color.a = 1.0f;

        marker_pub_->publish(msg);
    }

    void publishActualPath(const rclcpp::Time & stamp)
    {
        geometry_msgs::msg::PoseStamped pose;

        pose.header.frame_id = "map";
        pose.header.stamp = stamp;

        pose.pose.position.x = state_.x;
        pose.pose.position.y = state_.y;
        pose.pose.position.z = 0.02;

        pose.pose.orientation =
            makeQuaternion(state_.theta);

        actual_path_.header.stamp = stamp;

        actual_path_.poses.push_back(pose);

        // 限制历史路径长度
        if (actual_path_.poses.size() > 5000)
        {
            actual_path_.poses.erase(
                actual_path_.poses.begin());
        }

        path_pub_->publish(actual_path_);
    }

    robot::VehicleModel vehicle_;
    robot::State state_;

    double dt_ = 0.05;
    double max_steering_ = 0.0;
    double max_steering_rate_ = 0.0;
    double command_timeout_ = 0.5;

    double target_speed_ = 0.0;
    double target_steering_ = 0.0;
    double actual_steering_ = 0.0;

    bool received_command_ = false;

    rclcpp::Time last_command_time_;

    nav_msgs::msg::Path actual_path_;

    rclcpp::Subscription<
        ackermann_msgs::msg::
            AckermannDriveStamped>::SharedPtr drive_sub_;

    rclcpp::Publisher<
        nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

    rclcpp::Publisher<
        nav_msgs::msg::Path>::SharedPtr path_pub_;

    rclcpp::Publisher<
        visualization_msgs::msg::Marker>::SharedPtr
            marker_pub_;

    std::unique_ptr<
        tf2_ros::TransformBroadcaster> tf_broadcaster_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(
        std::make_shared<VehicleSimNode>());

    rclcpp::shutdown();

    return 0;
}
