
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using namespace std::chrono_literals;

class PurePursuitController : public rclcpp::Node
{
public:
    PurePursuitController()
        : Node("controller_node")
    {
        wheelbase_ = declare_parameter("wheelbase", 1.2);
        lookahead_ = declare_parameter("lookahead", 1.5);
        speed_ = declare_parameter("speed", 0.5);
        max_steer_ = declare_parameter("max_steer", 0.5235987756);
        goal_tolerance_ = declare_parameter("goal_tolerance", 0.6);

        auto path_qos =
            rclcpp::QoS(1).reliable().transient_local();

        path_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/planned_path",
            path_qos,
            [this](nav_msgs::msg::Path::SharedPtr msg)
            {
                if (msg->poses.size() < 2) {
                    RCLCPP_WARN(get_logger(), "Path too short.");
                    return;
                }

                if (msg->header.frame_id != "map") {
                    RCLCPP_ERROR(get_logger(),
                        "Expected path frame: map");
                    return;
                }

                path_ = *msg;
                nearest_index_ = 0;
                goal_reached_ = false;
                has_path_ = true;

                RCLCPP_INFO(get_logger(),
                    "Received path with %zu poses",
                    path_.poses.size());
            });

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            10,
            [this](nav_msgs::msg::Odometry::SharedPtr msg)
            {
                if (msg->header.frame_id != "map") {
                    RCLCPP_ERROR_THROTTLE(
                        get_logger(), *get_clock(), 2000,
                        "Expected odometry frame: map");
                    return;
                }

                x_ = msg->pose.pose.position.x;
                y_ = msg->pose.pose.position.y;

                const auto & q = msg->pose.pose.orientation;

                // Yaw from quaternion
                yaw_ = std::atan2(
                    2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                );

                last_odom_time_ = now();
                has_odom_ = true;
            });

        drive_pub_ =
            create_publisher<
                ackermann_msgs::msg::AckermannDriveStamped>(
                "/drive", 10);

        target_pub_ =
            create_publisher<geometry_msgs::msg::PointStamped>(
                "/lookahead_point", 10);

        timer_ = create_wall_timer(
            50ms, [this]() { controlLoop(); });

        RCLCPP_INFO(
            get_logger(),
            "Pure Pursuit controller started.");
    }

private:
    struct Target
    {
        double x = 0.0;
        double y = 0.0;
        bool valid = false;
    };

    double distance(double ax, double ay,
                    double bx, double by) const
    {
        return std::hypot(ax - bx, ay - by);
    }

    Target findLookaheadPoint()
    {
        Target target;

        const auto & poses = path_.poses;
        const std::size_t size = poses.size();

        // Find nearest path point, without moving backward
        double best = std::numeric_limits<double>::infinity();

        for (std::size_t i = nearest_index_; i < size; ++i) {
            const auto & p = poses[i].pose.position;
            double d = distance(x_, y_, p.x, p.y);

            if (d < best) {
                best = d;
                nearest_index_ = i;
            }
        }

        // Start from the closest point and walk along path arc length
        std::size_t i = nearest_index_;
        double remaining = lookahead_;

        while (i + 1 < size) {
            const auto & a = poses[i].pose.position;
            const auto & b = poses[i + 1].pose.position;

            double segment = distance(a.x, a.y, b.x, b.y);

            if (segment < 1e-9) {
                ++i;
                continue;
            }

            if (segment >= remaining) {
                double t = remaining / segment;

                target.x = a.x + t * (b.x - a.x);
                target.y = a.y + t * (b.y - a.y);
                target.valid = true;
                return target;
            }

            remaining -= segment;
            ++i;
        }

        // Near the end: use final waypoint
        const auto & goal = poses.back().pose.position;

        target.x = goal.x;
        target.y = goal.y;
        target.valid = true;

        return target;
    }

    void publishDrive(double speed, double steering)
    {
        ackermann_msgs::msg::AckermannDriveStamped msg;

        msg.header.stamp = now();
        msg.header.frame_id = "base_link";

        msg.drive.speed = static_cast<float>(speed);
        msg.drive.steering_angle =
            static_cast<float>(steering);

        drive_pub_->publish(msg);
    }

    void controlLoop()
    {
        // Stop if missing a valid path or odometry
        if (!has_path_ || !has_odom_) {
            publishDrive(0.0, 0.0);
            return;
        }

        // Prevent driving with stale odometry
        if ((now() - last_odom_time_).seconds() > 0.5) {
            publishDrive(0.0, 0.0);
            return;
        }

        if (goal_reached_) {
            publishDrive(0.0, 0.0);
            return;
        }

        const auto & goal = path_.poses.back().pose.position;

        double goal_distance =
            distance(x_, y_, goal.x, goal.y);

        // First-stage position-based stopping criterion
        if (goal_distance < goal_tolerance_) {
            goal_reached_ = true;
            publishDrive(0.0, 0.0);

            RCLCPP_INFO(get_logger(),
                "Goal reached. Position error: %.3f m",
                goal_distance);
            return;
        }

        Target target = findLookaheadPoint();

        if (!target.valid) {
            publishDrive(0.0, 0.0);
            return;
        }

        const double dx = target.x - x_;
        const double dy = target.y - y_;

        const double target_angle = std::atan2(dy, dx);

        const double alpha = std::atan2(
            std::sin(target_angle - yaw_),
            std::cos(target_angle - yaw_)
        );

        // Actual distance to the selected target
        const double ld = std::max(
            0.1, std::hypot(dx, dy));

        double steering = std::atan2(
            2.0 * wheelbase_ * std::sin(alpha), ld);

        steering = std::clamp(
            steering, -max_steer_, max_steer_);

        publishDrive(speed_, steering);

        geometry_msgs::msg::PointStamped point;
        point.header.stamp = now();
        point.header.frame_id = "map";
        point.point.x = target.x;
        point.point.y = target.y;
        point.point.z = 0.1;
        target_pub_->publish(point);

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Nearest=%zu, steering=%.3f rad, goal_error=%.3f m",
            nearest_index_, steering, goal_distance);
    }

    double wheelbase_ = 1.2;
    double lookahead_ = 1.5;
    double speed_ = 0.5;
    double max_steer_ = 0.5235987756;
    double goal_tolerance_ = 0.6;

    double x_ = 0.0;
    double y_ = 0.0;
    double yaw_ = 0.0;

    bool has_path_ = false;
    bool has_odom_ = false;
    bool goal_reached_ = false;

    rclcpp::Time last_odom_time_;
    nav_msgs::msg::Path path_;
    std::size_t nearest_index_ = 0;

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr
        path_sub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
        odom_sub_;

    rclcpp::Publisher<
        ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr
        drive_pub_;

    rclcpp::Publisher<
        geometry_msgs::msg::PointStamped>::SharedPtr
        target_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitController>());
    rclcpp::shutdown();
    return 0;
}
