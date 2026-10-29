
#include <cmath>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "VehicleModel.h"
#include "GridMap.h"
#include "HybridAStar.h"

class PlannerNode : public rclcpp::Node
{
public:
    PlannerNode() : Node("ackermann_planner")
    {
        auto qos = rclcpp::QoS(1)
            .reliable()
            .transient_local();

        path_pub_ =
            create_publisher<nav_msgs::msg::Path>(
                "/planned_path", qos);

        map_pub_ =
            create_publisher<nav_msgs::msg::OccupancyGrid>(
                "/map", qos);

        // 第一阶段：20m × 20m 空旷场景
        robot::GridMap map;
        map.obstacles.clear();

        robot::VehicleModel vehicle(1.2);

        robot::State start{2.0, 2.0, 0.0};
        robot::State goal{17.0, 16.0, robot::PI / 4.0};

        robot::HybridConfig config;

        RCLCPP_INFO(
            get_logger(),
            "Starting Hybrid A* planning..."
        );

        auto result = robot::planHybridAStar(
            map, vehicle, start, goal, config);

        if (!result.success) {
            RCLCPP_ERROR(
                get_logger(),
                "Hybrid A* planning failed");
            return;
        }

        RCLCPP_INFO(
            get_logger(),
            "Planning success: %zu points, length %.2f m",
            result.path.size(), result.length);

        publishMap(map);
        publishPath(result);
    }

private:
    void publishPath(const robot::PlanResult & result)
    {
        nav_msgs::msg::Path msg;
        msg.header.frame_id = "map";
        msg.header.stamp = now();

        for (const auto & s : result.path) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = msg.header;

            pose.pose.position.x = s.x;
            pose.pose.position.y = s.y;
            pose.pose.position.z = 0.0;

            pose.pose.orientation.z = std::sin(s.theta / 2.0);
            pose.pose.orientation.w = std::cos(s.theta / 2.0);

            msg.poses.push_back(pose);
        }

        path_pub_->publish(msg);
    }

    void publishMap(const robot::GridMap & map)
    {
        nav_msgs::msg::OccupancyGrid msg;
        msg.header.frame_id = "map";
        msg.header.stamp = now();

        constexpr double resolution = 0.1;

        msg.info.resolution = resolution;
        msg.info.width =
            static_cast<unsigned int>(map.width / resolution);
        msg.info.height =
            static_cast<unsigned int>(map.height / resolution);
        msg.info.map_load_time = now();
        msg.info.origin.orientation.w = 1.0;

        msg.data.assign(
            msg.info.width * msg.info.height, 0);

        // 根据 GridMap 的障碍物判断每个栅格
        for (unsigned int y = 0; y < msg.info.height; ++y) {
            for (unsigned int x = 0; x < msg.info.width; ++x) {
                double wx = (x + 0.5) * resolution;
                double wy = (y + 0.5) * resolution;

                if (!map.pointFree(wx, wy)) {
                    msg.data[y * msg.info.width + x] = 100;
                }
            }
        }

        map_pub_->publish(msg);
    }

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerNode>());
    rclcpp::shutdown();
    return 0;
}
