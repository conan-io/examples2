#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <yaml-cpp/yaml.h>

using NavigateToPose = nav2_msgs::action::NavigateToPose;


class YamlNavigationNode : public rclcpp::Node {
public:
    YamlNavigationNode(const std::string &yaml_file_path) : Node("yaml_navigation_node") {
        // Create action client
        action_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // Read locations from YAML file
        RCLCPP_INFO(this->get_logger(), "Reading locations from YAML...");
        if (!loadLocations(yaml_file_path)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load locations.");
            return;
        }

        if (locations_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "No locations found in the YAML file.");
            return;
        }

        sendAllGoals();
    }

private:
    struct Location {
        std::string name;
        double x;
        double y;
    };

    std::vector<Location> locations_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;

    bool loadLocations(const std::string &file_path) {
        try {
            YAML::Node yaml_file = YAML::LoadFile(file_path);
            for (const auto &node : yaml_file["locations"]) {
                Location location;
                location.name = node["name"].as<std::string>();
                location.x = node["x"].as<double>();
                location.y = node["y"].as<double>();
                locations_.emplace_back(location);
            }
            return true;
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Error parsing YAML: %s", e.what());
            return false;
        }
    }

    void sendAllGoals() {
        for (const auto &location : locations_) {
            RCLCPP_INFO(this->get_logger(), "Sending %s goal (%.2f, %.2f) to robot", location.name.c_str(), location.x, location.y);

            auto goal_msg = NavigateToPose::Goal();
            goal_msg.pose.header.frame_id = "map";
            goal_msg.pose.header.stamp = this->now();
            goal_msg.pose.pose.position.x = location.x;
            goal_msg.pose.pose.position.y = location.y;
            goal_msg.pose.pose.orientation.w = 1.0;

            action_client_->async_send_goal(goal_msg);
        }

        RCLCPP_INFO(this->get_logger(), "All goals have been sent.");
    }
};


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    if (argc < 2) {
        RCLCPP_ERROR(rclcpp::get_logger("yaml_navigation_node"), "Usage: yaml_navigation_node <yaml_file_path>");
        return 1;
    }

    std::string yaml_file_path = argv[1];
    std::make_shared<YamlNavigationNode>(yaml_file_path);

    rclcpp::shutdown();
    return 0;
}
