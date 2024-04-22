// cmd_vel_remap_node.cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("cmd_vel_raw");

    // Create a publisher for the remapped topic
    auto remapped_pub = node->create_publisher<nav_msgs::msg::Odometry>(
        "odom", 3);

    // Create lambda
    auto cmd_clb = [remapped_pub](const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        auto new_odom_msg = msg;
        if (msg->child_frame_id[0] == '/'){
            new_odom_msg->child_frame_id.erase(new_odom_msg->child_frame_id.begin());
        }
        if (msg->header.frame_id[0] == '/'){
            msg->header.frame_id.erase(msg->header.frame_id.begin());
        }
        remapped_pub->publish(*msg);
    };

    // Create a subscriber for the original /cmd_vel topic
    auto cmd_vel_sub = node->create_subscription<nav_msgs::msg::Odometry>(
        "odom_raw", 3,
        cmd_clb
    );

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
