#pragma once
#include "rclcpp/rclcpp.hpp"

class Main : public rclcpp::Node
{
public:
    explicit Main(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    ~Main() {}
private:
};