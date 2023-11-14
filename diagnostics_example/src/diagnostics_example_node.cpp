#include "diagnostics_example/diagnostics_example_node.h"

Main::Main(const rclcpp::NodeOptions &options): Node("diagnostics_example", options){
	RCLCPP_INFO(get_logger(),"diagnostics_example node has started.");
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Main>());
    rclcpp::shutdown();
    return 0;
}