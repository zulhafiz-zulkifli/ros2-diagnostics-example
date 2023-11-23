#include "rclcomm.h"

rclcomm::rclcomm()  {
    int argc=0;
    char **argv=NULL;
    rclcpp::init(argc,argv);
    node=rclcpp::Node::make_shared("diagnostics_qt");

    diagnostics_agg_sub = node->create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics_agg", 10,std::bind(&rclcomm::diagnostics_agg_cb, this, std::placeholders::_1));

    this->start();
}
void rclcomm::run(){
    
    // Create a multithreaded executor with a certain number of threads
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    
    
    rclcpp::WallRate loop_rate(10);
    while (rclcpp::ok())
    {
        // Spin
        // rclcpp::spin_some(node);
        loop_rate.sleep();

        executor.spin_some();
    }
    rclcpp::shutdown();
}


void rclcomm::diagnostics_agg_cb(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{
    emitClearTreeWidget();
    for (const auto& status : msg->status) {
        
        std::string levelString;
        if (status.level == diagnostic_msgs::msg::DiagnosticStatus::OK) {
            levelString = "OK";
        } else if (status.level == diagnostic_msgs::msg::DiagnosticStatus::WARN) {
            levelString = "WARN";
        } else if (status.level == diagnostic_msgs::msg::DiagnosticStatus::ERROR) {
            levelString = "ERROR";
        } else if (status.level == diagnostic_msgs::msg::DiagnosticStatus::STALE) {
            levelString = "STALE";
        } else {
            levelString = "STALE";
        }

        emitUpdateTreeWidget(QString::fromStdString(status.name), QString::fromStdString(status.message), QString::fromStdString(levelString));
    }

    

    // std::string diagnostics_agg = msg->data;
    // emitStatus(QString::fromStdString(diagnostics_agg));
    
}