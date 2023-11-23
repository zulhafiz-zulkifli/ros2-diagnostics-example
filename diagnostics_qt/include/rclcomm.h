#ifndef RCLCOMM_H
#define RCLCOMM_H
#include <QObject>
#include <QThread>
#include <iostream>
#include <cmath>
#include <iomanip> // For std::fixed and std::setprecision
#include <sstream> // For std::stringstream
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include "diagnostic_msgs/msg/diagnostic_array.hpp"

#include <thread>
#include <rclcpp/executors/multi_threaded_executor.hpp>

class rclcomm:public QThread
{
    Q_OBJECT
public:
    rclcomm();
    void diagnostics_agg_cb(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);
protected:
    void run();
private:
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_agg_sub;
    std::shared_ptr<rclcpp::Node> node;
signals:
    void emitDiagnostics(QString);
    void emitUpdateTreeWidget(QString,QString,QString);
    void emitClearTreeWidget();
public slots:
};
#endif // RCLCOMM_H
