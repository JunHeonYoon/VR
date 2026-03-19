// #include "widget.h"
#include "overlay.h"
#include <QtWidgets/QApplication>

#include <rclcpp/rclcpp.hpp>
#include <thread>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    QApplication a(argc, argv);
    std::cout << "ros2 initialized" << std::endl;

    OverlayWidget *pOverlayWidget = new OverlayWidget;

    OverlayController::SharedInstance()->Init();
    OverlayController::SharedInstance()->SetWidget(pOverlayWidget);

    // Run ROS2 spinning/subscriptions in a separate thread (WInit blocks with rclcpp::spin)
    std::thread ros_thread([&]() {
        pOverlayWidget->WInit();
    });

    int ret = a.exec();

    rclcpp::shutdown();
    if (ros_thread.joinable()) ros_thread.join();

    return ret;
}
