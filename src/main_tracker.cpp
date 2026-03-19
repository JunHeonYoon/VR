#include <rclcpp/rclcpp.hpp>
#include "tracker.h"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("HMD");

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);

  try
  {
    HMD hmdSystem(argc, argv);   // 기존 생성자 유지해도 됨(다만 init에서 rclcpp::init은 제거해야 함)
    hmdSystem.set_ros_node(node);

    RCLCPP_INFO(node->get_logger(), "HMD::init() start");
    hmdSystem.init();
    RCLCPP_INFO(node->get_logger(), "HMD::init() done");

    // HMD 루프는 별도 함수로 돌리고, ROS 스핀은 main에서 책임
    // 1) HMD가 내부 루프를 돌린다면, 그 루프에서 spin을 하지 않도록(위에서 제거함)
    // 2) main에서 spin_some을 함께 돌려준다.

    rclcpp::WallRate rate(130);
    while (rclcpp::ok())
    {
      // HMD 쪽 publish/update를 1스텝 실행하는 함수가 있으면 제일 좋음.
      // 지금 구조가 RunMainLoop()면 그 안에 while이 있어서 아래 방식은 못 씀.

      // 가장 간단한 구조:
      // - HMD::RunMainLoop()의 while을 없애고 step() 함수로 바꾸는 게 베스트인데,
      //   지금은 최소 수정으로 가기 위해 아래처럼 스레드로 RunMainLoop를 돌린다.

      break;
    }

    // 최소 변경안: HMD 루프를 스레드로 실행하고, main은 exec.spin()
    std::thread hmd_thread([&]() {
      hmdSystem.RunMainLoop();     // 여기서 spin_some()은 제거된 상태여야 함
    });

    exec.spin();                   // 여기서 ROS 콜백 처리 (지금은 subscription 없어도 안전)
    if (hmd_thread.joinable()) hmd_thread.join();

    RCLCPP_WARN(node->get_logger(), "tracker exiting normally");
  }
  catch (const std::exception &e)
  {
    RCLCPP_FATAL(node->get_logger(), "Exception: %s", e.what());
  }
  catch (...)
  {
    RCLCPP_FATAL(node->get_logger(), "Unknown exception");
  }

  rclcpp::shutdown();
  return 0;
}
