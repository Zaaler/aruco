#include <memory>

#include "aruco/Aruco.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  // Force flush of the stdout buffer.
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::WallRate loop_rate(20);
  rclcpp::init(argc, argv);

  // Create an executor that will be responsible for execution of callbacks for a set of nodes.
  // With this version, all callbacks will be called from within this thread (the main one).
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;

  //ROS node instance
  auto opts = rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true);
  auto aruco = std::make_shared<aruco::Aruco>(opts);
  exec.add_node(aruco);

  // spin will block until work comes in, execute work as it becomes available, and keep blocking.
  // It will only be interrupted by Ctrl-C.
  while (rclcpp::ok()) {
    try {
      exec.spin_once();
    } catch (const rclcpp::exceptions::RCLError & e) {
      RCLCPP_ERROR(
        rclcpp::get_logger(""),
        "unexpectedly failed with %s",
        e.what());
    }
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
