#include <rclcpp/rclcpp.hpp>
#include "opponent_estimator/opponent_estimator.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::cout << "Starting opponent estimator" << std::endl;
  rclcpp::spin(std::make_shared<OpponentEstimator>());
  rclcpp::shutdown();
  return 0;
}
