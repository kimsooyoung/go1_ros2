/*********************************************************************
 * Software License
 *  Copyright (c) 2022, WeGo Robotices Co., Ltd. && Road Balance Co. All Rights
 *Reserved.
 *********************************************************************/

#include "unitree_twist_cmd/UnitreeTwistCmd.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<UnitreeTwistCmd>());
  rclcpp::shutdown();

  return 0;
}
