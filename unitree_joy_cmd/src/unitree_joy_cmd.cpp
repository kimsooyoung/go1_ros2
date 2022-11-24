/*********************************************************************
 * Software License
 *  Copyright (c) 2022, WeGo Robotices Co., Ltd. && Road Balance Co. All Rights
 *Reserved.
 *********************************************************************/

#include "unitree_joy_cmd/UnitreeJoyCmd.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<UnitreeJoyCmd>());
  rclcpp::shutdown();

  return 0;
}
