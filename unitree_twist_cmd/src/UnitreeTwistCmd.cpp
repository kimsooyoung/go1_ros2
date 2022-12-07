/*********************************************************************
 * Software License
 *  Copyright (c) 2022, WeGo Robotices Co., Ltd. && Road Balance Co. All Rights
 *Reserved.
 *********************************************************************/

#include "unitree_twist_cmd/UnitreeTwistCmd.hpp"
#include <stdio.h>

UnitreeTwistCmd::UnitreeTwistCmd() : Node("unitree_twist_cmd") {
  /// Publisher, Subscriber, Timer 생성
  // twist_sub_ =
  //     nh->subscribe("/cmd_vel", 10, &UnitreeTwistCmd::twistSubCallback,
  //     this);
  // control_mode_sub_ =
  //     nh->subscribe("/unitree_control_mode", 10,
  //                   &UnitreeTwistCmd::controlModeSubCallback, this);
  // body_height_sub_ =
  //     nh->subscribe("/unitree_body_height", 10,
  //                   &UnitreeTwistCmd::bodyHeightSubCallback, this);
  // foot_height_sub_ =
  //     nh->subscribe("/unitree_foot_height", 10,
  //                   &UnitreeTwistCmd::footHeightSubCallback, this);

  twist_sub_ = this->create_subscription<Twist>(
      "cmd_vel", 10, std::bind(&UnitreeTwistCmd::twistSubCallback, this, _1));

  control_mode_sub_ = this->create_subscription<UInt8>(
      "unitree_control_mode", 10,
      std::bind(&UnitreeTwistCmd::controlModeSubCallback, this, _1));

  body_height_sub_ = this->create_subscription<UInt8>(
      "unitree_body_height", 10,
      std::bind(&UnitreeTwistCmd::bodyHeightSubCallback, this, _1));

  foot_height_sub_ = this->create_subscription<UInt8>(
      "unitree_foot_height", 10,
      std::bind(&UnitreeTwistCmd::footHeightSubCallback, this, _1));

  // unitree_cmd_pub_ = nh->advertise<unitree_legged_msgs::HighCmd>(
  //     "/untiree_quadrupped_high_cmd", 10);

  unitree_cmd_pub_ = this->create_publisher<HighCmd>("high_cmd", 10);

  // control_timer_ = nh->createTimer(ros::Duration(0.02),
  //                                  &UnitreeTwistCmd::timerCallback, this);

  control_timer_ = this->create_wall_timer(
      20ms, std::bind(&UnitreeTwistCmd::timerCallback, this));

  // /// Parameter 설정을 위해 추가 NodeHandler 생성
  // ros::NodeHandle nh_private("~");

  // nh_private.param<int>("default_mode", default_mode_, 1);
  // ROS_INFO("default_mode : %d", default_mode_);

  // nh_private.param<bool>("verbose", verbose_, true);
  // ROS_INFO("verbose : %s", verbose_ ? "true" : "false");

  default_mode_ = this->declare_parameter("default_mode", 1);
  verbose_ = this->declare_parameter("verbose", true);

  printf("default_mode : %d\n", default_mode_);
  printf("verbose : %s\n", verbose_ ? "true" : "false");

  // HIGHLEVEL Control Flag is 238
  unitree_cmd_msg_.head[0] = 0xFE;
  unitree_cmd_msg_.head[1] = 0xEF;
  unitree_cmd_msg_.level_flag = 238;

  /// parameter에 의해 초기 1회 mode가 선택된다.
  selectMode(default_mode_);
}

void UnitreeTwistCmd::twistSubCallback(const Twist::SharedPtr twist) {
  /// position mode라는 가정 하에 twist msg를 변환함
  unitree_cmd_msg_.velocity[0] = twist->linear.x;
  unitree_cmd_msg_.velocity[1] = twist->linear.y;

  unitree_cmd_msg_.yaw_speed = twist->angular.z;
}

bool UnitreeTwistCmd::selectMode(const uint8_t &mode) {
  /// mode 0 => Standing Mode
  /// mode 1 => Walking Mode
  /// mode 2 => Running Mode
  /// mode 3 => Stairs Climbing Mode

  switch (mode) {
  case 0:
    unitree_cmd_msg_.mode = 1;
    unitree_cmd_msg_.gait_type = 1;
    break;
  case 1:
    unitree_cmd_msg_.mode = 2;
    unitree_cmd_msg_.gait_type = 1;
    break;
  case 2:
    unitree_cmd_msg_.mode = 2;
    unitree_cmd_msg_.gait_type = 2;
    break;
  case 3:
    unitree_cmd_msg_.mode = 2;
    unitree_cmd_msg_.gait_type = 3;
    break;
  default:
    break;
  }

  return true;
}

void UnitreeTwistCmd::controlModeSubCallback(const UInt8::SharedPtr msg) {
  /// mode 선택 후 디버깅 메세지 출력
  selectMode(msg->data);

  if (verbose_)
    RCLCPP_INFO(this->get_logger(), "Control Mode Changed to %d", msg->data);
}

void UnitreeTwistCmd::bodyHeightSubCallback(const Float32::SharedPtr msg) {
  /// bodyHeight 변경 후 디버깅 메세지 출력
  unitree_cmd_msg_.body_height = msg->data;

  if (verbose_)
    RCLCPP_INFO(this->get_logger(), "Body Height Changed to %f", msg->data);
}

void UnitreeTwistCmd::footHeightSubCallback(const Float32::SharedPtr msg) {
  /// footRaiseHeight 변경 후 디버깅 메세지 출력
  unitree_cmd_msg_.foot_raise_height = msg->data;

  if (verbose_)
    RCLCPP_INFO(this->get_logger(), "Foot Raise Height Changed to %f",
                msg->data);
}

void UnitreeTwistCmd::timerCallback() {
  unitree_cmd_pub_->publish(unitree_cmd_msg_);
}