/*********************************************************************
 * Software License
 *  Copyright (c) 2022, WeGo Robotices Co., Ltd. && Road Balance Co. All Rights
 *Reserved.
 *********************************************************************/

#include "unitree_joy_cmd/UnitreeJoyCmd.hpp"
#include <stdio.h>

UnitreeJoyCmd::UnitreeJoyCmd() : Node("unitree_joy_cmd") {
  /// Publisher, Subscriber, Timer 생성
  unitree_cmd_pub_ = this->create_publisher<HighCmd>("high_cmd", 10);
  joy_sub_ = this->create_subscription<Joy>(
      "joy", 10, std::bind(&UnitreeJoyCmd::subCallback, this, _1));
  control_timer_ = this->create_wall_timer(
      20ms, std::bind(&UnitreeJoyCmd::timerCallback, this));

  roll_gain_ = this->declare_parameter("roll_gain", 0.8);
  pitch_gain_ = this->declare_parameter("pitch_gain", 0.5);
  yaw_gain_ = this->declare_parameter("yaw_gain", 0.3);

  bodyheight_gain_ = this->declare_parameter("bodyheight_gain", 0.2);
  speed_gain_ = this->declare_parameter("speed_gain", 1.0);

  // HIGHLEVEL Control Flag is 238
  unitree_cmd_msg_.head[0] = 0xFE;
  unitree_cmd_msg_.head[1] = 0xEF;
  unitree_cmd_msg_.level_flag = 238;

  printf("roll_gain : %f\n", roll_gain_);
  printf("pitch_gain : %f\n", pitch_gain_);
  printf("yaw_gain : %f\n", yaw_gain_);
  printf("bodyheight_gain : %f\n", bodyheight_gain_);
  printf("speed_gain : %f\n", speed_gain_);
}

void UnitreeJoyCmd::timerCallback() {
  unitree_cmd_pub_->publish(unitree_cmd_msg_);
}

void UnitreeJoyCmd::subCallback(const sensor_msgs::msg::Joy::SharedPtr data) {
  // Assume JoyStick is on "X" mode
  joy_keys.left_updown = data->axes[1];
  joy_keys.left_leftright = data->axes[0];
  joy_keys.right_updown = data->axes[4];
  joy_keys.right_leftright = data->axes[3];

  // +1 btn up / -1 btn down
  joy_keys.btn_updown = data->axes[7];
  // +1 btn left / -1 btn right
  joy_keys.btn_leftright = data->axes[6];

  joy_keys.btn_a = isTrue(data->buttons[0]);
  joy_keys.btn_b = isTrue(data->buttons[1]);
  joy_keys.btn_x = isTrue(data->buttons[2]);
  joy_keys.btn_y = isTrue(data->buttons[3]);

  joy_keys.btn_LB = isTrue(data->buttons[4]);
  joy_keys.btn_RB = isTrue(data->buttons[5]);

  joy_keys.btn_back = isTrue(data->buttons[6]);
  joy_keys.btn_start = isTrue(data->buttons[7]);

  parseControlMode();

  /// X mode의 경우 roll, pitch, yaw, bodyHeight 값만을 파싱함
  if (contorl_mode_ == ContorlMode::X_MODE) {
    parsePoseMode();
  } else {
    parseBodyHeight();
    parseFootRaiseHeight();
    parsePositionMode();
    parseVelocity();
  }

  /// 조이스틱 키값 갱신
  prev_joy_keys = joy_keys;
}

bool UnitreeJoyCmd::parsePoseMode() {
  unitree_cmd_msg_.mode = 1;

  unitree_cmd_msg_.velocity[0] = 0.0;
  unitree_cmd_msg_.velocity[1] = 0.0;
  unitree_cmd_msg_.yaw_speed = 0.0;

  unitree_cmd_msg_.euler[0] = joy_keys.left_leftright * -0.8;
  unitree_cmd_msg_.euler[1] = joy_keys.left_updown * 0.5;
  unitree_cmd_msg_.euler[2] = joy_keys.right_leftright * 0.3;
  unitree_cmd_msg_.body_height = joy_keys.right_updown * 0.2;

  return true;
}

bool UnitreeJoyCmd::parsePositionMode() {
  unitree_cmd_msg_.mode = 2;

  unitree_cmd_msg_.velocity[0] = joy_keys.left_updown * speed_gain_;
  unitree_cmd_msg_.velocity[1] = joy_keys.left_leftright * speed_gain_;

  unitree_cmd_msg_.yaw_speed = joy_keys.right_leftright * 2;

  switch (contorl_mode_) {
  case ContorlMode::A_MODE:
    unitree_cmd_msg_.gait_type = 1;
    break;
  case ContorlMode::B_MODE:
    unitree_cmd_msg_.gait_type = 2;
    break;
  case ContorlMode::Y_MODE:
    unitree_cmd_msg_.gait_type = 3;
    break;
  // TODO: X mode
  case ContorlMode::X_MODE:
    unitree_cmd_msg_.gait_type = 1;
    break;
  }

  return true;
}

bool UnitreeJoyCmd::parseControlMode() {
  if (joy_keys.btn_x && !prev_joy_keys.btn_x)
    contorl_mode_ = ContorlMode::X_MODE;
  if (joy_keys.btn_a && !prev_joy_keys.btn_a)
    contorl_mode_ = ContorlMode::A_MODE;
  if (joy_keys.btn_b && !prev_joy_keys.btn_b)
    contorl_mode_ = ContorlMode::B_MODE;
  if (joy_keys.btn_y && !prev_joy_keys.btn_y)
    contorl_mode_ = ContorlMode::Y_MODE;

  return true;
}

bool UnitreeJoyCmd::parseBodyHeight() {
  /// floating point 연산을 위해 특정한 값 사용
  if (joy_keys.btn_updown == 1.0 && prev_joy_keys.btn_updown == 0.0)
    unitree_cmd_msg_.body_height += 0.03125f;
  if (joy_keys.btn_updown == -1.0 && prev_joy_keys.btn_updown == 0.0)
    unitree_cmd_msg_.body_height -= 0.03125f;

  return true;
}

bool UnitreeJoyCmd::parseFootRaiseHeight() {
  /// floating point 연산을 위해 특정한 값 사용
  if (joy_keys.btn_LB && !prev_joy_keys.btn_LB)
    unitree_cmd_msg_.foot_raise_height -= 0.0625f;
  if (joy_keys.btn_RB && !prev_joy_keys.btn_RB)
    unitree_cmd_msg_.foot_raise_height += 0.0625f;

  return true;
}

bool UnitreeJoyCmd::parseVelocity() {
  /// floating point 연산을 위해 특정한 값 사용
  if (joy_keys.btn_back && !prev_joy_keys.btn_back)
    speed_gain_ -= 0.0625f;
  if (joy_keys.btn_start && !prev_joy_keys.btn_start)
    speed_gain_ += 0.0625f;

  return true;
}
