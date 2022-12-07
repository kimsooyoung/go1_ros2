/*********************************************************************
 * Software License
 *  Copyright (c) 2022, WeGo Robotices Co., Ltd. && Road Balance Co. All Rights
 *Reserved.
 *********************************************************************/

#ifndef _UNITREE_TWIST_CMD_
#define _UNITREE_TWIST_CMD_

// #include <ros/ros.h>
// #include <std_msgs/UInt8.h>
// #include <std_msgs/Float32.h>
// #include <geometry_msgs/Twist.h>
// #include <unitree_legged_msgs/HighCmd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include <rclcpp/rclcpp.hpp>

using Twist = geometry_msgs::msg::Twist;
using UInt8 = std_msgs::msg::UInt8;
using Float32 = std_msgs::msg::Float32;
using HighCmd = ros2_unitree_legged_msgs::msg::HighCmd;

using namespace std::chrono_literals;
using namespace std::placeholders;

/**
 * @brief geometry_msgs/Twist 및 기타 여러 topic msg들을
 * unitree_legged_msgs::HighCmd로 변환
 *
 */
class UnitreeTwistCmd : public rclcpp::Node {
private:
  /// ROS topic Sub/Pub

  // ros::Subscriber twist_sub_;
  // ros::Subscriber control_mode_sub_;
  // ros::Subscriber body_height_sub_;
  // ros::Subscriber foot_height_sub_;

  rclcpp::Subscription<Twist>::SharedPtr twist_sub_;
  rclcpp::Subscription<UInt8>::SharedPtr control_mode_sub_;
  rclcpp::Subscription<Float32>::SharedPtr body_height_sub_;
  rclcpp::Subscription<Float32>::SharedPtr foot_height_sub_;

  // ros::Publisher unitree_cmd_pub_;

  rclcpp::Publisher<HighCmd>::SharedPtr unitree_cmd_pub_;

  /// 주기적인 제어 데이터 publish를 위해 timer사용
  rclcpp::TimerBase::SharedPtr control_timer_;

  /// unitree에서 제공되는 제어 데이터 DB를 ROS msg로 변환한 타입
  HighCmd unitree_cmd_msg_;

  /// parameters
  int default_mode_;
  bool verbose_;

public:
  /**
   * @brief Construct a new Unitree Twist Cmd object
   * ROS topic Sub/Pub을 위한 초기 작업 실행
   * Timer 설정
   * 각종 parameter 설정 및 초기화
   *
   * @param nh Class 생성 시 main으로 부터 NodeHandler를 전달받음
   */
  UnitreeTwistCmd();

  /**
   * @brief Position Control Mode라는 가정 하에 twist 타입 데이터를 Subscribe
   * 하여 unitree 호환의 제어 데이터로 변환한다.
   *
   * @param twist geometry_msgs::Twist 타입 데이터
   */
  void twistSubCallback(const Twist::SharedPtr twist);

  /**
   * @brief Standing, Walking, Running, Stairs Climbing Mode 중 선택 가능
   * 순서대로 mode 0, 1, 2, 3에 해당한다.
   * input mode가 주어지면 지정된 변환에 따라 mode/gaitType이 정해진다.
   *
   * @param mode 선택할 mode
   * @return true 파싱 성공
   * @return false 파싱 실패 (TODO)
   */
  bool selectMode(const uint8_t &mode);

  /**
   * @brief UInt8 타입의 topic msg를 Subscribe 하여 주행 mode를 설정한다.
   *
   * @param msg std_msgs::UInt8 타입 데이터
   */
  void controlModeSubCallback(const UInt8::SharedPtr msg);

  /**
   * @brief Float32 타입의 topic msg를 Subscribe 하여 로봇의 몸체 높이를
   * 설정한다. default = 0
   *
   * @param msg std_msgs::Float32 타입 데이터
   */
  void bodyHeightSubCallback(const Float32::SharedPtr msg);

  /**
   * @brief Float32 타입의 topic msg를 Subscribe 하여 로봇의 몸체 높이를
   * 설정한다. default = 0
   *
   * @param msg std_msgs::Float32 타입 데이터
   */
  void footHeightSubCallback(const Float32::SharedPtr msg);

  /**
   * @brief 50Hz 주기로 실행되는 Timer
   * unitree_legged_msgs::HighCmd로 변환된 topic msg publish
   *
   * @param event Timer 사용을 위해 필수로 추가되는 매개변수
   */
  void timerCallback();
};

#endif