/*********************************************************************
 * Software License
 *  Copyright (c) 2022, WeGo Robotices Co., Ltd. && Road Balance Co. All Rights
 *Reserved.
 *********************************************************************/

#pragma once

#include <stdio.h>
#include <chrono>
#include <functional>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2_ros/transform_broadcaster.h"

// #include <std_msgs/Bool.h>
// #include <sensor_msgs/Imu.h>
// #include <nav_msgs/Odometry.h>

// #include <tf2/utils.h>
// #include <tf/transform_datatypes.h>
// #include <tf/transform_broadcaster.h>
// #include <tf2_ros/transform_listener.h>
// #include <geometry_msgs/TransformStamped.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "tf2/LinearMath/Quaternion.h"
// #include "unitree_legged_real/convert.h"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/imu.hpp"
// #include "unitree_legged_real/convert.h"

// 1. high_state sub
// 2. imu parse
// 3. odom tf broadcast
// 4. odom msg pub

// using namespace UNITREE_LEGGED_SDK;

class HighLevelHelper : public rclcpp::Node {

private:
  /// SDK DB와 동일한 요소를 갖진 ROS msg
  ros2_unitree_legged_msgs::msg::HighState high_state_;
  //   ros2_unitree_legged_msgs::msg::HighCmd high_cmd_;

  /// 일정 시간 제어 데이터 수신이 없다면 LCM 서버가 중단됨
  /// 이에 따라 LCM/ROS가 주기적인 Pub/Sub이 이루어짐
  //   ros::Timer timer_;
  rclcpp::TimerBase::SharedPtr timer_;

  /// ROS Publisher, Subscriber
  //   ros::Publisher imu_pub_;
  //   ros::Publisher movable_pub_;
  //   ros::Publisher odom_pub_;
  //   ros::Publisher control_pub_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  //   ros::Subscriber control_sub_;
  //   ros::Subscriber high_sub_;

  rclcpp::Subscription<ros2_unitree_legged_msgs::msg::HighState>::SharedPtr
      high_sub_;

  /// HighState 데이터를 여러 topic으로 쪼갠다.
  //   sensor_msgs::Imu dog_imu_;
  //   nav_msgs::Odometry dog_odom_;
  //   std_msgs::Bool dog_can_move_;

  sensor_msgs::msg::Imu dog_imu_;
  nav_msgs::msg::Odometry dog_odom_;

  /// tf 관련 변수들
  /// imu tf, odom tf를 생성하며 둘은 같은 좌표를 갖는다.
  //   tf::Quaternion q_raw_, q_offset_;
  tf2::Quaternion q_raw_, q_offset_;

  //   tf::Transform imu_transform_;
  //   tf::Transform odom_transform_;

  geometry_msgs::msg::TransformStamped imu_tf_stamp_;
  geometry_msgs::msg::TransformStamped odom_tf_stamp_;

  //   tf::TransformBroadcaster imu_broadcaster_;
  //   tf::TransformBroadcaster odom_broadcaster_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> imu_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;

  // ROS Parameters
  bool publish_tf_;
  bool verbose_;
  std::string imu_link_id_, base_link_id_, odom_link_id_;

public:
  HighLevelHelper() : Node("high_level_helper") {
    /// ROS 관련 초기 설정
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&HighLevelHelper::timerCallback, this));

    high_sub_ =
        this->create_subscription<ros2_unitree_legged_msgs::msg::HighState>(
            "high_state", 10,
            std::bind(&HighLevelHelper::highStateCallback, this,
                      std::placeholders::_1));

    // control_sub_ = nh->subscribe("untiree_quadrupped_high_cmd", 10,
    // &HighLevelHelper::controlSubCallback, this);

    // control_pub_ =
    // nh->advertise<ros2_unitree_legged_msgs::HighCmd>("high_cmd", 1000);
    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_raw", 10);
    // movable_pub_ = nh->advertise<std_msgs::Bool>("dog_can_move_", 10);
    // odom_pub_ = nh->advertise<nav_msgs::Odometry>("dog_odom", 10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("dog_odom", 10);

    imu_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    odom_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    imu_link_id_ = this->declare_parameter("imu_link", "imu_link");
    base_link_id_ = this->declare_parameter("base_link", "base_link");
    odom_link_id_ = this->declare_parameter("odom_link", "odom");

    publish_tf_ = this->declare_parameter("publish_tf", true);
    verbose_ = this->declare_parameter("verbose", false);

    printf("imu_link_id : %s\n", imu_link_id_.c_str());
    printf("base_link_id : %s\n", base_link_id_.c_str());
    printf("odom_link_id : %s\n", odom_link_id_.c_str());

    printf("publish_tf : %s\n", publish_tf_ ? "true" : "false");
    printf("verbose : %s\n", verbose_ ? "true" : "false");

    /// imu 오차 발생 시 보정을 위한 offset
    q_offset_.setRPY(0, 0, 3.1415);
  }

  /**
   * @brief ros_udp node로부터 로봇의 High State가 subscribe될 때마다 실행된는
   * 콜백 함수 다양한 데이터를 담고 있는 HighState에서 원하는 데이터만 추출하여
   * 별도의 topic publish를 수행한다.
   *
   * @param msg ROS 타압의 HighState msg, ros2_unitree_legged_msgs에서 구체적인
   * 내용을 조회할 수 있다.
   */
  void highStateCallback(
      const ros2_unitree_legged_msgs::msg::HighState::SharedPtr msg) {
    /// msg에서 원하는 데이터를 추출하고 클래스 변수로 저장한다.
    // parseCanMove(msg->mode);
    parseIMU(msg->imu);
    // parseOrientation(msg->imu);
    parseOdom(msg);

    /// topic publish
    imu_pub_->publish(dog_imu_);
    // movable_pub_.publish(dog_can_move_);
    odom_pub_->publish(dog_odom_);
  }

  /**
   * @brief ros2_unitree_legged_msgs::HighCmd를 다시 한 번 파싱한다.
   * 즉각 ros_udp node로 넘기지 않고, 한번 더 거치는 이유는 이전 버전과의 호환을
   * 위해서이다.
   * TODO : 예외처리 로직 추가하기
   * ex) mode가 9로 잘못 변경되는 경우, 갑자기 백덤블링을 할 수 있음
   *
   * @param high_cmd ros2_unitree_legged_msgs::HighCmd 타입 Subscribe msg
   */
  //   void controlSubCallback(const ros2_unitree_legged_msgs::HighCmd
  //   &high_cmd)
  //   {
  //     high_cmd_.mode = high_cmd.mode;

  //     high_cmd_.gaitType = high_cmd.gaitType;

  //     high_cmd_.euler[0] = high_cmd.euler[0];
  //     high_cmd_.euler[1] = high_cmd.euler[1];
  //     high_cmd_.euler[2] = high_cmd.euler[2];

  //     high_cmd_.velocity[0] = high_cmd.velocity[0];
  //     high_cmd_.velocity[1] = high_cmd.velocity[1];

  //     high_cmd_.yawSpeed = high_cmd.yawSpeed;

  //     high_cmd_.bodyHeight = high_cmd.bodyHeight;
  //     high_cmd_.footRaiseHeight = high_cmd.footRaiseHeight;
  //   }

  /**
   * @brief 50Hz 주기로 실행되는 Timer, 다음과 같은 작업을 수행한다.
   * 제어 데이터 publish : 일정 시간 이상 제어 데이터가 없으면 자동 차단되기
   * 때문 tf publish : rviz에서 시각화할 시, 비슷한 이유로 주기적인 publish가
   * 없으면 동작하지 않기 때문
   * @param event Timer 사용을 위해 필수로 추가되는 매개변수
   */
  void timerCallback() {
    // control_pub_.publish(high_cmd_);

    /// tf publish
    if (publish_tf_) {
      publishImuTF();
      publishOdomTF();
    }
  }

  /**
   * @brief 로봇의 조종 가능 여부를 확인한다.
   *
   * @return true 조종 가능
   * @return false 조종 불가능
   */
  //   bool parseCanMove(const uint8_t &mode)
  //   {
  //     /// 리모컨의 start키 입력 수신 여부에 따라 로봇의 작동 가능 여부를
  //     확인한다. static uint8_t print_count = 0; if (mode == 2)
  //     {
  //       dog_can_move_.data = true;
  //       if (print_count == 0)
  //       {
  //         ROS_INFO("robot can move: %s", dog_can_move_.data ? "true" :
  //         "false"); print_count++;
  //       }
  //     }
  //     else
  //     {
  //       dog_can_move_.data = false;
  //       print_count = 0;
  //     }

  //     return true;
  //   }

  /**
   * @brief HighState 중 imu 데이터만을 추출하여 topic msg를 구성한다.
   *
   * @return true 파싱 성공
   * @return false 파싱 실패 (TODO)
   */
  bool parseIMU(const ros2_unitree_legged_msgs::msg::IMU &imu_raw) {
    // Prepare IMU MSG
    dog_imu_.header.frame_id = imu_link_id_;
    dog_imu_.header.stamp = this->get_clock()->now();

    // 방향 데이터
    // dog_imu_.orientation.x = imu_raw.quaternion[0];
    // dog_imu_.orientation.y = imu_raw.quaternion[1];
    // dog_imu_.orientation.z = imu_raw.quaternion[2];
    // dog_imu_.orientation.w = imu_raw.quaternion[3];

    // 확인 필요
    // tf::Quaternion q_temp;

    q_raw_[2] = imu_raw.quaternion[0];
    q_raw_[1] = imu_raw.quaternion[1];
    q_raw_[0] = -imu_raw.quaternion[2];
    q_raw_[3] = -imu_raw.quaternion[3];

    q_raw_ = q_offset_ * q_raw_;
    q_raw_.normalize();

    dog_imu_.orientation.x = q_raw_[0];
    dog_imu_.orientation.y = q_raw_[1];
    dog_imu_.orientation.z = q_raw_[2];
    dog_imu_.orientation.w = q_raw_[3];

    // 각속도 데이터
    dog_imu_.angular_velocity.x = imu_raw.gyroscope[0];
    dog_imu_.angular_velocity.y = imu_raw.gyroscope[1];
    dog_imu_.angular_velocity.z = imu_raw.gyroscope[2];

    // 가속도 데이터
    dog_imu_.linear_acceleration.x = imu_raw.accelerometer[0];
    dog_imu_.linear_acceleration.y = imu_raw.accelerometer[1];
    dog_imu_.linear_acceleration.z = imu_raw.accelerometer[2];

    return true;
  }

  /**
   * @brief HighState 중 각도 데이터만을 추출한다.
   * IMU 데이터 내부에 rpy 값이 있지만 이전 버전과의 호환을 위해 quaternion 사용
   * 이것은 이후 디버깅 및 tf publish 시 사용하게 된다.
   *
   * @return true 파싱 성공
   * @return false 파싱 실패 (TODO)
   */
  //   bool parseOrientation(const ros2_unitree_legged_msgs::IMU &imu_raw)
  //   {
  //     q_raw_[2] = imu_raw.quaternion[0];
  //     q_raw_[1] = imu_raw.quaternion[1];
  //     q_raw_[0] = -imu_raw.quaternion[2];
  //     q_raw_[3] = -imu_raw.quaternion[3];

  //     q_raw_ = q_offset_ * q_raw_;

  //     return true;
  //   }

  /**
   * @brief HighState 중 odom과 관련된 데이터만을 추출하여 topic msg를 구성한다.
   *
   * @return true 파싱 성공
   * @return false 파싱 실패 (TODO)
   */
  bool
  parseOdom(const ros2_unitree_legged_msgs::msg::HighState::SharedPtr msg) {
    // Prepare Odom MSG
    dog_odom_.header.frame_id = odom_link_id_;
    dog_odom_.child_frame_id = base_link_id_;
    dog_odom_.header.stamp = this->get_clock()->now();

    // 위치 데이터 - 굴곡진 환경에서 큰 오차를 보이므로 완전히 신뢰할 수는 없다.
    dog_odom_.pose.pose.position.x = (msg->position)[0];
    dog_odom_.pose.pose.position.y = (msg->position)[1];
    dog_odom_.pose.pose.position.z = (msg->position)[2];
    dog_odom_.pose.pose.orientation = dog_imu_.orientation;
    dog_odom_.pose.covariance.fill(0.0);

    // 속도 데이터
    dog_odom_.twist.twist.linear.x = (msg->velocity)[0];
    dog_odom_.twist.twist.linear.y = (msg->velocity)[1];
    dog_odom_.twist.twist.angular.z = msg->yaw_speed;
    dog_odom_.twist.covariance.fill(0.0);

    return true;
  }

  /**
   * @brief imu_link를 이름으로 갖는 tf frame publish
   * 현재는 imu_link와 base_link가 일치하도록 세팅되어 있다.
   * (로봇 내부 어디에 imu가 위치하는지 모르기 때문)
   *
   * @return true publish 성공
   * @return false publish 실패
   */
  bool publishImuTF() {
    // tf::Quaternion unit_q;
    // unit_q[0] = 0.0;
    // unit_q[1] = 0.0;
    // unit_q[2] = 0.0;
    // unit_q[3] = 1.0;

    // imu_transform_.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    // imu_transform_.setRotation(unit_q);
    // imu_broadcaster_.sendTransform(tf::StampedTransform(
    //     imu_transform_, ros::Time::now(), base_link_id_, imu_link_id_));

    imu_tf_stamp_.header.stamp = this->get_clock()->now();
    imu_tf_stamp_.header.frame_id = base_link_id_;
    imu_tf_stamp_.child_frame_id = imu_link_id_;
    imu_tf_stamp_.transform.translation.x = 0.0;
    imu_tf_stamp_.transform.translation.y = 0.0;
    imu_tf_stamp_.transform.translation.z = 0.0;
    imu_tf_stamp_.transform.rotation.x = 0.0;
    imu_tf_stamp_.transform.rotation.y = 0.0;
    imu_tf_stamp_.transform.rotation.z = 0.0;
    imu_tf_stamp_.transform.rotation.w = 1.0;

    imu_broadcaster_->sendTransform(imu_tf_stamp_);

    return true;
  }

  /**
   * @brief odom_link_id_를 이름으로 갖는 odom tf frame publish
   * 해당 odom은 unitree 로봇 내에서 자체적으로 계산된 값으로
   * 굴곡진 지형을 지나거나, 급작스런 속도 변화 발생 시 odom이 불안정해진다.
   * 자체적으로 사용하기보다 sensor fusion을 거친 뒤 사용하기를 추천
   *
   * @return true publish 성공
   * @return false publish 실패
   */
  bool publishOdomTF() {
    // odom_transform_.setOrigin(tf::Vector3(dog_odom_.pose.pose.position.x,
    // dog_odom_.pose.pose.position.y, 0.0));
    // odom_transform_.setRotation(q_raw_);
    // odom_broadcaster_.sendTransform(tf::StampedTransform(
    //     odom_transform_, ros::Time::now(), odom_link_id_, base_link_id_));

    odom_tf_stamp_.header.stamp = this->get_clock()->now();
    odom_tf_stamp_.header.frame_id = odom_link_id_;
    odom_tf_stamp_.child_frame_id = base_link_id_;

    odom_tf_stamp_.transform.translation.x = dog_odom_.pose.pose.position.x;
    odom_tf_stamp_.transform.translation.y = dog_odom_.pose.pose.position.y;
    odom_tf_stamp_.transform.translation.z = dog_odom_.pose.pose.position.z;
    odom_tf_stamp_.transform.rotation.x = q_raw_[0];
    odom_tf_stamp_.transform.rotation.y = q_raw_[1];
    odom_tf_stamp_.transform.rotation.z = q_raw_[2];
    odom_tf_stamp_.transform.rotation.w = q_raw_[3];

    odom_broadcaster_->sendTransform(odom_tf_stamp_);

    return true;
  }

//   ~HighLevelHelper() {}
};
