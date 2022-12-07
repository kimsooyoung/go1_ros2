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

#include "tf2/LinearMath/Quaternion.h"
#include "ros2_unitree_legged_msgs/msg/high_state.hpp"
#include "ros2_unitree_legged_msgs/msg/imu.hpp"

class HighLevelHelper : public rclcpp::Node {

private:
  /// SDK DB와 동일한 요소를 갖진 ROS msg
  ros2_unitree_legged_msgs::msg::HighState high_state_;

  /// 일정 시간 제어 데이터 수신이 없다면 LCM 서버가 중단됨
  /// 이에 따라 LCM/ROS가 주기적인 Pub/Sub이 이루어짐
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  rclcpp::Subscription<ros2_unitree_legged_msgs::msg::HighState>::SharedPtr
      high_sub_;

  sensor_msgs::msg::Imu dog_imu_;
  nav_msgs::msg::Odometry dog_odom_;

  /// tf 관련 변수들
  /// imu tf, odom tf를 생성하며 둘은 같은 좌표를 갖는다.
  //   tf::Quaternion q_raw_, q_offset_;
  tf2::Quaternion q_raw_, q_offset_;

  geometry_msgs::msg::TransformStamped imu_tf_stamp_;
  geometry_msgs::msg::TransformStamped odom_tf_stamp_;
  geometry_msgs::msg::TransformStamped base_fp_stamp_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> imu_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> bf_broadcaster_;

  // ROS Parameters
  bool publish_tf_;
  bool verbose_;
  bool dimension_3d_;
  std::string imu_link_id_, base_link_id_, odom_link_id_, base_fp_id_;

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

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_raw", 10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("dog_odom", 10);

    imu_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    odom_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    bf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    imu_link_id_ = this->declare_parameter("imu_link", "imu_link");
    base_link_id_ = this->declare_parameter("base_link", "base_link");
    odom_link_id_ = this->declare_parameter("odom_link", "odom");
    base_fp_id_ = "base_footprint";

    publish_tf_ = this->declare_parameter("publish_tf", true);
    verbose_ = this->declare_parameter("verbose", false);
    dimension_3d_ = this->declare_parameter("dimension_3d", false);

    printf("imu_link_id : %s\n", imu_link_id_.c_str());
    printf("base_link_id : %s\n", base_link_id_.c_str());
    printf("odom_link_id : %s\n", odom_link_id_.c_str());

    printf("publish_tf : %s\n", publish_tf_ ? "true" : "false");
    printf("verbose : %s\n", verbose_ ? "true" : "false");
    printf("dimension_3d : %s\n", dimension_3d_ ? "true" : "false");

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
    parseIMU(msg->imu);
    parseOdom(msg);

    /// topic publish
    imu_pub_->publish(dog_imu_);
    odom_pub_->publish(dog_odom_);
  }

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
      publishBPTF();
    }
  }

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
    dog_imu_.orientation.x = imu_raw.quaternion[0];
    dog_imu_.orientation.y = imu_raw.quaternion[1];
    dog_imu_.orientation.z = imu_raw.quaternion[2];
    dog_imu_.orientation.w = imu_raw.quaternion[3];

    // Uncomment below lines for Black Go1
    // q_raw_[2] = imu_raw.quaternion[0];
    // q_raw_[1] = imu_raw.quaternion[1];
    // q_raw_[0] = -imu_raw.quaternion[2];
    // q_raw_[3] = -imu_raw.quaternion[3];

    // q_raw_ = q_offset_ * q_raw_;
    // q_raw_.normalize();

    // dog_imu_.orientation.x = q_raw_[0];
    // dog_imu_.orientation.y = q_raw_[1];
    // dog_imu_.orientation.z = q_raw_[2];
    // dog_imu_.orientation.w = q_raw_[3];

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
   * @brief HighState 중 odom과 관련된 데이터만을 추출하여 topic msg를 구성한다.
   *
   * @return true 파싱 성공
   * @return false 파싱 실패 (TODO)
   */
  bool
  parseOdom(const ros2_unitree_legged_msgs::msg::HighState::SharedPtr msg) {
    // Prepare Odom MSG
    dog_odom_.header.frame_id = odom_link_id_;
    dog_odom_.child_frame_id = base_fp_id_; // base_link_id_;
    dog_odom_.header.stamp = this->get_clock()->now();

    // 위치 데이터 - 굴곡진 환경에서 큰 오차를 보이므로 완전히 신뢰할 수는 없다.
    dog_odom_.pose.pose.position.x = (msg->position)[0];
    dog_odom_.pose.pose.position.y = (msg->position)[1];
    if(dimension_3d_)
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

  bool publishBPTF() {

    base_fp_stamp_.header.stamp = this->get_clock()->now();
    base_fp_stamp_.header.frame_id = base_fp_id_;
    base_fp_stamp_.child_frame_id = base_link_id_;
    base_fp_stamp_.transform.translation.x = 0.0;
    base_fp_stamp_.transform.translation.y = 0.0;
    base_fp_stamp_.transform.translation.z = 0.0;
    base_fp_stamp_.transform.rotation.x = 0.0;
    base_fp_stamp_.transform.rotation.y = 0.0;
    base_fp_stamp_.transform.rotation.z = 0.0;
    base_fp_stamp_.transform.rotation.w = 1.0;

    bf_broadcaster_->sendTransform(base_fp_stamp_);

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

    odom_tf_stamp_.header.stamp = this->get_clock()->now();
    odom_tf_stamp_.header.frame_id = odom_link_id_;
    odom_tf_stamp_.child_frame_id = base_fp_id_; // base_link_id_;

    odom_tf_stamp_.transform.translation.x = dog_odom_.pose.pose.position.x;
    odom_tf_stamp_.transform.translation.y = dog_odom_.pose.pose.position.y;
    odom_tf_stamp_.transform.translation.z = dog_odom_.pose.pose.position.z;

    odom_tf_stamp_.transform.rotation.x = dog_imu_.orientation.x;
    odom_tf_stamp_.transform.rotation.y = dog_imu_.orientation.y;
    odom_tf_stamp_.transform.rotation.z = dog_imu_.orientation.z;
    odom_tf_stamp_.transform.rotation.w = dog_imu_.orientation.w;

    // Uncomment Below lines for Black Go1
    // odom_tf_stamp_.transform.rotation.x = q_raw_[0];
    // odom_tf_stamp_.transform.rotation.y = q_raw_[1];
    // odom_tf_stamp_.transform.rotation.z = q_raw_[2];
    // odom_tf_stamp_.transform.rotation.w = q_raw_[3];

    odom_broadcaster_->sendTransform(odom_tf_stamp_);

    return true;
  }
};
