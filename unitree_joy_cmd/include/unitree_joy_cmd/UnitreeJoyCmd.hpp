/*********************************************************************
 * Software License
 *  Copyright (c) 2022, WeGo Robotices Co., Ltd. && Road Balance Co. All Rights
 *Reserved.
 *********************************************************************/

#ifndef _UNITREE_JOY_CMD_
#define _UNITREE_JOY_CMD_

#include <rclcpp/rclcpp.hpp>

// #include <std_msgs/UInt8.h>
// #include <std_msgs/Float32.h>
// #include <sensor_msgs/Joy.h>

#include "ros2_unitree_legged_msgs/msg/high_cmd.hpp"
#include "sensor_msgs/msg/joy.hpp"

using HighCmd = ros2_unitree_legged_msgs::msg::HighCmd;
using Joy = sensor_msgs::msg::Joy;
using namespace std::chrono_literals;
using namespace std::placeholders;

/**
 * @brief integer값의 true/false checker
 *
 * @param val_in
 * @return true 입력값이 1인 경우만 true 반환
 * @return false 그 외는 모두 false
 */
inline bool isTrue(const int &val_in) { return (val_in == 1) ? true : false; }

/**
 * @brief sensor_msgs/Joy를 unitree_legged_msgs::HighCmd로 변환
 *
 */
class UnitreeJoyCmd : public rclcpp::Node {
public:
  /**
   * @brief Logitech F710 조이스틱가 X mode일 때의 모든 키 파싱
   *
   */
  struct XMode {
    /// 좌우 조이패드값 저장
    float left_updown;
    float left_leftright;

    float right_updown;
    float right_leftright;

    /// 위아래 버튼값 저장
    int btn_updown;
    int btn_leftright;

    /// A B X Y 버튼값 저장
    bool btn_a;
    bool btn_b;
    bool btn_x;
    bool btn_y;

    // LB RB 버튼값 저장
    bool btn_LB;
    bool btn_RB;

    // TODO: Back, Start 버튼값 저장
    bool btn_back;
    bool btn_start;
  };

  /**
   * @brief 4가지 로봇 주행 모드 판별
   *
   */
  enum class ContorlMode {
    X_MODE,
    A_MODE,
    B_MODE,
    Y_MODE,
  };

private:
  /// 조이스틱 현재값, 이전값 저장
  XMode prev_joy_keys;
  XMode joy_keys;

  /// 현재 로봇 조종 모드
  ContorlMode contorl_mode_ = ContorlMode::X_MODE;

  /// unitree에서 제공되는 제어 데이터 DB를 ROS msg로 변환한 타입
  HighCmd unitree_cmd_msg_;

  /// ROS topic Sub/Pub
  rclcpp::Subscription<Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<HighCmd>::SharedPtr unitree_cmd_pub_;

  /// 주기적인 제어 데이터 publish를 위해 timer사용
  rclcpp::TimerBase::SharedPtr control_timer_;

  /// ROS parameters
  double roll_gain_, pitch_gain_, yaw_gain_;
  double bodyheight_gain_;
  double speed_gain_;

public:
  /**
   * @brief Construct a new Unitree Joy Cmd object
   * ROS topic Sub/Pub을 위한 초기 작업 실행
   * Timer 설정
   * 각종 parameter 설정 및 초기화
   *
   */
  UnitreeJoyCmd();

  /**
   * @brief 50Hz 주기로 실행되는 Timer
   * HighCmd로 변환된 topic msg publish
   * 주기적인 제어 신호가 없으면 로봇이 자동 정지하기 때문에 timer 사용
   *
   */
  void timerCallback();

  /**
   * @brief sensor_msgs/Joy topic Subscribe
   * Subscribe 이후 XMode 타입의 키 매핑 갱신
   * Subscribe 이후 Joy 데이터로부터 매칭되는 제어 데이터를 파싱
   * ex) unitree_cmd_msg_.velocity[0] = joy_keys.left_updown;
   * => x 선속도는 왼쪽 조이패드의 상하 움직임에 매핑됨
   *
   * @param data sensor_msgs::msg::Joy 타입 데이터
   */
  void subCallback(const sensor_msgs::msg::Joy::SharedPtr data);

  /**
   * @brief 제어 모드 중 Pose control mode에 해당하는 파싱 작업이 이루어짐
   *
   * @return true 파싱 성공
   * @return false 파싱 실패 (TODO)
   */
  bool parsePoseMode();

  /**
   * @brief 제어 모드 중 Position control mode에 해당하는 파싱 작업이 이루어지며
   * Walking, Running, Stairs Climbing Mode에 해당하는 gaitType 구분이 이루어짐
   *
   * @return true 파싱 성공
   * @return false 파싱 실패 (TODO)
   */
  bool parsePositionMode();

  /**
   * @brief 조이스틱의 X, A, B, Y버튼을 누를 시마다 모드가 변경되며,
   * 이 모드의 변경을 탐지하는 부분
   *
   * @return true 파싱 성공
   * @return false 파싱 실패 (TODO)
   */
  bool parseControlMode();

  /**
   * @brief 로봇의 몸체 높이 (BodyHeight)을 제어하기 위해
   * 조이스틱의 위아래 버튼을 tracking 및 파싱
   *
   * @return true 파싱 성공
   * @return false 파싱 실패 (TODO)
   */
  bool parseBodyHeight();

  /**
   * @brief 로봇의 보행 시 발끝의 높이 (FootRaiseHeight)을 제어하기 위해
   * 조이스틱의 LB, RB 버튼을 tracking 및 파싱
   *
   * @return true 파싱 성공
   * @return false 파싱 실패 (TODO)
   */
  bool parseFootRaiseHeight();

  /**
   * @brief 속도 gain값 조절용 함수, 조이스틱의 back/start 버튼을 통해 속도를
   * 조정할 수 있다.
   *
   * @return true 파싱 성공
   * @return false 파싱 실패 (TODO)
   */
  bool parseVelocity();
};

#endif