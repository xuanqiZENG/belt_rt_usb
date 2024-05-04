
//
// Created by Shengzhi Wang on 2023/12/22. (email: wahrheitwsz@gmail.com).
//

#pragma once
#include <legged_hw/LeggedHW.h>
#include <legged_sirius_hw/rt_usb_interface.h>
#include <legged_sirius_hw/rt_imu.h>
#include "thread"

namespace legged {
const std::vector<std::string> CONTACT_SENSOR_NAMES = {"RF_FOOT", "LF_FOOT", "RH_FOOT", "LH_FOOT"};

struct SiriusMotorData {
  double pos_, vel_, tau_;                 // state
  double posDes_, velDes_, kp_, kd_, ff_;  // command
};

struct SiriusImuData {
  double ori_[4];            // NOLINT(modernize-avoid-c-arrays)
  double oriCov_[9];         // NOLINT(modernize-avoid-c-arrays)
  double angularVel_[3];     // NOLINT(modernize-avoid-c-arrays)
  double angularVelCov_[9];  // NOLINT(modernize-avoid-c-arrays)
  double linearAcc_[3];      // NOLINT(modernize-avoid-c-arrays)
  double linearAccCov_[9];   // NOLINT(modernize-avoid-c-arrays)
};

class SiriusHW : public LeggedHW {
 public:
  SiriusHW();
  ~SiriusHW() {
    usb2canRunning_ = false;
    spUSB2CAN.destroy_thread();
    if (usb2can_thread_.joinable()) {
      usb2can_thread_.join();
    }
  };
  /** \brief Get necessary params from param server. Init hardware_interface.
   *
   * Get params from param server and check whether these params are set. Load urdf of robot. Set up transmission and
   * joint limit. Get configuration of can bus and create data pointer which point to data received from Can bus.
   *
   * @param root_nh Root node-handle of a ROS node.
   * @param robot_hw_nh Node-handle for robot hardware.
   * @return True when init successful, False when failed.
   */
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  /** \brief Communicate with hardware. Get data, status of robot.
   *
   * Call @ref UNITREE_LEGGED_SDK::UDP::Recv() to get robot's state.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void read(const ros::Time& time, const ros::Duration& period) override;

  /** \brief Comunicate with hardware. Publish command to robot.
   *
   * Propagate joint state to actuator state for the stored
   * transmission. Limit cmd_effort into suitable value. Call @ref UNITREE_LEGGED_SDK::UDP::Recv(). Publish actuator
   * current state.
   *
   * @param time Current time
   * @param period Current time - last time
   */
  void write(const ros::Time& time, const ros::Duration& period) override;

  // USB2CAN Interface
  Sirius_USB2CAN_Board spUSB2CAN;
  spi_data_t * usb_data_to_controller;
  spi_command_t * usb_cmd_from_controller;
  std::atomic_bool usb2canRunning_{};
  std::thread usb2can_thread_;
  N_Communication::USB_COM_IMU _usb_IMU;
  N_Communication::VectorNavData usb_data_imu;
  int enable_counter = 0;

 private:
  bool setupJoints();

  bool setupImu();

  bool setupContactSensor(ros::NodeHandle& nh);

  SiriusMotorData jointData_[12]{};  // NOLINT(modernize-avoid-c-arrays)
  SiriusImuData imuData_{};
  bool contactState_[4]{};  // NOLINT(modernize-avoid-c-arrays)

  int powerLimit_{};
  int contactThreshold_{};
  std::thread recv_thread;
};

}  // namespace legged
