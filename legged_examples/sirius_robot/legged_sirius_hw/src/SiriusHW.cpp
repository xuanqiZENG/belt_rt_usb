
//
// Created by Shengzhi Wang on 2023/12/22. (email: wahrheitwsz@gmail.com).
//

#include "legged_sirius_hw/SiriusHW.h"
#include <legged_sirius_hw/rt_imu.h>

namespace legged {
const uint16_t vendor_id = 0x1234;
const uint16_t product_id = 0x6789;
const uint16_t sirius_usb2can_vendor_id = 0x1111;
const uint16_t sirius_usb2can_product_id = 0x2222;
const uint8_t  motors_ep_in = 0x81;
const uint8_t  motors_ep_out = 0x01;
const unsigned char endpoint_1 = 0x81;
const int IMU_RX_WORDS_PER_MESSAGE = 44;
SiriusHW::SiriusHW():
    _usb_IMU(IMU_RX_WORDS_PER_MESSAGE, 4, N_Communication::Vendor_id_Hex(vendor_id), N_Communication::Product_id_Hex(product_id), endpoint_1, 0x00),
    spUSB2CAN(sirius_usb2can_vendor_id, sirius_usb2can_product_id, motors_ep_in, motors_ep_out)
    {LeggedHW();}

bool SiriusHW::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  usb_data_imu.quat << 1.0, 0.0, 0.0, 0.0;
  usb_data_imu.gyro << 0.0, 0.0, 0.0;
  usb_data_imu.accelerometer << 0.0, 0.0, 0.0;
  _usb_IMU.usb_imu_set_rx_buffer(&usb_data_imu);
  if (!LeggedHW::init(root_nh, robot_hw_nh)) {
    return false;
  }
  robot_hw_nh.getParam("power_limit", powerLimit_);
  setupJoints();
  setupImu();
  setupContactSensor(robot_hw_nh);
  return true;
}

void SiriusHW::read(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  libusb_handle_events(_usb_IMU.get_usb_ctx());
  imuData_.ori_[0] = usb_data_imu.quat[0];
  imuData_.ori_[1] = usb_data_imu.quat[1];
  imuData_.ori_[2] = usb_data_imu.quat[2];
  imuData_.ori_[3] = usb_data_imu.quat[3];
  imuData_.angularVel_[0] = usb_data_imu.gyro[0];
  imuData_.angularVel_[1] = usb_data_imu.gyro[1];
  imuData_.angularVel_[2] = usb_data_imu.gyro[2];
  imuData_.linearAcc_[0] = usb_data_imu.accelerometer[0];
  imuData_.linearAcc_[1] = usb_data_imu.accelerometer[1];
  imuData_.linearAcc_[2] = usb_data_imu.accelerometer[2];
  // std::cout<<imuData_.ori_[0]<<"<---> \n";
  //   std::cout<<imuData_.ori_[1]<<"<---> \n";
  //     std::cout<<imuData_.ori_[2]<<"<---> \n";
  //       std::cout<<imuData_.ori_[3]<<"<---> \n";
  //   std::cout<<imuData_.linearAcc_[0]<<"<---> \n";
  //   std::cout<<imuData_.linearAcc_[1]<<"<---> \n";
  //   std::cout<<imuData_.linearAcc_[2]<<"<---> \n";
  // std::cout<<std::endl;

  // No contact feedback.
  for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
    contactState_[i] =0;
  }
  
  // Read the arm data
  this->spUSB2CAN.lock_in_mutex();
  for (int leg_num = 0; leg_num < 4; leg_num++) {
    jointData_[3*leg_num].pos_ = static_cast<double>(this->usb_data_to_controller->q_abad[leg_num]);
    jointData_[3*leg_num+1].pos_ = static_cast<double>(this->usb_data_to_controller->q_knee[leg_num]);
    jointData_[3*leg_num+2].pos_ = static_cast<double>(this->usb_data_to_controller->q_hip[leg_num]);
    jointData_[3*leg_num].vel_ = static_cast<double>(this->usb_data_to_controller->qd_abad[leg_num]);
    jointData_[3*leg_num+1].vel_ = static_cast<double>(this->usb_data_to_controller->qd_knee[leg_num]);
    jointData_[3*leg_num+2].vel_ = static_cast<double>(this->usb_data_to_controller->qd_hip[leg_num]);
}
  this->spUSB2CAN.unlock_in_mutex();

  // Set feedforward and velocity cmd to zero to avoid for safety when not controller setCommand
  std::vector<std::string> names = hybridJointInterface_.getNames();
  for (const auto& name : names) {
    HybridJointHandle handle = hybridJointInterface_.getHandle(name);
    handle.setFeedforward(0.);
    handle.setVelocityDesired(0.);
    handle.setKd(3.);
  }
}

void SiriusHW::write(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  // Set arm command
  this->spUSB2CAN.lock_out_mutex();

  // Set joint position & velocity
  for (int leg_num = 0; leg_num < 4; leg_num++) {
    usb_cmd_from_controller->q_des_abad[leg_num] = static_cast<float>(jointData_[3*leg_num].posDes_);
    usb_cmd_from_controller->q_des_knee[leg_num] = static_cast<float>(jointData_[3*leg_num+1].posDes_);
    usb_cmd_from_controller->q_des_hip[leg_num]  = static_cast<float>(jointData_[3*leg_num+2].posDes_);

    usb_cmd_from_controller->qd_des_abad[leg_num] = static_cast<float>(jointData_[3*leg_num].velDes_);
    usb_cmd_from_controller->qd_des_knee[leg_num] = static_cast<float>(jointData_[3*leg_num+1].velDes_);
    usb_cmd_from_controller->qd_des_hip[leg_num]  = static_cast<float>(jointData_[3*leg_num+2].velDes_);

    usb_cmd_from_controller->tau_abad_ff[leg_num] = static_cast<float>(jointData_[3*leg_num].ff_);
    usb_cmd_from_controller->tau_knee_ff[leg_num] = static_cast<float>(jointData_[3*leg_num+1].ff_);
    usb_cmd_from_controller->tau_hip_ff[leg_num]  = static_cast<float>(jointData_[3*leg_num+2].ff_);
  }
  
  // Set kp, kd and enable the joints
  for (int leg_num = 0; leg_num < 4; leg_num++) {
  usb_cmd_from_controller->kp_abad[leg_num] = static_cast<float>(jointData_[3*leg_num].kp_);
  usb_cmd_from_controller->kp_knee[leg_num] = static_cast<float>(jointData_[3*leg_num+1].kp_);
  usb_cmd_from_controller->kp_hip[leg_num]  = static_cast<float>(jointData_[3*leg_num+2].kp_);
  usb_cmd_from_controller->kd_abad[leg_num] = static_cast<float>(jointData_[3*leg_num].kd_);
  usb_cmd_from_controller->kd_hip[leg_num]  = static_cast<float>(jointData_[3*leg_num+1].kd_);
  usb_cmd_from_controller->kd_knee[leg_num] = static_cast<float>(jointData_[3*leg_num+2].kd_);
  }

  for (int leg_num = 0; leg_num < 2; leg_num++) {
  if (enable_counter < 30){
  usb_cmd_from_controller->flags[leg_num] = 0x01 <<1;
  }
  else {
    usb_cmd_from_controller->flags[leg_num] = 0;
  }
  }
  if (enable_counter>50){
  enable_counter=50;
  }
  enable_counter++;
  this->spUSB2CAN.unlock_out_mutex();
}

bool SiriusHW::setupJoints() {
  for (const auto& joint : urdfModel_->joints_) {
    int leg_index = 0;
    int joint_index = 0;
    if (joint.first.find("RF") != std::string::npos) {
      leg_index = 0;
    } else if (joint.first.find("LF") != std::string::npos) {
      leg_index = 1;
    } else if (joint.first.find("RH") != std::string::npos) {
      leg_index = 2;
    } else if (joint.first.find("LH") != std::string::npos) {
      leg_index = 3;
    }else {
      continue;
    }

    if (joint.first.find("HAA") != std::string::npos) {
      joint_index = 0;
    } else if (joint.first.find("HFE") != std::string::npos) {
      joint_index = 1;
    } else if (joint.first.find("KFE") != std::string::npos) {
      joint_index = 2;
    } else {
      continue;
    }
    int index = leg_index * 3 + joint_index;
    hardware_interface::JointStateHandle state_handle(joint.first, &jointData_[index].pos_, &jointData_[index].vel_,
                                                      &jointData_[index].tau_);
    jointStateInterface_.registerHandle(state_handle);
    hybridJointInterface_.registerHandle(HybridJointHandle(state_handle, &jointData_[index].posDes_, &jointData_[index].velDes_,
                                                           &jointData_[index].kp_, &jointData_[index].kd_, &jointData_[index].ff_));
  }

  // for (const auto& joint : urdfModel_->joints_) {
  //   int arm_index = 0;
  //   if (joint.first.find("Joint1") != std::string::npos) {
  //     arm_index = 0;
  //   } else if (joint.first.find("Joint2") != std::string::npos) {
  //     arm_index = 1;
  //   } else if (joint.first.find("Joint3") != std::string::npos) {
  //     arm_index = 2;
  //   }

    // int index = 12 + arm_index;
    // hardware_interface::JointStateHandle state_handle(joint.first, &jointData_[index].pos_, &jointData_[index].vel_,
    //                                                   &jointData_[index].tau_);
    // jointStateInterface_.registerHandle(state_handle);
    // hybridJointInterface_.registerHandle(HybridJointHandle(state_handle, &jointData_[index].posDes_, &jointData_[index].velDes_,
    //                                                        &jointData_[index].kp_, &jointData_[index].kd_, &jointData_[index].ff_));
  // }

  return true;
}

bool SiriusHW::setupImu() {
  imuSensorInterface_.registerHandle(hardware_interface::ImuSensorHandle("unitree_imu", "unitree_imu", imuData_.ori_, imuData_.oriCov_,
                                                                         imuData_.angularVel_, imuData_.angularVelCov_, imuData_.linearAcc_,
                                                                         imuData_.linearAccCov_));
  _usb_IMU.USB_Com_Start_Trans_Asy();
  imuData_.oriCov_[0] = 0.0012;
  imuData_.oriCov_[4] = 0.0012;
  imuData_.oriCov_[8] = 0.0012;

  imuData_.angularVelCov_[0] = 0.0004;
  imuData_.angularVelCov_[4] = 0.0004;
  imuData_.angularVelCov_[8] = 0.0004;

  return true;
}

bool SiriusHW::setupContactSensor(ros::NodeHandle& nh) {
  nh.getParam("contact_threshold", contactThreshold_);
  for (size_t i = 0; i < CONTACT_SENSOR_NAMES.size(); ++i) {
    contactSensorInterface_.registerHandle(ContactSensorHandle(CONTACT_SENSOR_NAMES[i], &contactState_[i]));
  }
  return true;
}
}  // namespace legged
