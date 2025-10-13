// Copyright 2025 TESOLLO
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the TESOLLO nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include "dg5f_driver/system_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dg5f_driver {

static const int leftMotorDir[20] = {1, 1, 1,  1, -1, 1, 1,  1, -1, 1,
                                     1, 1, -1, 1, 1,  1, -1, 1, -1, -1};

static const int rightMotorDir[20] = {1, 1, -1, -1,
                                       -1, 1, 1,  1,
                                      -1, 1, 1, 1,
                                       -1, 1, 1, 1,
                                       -1, -1, -1, -1};

hardware_interface::SystemInterface::CallbackReturn SystemInterface::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::CallbackReturn::SUCCESS !=
      hardware_interface::SystemInterface::on_init(info)) {
    return CallbackReturn::ERROR;
  }

  positions_.resize(info_.joints.size(), 0.0);
  velocities_.resize(info_.joints.size(), 0.0);
  efforts_.resize(info_.joints.size(), 0.0);
  temperature_.resize(info_.joints.size(), 0.0);
  effort_commands_.resize(info_.joints.size(), 0.0);
  firmware_version_.resize(2,0);
  current_limit_flag_.resize(info_.joints.size(), 0.0);
  current_integral_.resize(info_.joints.size(), 0.0);
  for (const hardware_interface::ComponentInfo& joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_ERROR(rclcpp::get_logger("SystemInterface"),
                   "Joint '%s' needs a command interface.", joint.name.c_str());

      return CallbackReturn::ERROR;
    }

    if (!(joint.state_interfaces[0].name ==
              hardware_interface::HW_IF_POSITION ||
          joint.state_interfaces[1].name ==
              hardware_interface::HW_IF_VELOCITY)) {
      RCLCPP_ERROR(rclcpp::get_logger("SystemInterface"),
                   "Joint '%s' needs the following state interfaces in this "
                   "order: %s and %s.",
                   joint.name.c_str(), hardware_interface::HW_IF_POSITION,
                   hardware_interface::HW_IF_VELOCITY);
      return CallbackReturn::ERROR;
    }
  }
  // Set default values
  delto_ip_ = "169.254.186.72";  // Default IP
  delto_port_ = 502;             // Default port
  fingertip_sensor_ = false;     // Default value
  io_ = false;                   // Default value
  model_ = 0x5F12;               // Default model

  // Safely get parameters with defaults
  if (info.hardware_parameters.find("delto_ip") !=
      info.hardware_parameters.end()) {
    delto_ip_ = info.hardware_parameters.at("delto_ip");
  } else {
    RCLCPP_WARN(rclcpp::get_logger("SystemInterface"),
                "Parameter 'delto_ip' not found, using default: %s",
                delto_ip_.c_str());
  }

  if (info.hardware_parameters.find("delto_port") !=
      info.hardware_parameters.end()) {
    try {
      delto_port_ = std::stoi(info.hardware_parameters.at("delto_port"));
    } catch (...) {
      RCLCPP_WARN(rclcpp::get_logger("SystemInterface"),
                  "Invalid port parameter, using default: %d", delto_port_);
    }
  } else {
    RCLCPP_WARN(rclcpp::get_logger("SystemInterface"),
                "Parameter 'delto_port' not found, using default: %d",
                delto_port_);
  }

  if (info.hardware_parameters.find("delto_model") !=
      info.hardware_parameters.end()) {
    try {
      model_ = std::stoi(info.hardware_parameters.at("delto_model"));
    } catch (...) {
      RCLCPP_WARN(rclcpp::get_logger("SystemInterface"),
                  "Invalid port parameter, using default: %d", model_);
    }
  } else {
    RCLCPP_WARN(rclcpp::get_logger("SystemInterface"),
                "Parameter 'delto_port' not found, using default: %d", model_);
  }
  RCLCPP_INFO(rclcpp::get_logger("SystemInterface"), "Delto model: %d", model_);
  // Check fingertip parameter
  if (info.hardware_parameters.find("fingertip_sensor") !=
      info.hardware_parameters.end()) {
    fingertip_sensor_ =
        info.hardware_parameters.at("fingertip_sensor") == "true";
  }

  // Check IO parameter
  if (info.hardware_parameters.find("IO") != info.hardware_parameters.end()) {
    io_ = info.hardware_parameters.at("IO") == "true";
  }

  if (info_.hardware_parameters.find("hand_type") !=
      info.hardware_parameters.end()) {
    hand_type_ = info.hardware_parameters.at("hand_type");
  } else {
    RCLCPP_WARN(rclcpp::get_logger("SystemInterface"),
                "Parameter 'hand_type' not found, using default: %s",
                hand_type_.c_str());
  }
  if (hand_type_ == "left") {
    for (size_t i = 0; i < positions_.size(); ++i) {
      motor_dir_[i] = leftMotorDir[i];
    }
  } else if (hand_type_ == "right") {
    for (size_t i = 0; i < positions_.size(); ++i) {
      motor_dir_[i] = rightMotorDir[i];
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("SystemInterface"), "Invalid hand type: %s",
                 hand_type_.c_str());
    return CallbackReturn::ERROR;
  }

  // if (info_.hardware_parameters.find("firmware_version") !=
  //     info.hardware_parameters.end()) {
  //   firmware_version_ = stof(info.hardware_parameters.at("firmware_version"));
  // } else {
  //   RCLCPP_WARN(rclcpp::get_logger("SystemInterface"),
  //               "Parameter 'firmware_version_' not found.");
  // }

  delto_client_ = std::make_unique<DeltoTCP::Communication>(
      delto_ip_, delto_port_, model_, fingertip_sensor_, io_);
  
      try{
  init();
  firmware_version_ = delto_client_->GetFirmwareVersion();
      }
      catch(...)
      {
          RCLCPP_WARN(rclcpp::get_logger("SystemInterface"),
                "Connect Failed.");
                return CallbackReturn::FAILURE;
      }
      // m_init_thread_ = std::thread(&SystemInterface::init, this);
  // m_init_thread_.detach();

  return CallbackReturn::SUCCESS;
}

hardware_interface::SystemInterface::CallbackReturn
SystemInterface::on_deactivate(
    [[maybe_unused]] const rclcpp_lifecycle::State& previous_state) {
  if (delto_client_) {
    delto_client_->Disconnect();
  }

  RCLCPP_INFO(rclcpp::get_logger("SystemInterface"), "Deactivated driver");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
SystemInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(info_.joints.size() *
                           4);  // position, velocity, effort, temperature

  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &positions_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &velocities_[i]));

    // Add effort state interface
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &efforts_[i]));
        
    // Add temperature state interface
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, delto_interface::HW_IF_TEMPERATURE,
        &temperature_[i]));
        
    std::cout << "export_state_interfaces: " << info_.joints[i].name
              << " (position, velocity, effort, temperature)" << std::endl;
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
SystemInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
        &effort_commands_[i]));
  }

  return command_interfaces;
}

SystemInterface::return_type SystemInterface::prepare_command_mode_switch(
    [[maybe_unused]] const std::vector<std::string>& start_interfaces,
    [[maybe_unused]] const std::vector<std::string>& stop_interfaces) {
  return return_type::OK;
}

SystemInterface::CallbackReturn SystemInterface::on_activate(
    [[maybe_unused]] const rclcpp_lifecycle::State& previous_state) {
  RCLCPP_INFO(rclcpp::get_logger("SystemInterface"), "Started driver");

  return CallbackReturn::SUCCESS;
}

hardware_interface::SystemInterface::CallbackReturn
SystemInterface::on_shutdown(
    [[maybe_unused]] const rclcpp_lifecycle::State& previous_state) {
  RCLCPP_INFO(rclcpp::get_logger("SystemInterface"), "Stopped driver");
  return CallbackReturn::SUCCESS;
}

void SystemInterface::init() { delto_client_->Connect(); }

SystemInterface::return_type SystemInterface::read(
    [[maybe_unused]] const rclcpp::Time& time,
    [[maybe_unused]] const rclcpp::Duration& period) {
  try {
    if (!delto_client_) {
      std::cerr << "Client is not initialized" << std::endl;
      return return_type::ERROR;
    }

    DeltoReceivedData received_data;
    try {
      received_data = delto_client_->GetData();
    } catch (const std::exception& e) {
      std::cerr << "Failed to read data: " << e.what() << std::endl;
      return return_type::ERROR;
    }

    // 데이터 크기 검증
    if (received_data.joint.size() != positions_.size()) {
      std::cerr << "Insufficient position data. Expected: " << positions_.size()
                << ", Got: " << received_data.joint.size() << std::endl;
      return return_type::ERROR;
    }

    if (received_data.velocity.size() != velocities_.size()) {
      std::cerr << "Insufficient velocity data. Expected: "
                << velocities_.size()
                << ", Got: " << received_data.velocity.size() << std::endl;
      return return_type::ERROR;
    }

    if (received_data.current.size() != efforts_.size()) {
      std::cerr << "Insufficient current data. Expected: " << efforts_.size()
                << ", Got: " << received_data.current.size() << std::endl;
      return return_type::ERROR;
    }

    positions_ = received_data.joint;
    velocities_ = received_data.velocity;
    current_ = received_data.current;
    efforts_ = current_;
    temperature_ = received_data.temperature;

    return return_type::OK;
  } catch (const std::exception& e) {
    std::cerr << "Unexpected error in read: " << e.what() << std::endl;
    return return_type::ERROR;
  }
}

SystemInterface::return_type SystemInterface::write(
    [[maybe_unused]] const rclcpp::Time& time,
    [[maybe_unused]] const rclcpp::Duration& period) {
  std::vector<double> filter_effort_commands(effort_commands_.size());
  std::vector<double> duty(effort_commands_.size());
  std::vector<int> int_duty(effort_commands_.size());
  std::vector<int> current_mA(effort_commands_.size());

  for (size_t i = 0; i < effort_commands_.size(); ++i) {
    current_mA[i] = static_cast<int>(current_[i]);
  }
  try {
    filter_effort_commands = delto_gripper_helper::CurrentControl(
        effort_commands_.size(), current_mA, effort_commands_,
        current_limit_flag_, current_integral_);

    duty = delto_gripper_helper::ConvertDuty(effort_commands_.size(),
                                             filter_effort_commands);

    for (size_t i = 0; i < effort_commands_.size(); ++i) {
      int_duty[i] = static_cast<int>(duty[i] * 10);
      int_duty[i] = std::clamp(int_duty[i], -1000, 1000);

      if ((firmware_version_[0] >= 2 && firmware_version_[1]>=8) ||
         firmware_version_[0] >2 ) {
        int_duty[i] *= 1;
      } else {
        int_duty[i] *= motor_dir_[i];
      }
    }

  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("SystemInterface"), "write error: %s",
                 e.what());
    return return_type::ERROR;
  }

  delto_client_->SendDuty(int_duty);

  return return_type::OK;
}
}  // namespace dg5f_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dg5f_driver::SystemInterface,
                       hardware_interface::SystemInterface)
