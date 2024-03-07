// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "diffdrive_roboteq_sbl/diffbot_system.hpp"
#include "diffdrive_roboteq_sbl/Constants.h"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diffdrive_roboteq_sbl
{
hardware_interface::CallbackReturn DiffDriveRoboteqHardwareSbl::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // harware info -> parameters from xacro
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Save parameters from xacro
  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device_name = info_.hardware_parameters["device_name"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  cfg_.reducer_ratio = std::stoi(info_.hardware_parameters["reducer_ratio"]);

  wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev * cfg_.reducer_ratio);
  wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev * cfg_.reducer_ratio);


  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveRoboteqHardwareSbl"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveRoboteqHardwareSbl"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveRoboteqHardwareSbl"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveRoboteqHardwareSbl"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("DiffDriveRoboteqHardwareSbl"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // TODO clean odometry data from controller, get max speed

  return hardware_interface::CallbackReturn::SUCCESS;
}

/*
* What things can be readed
*/
std::vector<hardware_interface::StateInterface> DiffDriveRoboteqHardwareSbl::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

  return state_interfaces;
}

/*
* What things can be controlled
*/
std::vector<hardware_interface::CommandInterface> DiffDriveRoboteqHardwareSbl::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));


  return command_interfaces;
}


hardware_interface::CallbackReturn DiffDriveRoboteqHardwareSbl::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveRoboteqHardwareSbl"), "Configuring ...please wait...");

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveRoboteqHardwareSbl"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveRoboteqHardwareSbl::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveRoboteqHardwareSbl"), "Cleaning ...please wait...");
  device_.Disconnect();
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveRoboteqHardwareSbl"), "Successfully cleaned!");

  return hardware_interface::CallbackReturn::SUCCESS;
}



hardware_interface::CallbackReturn DiffDriveRoboteqHardwareSbl::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveRoboteqHardwareSbl"), "Activating ...please wait...");

  RCLCPP_INFO(rclcpp::get_logger("DiffDriveRoboteqHardwareSbl"), "Opening port ... %s", cfg_.device_name.c_str()); 

  int status = device_.Connect(cfg_.device_name);
  if (status != RQ_SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveRoboteqHardwareSbl"), "Could not connect to controller," \
              " got status code: %d", status);
    device_.Disconnect();
              
    //return hardware_interface::CallbackReturn::ERROR;
  } else 
  {
    this->init_settings();
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveRoboteqHardwareSbl"), "Successfully activated!");
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DiffDriveRoboteqHardwareSbl::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveRoboteqHardwareSbl"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveRoboteqHardwareSbl"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DiffDriveRoboteqHardwareSbl::read(
  const rclcpp::Time & /*time*/, [[maybe_unused]] const rclcpp::Duration & period)
{
  int status;

  // Check if connected
  if (!device_.IsConnected())
  {
    status = device_.Connect(cfg_.device_name);
    if (status != RQ_SUCCESS)
    {
      device_.Disconnect();
    }else
    {
      RCLCPP_INFO(rclcpp::get_logger("DiffDriveRoboteqHardwareSbl"), "Connected back to controller!");
    }
    return hardware_interface::return_type::OK;
  }

  // Check if configured
  if(!cfg_.initialized)
  {
    init_settings();
    return hardware_interface::return_type::OK;
  }
  
  if ((status = device_.GetValue(_CB, 1, wheel_l_.enc)) != RQ_SUCCESS)
  {
    if (status == RQ_ERR_TRANSMIT_FAILED)
    {
      device_.Disconnect();
            RCLCPP_ERROR(rclcpp::get_logger("DiffDriveRoboteqHardwareSbl"), "BLDC disconecting - technical reason???");
      return hardware_interface::return_type::OK;
    }
  }

  if ((status = device_.GetValue(_CB, 2, wheel_r_.enc)) != RQ_SUCCESS)
  {
    if (status == RQ_ERR_TRANSMIT_FAILED)
    {
      device_.Disconnect();
            RCLCPP_ERROR(rclcpp::get_logger("DiffDriveRoboteqHardwareSbl"), "BLDC disconecting - technical reason???");
      return hardware_interface::return_type::OK;
    }
  }
  
  double delta_seconds = period.seconds();

  double pos_prev;
  pos_prev = wheel_l_.pos;
  wheel_l_.pos = wheel_l_.calc_enc_angle();
  wheel_l_.vel = (wheel_l_.pos - pos_prev) / delta_seconds;

  pos_prev = wheel_r_.pos;
  wheel_r_.pos = wheel_r_.calc_enc_angle();
  wheel_r_.vel = (wheel_r_.pos - pos_prev) / delta_seconds;

  //RCLCPP_INFO(rclcpp::get_logger("DiffDriveRoboteqHardwareSbl"), "Read: %.2f", wheel_l_.pos);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type diffdrive_roboteq_sbl ::DiffDriveRoboteqHardwareSbl::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  //RCLCPP_INFO(rclcpp::get_logger("DiffDriveRoboteqHardwareSbl"), "Write cmd: %.2f %.2f", wheel_l_.cmd, wheel_r_.cmd);
  
  // Input (rad/sec)

  // Check if connected
  if (!device_.IsConnected())
  {
    return hardware_interface::return_type::OK;
  }
  // Check if configured
  if(!cfg_.initialized)
  {
    init_settings(); 
    return hardware_interface::return_type::OK;
  }

  int max_vel = 1800; // From controller

  // Convert to rpm and multiply with reducer ratio
  float rpm_left = wheel_l_.cmd * 60 / (2 * 3.14) * cfg_.reducer_ratio;
  float rpm_right = wheel_r_.cmd * 60 / (2 * 3.14) * cfg_.reducer_ratio;

  // Convert to controller scale
  // max_vel -> 1000

  int rpm_left_scaled = (rpm_left / max_vel) * 1000;
  int rpm_right_scaled = (rpm_right / max_vel) * 1000;

  int status;

  if ((status = device_.SetCommand(_G, 1, rpm_left_scaled)) != RQ_SUCCESS)
  {
    if (status == RQ_ERR_TRANSMIT_FAILED)
    {
      device_.Disconnect();
            RCLCPP_ERROR(rclcpp::get_logger("DiffDriveRoboteqHardwareSbl"), "BLDC disconecting - technical reason???");
      return hardware_interface::return_type::OK;
      //cout << "Manual disconecting" << status << endl;
      //RCLCPP_ERROR(rclcpp::get_logger("DiffDriveRoboteqHardwareSbl"), "BLDC disconecting - technical reason???");
      //return hardware_interface::return_type::ERROR;
    }
  }
  else
  {

  }

  if ((status = device_.SetCommand(_G, 2, rpm_right_scaled)) != RQ_SUCCESS)
  {
    if (status == RQ_ERR_TRANSMIT_FAILED)
    {
      device_.Disconnect();
            RCLCPP_ERROR(rclcpp::get_logger("DiffDriveRoboteqHardwareSbl"), "BLDC disconecting - technical reason???");
      return hardware_interface::return_type::OK;
    }
  }
  else
  {

  }


  return hardware_interface::return_type::OK;
}

void DiffDriveRoboteqHardwareSbl::init_settings()
{
  int result;
  int status;

	RCLCPP_INFO(rclcpp::get_logger("DiffDriveRoboteqHardwareSbl"), "- Read setting MAXRPM: GetValue(_VAR, 10)...");
	
  if ((status = device_.GetConfig(_MXRPM, 1, result)) != RQ_SUCCESS)
  {
		RCLCPP_INFO(rclcpp::get_logger("DiffDriveRoboteqHardwareSbl"), "- GetConfig(_MXRPM, 1) failed");
    return;
  } else
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveRoboteqHardwareSbl"), "- GetConfig(_MXRPM, 1) succeded: %d", result);
    wheel_l_.max_rpm = result;
  }

  sleepms(10);  // Sleep 10 ms

  if ((status = device_.GetConfig(_MXRPM, 2, result)) != RQ_SUCCESS)
  {
		RCLCPP_INFO(rclcpp::get_logger("DiffDriveRoboteqHardwareSbl"), "- GetConfig(_MXRPM, 2) failed");
    return;
  } else
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveRoboteqHardwareSbl"), "- GetConfig(_MXRPM, 2) succeded: %d", result);
    wheel_r_.max_rpm = result;
  }

  sleepms(10);  // Sleep 10 ms

  if ((status = device_.SetCommand(_CB, 1, 0)) != RQ_SUCCESS)
  {
		RCLCPP_INFO(rclcpp::get_logger("DiffDriveRoboteqHardwareSbl"), "- SetCommand(_CB, 1, 0) failed");
    return;
  } else
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveRoboteqHardwareSbl"), "- SetCommand(_CB, 1, 0) succeded");
  }

  sleepms(10);  // Sleep 10 ms

  if ((status = device_.SetCommand(_CB, 2, 0)) != RQ_SUCCESS)
  {
		RCLCPP_INFO(rclcpp::get_logger("DiffDriveRoboteqHardwareSbl"), "- SetCommand(_CB, 2, 0) failed");
    return;
  } else
  {
    RCLCPP_INFO(rclcpp::get_logger("DiffDriveRoboteqHardwareSbl"), "- SetCommand(_CB, 2, 0) secceded");
  }

  cfg_.initialized = true;
  RCLCPP_INFO(rclcpp::get_logger("DiffDriveRoboteqHardwareSbl"), "HW interfaces configured!");

}


}  // namespace diffdrive_roboteq_sbl

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diffdrive_roboteq_sbl::DiffDriveRoboteqHardwareSbl, hardware_interface::SystemInterface)
