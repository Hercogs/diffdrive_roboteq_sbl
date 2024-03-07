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

#ifndef DIFFDRIVE_ROBOTEQ_SBL__DIFFBOT_SYSTEM_HPP_
#define DIFFDRIVE_ROBOTEQ_SBL__DIFFBOT_SYSTEM_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "diffdrive_roboteq_sbl/visibility_control.h"
 
// Custom lib
#include "diffdrive_roboteq_sbl/RoboteqDevice.h"
#include "diffdrive_roboteq_sbl/ErrorCodes.h"
#include "diffdrive_roboteq_sbl/wheel.hpp"

namespace diffdrive_roboteq_sbl
{
class DiffDriveRoboteqHardwareSbl : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(DiffDriveRoboteqHardwareSbl);

  DIFFDRIVE_ROBOTEQ_SBL_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  DIFFDRIVE_ROBOTEQ_SBL_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  DIFFDRIVE_ROBOTEQ_SBL_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  DIFFDRIVE_ROBOTEQ_SBL_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  DIFFDRIVE_ROBOTEQ_SBL_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  DIFFDRIVE_ROBOTEQ_SBL_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  DIFFDRIVE_ROBOTEQ_SBL_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  DIFFDRIVE_ROBOTEQ_SBL_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  DIFFDRIVE_ROBOTEQ_SBL_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  DIFFDRIVE_ROBOTEQ_SBL_PUBLIC
  void init_settings();


private:
  // Declare structure
  struct Config
  {
    std::string left_wheel_name = "";
    std::string right_wheel_name = "";
    float loop_rate {0.0};
    std::string device_name = "";
    int baud_rate {0};
    int timeout_ms {0};
    int enc_counts_per_rev {0};
    int reducer_ratio {0};

    bool initialized {false};
  };

  // Controler object
  RoboteqDevice device_;

  // Config
  Config cfg_;

  Wheel wheel_l_;
  Wheel wheel_r_;


};

}  // namespace diffdrive_roboteq_sbl

#endif  // DIFFDRIVE_ROBOTEQ_SBL__DIFFBOT_SYSTEM_HPP_
