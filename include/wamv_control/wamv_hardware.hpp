// Copyright (c) 2019 OUXT Polaris
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

#ifndef WAMV_CONTROL__WAMV_HARDWARE_HPP_
#define WAMV_CONTROL__WAMV_HARDWARE_HPP_

#if defined(GALACTIC) || defined(HUMBLE)
#include <hardware_interface/system_interface.hpp>
#else
#include <hardware_interface/base_interface.hpp>
#endif
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#if defined(GALACTIC) || defined(HUMBLE)
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#else
#include <hardware_interface/types/hardware_interface_status_values.hpp>
#endif
#include <memory>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <wamv_control/visibility_control.hpp>
#include <wamv_control/wamv_driver.hpp>

using hardware_interface::return_type;

namespace wamv_control
{
class WamVHardware
#if defined(GALACTIC) || defined(HUMBLE)
: public hardware_interface::SystemInterface
#else
: public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
#endif
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(WamVHardware)

  WAMV_CONTROL_PUBLIC
  ~WamVHardware();

#if defined(GALACTIC) || defined(HUMBLE)
  WAMV_CONTROL_PUBLIC
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
#else
  WAMV_CONTROL_PUBLIC
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;
#endif

  WAMV_CONTROL_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  WAMV_CONTROL_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

#if defined(HUMBLE)

  WAMV_CONTROL_PUBLIC
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  WAMV_CONTROL_PUBLIC
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

#else

  WAMV_CONTROL_PUBLIC
  return_type read() override;

  WAMV_CONTROL_PUBLIC
  return_type write() override;

#endif

private:
  std::shared_ptr<WamVDriver> driver_;
  double left_thrust_cmd_;
  double right_thrust_cmd_;
  std::string left_thruster_joint_;
  std::string right_thruster_joint_;
};
}  // namespace wamv_control

#endif  // WAMV_CONTROL__WAMV_HARDWARE_HPP_
