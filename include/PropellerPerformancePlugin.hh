/*
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef PROPELLERPERFORMANCEPLUGIN_HH_
#define PROPELLERPERFORMANCEPLUGIN_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace gz {
namespace sim {
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems {

/// \brief Data-driven propeller plugin.
///
/// Reads a flat CSV of measured propeller performance data (e.g. derived from
/// APC / Flow5 PERFILES tables) and applies the resulting thrust and drag
/// torque to the rotor link each step, interpolating on (RPM, axial airspeed).
///
/// Intended as a physics-accurate replacement for the stock
/// `gz-sim-lift-drag-rotor-system` on a multirotor rotor. One plugin instance
/// per rotor.
///
/// Expected CSV format: header row `rpm,v_ms,thrust_N,torque_Nm`; one data
/// row per (RPM, airspeed) sample; rows sorted by (rpm, v_ms).
///
/// ## System Parameters:
///
///   `<link_name>` Rotor link the thrust/torque is applied to. Required.
///
///   `<joint_name>` Revolute joint that spins the rotor link. Its angular
///   velocity is read to derive RPM; its axis defines the thrust direction.
///   Required.
///
///   `<performance_file>` Path or `model://` URI to the CSV produced by
///   `tools/flow5_to_csv.py`. Required.
///
///   `<cw>` Override the sign of the reaction drag torque. Normally the sign
///   of the joint angular velocity determines the direction of the drag
///   torque automatically; set this only if the joint axis convention is
///   inverted. Default: unset (use `sign(omega)`).
///
class PropellerPerformancePlugin :
    public System,
    public ISystemPreUpdate,
    public ISystemConfigure
{
  /// \brief Destructor
  public: virtual ~PropellerPerformancePlugin();

  /// \brief Constructor
  public: PropellerPerformancePlugin();

  // Documentation inherited
  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                         gz::sim::EntityComponentManager &_ecm) final;

  // Documentation inherited
  public: void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &) final;

  /// \internal
  /// \brief Private implementation
  private: class Impl;
  private: std::unique_ptr<Impl> impl;
};

}  // namespace systems
}
}  // namespace sim
}  // namespace gz

#endif  // PROPELLERPERFORMANCEPLUGIN_HH_
