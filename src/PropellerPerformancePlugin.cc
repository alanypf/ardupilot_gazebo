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

#include "PropellerPerformancePlugin.hh"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#include <gz/common/Profiler.hh>
#include <gz/common/SystemPaths.hh>
#include <gz/common/URI.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/vector3d.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/transport/Node.hh>

namespace gz {
namespace sim {
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems {

namespace {

constexpr double kRadPerSecToRpm = 60.0 / (2.0 * M_PI);

/// \brief Resolve a user-supplied file path. Supports absolute paths, paths
/// relative to the SDF file that declares the plugin, and `model://` URIs
/// (resolved via `GZ_SIM_RESOURCE_PATH`).
std::string ResolvePath(const std::string &_path,
                        const std::string &_sdfFilePath)
{
  if (_path.empty())
  {
    return _path;
  }
  if (_path.find("://") != std::string::npos)
  {
    common::SystemPaths sp;
    sp.SetFilePathEnv("GZ_SIM_RESOURCE_PATH");
    return sp.FindFileURI(common::URI(_path));
  }
  std::filesystem::path p(_path);
  if (p.is_absolute())
  {
    return _path;
  }
  if (!_sdfFilePath.empty())
  {
    return (std::filesystem::path(_sdfFilePath).parent_path() / p).string();
  }
  return _path;
}

/// \brief One sample in the propeller table.
struct Sample
{
  double rpm;
  double v_ms;
  double thrust_N;
  double torque_Nm;
};

/// \brief 1D linear interpolation with edge clamping.
double Lerp1D(const std::vector<double> &xs,
              const std::vector<double> &ys,
              double x)
{
  if (xs.empty()) return 0.0;
  if (x <= xs.front()) return ys.front();
  if (x >= xs.back()) return ys.back();
  auto it = std::upper_bound(xs.begin(), xs.end(), x);
  size_t hi = std::distance(xs.begin(), it);
  size_t lo = hi - 1;
  double t = (x - xs[lo]) / (xs[hi] - xs[lo]);
  return ys[lo] + t * (ys[hi] - ys[lo]);
}

/// \brief 2D bilinear lookup table grouped by RPM.
class PerfTable
{
  public: bool Load(const std::string &_path, std::string &_err);

  /// \brief Interpolate (thrust_N, torque_Nm) at the given RPM and axial
  /// airspeed. Clamped to table edges (no extrapolation).
  public: std::pair<double, double> At(double _rpm, double _v) const;

  public: bool Empty() const { return this->rpms.empty(); }

  /// \brief Unique sorted RPM values.
  private: std::vector<double> rpms;

  /// \brief Per-RPM block: airspeeds sorted ascending.
  private: std::vector<std::vector<double>> speeds;
  private: std::vector<std::vector<double>> thrust;
  private: std::vector<std::vector<double>> torque;
};

bool PerfTable::Load(const std::string &_path, std::string &_err)
{
  std::ifstream f(_path);
  if (!f.is_open())
  {
    _err = "could not open '" + _path + "'";
    return false;
  }

  std::vector<Sample> samples;
  std::string line;
  bool header = true;
  size_t lineNo = 0;
  while (std::getline(f, line))
  {
    ++lineNo;
    if (line.empty()) continue;
    if (header)
    {
      header = false;
      continue;  // skip header row
    }
    std::istringstream ss(line);
    std::string tok;
    std::vector<double> vals;
    while (std::getline(ss, tok, ','))
    {
      try { vals.push_back(std::stod(tok)); }
      catch (const std::exception &)
      {
        _err = "malformed number at line " + std::to_string(lineNo);
        return false;
      }
    }
    if (vals.size() < 4)
    {
      _err = "too few columns at line " + std::to_string(lineNo);
      return false;
    }
    samples.push_back({vals[0], vals[1], vals[2], vals[3]});
  }
  if (samples.empty())
  {
    _err = "no data rows in '" + _path + "'";
    return false;
  }

  std::sort(samples.begin(), samples.end(),
      [](const Sample &a, const Sample &b) {
        return std::tie(a.rpm, a.v_ms) < std::tie(b.rpm, b.v_ms);
      });

  double currentRpm = samples.front().rpm - 1.0;
  for (const auto &s : samples)
  {
    if (s.rpm != currentRpm)
    {
      this->rpms.push_back(s.rpm);
      this->speeds.emplace_back();
      this->thrust.emplace_back();
      this->torque.emplace_back();
      currentRpm = s.rpm;
    }
    this->speeds.back().push_back(s.v_ms);
    this->thrust.back().push_back(s.thrust_N);
    this->torque.back().push_back(s.torque_Nm);
  }
  return true;
}

std::pair<double, double> PerfTable::At(double _rpm, double _v) const
{
  if (this->rpms.empty()) return {0.0, 0.0};

  // Find bracketing RPM blocks.
  size_t loIdx = 0;
  size_t hiIdx = 0;
  double t = 0.0;
  if (_rpm <= this->rpms.front())
  {
    loIdx = hiIdx = 0;
  }
  else if (_rpm >= this->rpms.back())
  {
    loIdx = hiIdx = this->rpms.size() - 1;
  }
  else
  {
    auto it = std::upper_bound(this->rpms.begin(), this->rpms.end(), _rpm);
    hiIdx = std::distance(this->rpms.begin(), it);
    loIdx = hiIdx - 1;
    t = (_rpm - this->rpms[loIdx]) /
        (this->rpms[hiIdx] - this->rpms[loIdx]);
  }

  const double thrLo = Lerp1D(this->speeds[loIdx], this->thrust[loIdx], _v);
  const double torLo = Lerp1D(this->speeds[loIdx], this->torque[loIdx], _v);
  if (loIdx == hiIdx)
  {
    return {thrLo, torLo};
  }
  const double thrHi = Lerp1D(this->speeds[hiIdx], this->thrust[hiIdx], _v);
  const double torHi = Lerp1D(this->speeds[hiIdx], this->torque[hiIdx], _v);
  return {thrLo + t * (thrHi - thrLo), torLo + t * (torHi - torLo)};
}

}  // namespace

//////////////////////////////////////////////////
class PropellerPerformancePlugin::Impl
{
  /// \brief Model the plugin is attached to.
  public: Model parentModel{kNullEntity};

  /// \brief SDF-configured link and joint names.
  public: std::string linkName;
  public: std::string jointName;

  /// \brief Resolved link and joint entities.
  public: Link link{kNullEntity};
  public: Entity jointEntity{kNullEntity};

  /// \brief Joint axis in child-link frame (unit vector).
  public: math::Vector3d axisLocal{0.0, 0.0, 1.0};

  /// \brief Optional explicit torque-sign override. When set, replaces the
  /// sign derived from omega: +1.0 for CW, -1.0 for CCW (in the
  /// child-link-axis convention).
  public: std::optional<double> torqueSignOverride;

  /// \brief Propeller performance data.
  public: PerfTable table;

  /// \brief Debug telemetry publisher. Advertises a `msgs::Vector3d` per
  /// rotor carrying (x=rpm, y=v_axial_ms, z=thrust_N). Topic is
  /// `/model/<model>/<link>/perf`.
  public: transport::Node node;
  public: transport::Node::Publisher perfPub;

  /// \brief True once Configure finished successfully.
  public: bool validConfig{false};

  /// \brief True once entities are resolved in PreUpdate.
  public: bool entitiesResolved{false};
};

//////////////////////////////////////////////////
PropellerPerformancePlugin::~PropellerPerformancePlugin() = default;

//////////////////////////////////////////////////
PropellerPerformancePlugin::PropellerPerformancePlugin() :
    impl(std::make_unique<PropellerPerformancePlugin::Impl>())
{
}

//////////////////////////////////////////////////
void PropellerPerformancePlugin::Configure(
    const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &)
{
  this->impl->parentModel = Model(_entity);
  if (!this->impl->parentModel.Valid(_ecm))
  {
    gzerr << "PropellerPerformancePlugin must be attached to a model.\n";
    return;
  }

  auto required = [&](const char *tag, std::string &out) -> bool {
    if (!_sdf->HasElement(tag))
    {
      gzerr << "PropellerPerformancePlugin requires <" << tag << ">.\n";
      return false;
    }
    out = _sdf->Get<std::string>(tag);
    return true;
  };

  std::string perfFile;
  if (!required("link_name", this->impl->linkName) ||
      !required("joint_name", this->impl->jointName) ||
      !required("performance_file", perfFile))
  {
    return;
  }

  const std::string resolved = ResolvePath(perfFile, _sdf->FilePath());
  std::string err;
  if (!this->impl->table.Load(resolved, err))
  {
    gzerr << "PropellerPerformancePlugin failed to load "
          << "performance table: " << err << " (input '" << perfFile
          << "' resolved to '" << resolved << "')\n";
    return;
  }

  if (_sdf->HasElement("cw"))
  {
    this->impl->torqueSignOverride =
        _sdf->Get<bool>("cw") ? 1.0 : -1.0;
  }

  const std::string topic = transport::TopicUtils::AsValidTopic(
      "/model/" + this->impl->parentModel.Name(_ecm) + "/" +
      this->impl->linkName + "/perf");
  if (!topic.empty())
  {
    this->impl->perfPub =
        this->impl->node.Advertise<msgs::Vector3d>(topic);
  }

  this->impl->validConfig = true;
}

//////////////////////////////////////////////////
void PropellerPerformancePlugin::PreUpdate(
    const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("PropellerPerformancePlugin::PreUpdate");

  if (!this->impl->validConfig || _info.paused)
  {
    return;
  }

  if (!this->impl->entitiesResolved)
  {
    this->impl->link = Link(_ecm.EntityByComponents(
        components::Link(),
        components::ParentEntity(this->impl->parentModel.Entity()),
        components::Name(this->impl->linkName)));
    if (!this->impl->link.Valid(_ecm))
    {
      gzerr << "PropellerPerformancePlugin: link '" << this->impl->linkName
            << "' not found.\n";
      this->impl->validConfig = false;
      return;
    }
    this->impl->link.EnableVelocityChecks(_ecm);

    this->impl->jointEntity = this->impl->parentModel.JointByName(
        _ecm, this->impl->jointName);
    if (this->impl->jointEntity == kNullEntity)
    {
      gzerr << "PropellerPerformancePlugin: joint '"
            << this->impl->jointName << "' not found.\n";
      this->impl->validConfig = false;
      return;
    }
    if (!_ecm.EntityHasComponentType(
            this->impl->jointEntity, components::JointVelocity::typeId))
    {
      _ecm.CreateComponent(this->impl->jointEntity,
          components::JointVelocity());
    }

    const auto *axisComp =
        _ecm.Component<components::JointAxis>(this->impl->jointEntity);
    if (axisComp != nullptr)
    {
      auto xyz = axisComp->Data().Xyz();
      if (xyz.Length() > 1e-9)
      {
        this->impl->axisLocal = xyz.Normalized();
      }
    }

    this->impl->entitiesResolved = true;
  }

  const auto *velComp = _ecm.Component<components::JointVelocity>(
      this->impl->jointEntity);
  if (velComp == nullptr || velComp->Data().empty())
  {
    return;  // velocity not populated yet
  }
  const double omega = velComp->Data()[0];
  const double rpm = std::abs(omega) * kRadPerSecToRpm;

  const auto linkPose = this->impl->link.WorldPose(_ecm);
  const auto linkVel = this->impl->link.WorldLinearVelocity(_ecm);
  if (!linkPose.has_value() || !linkVel.has_value())
  {
    return;
  }

  const math::Vector3d axisWorld =
      linkPose->Rot().RotateVector(this->impl->axisLocal);
  const double vAxial = std::max(0.0, axisWorld.Dot(linkVel.value()));

  const auto [thrust, torque] = this->impl->table.At(rpm, vAxial);

  const math::Vector3d force = thrust * axisWorld;

  double torqueSign;
  if (this->impl->torqueSignOverride.has_value())
  {
    torqueSign = this->impl->torqueSignOverride.value();
  }
  else
  {
    torqueSign = (omega >= 0.0) ? -1.0 : 1.0;
  }
  const math::Vector3d torqueVec = (torqueSign * torque) * axisWorld;

  this->impl->link.AddWorldWrench(_ecm, force, torqueVec);

  if (this->impl->perfPub.Valid())
  {
    msgs::Vector3d msg;
    msg.set_x(rpm);
    msg.set_y(vAxial);
    msg.set_z(thrust);
    this->impl->perfPub.Publish(msg);
  }
}

//////////////////////////////////////////////////

}  // namespace systems
}
}  // namespace sim
}  // namespace gz

GZ_ADD_PLUGIN(
    gz::sim::systems::PropellerPerformancePlugin,
    gz::sim::System,
    gz::sim::systems::PropellerPerformancePlugin::ISystemConfigure,
    gz::sim::systems::PropellerPerformancePlugin::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
    gz::sim::systems::PropellerPerformancePlugin,
    "PropellerPerformancePlugin")
