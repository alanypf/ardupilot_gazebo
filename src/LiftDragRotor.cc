/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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

#include "LiftDragRotor.hh"

#include <algorithm>
#include <string>
#include <vector>
#include <cmath>

#include <gz/common/Profiler.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <sdf/Element.hh>

#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/Util.hh"

#include "gz/sim/components/AngularVelocity.hh"
#include "gz/sim/components/Inertial.hh"
#include "gz/sim/components/Joint.hh"
#include "gz/sim/components/JointPosition.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/components/Link.hh"
#include "gz/sim/components/Name.hh"
#include "gz/sim/components/ExternalWorldWrenchCmd.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Wind.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::LiftDragRotorPrivate
{
  // Initialize the system
  public: void Load(const EntityComponentManager &_ecm,
                    const sdf::ElementPtr &_sdf);

  /// \brief Compute lift and drag forces and update the corresponding
  /// components
  /// \param[in] _ecm Immutable reference to the EntityComponentManager
  public: void Update(EntityComponentManager &_ecm);

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief Coefficient of Drag / alpha slope.
  /// Drag = C_D * q * S
  /// where q (dynamic pressure) = 0.5 * rho * v^2
  public: double cda = 0.01;

  /// \brief Coefficient of Moment / alpha slope.
  /// Moment = C_M * q * S
  /// where q (dynamic pressure) = 0.5 * rho * v^2
  public: double cma = 0.0;

  /// \brief angle of attach when airfoil stalls
  public: double alphaStall = GZ_PI_2;

  /// \brief Cd-alpha rate after stall
  /// \todo(anyone): what's flat plate drag?
  public: double cdaStall = 1.0;

  /// \brief Cm-alpha rate after stall
  public: double cmaStall = 0.0;

  /// \brief How much Cm changes with a change in control
  /// surface deflection angle
  public: double cm_delta = 0.0;

  /// \brief air density
  /// at 25 deg C it's about 1.1839 kg/m^3
  /// At 20 °C and 101.325 kPa, dry air has a density of 1.2041 kg/m3.
  public: double rho = 1.2041;

  /// \brief rotor diameter, needed for calculating advance ratio J
  public: double Diameter = 0.2041;

  /// \brief if the shape is aerodynamically radially symmetric about
  /// the forward direction. Defaults to false for wing shapes.
  /// If set to true, the upward direction is determined by the
  /// angle of attack.
  public: bool radialSymmetry = false;

  /// \brief effective planeform surface area
  public: double area = 1.0;

  /// \brief initial angle of attack
  public: double alpha0 = 0.0;

  /// \brief center of pressure in link local coordinates with respect to the
  /// link's center of mass
  public: gz::math::Vector3d cp = math::Vector3d::Zero;

  /// \brief Normally, this is taken as a direction parallel to the chord
  /// of the airfoil in zero angle of attack forward flight.
  public: math::Vector3d forward = math::Vector3d::UnitX;

  /// \brief A vector in the lift/drag plane, perpendicular to the forward
  /// vector. Inflow velocity orthogonal to forward and upward vectors
  /// is considered flow in the wing sweep direction.
  public: math::Vector3d upward = math::Vector3d::UnitZ;

  /// \brief how much to change CL per radian of control surface joint
  /// value.
  public: double controlJointRadToCL = 4.0;

  /// \brief Link entity targeted this plugin.
  public: Entity linkEntity;

  /// \brief Joint entity that actuates a control surface for this lifting body
  public: Entity controlJointEntity;

  /// \brief Set during Load to true if the configuration for the system is
  /// valid and the post-update can run
  public: bool validConfig{false};

  /// \brief Copy of the sdf configuration used for this plugin
  public: sdf::ElementPtr sdfConfig;

  /// \brief Initialization flag
  public: bool initialized{false};
};

//////////////////////////////////////////////////
void LiftDragRotorPrivate::Load(const EntityComponentManager &_ecm,
                           const sdf::ElementPtr &_sdf)
{
  if (!this->model.Valid(_ecm))
  {
    gzerr << "The LiftDragRotor system should be attached to a model entity. "
           << "Failed to initialize." << std::endl;
    return;
  }
  this->cda = _sdf->Get<double>("cda", this->cda).first;
  this->cma = _sdf->Get<double>("cma", this->cma).first;
  this->alphaStall = _sdf->Get<double>("alpha_stall", this->alphaStall).first;
  this->cdaStall = _sdf->Get<double>("cda_stall", this->cdaStall).first;
  this->cmaStall = _sdf->Get<double>("cma_stall", this->cmaStall).first;
  this->rho = _sdf->Get<double>("air_density", this->rho).first;
  this->Diameter = _sdf->Get<double>("diameter", this->Diameter).first;
  this->radialSymmetry = _sdf->Get<bool>("radial_symmetry",
      this->radialSymmetry).first;
  this->area = _sdf->Get<double>("area", this->area).first;
  this->alpha0 = _sdf->Get<double>("a0", this->alpha0).first;
  this->cp = _sdf->Get<math::Vector3d>("cp", this->cp).first;
  this->cm_delta = _sdf->Get<double>("cm_delta", this->cm_delta).first;

  // blade forward (-drag) direction in link frame
  this->forward =
      _sdf->Get<math::Vector3d>("forward", this->forward).first;
  this->forward.Normalize();

  // blade upward (+lift) direction in link frame
  this->upward = _sdf->Get<math::Vector3d>(
      "upward", this->upward) .first;
  this->upward.Normalize();

  this->controlJointRadToCL = _sdf->Get<double>(
      "control_joint_rad_to_cl", this->controlJointRadToCL).first;

  if (_sdf->HasElement("link_name"))
  {
    sdf::ElementPtr elem = _sdf->GetElement("link_name");
    auto linkName = elem->Get<std::string>();
    auto entities =
        entitiesFromScopedName(linkName, _ecm, this->model.Entity());

    if (entities.empty())
    {
      gzerr << "Link with name[" << linkName << "] not found. "
             << "The LiftDragRotor will not generate forces\n";
      this->validConfig = false;
      return;
    }
    else if (entities.size() > 1)
    {
      gzwarn << "Multiple link entities with name[" << linkName << "] found. "
             << "Using the first one.\n";
    }

    this->linkEntity = *entities.begin();
    if (!_ecm.EntityHasComponentType(this->linkEntity,
                                     components::Link::typeId))
    {
      this->linkEntity = kNullEntity;
      gzerr << "Entity with name[" << linkName << "] is not a link\n";
      this->validConfig = false;
      return;
    }
  }
  else
  {
    gzerr << "The LiftDragRotor system requires the 'link_name' parameter\n";
    this->validConfig = false;
    return;
  }

  if (_sdf->HasElement("control_joint_name"))
  {
    auto controlJointName = _sdf->Get<std::string>("control_joint_name");
    auto entities =
        entitiesFromScopedName(controlJointName, _ecm, this->model.Entity());

    if (entities.empty())
    {
      gzerr << "Joint with name[" << controlJointName << "] not found. "
             << "The LiftDragRotor will not generate forces\n";
      this->validConfig = false;
      return;
    }
    else if (entities.size() > 1)
    {
      gzwarn << "Multiple joint entities with name[" << controlJointName
              << "] found. Using the first one.\n";
    }

    this->controlJointEntity = *entities.begin();
    if (!_ecm.EntityHasComponentType(this->controlJointEntity,
                                     components::Joint::typeId))
    {
      this->controlJointEntity = kNullEntity;
      gzerr << "Entity with name[" << controlJointName << "] is not a joint\n";
      this->validConfig = false;
      return;
    }
  }

  // If we reached here, we have a valid configuration
  this->validConfig = true;
}

//////////////////////////////////////////////////
LiftDragRotor::LiftDragRotor()
    : System(), dataPtr(std::make_unique<LiftDragRotorPrivate>())
{
}

//////////////////////////////////////////////////
void LiftDragRotorPrivate::Update(EntityComponentManager &_ecm)
{
  GZ_PROFILE("LiftDragRotorPrivate::Update");
  // get linear velocity at cp in world frame
  const auto worldLinVel =
      _ecm.Component<components::WorldLinearVelocity>(this->linkEntity);
  const auto worldAngVel =
      _ecm.Component<components::WorldAngularVelocity>(this->linkEntity);
  const auto worldPose =
      _ecm.Component<components::WorldPose>(this->linkEntity);

  // get wind as a component from the _ecm
  components::WorldLinearVelocity *windLinearVel = nullptr;
  if(_ecm.EntityByComponents(components::Wind()) != kNullEntity){
    Entity windEntity = _ecm.EntityByComponents(components::Wind());
    windLinearVel =
        _ecm.Component<components::WorldLinearVelocity>(windEntity);
  }
  components::JointPosition *controlJointPosition = nullptr;
  if (this->controlJointEntity != kNullEntity)
  {
    controlJointPosition =
        _ecm.Component<components::JointPosition>(this->controlJointEntity);
  }

  if (!worldLinVel || !worldAngVel || !worldPose)
  {
    return;
  }

  const auto &pose = worldPose->Data();
  const auto cpWorld = pose.Rot().RotateVector(this->cp);
  auto vel = worldLinVel->Data() + worldAngVel->Data().Cross(
  cpWorld);
  if (windLinearVel != nullptr){
    vel = worldLinVel->Data() + worldAngVel->Data().Cross(
    cpWorld) - windLinearVel->Data();
  }

  if (vel.Length() <= 0.01)
  {
    return;
  }

  const auto velI = vel.Normalized();

  // rotate forward and upward vectors into world frame
  const auto forwardI = pose.Rot().RotateVector(this->forward);

  if (forwardI.Dot(vel) <= 0.0){
    // Only calculate lift or drag if the wind relative velocity
    // is in the same direction
    return;
  }

  math::Vector3d upwardI;
  if (this->radialSymmetry)
  {
    // use inflow velocity to determine upward direction
    // which is the component of inflow perpendicular to forward direction.
    math::Vector3d tmp = forwardI.Cross(velI);
    upwardI = forwardI.Cross(tmp).Normalize();
  }
  else
  {
    upwardI = pose.Rot().RotateVector(this->upward);
  }

  // spanwiseI: a vector normal to lift-drag-plane described in world frame
  const auto spanwiseI = forwardI.Cross(upwardI).Normalize();

  const double minRatio = -1.0;
  const double maxRatio = 1.0;
  // check sweep (angle between velI and lift-drag-plane)
  double sinSweepAngle = math::clamp(
      spanwiseI.Dot(velI), minRatio, maxRatio);

  // The sweep adjustment depends on the velocity component normal to
  // the wing leading edge which appears quadratically in the
  // dynamic pressure, so scale by cos^2 .
  double cos2SweepAngle = 1.0 - sinSweepAngle * sinSweepAngle;
  double sweep = std::asin(sinSweepAngle);

  // truncate sweep to within +/-90 deg
  while (std::fabs(sweep) > 0.5 * GZ_PI)
  {
    sweep = sweep > 0 ? sweep - GZ_PI : sweep + GZ_PI;
  }

  // angle of attack is the angle between
  // velI projected into lift-drag plane
  //  and
  // forward vector
  //
  // projected = spanwiseI Xcross ( vector Xcross spanwiseI)
  //
  // so,
  // removing spanwise velocity from vel
  // Note: Original code had:
  //    const auto velInLDPlane = vel - vel.Dot(spanwiseI)*velI;
  // I believe the projection should be onto spanwiseI which then gets removed
  // from vel
  const auto velInLDPlane = vel - vel.Dot(spanwiseI)*spanwiseI;

  // get direction of drag
  const auto dragDirection = -velInLDPlane.Normalized();

  // get direction of lift
  const auto liftI = spanwiseI.Cross(velInLDPlane).Normalized();

  // compute angle between upwardI and liftI
  // in general, given vectors a and b:
  //   cos(theta) = a.Dot(b)/(a.Length()*b.Length())
  // given upwardI and liftI are both unit vectors, we can drop the denominator
  //   cos(theta) = a.Dot(b)
  const double cosAlpha =
      math::clamp(liftI.Dot(upwardI), minRatio, maxRatio);

  // Is alpha positive or negative? Test:
  // forwardI points toward zero alpha
  // if forwardI is in the same direction as lift, alpha is positive.
  // liftI is in the same direction as forwardI?
  double alpha = this->alpha0 - std::acos(cosAlpha);
  if (liftI.Dot(forwardI) >= 0.0)
    alpha = this->alpha0 + std::acos(cosAlpha);

  // normalize to within +/-90 deg
  while (fabs(alpha) > 0.5 * GZ_PI)
  {
    alpha = alpha > 0 ? alpha - GZ_PI : alpha + GZ_PI;
  }

  // compute dynamic pressure
  const double speedInLDPlane = velInLDPlane.Length();
  const double q = 0.5 * this->rho * speedInLDPlane * speedInLDPlane;

  // compute lift force at cp
  // math::Vector3d lift = cl * q * this->area * liftI;

  // ---------------获取连杆的角速度
  // 1. 获取旋翼轴方向（在连杆局部坐标系）
  // 假设旋翼轴是局部Z轴
  math::Vector3d rotorAxisLocal(0, 0, 1);
  
  // 2. 转换到世界坐标系
  math::Vector3d rotorAxisWorld = worldPose->Data().Rot().RotateVector(
      rotorAxisLocal);
  rotorAxisWorld.Normalize();
  
  // 3. 投影：提取沿旋翼轴方向的角速度分量
  double rotorSpeed = worldAngVel->Data().Dot(rotorAxisWorld);
  rotorSpeed = rotorSpeed * 2.0f;
  // 4. 输出结果
  gzdbg << "旋翼转速: " << rotorSpeed << " rad/s, "
          << rotorSpeed * 60.0 / (2.0 * M_PI) << " RPM" << std::endl;
    
  // 1. ---------------获取连杆质心的线速度（世界坐标系）
  math::Vector3d linkLinearVelocity = worldLinVel->Data();  // 单位：m/s
  
  // 2. 压力中心处的速度
  auto velAtCp = linkLinearVelocity + worldAngVel->Data().Cross(cpWorld);
  
  if (windLinearVel != nullptr)
  {
    velAtCp = velAtCp - windLinearVel->Data();
  }
    
  // 计算在不同方向上的速度分量
  double upwardSpeed = velAtCp.Dot(upwardI);       // 升力方向速度  
  gzdbg << "升力方向速度: " << upwardSpeed << " m/s" << std::endl;
  auto J = 10 * upwardSpeed / (std::fabs(rotorSpeed) * this->Diameter); // 计算进气比
  if(J < 0.0 || J > 2.0){ // sanity check, J should be positive and not unreasonably large
    J = 0.0; // 进气比不合理，设置为0，表示没有升力产生
  }
  gzdbg << "进气比 J: " << J << std::endl;
  double P1 = 0.0013;
  double P2 = -0.0039;
  double P3 = 0.0021;
  double P4 = 0.0006;
  double P5 = -0.0003;
  double P6 = 0.0042;
  // double Ct = (P1 * std::fabs(rotorSpeed) + P2) * (P3*J*J*J*J + P4*J*J*J + P5*J*J + P6*J + P7);
  double Ct = P1*J*J*J*J*J + P2*J*J*J*J + P3*J*J*J + P4*J*J + P5*J +P6;
  gzdbg << "Thrust coefficient Ct: " << Ct << std::endl;
  // 计算推力
  // 推力计算公式：T = Ct * rho * n^2 * D^4
  auto T = Ct * this->rho * rotorSpeed * rotorSpeed * this->Diameter * this->Diameter * this->Diameter * this->Diameter;
  T = 0.5 * T;  //这里取一半是因为以上公式对应的是旋翼拉力，我们使用的是二叶桨，因此计算的拉力只有一半
  if (T < 0.0 || T > 100.0) // sanity check, T should be positive and not unreasonably large
  {
    T = 0.0; // 推力不合理，设置为0，表示没有升力产生
  }
  gzdbg << "T: " << T << " N" << std::endl;
  math::Vector3d lift = T * liftI;

  // ---------------计算阻力系数
  double Q1 = 0.0005;
  double Q2 = -0.0019;
  double Q3 = 0.0019;
  double Q4 = -0.0004;
  double Q5 = 0.0001;
  double Q6 = 5.2050e-4;
  double Cq = Q1*J*J*J*J*J + Q2*J*J*J*J + Q3*J*J*J + Q4*J*J + Q5*J + Q6;
  gzdbg << "Torque coefficient Cq: " << Cq << std::endl;
  // 计算推力
  // 力矩计算公式：Q = Cq * rho * n^2 * D^5
  auto Q = Cq * this->rho * rotorSpeed * rotorSpeed * this->Diameter * this->Diameter * this->Diameter * this->Diameter * this->Diameter;
  Q = 0.5 * Q;
  if (Q < 0.0 || Q > 100.0) // sanity check, Q should be positive and not unreasonably large
  {
    Q = 0.0; // 推力不合理，设置为0，表示没有升力产生
  }
  gzdbg << "Q: " << Q << " Nm" << std::endl;

  // compute cd at cp, check for stall, correct for sweep
  double cd;
  if (alpha > this->alphaStall)
  {
    cd = (this->cda * this->alphaStall +
          this->cdaStall * (alpha - this->alphaStall))
         * cos2SweepAngle;
  }
  else if (alpha < -this->alphaStall)
  {
    cd = (-this->cda * this->alphaStall +
          this->cdaStall * (alpha + this->alphaStall))
         * cos2SweepAngle;
  }
  else
    cd = (this->cda * alpha) * cos2SweepAngle;

  // make sure drag is positive
  cd = std::fabs(cd);

  // drag at cp
  math::Vector3d drag = cd * q * this->area * dragDirection;

  // compute cm at cp, check for stall, correct for sweep
  // double cm;
  // if (alpha > this->alphaStall)
  // {
  //   cm = (this->cma * this->alphaStall +
  //         this->cmaStall * (alpha - this->alphaStall))
  //        * cos2SweepAngle;
  //   // make sure cm is still great than 0
  //   cm = std::max(0.0, cm);
  // }
  // else if (alpha < -this->alphaStall)
  // {
  //   cm = (-this->cma * this->alphaStall +
  //         this->cmaStall * (alpha + this->alphaStall))
  //        * cos2SweepAngle;
  //   // make sure cm is still less than 0
  //   cm = std::min(0.0, cm);
  // }
  // else
  //   cm = this->cma * alpha * cos2SweepAngle;

  // // Take into account the effect of control surface deflection angle to cm
  // if (controlJointPosition && !controlJointPosition->Data().empty())
  // {
  //   cm += this->cm_delta * controlJointPosition->Data()[0];
  // }

  // compute moment (torque) at cp
  // spanwiseI used to be momentDirection
  // math::Vector3d moment = cm * q * this->area * spanwiseI;
  math::Vector3d moment = Q * spanwiseI;

  // force and torque about cg in world frame
  // math::Vector3d force = math::Vector3d::Zero;
  math::Vector3d force = lift + drag;
  math::Vector3d torque = moment;
  // Correct for nan or inf
  force.Correct();
  this->cp.Correct();
  torque.Correct();

  // We want to apply the force at cp. The old LiftDragRotor plugin did the
  // following:
  //     this->link->AddForceAtRelativePosition(force, this->cp);
  // The documentation of AddForceAtRelativePosition says:
  //> Add a force (in world frame coordinates) to the body at a
  //> position relative to the center of mass which is expressed in the
  //> link's own frame of reference.
  // But it appears that 'cp' is specified in the link frame so it probably
  // should have been
  //     this->link->AddForceAtRelativePosition(
  //         force, this->cp - this->link->GetInertial()->CoG());
  //
  // \todo(addisu) Create a convenient API for applying forces at offset
  // positions
  const auto totalTorque = torque + cpWorld.Cross(force);
  Link link(this->linkEntity);
  link.AddWorldWrench(_ecm, force, totalTorque);

  // Debug
  auto linkName = _ecm.Component<components::Name>(this->linkEntity)->Data();
  gzdbg << "=============================\n";
  // gzdbg << "Link: [" << linkName << "] pose: [" << pose
  //        << "] dynamic pressure: [" << q << "]\n";
  // gzdbg << "spd: [" << vel.Length() << "] vel: [" << vel << "]\n";
  // gzdbg << "LD plane spd: [" << velInLDPlane.Length() << "] vel : ["
  //        << velInLDPlane << "]\n";
  // gzdbg << "forward (inertial): " << forwardI << "\n";
  // gzdbg << "upward (inertial): " << upwardI << "\n";
  // gzdbg << "q: " << q << "\n";
  // gzdbg << "cl: " << cl << "\n";
  // gzdbg << "lift dir (inertial): " << liftI << "\n";
  // gzdbg << "Span direction (normal to LD plane): " << spanwiseI << "\n";
  // gzdbg << "sweep: " << sweep << "\n";
  // gzdbg << "alpha: " << alpha << "\n";
  // gzdbg << "lift: " << lift << "\n";
  // gzdbg << "drag: " << drag << " cd: " << cd << " cda: "
  //        << this->cda << "\n";
  // gzdbg << "moment: " << moment << "\n";
  // gzdbg << "force: " << force << "\n";
  // gzdbg << "torque: " << torque << "\n";
  // gzdbg << "totalTorque: " << totalTorque << "\n";

  // // ---------------获取连杆的角速度
  // // 1. 获取旋翼轴方向（在连杆局部坐标系）
  // // 假设旋翼轴是局部Z轴
  // math::Vector3d rotorAxisLocal(0, 0, 1);
  
  // // 2. 转换到世界坐标系
  // math::Vector3d rotorAxisWorld = worldPose->Data().Rot().RotateVector(
  //     rotorAxisLocal);
  // rotorAxisWorld.Normalize();
  
  // // 3. 投影：提取沿旋翼轴方向的角速度分量
  // double rotorSpeed = worldAngVel->Data().Dot(rotorAxisWorld);
  
  // // 4. 输出结果
  // gzdbg << "旋翼转速: " << rotorSpeed << " rad/s, "
  //         << rotorSpeed * 60.0 / (2.0 * M_PI) << " RPM" << std::endl;
    
  // // 1. ---------------获取连杆质心的线速度（世界坐标系）
  // math::Vector3d linkLinearVelocity = worldLinVel->Data();  // 单位：m/s
  
  // // 2. 压力中心处的速度
  // auto velAtCp = linkLinearVelocity + worldAngVel->Data().Cross(cpWorld);
  
  // if (windLinearVel != nullptr)
  // {
  //   velAtCp = velAtCp - windLinearVel->Data();
  // }
    
  // // 计算在不同方向上的速度分量
  // // double forwardSpeed = velAtCp.Dot(forwardI);      // 前进方向速度
  // double upwardSpeed = velAtCp.Dot(upwardI);       // 升力方向速度  
  // // double spanwiseSpeed = velAtCp.Dot(spanwiseI);   // 展向速度
  // // gzdbg << "前进方向速度: " << forwardSpeed << " m/s" << std::endl;
  // gzdbg << "升力方向速度: " << upwardSpeed << " m/s" << std::endl;
  // // gzdbg << "展向速度: " << spanwiseSpeed << " m/s" << std::endl;
  // auto J = 10 * upwardSpeed / (rotorSpeed * this->Diameter); // 计算进气比
  // gzdbg << "进气比 J: " << J << std::endl;
  // double P1 = 2.163e-6;
  // double P2 = 0.01672;
  // double P3 = 0.3937;
  // double P4 = -1.536;
  // double P5 = 0.8738;
  // double P6 = 1.761;
  // double Ct = (P1 * rotorSpeed + P2) * (P3*J*J*J*J + P4*J*J*J + P5*J*J + P6);
  // gzdbg << "Thrust coefficient Ct: " << Ct << std::endl;
  // auto thrust = Ct * this->rho * rotorSpeed * rotorSpeed * this->Diameter * this->Diameter * this->Diameter * this->Diameter;
  // gzdbg << "Thrust: " << thrust << " N" << std::endl;
  // gzdbg << "force: " << force << "\n";
}

//////////////////////////////////////////////////
void LiftDragRotor::Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &, EventManager &)
{
  this->dataPtr->model = Model(_entity);
  this->dataPtr->sdfConfig = _sdf->Clone();
}

//////////////////////////////////////////////////
void LiftDragRotor::PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm)
{
  GZ_PROFILE("LiftDragRotor::PreUpdate");

  // \TODO(anyone) Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
           << std::chrono::duration<double>(_info.dt).count()
           << "s]. System may not work properly." << std::endl;
  }

  if (!this->dataPtr->initialized)
  {
    // We call Load here instead of Configure because we can't be guaranteed
    // that all entities have been created when Configure is called
    this->dataPtr->Load(_ecm, this->dataPtr->sdfConfig);
    this->dataPtr->initialized = true;

    if (this->dataPtr->validConfig)
    {
      Link link(this->dataPtr->linkEntity);
      link.EnableVelocityChecks(_ecm, true);

      if ((this->dataPtr->controlJointEntity != kNullEntity) &&
          !_ecm.Component<components::JointPosition>(
              this->dataPtr->controlJointEntity))
      {
        _ecm.CreateComponent(this->dataPtr->controlJointEntity,
            components::JointPosition());
      }
    }
  }

  if (_info.paused)
    return;

  // This is not an "else" because "initialized" can be set in the if block
  // above
  if (this->dataPtr->initialized && this->dataPtr->validConfig)
  {
    this->dataPtr->Update(_ecm);
  }
}

GZ_ADD_PLUGIN(LiftDragRotor,
                    System,
                    LiftDragRotor::ISystemConfigure,
                    LiftDragRotor::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(LiftDragRotor, "gz::sim::systems::LiftDragRotor")
