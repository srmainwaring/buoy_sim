// Copyright 2022 Open Source Robotics Foundation, Inc. and Monterey Bay Aquarium Research Institute
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

#include "SinusoidalPiston.hpp"

#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>

#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/Model.hh>

#include <memory>
#include <string>


using namespace std::chrono_literals;

namespace buoy_gazebo
{
struct SinusoidalPistonPrivate
{
  /// \brief piston stroke (m)
  double stroke{2.03};

  /// \brief Joint Entity
  ignition::gazebo::Entity jointEntity;

  /// \brief Model interface
  ignition::gazebo::Model model{ignition::gazebo::kNullEntity};
};

/////////////////////////////////////////////////
double SdfParamDouble(
  const std::shared_ptr<const sdf::Element> & _sdf,
  const std::string & _field,
  double _default)
{
  return _sdf->Get<double>(_field, _default).first;
}

//////////////////////////////////////////////////
SinusoidalPiston::SinusoidalPiston()
: dataPtr(std::make_unique<SinusoidalPistonPrivate>())
{
}

//////////////////////////////////////////////////
void SinusoidalPiston::Configure(
  const ignition::gazebo::Entity & _entity,
  const std::shared_ptr<const sdf::Element> & _sdf,
  ignition::gazebo::EntityComponentManager & _ecm,
  ignition::gazebo::EventManager & /*_eventMgr*/)
{
  this->dataPtr->model = ignition::gazebo::Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm)) {
    ignerr << "SinusoidalPiston plugin should be attached to a model entity. " <<
      "Failed to initialize." << std::endl;
    return;
  }

  // Get params from SDF
  this->dataPtr->stroke = SdfParamDouble(_sdf, "stroke", this->dataPtr->stroke);

  auto jointName = _sdf->Get<std::string>("JointName");
  if (jointName.empty()) {
    ignerr << "SinusoidalPiston found an empty jointName parameter. " <<
      "Failed to initialize.";
    return;
  }

  this->dataPtr->jointEntity = this->dataPtr->model.JointByName(
    _ecm,
    jointName);
  if (this->dataPtr->jointEntity == ignition::gazebo::kNullEntity) {
    ignerr << "Joint with name[" << jointName << "] not found. " <<
      "The SinusoidalPiston may not influence this joint.\n";
    return;
  }
}

//////////////////////////////////////////////////
void SinusoidalPiston::PreUpdate(
  const ignition::gazebo::UpdateInfo & _info,
  ignition::gazebo::EntityComponentManager & _ecm)
{
  IGN_PROFILE("SinusoidalPiston::PreUpdate");

  // If the joint hasn't been identified yet, the plugin is disabled
  if (this->dataPtr->jointEntity == ignition::gazebo::kNullEntity) {
    return;
  }

  // TODO(anyone): Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero()) {
    ignwarn << "Detected jump back in time [" <<
      std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count() <<
      "s]. System may not work properly." << std::endl;
  }

  // Nothing left to do if paused.
  if (_info.paused) {
    return;
  }

  static const double T{10000.0};  // milliseconds
  static const double m{150.0};  // kg
  static const double shift{0.5};  // meters
  const double a = -2.0 * IGN_PI * IGN_PI * this->dataPtr->stroke * (sin(
      2.0 * IGN_PI *
      std::chrono::duration_cast<std::chrono::milliseconds>(
        _info.simTime).count() / T) + shift);
  auto forceComp =
    _ecm.Component<ignition::gazebo::components::JointForceCmd>(
    this->dataPtr->jointEntity);
  if (forceComp == nullptr) {
    _ecm.CreateComponent(
      this->dataPtr->jointEntity,
      ignition::gazebo::components::JointForceCmd({m * a}));
  } else {
    forceComp->Data()[0] += m * a;  // Add force to existing forces.
  }
}
}  // namespace buoy_gazebo

IGNITION_ADD_PLUGIN(
  buoy_gazebo::SinusoidalPiston,
  ignition::gazebo::System,
  buoy_gazebo::SinusoidalPiston::ISystemConfigure,
  buoy_gazebo::SinusoidalPiston::ISystemPreUpdate)
