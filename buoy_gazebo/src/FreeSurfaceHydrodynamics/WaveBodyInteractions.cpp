// Copyright 2022 Open Source Robotics Foundation, Inc. and Monterey Bay
// Aquarium Research Institute
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

#include "FS_Hydrodynamics.hpp"

#include "LinearIncidentWave.hpp"
#include "WaveBodyInteractions.hpp"
#include <gz/sim/Link.hh>
#include <gz/sim/components/AngularAcceleration.hh>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/World.hh>
#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/Name.hh>
#include <gz/math/PID.hh>
#include <gz/msgs.hh>
#include <gz/msgs/double.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <stdio.h>

#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

using namespace Eigen;
using namespace gz;
using namespace sim;
using namespace systems;

namespace buoy_gazebo {
class WaveBodyInteractionsPrivate {
public:
  WaveBodyInteractionsPrivate()
      : FloatingBody(
            IncRef){}; // Constructor needs to assign reference or compile error
                       // of un-assigned referenced occurs

  /// \brief Model interface
  gz::sim::Model model{gz::sim::kNullEntity};

  /// \brief Link entity
  gz::sim::Entity linkEntity;

  /// \brief Incident wave implementation
  LinearIncidentWave Inc;

  /// \brief Reference to Incident wave implementation
  LinearIncidentWave &IncRef = Inc;

  /// \brief  Free-Surface hydrodynamics implementation
  FS_HydroDynamics FloatingBody;

  /// \brief Location of Waterplane in Link Frame
  gz::math::Vector3d WaterplaneOrigin;

  /// \brief Pose of Waterplane (CS p) in Link Frame (CS b)
  gz::math::Pose3<double> b_Pose_p;

  /// \brief gz communication node.
  gz::transport::Node node;

  /// \brief Callback for User Commanded Current subscription
  /// \param[in] _msg Current
  void OnUserCmdCurr(const gz::msgs::Double &_msg);
};

//////////////////////////////////////////////////
WaveBodyInteractions::WaveBodyInteractions()
    : dataPtr(std::make_unique<WaveBodyInteractionsPrivate>()) {}

/////////////////////////////////////////////////
double SdfParamDouble(const std::shared_ptr<const sdf::Element> &_sdf,
                      const std::string &_field, double _default) {
  return _sdf->Get<double>(_field, _default).first;
}

//////////////////////////////////////////////////
void WaveBodyInteractions::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager & /*_eventMgr*/) {
  std::cout << "In Configure " << std::endl;
  this->dataPtr->model = gz::sim::Model(_entity);
  if (!this->dataPtr->model.Valid(_ecm)) {
    gzerr
        << "WaveBodyInteractions plugin should be attached to a model entity. "
        << "Failed to initialize." << std::endl;
    return;
  }

  // Get params from SDF.
  if (!_sdf->HasElement("LinkName")) {
    gzerr << "You musk specify a <LinkName> for the wavebodyinteraction "
              "plugin to act upon"
           << "Failed to initialize." << std::endl;
    return;
  }
  auto linkName = _sdf->Get<std::string>("LinkName");
  std::cout << "LinkName = " << linkName << std::endl;

  this->dataPtr->linkEntity = this->dataPtr->model.LinkByName(_ecm, linkName);
  if (!_ecm.HasEntity(this->dataPtr->linkEntity)) {
    gzerr << "Link name" << linkName << "does not exist";
    return;
  }

  double A = 1.5; //.5 + ((float)(rand() % 20) / 10);
  double T = 8; /// 3.0 + (rand() % 9);
  this->dataPtr->Inc.SetToPiersonMoskowitzSpectrum(2*A, 0);
  //this->dataPtr->Inc.SetToMonoChromatic(2 * A, T, 0);

  /// \todo(srmainwaring) set up parameters for locating these files
  std::string HydrodynamicsBaseFilename =
      "src/buoy_sim/buoy_gazebo/"
      "src/FreeSurfaceHydrodynamics/HydrodynamicCoeffs/BuoyA5";
  this->dataPtr->FloatingBody.ReadWAMITData_FD(HydrodynamicsBaseFilename);
  this->dataPtr->FloatingBody.ReadWAMITData_TD(HydrodynamicsBaseFilename);
  this->dataPtr->FloatingBody.SetTimestepSize(.001); // TODO:  Need to get timestep size from ecm.

  gz::sim::Link baseLink(this->dataPtr->linkEntity);

  baseLink.EnableAccelerationChecks(
      _ecm,
      true); // Allow access to last-timestep's acceleration in Configure()

  double S = SdfParamDouble(_sdf, "S", 5.47);
  double S11 = SdfParamDouble(_sdf, "S11", 1.37);
  double S22 = SdfParamDouble(_sdf, "S22", 1.37);
  this->dataPtr->FloatingBody.SetWaterplane(S, S11, S22);

  double COB_x = SdfParamDouble(_sdf, "COB_x", 0);
  double COB_y = SdfParamDouble(_sdf, "COB_y", 0);
  double COB_z = SdfParamDouble(_sdf, "COB_z", -.18);
  this->dataPtr->FloatingBody.SetCOB(COB_x, COB_y, COB_z);
  double Vol = SdfParamDouble(_sdf, "Vol", 1.75);
  this->dataPtr->FloatingBody.SetVolume(Vol);

  double WaterplaneOrigin_x = SdfParamDouble(_sdf, "WaterplaneOrigin_x", 0.0);
  double WaterplaneOrigin_y = SdfParamDouble(_sdf, "WaterplaneOrigin_y", 0.0);
  double WaterplaneOrigin_z = SdfParamDouble(_sdf, "WaterplaneOrigin_z", 2.45);
  this->dataPtr->b_Pose_p.Set(WaterplaneOrigin_x, WaterplaneOrigin_y, WaterplaneOrigin_z,0,0,0);
  std::cout << "CCCC b_Pose_p = " << this->dataPtr->b_Pose_p << std::endl;



#if 1
 gz::math::Pose3<double> w_Pose_b(0,0,-2.45,0,0.1,0);
 gz::math::Pose3<double> b_Pose_p(0,0,2.45,0,0,0);
 auto w_Pose_p = w_Pose_b*b_Pose_p;
 std::cout << "POSETEST w_Pose_b = " << w_Pose_b << std::endl;
 std::cout << "POSETEST b_Pose_p = " << b_Pose_p << std::endl;
 std::cout << "POSETEST w_Pose_p = " << w_Pose_p << std::endl;
 //std::cout << "POESTEST w_Pose_p components = " << w_Pose_p.Position().
 gz::math::Vector3<double> p_X(0,0,0);
 std::cout << "POSETEST p_X  = " << p_X << std::endl;
 std::cout << "POSETEST b_X  = " << b_Pose_p.CoordPositionAdd(p_X) << std::endl;
 std::cout << "POSETEST w_X  = " << w_Pose_p.CoordPositionAdd(p_X) << std::endl;
#endif

}

#define EPSILON 0.0000001;
bool AreSame(double a, double b)
{
    return fabs(a - b) < EPSILON;
}
//////////////////////////////////////////////////
void WaveBodyInteractions::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) {

  GZ_PROFILE("#WaveBodyInteractions::PreUpdate");
  // Nothing left to do if paused.
  if (_info.paused) {
    return;
  }
  auto SimTime = std::chrono::duration<double>(_info.simTime).count();
  std::cout << "In PreUpdate SimTime = " << SimTime << std::endl;

double dt = std::chrono::duration<double>(_info.dt).count();
if(!AreSame(dt,dataPtr->FloatingBody.GetTimestepSize()))  //Would prefer to set this in Configure, but not sure how to access it or if it's guaranteed to be setup when configure is called.
{
  std::cout << " Setting timestep size " << std::endl;
  //dataPtr->FloatingBody.SetTimestepSize(dt);
}
std::cout <<" DT = " << dataPtr->FloatingBody.GetTimestepSize() << std::endl;

  // \TODO(anyone): Support rewind
  if (_info.dt < std::chrono::steady_clock::duration::zero()) {
    gzwarn
        << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  gz::sim::Link baseLink(this->dataPtr->linkEntity);
  auto w_xddot = baseLink.WorldLinearAcceleration(_ecm);
  auto w_omegadot = baseLink.WorldAngularAcceleration(_ecm);

  auto w_Pose_b = gz::sim::worldPose(this->dataPtr->linkEntity, _ecm);
  auto w_Pose_p = w_Pose_b*this->dataPtr->b_Pose_p;
  std::cout << "w_Pose_b = " << w_Pose_b << std::endl;
  std::cout << "w_Pose_p = " << w_Pose_p << std::endl;


  gz::math::Vector3<double> p_xddot = w_Pose_p.Rot().Inverse() * *w_xddot;
  gz::math::Vector3<double> p_omegadot = w_Pose_p.Rot().Inverse() * *w_omegadot;

std::cout << "w_omegadot = " << w_omegadot.value() << std::endl;
std::cout << "p_omegadot = " << p_omegadot << std::endl;

  Eigen::VectorXd x(6);   
  x << w_Pose_p.X(),w_Pose_p.Y(),w_Pose_p.Z(),w_Pose_p.Roll(),w_Pose_p.Pitch(),w_Pose_p.Yaw();
  std::cout << "x(6) = " << x.transpose() << std::endl;
  Eigen::VectorXd BuoyancyForce(6);
  BuoyancyForce =  this->dataPtr->FloatingBody.BuoyancyForce(x); 
  std::cout << "Buoyancy Force = " << BuoyancyForce.transpose() << std::endl;


//Compute Buoyancy Force
  gz::math::Vector3d w_FBp(BuoyancyForce(0), BuoyancyForce(1),BuoyancyForce(2)); // Needs to be adjusted for yaw only
  gz::math::Vector3d w_MBp(
      cos(x(5)) * BuoyancyForce(3) - sin(x(5)) * BuoyancyForce(4),
      sin(x(5)) * BuoyancyForce(3) + cos(x(5)) * BuoyancyForce(4),
      BuoyancyForce(5)); // Needs to be adjusted for yaw only
      w_MBp += (w_Pose_b.Rot().RotateVector(this->dataPtr->b_Pose_p.Pos())).Cross(w_FBp); //Add contribution due to force offset from origin
std::cout << "Buoyancy: applied moment = " << w_MBp << std::endl;

//Compute Memory part of Radiation Force
  Eigen::VectorXd xddot(6);
  xddot << p_xddot.X(),p_xddot.Y(),p_xddot.Z(),p_omegadot.X(),p_omegadot.Y(),p_omegadot.Z();
  std::cout << "xddot = " << xddot.transpose() << std::endl;
  Eigen::VectorXd MemForce(6);
  MemForce = -this->dataPtr->FloatingBody.RadiationForce(xddot); //Note negative sign, FS_Hydrodynamics returns force required to move body in prescirbed way, force of water on body is opposite.
  std::cout << " MemForce = " << MemForce.transpose() << std::endl;
  gz::math::Vector3d w_FRp(MemForce(0), MemForce(1), MemForce(2)); // Needs to be adjusted for yaw only
  gz::math::Vector3d w_MRp(
      1*(cos(x(5)) * MemForce(3) - sin(x(5)) * MemForce(4)),
      1*(sin(x(5)) * MemForce(3) + cos(x(5)) * MemForce(4)),
      MemForce(5)); // Needs to be adjusted for yaw only
 w_MRp += (w_Pose_b.Rot().RotateVector(this->dataPtr->b_Pose_p.Pos())).Cross(w_FRp); //Add contribution due to force offset from origin
std::cout << "Radiation: applied moment = " << w_MRp << std::endl;

  Eigen::VectorXd ExtForce(6);
  ExtForce = this->dataPtr->FloatingBody.ExcitingForce();
   std::cout << "Exciting Force = " << ExtForce.transpose() << std::endl;
  gz::math::Vector3d w_FEp(ExtForce(0), ExtForce(1), ExtForce(2)); // Needs to be adjusted for yaw only
  gz::math::Vector3d w_MEp(
      1*(cos(x(5)) * ExtForce(3) - sin(x(5)) * ExtForce(4)),
      1*(sin(x(5)) * ExtForce(3) + cos(x(5)) * ExtForce(4)),
      ExtForce(5)); // Needs to be adjusted for yaw only

std::cout << "Exciting: applied force = " << w_FEp << std::endl;
std::cout << "Exciting: applied moment = " << w_MEp << std::endl;

 w_MEp += (w_Pose_b.Rot().RotateVector(this->dataPtr->b_Pose_p.Pos())).Cross(w_FEp); //Add contribution due to force offset from origin
 //baseLink.AddWorldWrench(_ecm, w_FBp + w_FRp + w_FEp, w_MBp + w_MRp + w_MEp); 
 baseLink.AddWorldWrench(_ecm, w_FBp + w_FRp, w_MBp - w_MRp);

std::cout << "ABCD " << SimTime << "  " <<w_MBp[0] << "  " <<  w_MRp[0] << std::endl;

  /*
  Eigen::Vector3d BuoyancyMomentAtLinkOrigin;
  //BuoyancyMomentAtLinkOrigin =
  (this->dataPtr->WaterplaneOrigin).cross(BuoyancyForce.segment(0,3))+BuoyancyForce.segment(3,3);
  BuoyancyMomentAtLinkOrigin =  BuoyancyForce.segment(3,3);
  std::cout << "Buoyancy Moment At Origin= " <<
  >BuoyancyMomentAtLinkOrigin.transpose() << std::endl << std::endl;

  gz::math::Vector3d
  AppliedForce(BuoyancyForce(0),BuoyancyForce(1),BuoyancyForce(2));
  gz::math::Vector3d
  AppliedMoment(BuoyancyMomentAtLinkOrigin(0),BuoyancyMomentAtLinkOrigin(1),BuoyancyMomentAtLinkOrigin(2));
  baseLink.AddWorldWrench( _ecm, AppliedForce, AppliedMoment);
*/
  /*
    gz::math::Vector3d
      totalForce(ForceAtWaterplaneOrigin(0),ForceAtWaterplaneOrigin(1),ForceAtWaterplaneOrigin(2));
      std::cout << "Total Force = "  << totalForce << std::endl;

        Eigen::Vector3d MomentAtLinkOriginDueToForceAtWaterplaneOrigin;
        MomentAtLinkOriginDueToForceAtWaterplaneOrigin =
  this->dataPtr->WaterplaneOrigin.cross(ForceAtWaterplaneOrigin);

    gz::math::Vector3d
  //    totalTorque(0,0,0);
  //    totalTorque(BuoyancyForce(3) +
  MomentAtLinkOriginDueToForceAtWaterplaneOrigin(0)-MemForce(3),
   //               BuoyancyForce(4) +
  MomentAtLinkOriginDueToForceAtWaterplaneOrigin(1)-MemForce(4),
    //              BuoyancyForce(5) +
  MomentAtLinkOriginDueToForceAtWaterplaneOrigin(2)-MemForce(5));
      totalTorque(-MemForce(3)+ExtForce(3)+BuoyancyForce(3) +
  MomentAtLinkOriginDueToForceAtWaterplaneOrigin(0),
                  -MemForce(4)+ExtForce(4)+BuoyancyForce(4) +
  MomentAtLinkOriginDueToForceAtWaterplaneOrigin(1),
                  -MemForce(5)+ExtForce(5)+BuoyancyForce(5) +
  MomentAtLinkOriginDueToForceAtWaterplaneOrigin(2));

  baseLink.AddWorldWrench( _ecm, pose.Rot()*(totalForce),
  pose.Rot()*totalTorque);

  */
}

//////////////////////////////////////////////////
void WaveBodyInteractions::Update(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm) {
  GZ_PROFILE("#WaveBodyInteractions::Update");
  // Nothing left to do if paused.
  if (_info.paused) {
    return;
  }

  // auto SimTime = std::chrono::duration < double > (_info.simTime).count();
}

//////////////////////////////////////////////////
void WaveBodyInteractions::PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm) {
  GZ_PROFILE("#WaveBodyInteractions::PostUpdate");
  // Nothing left to do if paused.
  if (_info.paused) {
    return;
  }

  // auto SimTime = std::chrono::duration < double > (_info.simTime).count();
}

//////////////////////////////////////////////////
void WaveBodyInteractionsPrivate::OnUserCmdCurr(
    const gz::msgs::Double &_msg) {
  std::cout << "In OnUserCmdCurr" << std::endl;
}
} // namespace buoy_gazebo

GZ_ADD_PLUGIN(buoy_gazebo::WaveBodyInteractions,
    gz::sim::System,
    buoy_gazebo::WaveBodyInteractions::ISystemConfigure,
    buoy_gazebo::WaveBodyInteractions::ISystemPreUpdate,
    buoy_gazebo::WaveBodyInteractions::ISystemUpdate,
    buoy_gazebo::WaveBodyInteractions::ISystemPostUpdate);

/*
  GZ_ADD_PLUGIN(
      WaveBodyInteractions,
      gz::sim::System,
      WaveBodyInteractions::ISystemConfigure,
      WaveBodyInteractions::ISystemPreUpdate,
      WaveBodyInteractions::ISystemUpdate,
      WaveBodyInteractions::ISystemPostUpdate);

  GZ_ADD_PLUGIN_ALIAS(WaveBodyInteractions,
      "gz::sim::systems::WaveBodyInteractions")
*/

