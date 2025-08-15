#include <sys/types.h>
#include <sys/wait.h>

#include <algorithm>
#include <iomanip>
#include <numeric>
#include <string>
#include <stdexcept>

#include <rbdl/addons/luamodel/luamodel.h>
#include <rbdl/addons/luamodel/luatables.h>
#include <rbdl/addons/muscle/muscle.h>
#include <rbdl/addons/muscle/TorqueMuscleFunctionFactory.h>
#include "LuaTypes.h"
#include "wholebodylifter2d_Model.h"
#include "csvtools.h"

using namespace std;
using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace RigidBodyDynamics::Addons::Muscle;
using namespace RigidBodyDynamics::Addons::Geometry;

// =============================================================================

WholeBodyLifterModel::WholeBodyLifterModel() :
  activeConstraintSet (-1),
  nDof (-1),
  nActuatedDof(0) {

  // Create invalid values for point infos
  for (unsigned int i = 0; i < PointNameLast; i++) {
    pointInfos[i] = Point();
  }

  // Create invalid values for constraint set infos
  for (unsigned int i = 0; i < ConstraintSetNameLast; i++) {
		cs_infos[i] = ConstraintSetInfo();
  }

  p = VectorNd::Zero (ParamNameLast);


}

// =============================================================================
// See detailed comments in wholebodylifter2d_Model.h
// =============================================================================
/*
void WholeBodyLifterModel::
calcMuscleActivations(
    const RigidBodyDynamics::Math::VectorNd &qIn,
    const RigidBodyDynamics::Math::VectorNd &qDotIn,
    const RigidBodyDynamics::Math::VectorNd &tauIn,
    bool addJointDamping,
    RigidBodyDynamics::Math::VectorNd &aOut,
    RigidBodyDynamics::Math::VectorNd &taOut,
    RigidBodyDynamics::Math::VectorNd &tvOut,
    RigidBodyDynamics::Math::VectorNd &tpOut) const
{
  aOut.resize(tqMuscleModel.size());
  taOut.resize(tqMuscleModel.size());
  tvOut.resize(tqMuscleModel.size());
  tpOut.resize(tqMuscleModel.size());

  TorqueMuscleSummary tms;

  RigidBodyDynamics::Math::VectorNd tauTemp;
  tauTemp.resize(NumberOfMuscleControls);

  for(unsigned int i=0; i<tauIn.size();++i){
    tauTemp(i)= tauIn(i);
  }
  for(unsigned int i=0; i<aOut.size();++i){
    aOut(i)  = 0;
    taOut(i) = 0;
    tvOut(i) = 0;
    tpOut(i) = 0;
  }

  if(addJointDamping){
    unsigned int i;
    for(i=StateRightHipRotY; i<StateBoxVelX; ++i ){
      //here the sign changes because muscles need
      //to generate the torque required by ID on top
      //of the damping torque.
      tauTemp(i) += jointDampingCoefficient(i)*qDotIn(i);
    }
    assert(i == StateBoxVelX);
  }

  unsigned int k         = ControlRightHipExtensionRotY;
  for (unsigned int j = PosRightHipRotY; j < PosNameLast; ++j) {

    //If this is a lumbar joint scale q and qdot so that the lumbar
    //develops forces according to the total bend.
    if ( k == ControlMiddleTrunkExtensionRotY ||
         k == ControlMiddleTrunkFlexionRotY   ||number of degrees of freedomn
         k == ControlUpperTrunkExtensionRotY  ||
         k == ControlUpperTrunkFlexionRotY ) {
      //Here we have 2 joints for the lumbar, and so we scale q and qdot
      //by 2. That way each of these lumbar joints will develop their
      //maximum forces when the lumbar has a total flexion/extension
      //angle and velocity equal to the sum of two.
      //
      //This comment is kind of confusing, but I'm not sure how to make it
      //clearer at the moment.
      tqMuscleModel[k].calcActivation(
              2*qIn(j), 2*qDotIn(j), tauTemp(j),tms);
      aOut(k) = tms.activation;
      taOut(k)= tms.fiberActiveTorqueAngleMultiplier;
      tvOut(k)= tms.fiberActiveTorqueAngularVelocityMultiplier;
      tpOut(k)= tms.fiberPassiveTorqueAngleMultiplier;

      ++k;
      tqMuscleModel[k].calcActivation(
            2*qIn(j), 2*qDotIn(j), tauTemp(j),tms);
      aOut(k) = tms.activation;
      taOut(k)= tms.fiberActiveTorqueAngleMultiplier;
      tvOut(k)= tms.fiberActiveTorqueAngularVelocityMultiplier;
      tpOut(k)= tms.fiberPassiveTorqueAngleMultiplier;
      ++k;
    } else {
      tqMuscleModel[k].calcActivation(
             qIn(j),  qDotIn(j), tauTemp(j),tms);
      aOut(k) = tms.activation;
      taOut(k)= tms.fiberActiveTorqueAngleMultiplier;
      tvOut(k)= tms.fiberActiveTorqueAngularVelocityMultiplier;
      tpOut(k)= tms.fiberPassiveTorqueAngleMultiplier;
      ++k;
      tqMuscleModel[k].calcActivation(
             qIn(j),  qDotIn(j), tauTemp(j),tms);
      aOut(k) = tms.activation;
      taOut(k)= tms.fiberActiveTorqueAngleMultiplier;
      tvOut(k)= tms.fiberActiveTorqueAngularVelocityMultiplier;
      tpOut(k)= tms.fiberPassiveTorqueAngleMultiplier;
      ++k;
    }
  }


}

// =============================================================================
// See detailed comments in wholebodylifter2d_Model.h
// =============================================================================

void WholeBodyLifterModel::
calcMaximumActiveIsometricTorqueScaleFactor(
    const RigidBodyDynamics::Math::VectorNd &qIn,
    const RigidBodyDynamics::Math::VectorNd &qDotIn,
    const RigidBodyDynamics::Math::VectorNd &tauIn,
    const RigidBodyDynamics::Math::VectorNd &activationIn,
    bool addJointDamping,
    RigidBodyDynamics::Math::VectorNd &sfOut) const
{
  sfOut.resize(tqMuscleModel.size());
  RigidBodyDynamics::Math::VectorNd tauTemp;
  tauTemp.resize(ControlNameLast);

  for(unsigned int i=0; i<tauIn.size();++i){
    tauTemp(i)= tauIn(i);
  }
  for(unsigned int i=0; i<sfOut.size();++i){
    sfOut(i)=0;
  }

  if(addJointDamping){
    unsigned int i;
    for(i = StateRightHipRotY; i<StateBoxVelX; ++i ){
      //here the sign changes because muscles need
      //to generate the torque required by ID on top
      //of the damping torque.
      tauTemp(i) += jointDampingCoefficient(i)*qDotIn(i);
    }
    assert(i == StateBoxVelX);
  }

  unsigned int k         = ControlRightHipExtensionRotY;
  for (unsigned int j = PosRightHipRotY; j < PosNameLast; ++j) {
    //If this is a lumbar joint scale q and qdot so that the lumbar
    //develops forces according to the total bend.
    if ( k == ControlMiddleTrunkExtensionRotY ||
         k == ControlMiddleTrunkFlexionRotY   ||
         k == ControlUpperTrunkExtensionRotY  ||
         k == ControlUpperTrunkFlexionRotY ) {
      //Here we have 2 joints for the lumbar, and so we scale q and qdot
      //by 2. That way each of these lumbar joints will develop their
      //maximum forces when the lumbar has a total flexion/extension
      //angle and velocity equal to the sum of two.
      //
      //This comment is kind of confusing, but I'm not sure how to make it
      //clearer at the moment.
      sfOut(k) =
          tqMuscleModel[k].calcMaximumActiveIsometricTorqueScalingFactor(
              2*qIn(j), 2*qDotIn(j), activationIn(k), tauTemp(j));
      ++k;
      sfOut(k) =
          tqMuscleModel[k].calcMaximumActiveIsometricTorqueScalingFactor(
            2*qIn(j), 2*qDotIn(j), activationIn(k), tauTemp(j));
      ++k;
    } else {
      sfOut(k) =
          tqMuscleModel[k].calcMaximumActiveIsometricTorqueScalingFactor(
             qIn(j),  qDotIn(j), activationIn(k), tauTemp(j));
      ++k;
      sfOut(k) =
          tqMuscleModel[k].calcMaximumActiveIsometricTorqueScalingFactor(
             qIn(j),  qDotIn(j), activationIn(k), tauTemp(j));
      ++k;
    }
  }


}
*/

CSId WholeBodyLifterModel::getConstraintSetId(unsigned int phase)
{

  CSId tmp = ConstraintSetNameLast;

  switch(phase){
    case 0:
      {
        tmp = CSBothFeetFlat2D;
      }
      break;
    case 1:
      {
        tmp = CSBothFeetFlat2DTransition;
      }
      break;
    case 2:
      {
        tmp = CSBothFeetFlat2DBoxAttach;
      }
    break;
  default:
    cerr << "There is no 3rd model stage!" << endl;
    abort();
  }

  return tmp;
}

void WholeBodyLifterModel::projectStateOntoMultibodyConstraintManifold(
    double *sd,
    const CSId cs_id){

  VectorNd qInit, qDotInit, qAsm, qDotAsm;
  qInit.resize(q.size());
  qDotInit.resize(qdot.size());
  qAsm.resize(q.size());
  qDotAsm.resize(qdot.size());

  //Copy the multibody state part into q and qdot.
  for (unsigned int i = 0; i < PosNameLast; i++) {
    qInit[i]    = sd[i];
    qDotInit[i] = sd[i + PosNameLast];
  }

  //Setup the weighting matrix:
  VectorNd assemblyWeights;
  assemblyWeights.resize(PosNameLast);

  //Make the weights of the human dofs large (so they don't move much)
  for(unsigned int i = 0; i < PosNameLast; ++i){
    assemblyWeights(i) = 1.0;
  }
  //Make the weights of the exo small (so they move more).
  for(unsigned int i = PosExoFirst; i <= PosExoLast; ++i){
    assemblyWeights(i) = 0.1;
  }

  //Assemble the constraints at the position level.
  CalcAssemblyQ(model, qInit, constraint_sets[cs_id],
                qAsm, assemblyWeights, 1e-8, 100);


  //Assemble the constraints at the velocity level.
  CalcAssemblyQDot(model, qAsm, qDotInit, constraint_sets[cs_id],
                qDotAsm, assemblyWeights);

  //Update the state vector.
  for (unsigned int i = 0; i < PosNameLast; i++) {
    sd[i] = qAsm[i];
    sd[i + PosNameLast] = qDotAsm[i];
  }


}

void WholeBodyLifterModel::updateState (
		const double * sd,
		const double * u,
		const double *p_in,
		const CSId cs_id) {
	activeConstraintSet = cs_id;
	dynamicsComputed  = false;
	kinematicsUpdated = false;

  for (unsigned int i = 0; i < PosNameLast; i++) {
		q[i]    = sd[i];
    qdot[i] = sd[i + PosNameLast];
  }

  unsigned int k = ExoControlFirst;
  for(unsigned int j=0; j < NumberOfExoControls; ++j){
    exoControl[j] = u[k];
    k++;
  }

#ifdef _ACTIVATION_DYNAMICS_

    k = 0;
    for(unsigned int j = StateMuscleActFirst;
        j <= StateMuscleActLast;
        j++)
    {
      act[k] = sd[j];
      actDot[k] = (u[k]-sd[j])/activationTimeConstant;
      k++;
    }

    /*
    k = 0;
    for(unsigned int j =  StateExoTauFirst;
                     j <= StateExoTauLast;
                     j++)
    {
      exoTauNorm[k]    = sd[j];
      exoTauDotNorm[k] = (exoControl[k] - sd[j])/activationTimeConstant;
      k++;
    }*/

    for(unsigned int j =  0;
                     j < NumberOfExoControls;
                     j++)
    {
      exoTauNorm[j]    = exoControl[j];
    }

#else
  k = MuscleControlFirst;
  for(unsigned int j=0; j < NumberOfMuscleControls; ++j){
    act[j] = u[k];
    k++;
  }

  for(unsigned int j =  0;
                   j < NumberOfExoControls;
                   j++)
  {
    exoTauNorm[j]    = exoControl[j];
  }
#endif;


  copy(p_in, p_in + ParamNameLast, p.data());
  zeroTau();
  addMuscleTorqueToTau();
  addJointDampingToTau();
  addExoTorqueToTau();
}


// =============================================================================

void WholeBodyLifterModel::zeroTau() {
  // Sum and copy resultant torques onto tau
  fill_n(tau.data(), tau.size(), 0.0);
}

// =============================================================================

void WholeBodyLifterModel::addJointDampingToTau(){

  unsigned int i;
  for(i=StateHumanInternalPosFirst; i<=StateHumanPosLast; ++i ){
    tau[i] += -jointDampingCoefficient(i)*qdot[i];
  }
  assert(i == StateVelFirst);
  
}

void WholeBodyLifterModel::addExoTorqueToTau(){
  for(unsigned int i = 0; i<NumberOfExoControls ; i++){
    tau[PosExoActuated[i]] = exoTauNorm[i]*exoActuatorTauMax[i];
  }

}


// =============================================================================

void WholeBodyLifterModel::addMuscleTorqueToTau () {
  unsigned int k         = 0;

  for (unsigned int j = PosHumanInternalFirst; j <= PosHumanLast; ++j) {
    //If this is a lumbar joint scale q and qdot so that the lumbar
    //develops forces according to the total bend.
    if ( k == ControlMiddleTrunkExtensionRotY ||
         k == ControlMiddleTrunkFlexionRotY   ||
         k == ControlUpperTrunkExtensionRotY  ||
         k == ControlUpperTrunkFlexionRotY ) {
      //Here we have 2 joints for the lumbar, and so we scale q and qdot 
      //by 2. That way each of these lumbar joints will develop their 
      //maximum forces when the lumbar has a total flexion/extension
      //angle and velocity equal to the sum of two.
      //
      //This comment is kind of confusing, but I'm not sure how to make it
      //clearer at the moment.
      tauMuscles[k] = tqMuscleModel[k].calcJointTorque(
            2*q[j], 2*qdot[j], act[k]);
      ++k;
      tauMuscles[k] = tqMuscleModel[k].calcJointTorque(
            2*q[j], 2*qdot[j], act[k]);
      ++k;
    } else {
      tauMuscles[k] = tqMuscleModel[k].calcJointTorque(
            q[j], qdot[j], act[k]);
      ++k;
      tauMuscles[k] = tqMuscleModel[k].calcJointTorque(
            q[j], qdot[j], act[k]);
      ++k;
    } 
  }

  

  k = 0;
  for (unsigned int i = PosHumanInternalFirst; i <= PosHumanLast; i++) {
    tau[i] += tauMuscles[k];
    ++k;
    tau[i] += tauMuscles[k];
    ++k;
  }
}

// =============================================================================

void WholeBodyLifterModel::updateKinematics () {
  UpdateKinematics (model, q, qdot, qddot);
  kinematicsUpdated = true;
}

// =============================================================================

void WholeBodyLifterModel::calcForwardDynamicsRhs (double *res) {
	ForwardDynamicsConstraintsDirect (model, q, qdot, tau,
		constraint_sets[activeConstraintSet], qddot);

	dynamicsComputed = true;
	for (unsigned int i = 0; i < PosNameLast; i++) {
		res[i] = qdot[i];
		res[i + PosNameLast] = qddot[i];
	}
#ifdef _ACTIVATION_DYNAMICS_

    unsigned int k = 0;
    for(unsigned int j = StateMuscleActFirst;
        j <= StateMuscleActLast;
        j++)
    {
      res[j] = actDot[k];
      k++;
    }
    /*
    k =0;
    for(unsigned int j =  StateExoTauFirst;
                     j <= StateExoTauLast;
                     j++)
    {
      res[j] = exoTauDotNorm[k]*exoActuatorTauMax[k];
      k++;
    }
    */

#endif
}

// =============================================================================

double WholeBodyLifterModel::getSumNormalizedMuscleTorquesSqr () {
	double result = 0.;
  for (int i = 0;i < NumberOfMuscleControls; i++) {
    result += std::pow(tauMuscles[i] / tauMaxMuscles[i], 2);
	}
	return result;
}

// =============================================================================

double WholeBodyLifterModel::getSumActSqr () {
  double result = std::inner_product(act.begin(), act.end(), act.begin(), 0.);
  return result;
}

// =============================================================================

Vector3d WholeBodyLifterModel::getPointPosition (const PointId &point_name) {
  if (!kinematicsUpdated) {
    updateKinematics();
  }

  unsigned int body_id = pointInfos[point_name].body_id;
  Vector3d point_local = pointInfos[point_name].point_local;

  return CalcBodyToBaseCoordinates (model, q, body_id, point_local, false);
}


// =============================================================================

Matrix3d WholeBodyLifterModel::getBodyWorldOrientation(const PointId &point_name) {
  if (!kinematicsUpdated) {
    updateKinematics();
  }

  unsigned int body_id = pointInfos[point_name].body_id;

  return CalcBodyWorldOrientation(model, q, body_id, false);
}

// =============================================================================

Vector3d WholeBodyLifterModel::getPointVelocity (const PointId &point_name) {
  if (!kinematicsUpdated) {
    updateKinematics();
  }

  unsigned int body_id = pointInfos[point_name].body_id;
  Vector3d point_local = pointInfos[point_name].point_local;

  return CalcPointVelocity (model, q, qdot, body_id, point_local, false);
}

// =============================================================================

SpatialVector WholeBodyLifterModel::getPointVelocity6D (const PointId &point_name) {
  if (!kinematicsUpdated) {
    updateKinematics();
  }

  unsigned int body_id = pointInfos[point_name].body_id;
  Vector3d point_local = pointInfos[point_name].point_local;

  return CalcPointVelocity6D (model, q, qdot, body_id, point_local, false);
}

// =============================================================================

SpatialVector WholeBodyLifterModel::getLoopConstraintForceInBaseCoordinates(
    const std::string& body_predecessor,
    const std::string& body_successor) {

  SpatialVector wB = getLoopConstraintForce (body_predecessor,body_successor);

  unsigned int body_p = model.GetBodyId(body_predecessor.c_str());
  unsigned int body_s = model.GetBodyId(body_successor.c_str());

  Vector3d tauB,forceB,tau0,force0;
  tauB[0]    = wB[0];
  tauB[1]    = wB[1];
  tauB[2]    = wB[2];
  forceB[0]  = wB[3];
  forceB[1]  = wB[4];
  forceB[2]  = wB[5];

  Matrix3d rm0B = RigidBodyDynamics::CalcBodyWorldOrientation(model,q,body_p,false);

  tau0    = rm0B.transpose()*tauB;
  force0  = rm0B.transpose()*forceB;

  SpatialVector w0;

  w0[0] = tau0[0];
  w0[1] = tau0[1];
  w0[2] = tau0[2];
  w0[3] = force0[0];
  w0[4] = force0[1];
  w0[5] = force0[2];

  return w0;
}

SpatialVector WholeBodyLifterModel::getLoopConstraintForce (
    const std::string& body_predecessor,
    const std::string& body_successor) {

  unsigned int body_p = model.GetBodyId(body_predecessor.c_str());
  unsigned int body_s = model.GetBodyId(body_successor.c_str());

  if (!dynamicsComputed) {
  ForwardDynamicsConstraintsDirect (model, q, qdot, tau,
    constraint_sets[activeConstraintSet], qddot);
    dynamicsComputed = true;
  }

  SpatialVector result (0., 0., 0.,0., 0., 0.);
  bool found = false;

  const RigidBodyDynamics::ConstraintSet &active_constraint_set
        = constraint_sets[activeConstraintSet];

  const ConstraintSetInfo constraint_set_info
        = cs_infos[activeConstraintSet];

  //std::vector<ConstraintInfo>::const_iterator constraint_iter
  //  = cs_infos[activeConstraintSet].constraints.begin();
  for (unsigned int ci = 0; ci < constraint_set_info.constraints.size(); ci++) {
    const ConstraintInfo& constraint_info = constraint_set_info.constraints[ci];

    if(constraint_info.type == ConstraintSet::LoopConstraint){

      if(   active_constraint_set.body_p[ci] == body_p &&
            active_constraint_set.body_s[ci] == body_s){
        found = true;
        assert (constraint_info.axis
                == active_constraint_set.constraintAxis[ci]);
        result += active_constraint_set.force[ci]
                 *active_constraint_set.constraintAxis[ci];
      }
    }

  }

  if (!found) {
    cerr  << "Error (" << __func__ << "): Loop constraint between bodies'"
          << model.GetBodyName(body_p) << " and " << model.GetBodyName(body_s)
          << "' is not contained in constraint set '"
          << cs_infos[activeConstraintSet].name << "'!" << endl;
    abort();
  }

  return result;
}
// =============================================================================

Vector3d WholeBodyLifterModel::getPointForce (const PointId &point_name) {

  if (!dynamicsComputed) {
	ForwardDynamicsConstraintsDirect (model, q, qdot, tau,
		constraint_sets[activeConstraintSet], qddot);
    dynamicsComputed = true;
  }

  Vector3d result (0., 0., 0.);
  bool found = false;

  const RigidBodyDynamics::ConstraintSet &active_constraint_set 
				= constraint_sets[activeConstraintSet];

  const ConstraintSetInfo constraint_set_info 
				= cs_infos[activeConstraintSet];

  std::vector<ConstraintInfo>::const_iterator constraint_iter 
		= cs_infos[activeConstraintSet].constraints.begin();

  for (unsigned int ci = 0; ci < constraint_set_info.constraints.size(); ci++) {
    const ConstraintInfo& constraint_info = constraint_set_info.constraints[ci];

    //Note: whoever coded the original version of this if statement did not
    //check what the proper name of the enums were for LoopConstraint and
    //ContactConstraint. Be warned that the compiler will miss this error
    //when it is in an if statement. This was *not* easy to find. Please
    //be more careful in the future.
    //MM 2016/01/05.
    if(constraint_info.type == ConstraintSet::ContactConstraint){
      found = true;
      if(constraint_info.point_id == point_name){
        assert (constraint_info.normal == active_constraint_set.normal[ci]);
        result += active_constraint_set.force[ci]*active_constraint_set.normal[ci];
      }
    }
  }

  if (!found) {
    cerr  << "Error (" << __func__ << "): Point '" 
          << pointInfos[point_name].name 
          << "' is not contained in constraint set '" 
					<< cs_infos[activeConstraintSet].name << "'!" << endl;
    abort();
  }

  return result;
}

// =============================================================================

int WholeBodyLifterModel::appendIeqHandBoxForceError(double *res, int j){

  SpatialVector RightHandBoxForce = getLoopConstraintForce("Box","Hand_R");
  //MM: Starting out by bounding the min and maximum forces in the Z direction.

  res[j++] = 0.5*maxHandBoxForces(5) - RightHandBoxForce(5);

  return j;
}

// =============================================================================
int WholeBodyLifterModel::
    appendIeqFootGroundForceError(double * res, int j)
{

  Vector3d RightHeelForce = getPointForce (PointHeel_R);
  Vector3d RightToeForce  = getPointForce (PointToe_R);

  //N.B.: you should never ever see a res[++j] in this file,
  //because it will cause a memory overrun. Be sure to search
  //for it before you run anything!
  res[j++] = RightHeelForce[2];
  res[j++] = RightToeForce[2];


  res[j++] = RightHeelForce[0]
           + (RightHeelForce[2] + RightToeForce[2])*coefficientOfFriction;

  res[j++] = (RightHeelForce[2] + RightToeForce[2])*coefficientOfFriction
            -RightHeelForce[0];

  return j;

}

int WholeBodyLifterModel::
    appendEqLumbarVelocityError(double * res, int j)
{
  res[j++] = qdot[PosMiddleTrunkRotY]
           - qdot[PosUpperTrunkRotY];
  return j;
}

int WholeBodyLifterModel::
  appendEqHandBoxRotYVelError(double * res, int j)
{

  SpatialVector RightHandVel6D =
      CalcPointVelocity6D(model,
                          q,
                          qdot,
                          model.GetBodyId("Hand_R"),
                          pointInfos[PointDistalMetacarpal_R].point_local,
                          true);
  SpatialVector BoxVel6D =
      CalcPointVelocity6D(model,
                          q,
                          qdot,
                          model.GetBodyId("Box"),
                          pointInfos[PointBoxCoM].point_local,
                          true);

  //Now pick out the angular velocity of the bodies about the
  //global y axis.
  res[j++] = RightHandVel6D(1) - BoxVel6D(1);
  return j;

}

int WholeBodyLifterModel::
  appendEqHandBoxRotYError(double * res, int j)
{

  Matrix3d E_RH_0 = CalcBodyWorldOrientation(
                      model,
                      q,
                      model.GetBodyId("Hand_R"),
                      true);

  Matrix3d E_B_0 = CalcBodyWorldOrientation(
                      model,
                      q,
                      model.GetBodyId("Box"),
                      true);

  //Since these rotation matrices are from the world to the body,
  //the row vectors are the direction vectors of the local body
  //resolved in the world frame.

  Vector3d E_Bx_0  = Vector3d( E_B_0(0,0), E_B_0(0,1), E_B_0(0,2));
  Vector3d E_RHz_0 = Vector3d(E_RH_0(2,0),E_RH_0(2,1),E_RH_0(2,2));

  res[j++] = E_Bx_0(0)*E_RHz_0(0)
           + E_Bx_0(1)*E_RHz_0(1)
           + E_Bx_0(2)*E_RHz_0(2);
  return j;
}


int WholeBodyLifterModel::
  appendEqHandBoxPosError(double * res, int j)
{
  Vector3d RightHandPos =
      getPointPosition (PointDistalMetacarpal_R);
  Vector3d BoxRightHandPos =
      getPointPosition (PointBoxAttach_R);

  res[j++] = RightHandPos[0] - BoxRightHandPos[0];
  res[j++] = RightHandPos[2] - BoxRightHandPos[2];

  return j;

}

int WholeBodyLifterModel::
  appendEqHandBoxVelError(double * res, int j)
{

  Vector3d BoxRightHandVel = getPointVelocity (PointBoxAttach_R);
  Vector3d RightHandVel    = getPointVelocity (PointDistalMetacarpal_R);

  res[j++] = RightHandVel[0] - BoxRightHandVel[0];
  res[j++] = RightHandVel[2] - BoxRightHandVel[2];

  return j;

}

int WholeBodyLifterModel::
  appendEqBoxParallelToLegPosError(double *res, int j)
{

  Vector3d r0b0 = getPointPosition (PointBoxBottom);
  Vector3d r0t0 = getPointPosition (PointBoxTop);

  Matrix3d EB0  =
      CalcBodyWorldOrientation(
        model,
        q,
        model.GetBodyId("Box"),
        false);

  Vector3d LocalPos(0.0,0.0,0.0);

  Vector3d r0K0r =
      CalcBodyToBaseCoordinates (
        model,
        q,
        model.GetBodyId("Shank_R"),
        LocalPos,
        false);

  Vector3d r0H0r =
      CalcBodyToBaseCoordinates (
        model,
        q,
        model.GetBodyId("Thigh_R"),
        LocalPos,
        false);

  Vector3d rKBBr = EB0*(r0b0-r0K0r);
  Vector3d rHTBr = EB0*(r0t0-r0H0r);

  res[j++] = rKBBr(0) - rHTBr(0);
  return j;

}

int WholeBodyLifterModel::
  appendIeqBoxLegPosError(double *res, int j)
{

  Vector3d r0b0 = getPointPosition (PointBoxBottom);
  Vector3d r0t0 = getPointPosition (PointBoxTop);

  Matrix3d EB0  =
      CalcBodyWorldOrientation(
        model,
        q,
        model.GetBodyId("Box"),
        false);

  Vector3d LocalPos(0.0,0.0,0.0);

  Vector3d r0K0r =
      CalcBodyToBaseCoordinates (
        model,
        q,
        model.GetBodyId("Shank_R"),
        LocalPos,
        false);

  Vector3d r0H0r =
      CalcBodyToBaseCoordinates (
        model,
        q,
        model.GetBodyId("Thigh_R"),
        LocalPos,
        false);


  Vector3d rKBBr = EB0*(r0b0-r0K0r);
  Vector3d rHTBr = EB0*(r0t0-r0H0r);

  res[j++] = rKBBr(0) - boxHalfWidth;
  res[j++] = rHTBr(0) - boxHalfWidth;

  return j;

}

// =============================================================================

bool WholeBodyLifterModel::loadFromFile (
		string modelfilename,
		string datfilename,
		bool verbose) {

	if (!Addons::LuaModelReadFromFile(modelfilename.c_str(), &model, verbose)) {
		cerr << "Error loading LuaModel: " << modelfilename << endl;
		abort();
	}

	nDof          = model.dof_count;
  nActuatedDof  = PosHumanLast - PosHumanInternalFirst + 1
                  + NumberOfExoControls;

	if (nDof != PosNameLast) {
		cerr  << "Error: Number of model degrees of freedom ("
			<< nDof << ") does not match number of positional variables ("
			<< PosNameLast << ")!" << endl;
		abort();
	}

	q           = VectorNd::Zero (nDof);
	qdot        = VectorNd::Zero (nDof);
	qddot       = VectorNd::Zero (nDof);
	tau         = VectorNd::Zero (nDof);

  tauMuscles.resize(NumberOfMuscleControls);
  tauMaxMuscles.resize(NumberOfMuscleControls);
  jointDampingCoefficient      = VectorNd::Zero(nDof);

  //highestJointPassiveStiffness = VectorNd::Zero(nDof);
  //smallestInertiaOnJoint       = VectorNd::Zero(nDof);
  //jointTimeConstant            = VectorNd::Zero(nDof);

  //for(unsigned int i = 0; i < nDof; ++i){
  // jointTimeConstant[i] =  8.0*2.0*M_PI;
  //}
  //The 2.7 Hz is from 
  /*
    Eur J Appl Physiol Occup Physiol. 1996;74(1-2):119-32.
    Mechanically corrected EMG for the continuous estimation of erector spinae muscle loading during repetitive lifting.
    Potvin JR, Norman RW, McGill SM.
  */
  //jointTimeConstant[PosMiddleTrunkRotY] = 2.7*2.0*M_PI;
  //jointTimeConstant[PosUpperTrunkRotY]  = 2.7*2.0*M_PI;

  //Get the smallest rotational inertia at each joint.
  //double bodyInertiaY;
  //double parentInertiaY;
  //double smallestInertiaY;
  //for(unsigned int i = 0; i < model.mBodies.size(); ++i){
    //bodyInertiaY   = model.mBodies[i].mInertia(1,1);
    //parentInertiaY = model.mBodies[model.lambda[i]].mInertia(1,1);
    //if(bodyInertiaY<parentInertiaY){
    //  smallestInertiaY = bodyInertiaY;
    //}else{
    //  smallestInertiaY = parentInertiaY;
    //}
    //smallestInertiaOnJoint(model.mJoints[i].q_index) = smallestInertiaY;
  //}



  /*
    If I solve for the activation signals of the muscles (assuming no
    co activation) I get these muscles with the highest values of
    d/dt activation

    max(|d/dt act|) min(|act_tau|) i   Name
    2.228657        0.223979      1   RightHipExtensionRotY
    2.234334        0.237071      7   LeftHipExtensionRotY
    1.798192        0.317295      13  MiddleTrunkExtensionRotY
    2.762980        0.189397      15  UpperTrunkExtensionRotY

    If say a plausible range is from half this size (2x as fast) to 
    the maximum in the data then I have [0.0946985, 0.189397]
    which between 2x and 3.5x slower than the time constant from Thelen et al. 
    which is normally used in the literature.

    Thelen, D. G. (2003). Adjustment of muscle mechanics model
    parameters to simulate dynamic contractions in older adults.
    Transactions-American Society Of Mechanical Engineers Journal Of
    Biomechanical Engineering, 125(1), 70-77.
*/

  //This value of act_tau = 1.262646666666667e-01 is in the middle of the 
  //plausible range
  //I'm having a hard time getting anything started. Making this faster.
  activationTimeConstant = 0.0946985;//1.262646666666667e-01;

  act.resize(NumberOfMuscleControls);
  actDot.resize(NumberOfMuscleControls);
  tqMuscleModel.resize(NumberOfMuscleControls);

  exoTauNorm.resize(NumberOfExoControls);
  exoTauDotNorm.resize(NumberOfExoControls);
  exoControl.resize(NumberOfExoControls);
  exoActuatorTauMax.resize(NumberOfExoControls);
  exoSpringStiffness.resize(NumberOfExoControls);
  exoDampingCoefficients.resize(NumberOfExoControls);

  for(unsigned int i=0; i < NumberOfExoControls; i++){
    exoActuatorTauMax[i] = 200/3.0; //This will support 30% of the
                                    //15kg pick up torque. Or 60% of
                                    //the no-load torque
  }


	setupTorqueMuscles (modelfilename.c_str(), true);
	setupJointDamping(true);
  coefficientOfFriction = 0.8;
  boxHalfWidth = 0.20;
  handOffset   = 0;//0.005;
  hipAngleOffset = 0;//0.025;

  //For Rubber & dry concrete
  //http://www.engineeringtoolbox.com/friction-coefficients-d_778.html

  unsigned int BoxId       = model.GetBodyId("Box");
  double boxMass          = model.mBodies[BoxId].mMass;
  Matrix3d boxInertia     = model.mBodies[BoxId].mInertia;
  Vector3d gravityVec     = model.gravity;

  //N.B. The ugly floating point numbers are the max/min accelerations
  //     from the experimental data - found by taking a numerical 2nd
  //     derivative of a filtered versions of the tracked positions.
  maxHandBoxForces(0) = boxInertia(0,0)*0.;
  maxHandBoxForces(1) = boxInertia(1,1)*2.162499999999997e+00;
  maxHandBoxForces(2) = boxInertia(2,2)*0.;
  maxHandBoxForces(3) = boxMass*1.443750000000019e+00 - boxMass*gravityVec(0);
  maxHandBoxForces(4) = boxMass*0. - boxMass*gravityVec(1);
  maxHandBoxForces(5) = boxMass*2.024999999999975e+00 - boxMass*gravityVec(2);

  minHandBoxForces(0) = boxInertia(0,0)*0.;
  minHandBoxForces(1) = boxInertia(1,1)*(-1.346874999999998e+00);
  minHandBoxForces(2) = boxInertia(2,2)*0.;
  minHandBoxForces(3) = boxMass*(-7.124999999999827e-01) - boxMass*gravityVec(0);
  minHandBoxForces(4) = boxMass*0. - boxMass*gravityVec(1);
  minHandBoxForces(5) = boxMass*(-1.537500000000086e+00) - boxMass*gravityVec(2);

  //if(verbose){
    cout << "Constants: " << endl;
    cout << "  coefficient of friction " << coefficientOfFriction << endl;
    cout << "  boxHalfWidth " << boxHalfWidth << endl;
    cout << "  hand offset " << handOffset << endl;
    cout << "  max. net hand-box forces "
         << maxHandBoxForces(0) << " "
         << maxHandBoxForces(1) << " "
         << maxHandBoxForces(2) << " "
         << maxHandBoxForces(3) << " "
         << maxHandBoxForces(4) << " "
         << maxHandBoxForces(5) << endl;
    cout << "  min. net hand-box forces "
         << minHandBoxForces(0) << " "
         << minHandBoxForces(1) << " "
         << minHandBoxForces(2) << " "
         << minHandBoxForces(3) << " "
         << minHandBoxForces(4) << " "
         << minHandBoxForces(5) << endl;
  //}

	return true;
}
// =============================================================================


void WholeBodyLifterModel::setupTorqueMuscles ( const char* filename,
                                                bool verbose) {

  std::vector < int > angleSigns (NumberOfMuscleControls);
  std::vector < int > torqueSigns(NumberOfMuscleControls);

  // Middletrunk to Head
  angleSigns.at(ControlMiddleTrunkExtensionRotY)    =  1.0;
  angleSigns.at(ControlMiddleTrunkFlexionRotY)      =  1.0;
  angleSigns.at(ControlUpperTrunkExtensionRotY)     =  1.0;
  angleSigns.at(ControlUpperTrunkFlexionRotY)       =  1.0;
  angleSigns.at(ControlHeadExtensionRotY)           =  1.0;
  angleSigns.at(ControlHeadFlexionRotY)             =  1.0;

  torqueSigns.at(ControlMiddleTrunkExtensionRotY)   = -1.0;
  torqueSigns.at(ControlMiddleTrunkFlexionRotY)     =  1.0;
  torqueSigns.at(ControlUpperTrunkExtensionRotY)    = -1.0;
  torqueSigns.at(ControlUpperTrunkFlexionRotY)      =  1.0;
  torqueSigns.at(ControlHeadExtensionRotY)          = -1.0;
  torqueSigns.at(ControlHeadFlexionRotY)            =  1.0;

  // Hips
  angleSigns.at(ControlRightHipExtensionRotY)       = -1.0;
  angleSigns.at(ControlRightHipFlexionRotY)         = -1.0;
  
  torqueSigns.at(ControlRightHipExtensionRotY)      =  1.0;
  torqueSigns.at(ControlRightHipFlexionRotY)        = -1.0;
  

  // Knees
  angleSigns.at(ControlRightKneeExtensionRotY)      =  1.0;
  angleSigns.at(ControlRightKneeFlexionRotY)        =  1.0;
  
  torqueSigns.at(ControlRightKneeExtensionRotY)     = -1.0;
  torqueSigns.at(ControlRightKneeFlexionRotY)       =  1.0;
  
  // Ankles
  angleSigns.at(ControlRightAnkleExtensionRotY)     = -1.0;
  angleSigns.at(ControlRightAnkleFlexionRotY)       = -1.0;

  torqueSigns.at(ControlRightAnkleExtensionRotY)    =  1.0;
  torqueSigns.at(ControlRightAnkleFlexionRotY)      = -1.0;

  // Ankles
  angleSigns.at(ControlRightUpperArmExtensionRotY)   = -1.0;
  angleSigns.at(ControlRightUpperArmFlexionRotY)    =  -1.0;

  torqueSigns.at(ControlRightUpperArmExtensionRotY) =   1.0;
  torqueSigns.at(ControlRightUpperArmFlexionRotY)   =  -1.0;

  // Lower Arms
  angleSigns.at(ControlRightLowerArmExtensionRotY)  =  -1.0;
  angleSigns.at(ControlRightLowerArmFlexionRotY)    =  -1.0;

  torqueSigns.at(ControlRightLowerArmExtensionRotY) =   1.0;
  torqueSigns.at(ControlRightLowerArmFlexionRotY)   =  -1.0;

  // Hands: This is not extension/flexion it is ulnar and radial deviation
  angleSigns.at(ControlRightHandExtensionRotY)      =   1.0;
  angleSigns.at(ControlRightHandFlexionRotY)        =   1.0;

  torqueSigns.at(ControlRightHandExtensionRotY)     =   1.0; //ulnar deviation
  torqueSigns.at(ControlRightHandFlexionRotY)       =  -1.0; //radial deviation

  // This array of ints corresponds to the torque muscle and directon in the
  //gymnast data set
  std::vector < int > gymTqIdx(NumberOfMuscleControls);
  gymTqIdx.at(ControlRightHipExtensionRotY)     = JointTorqueSet::HipExtension;
  gymTqIdx.at(ControlRightHipFlexionRotY)       = JointTorqueSet::HipFlexion;
  gymTqIdx.at(ControlRightKneeExtensionRotY)    = JointTorqueSet::KneeExtension;
  gymTqIdx.at(ControlRightKneeFlexionRotY)      = JointTorqueSet::KneeFlexion;
  gymTqIdx.at(ControlRightAnkleExtensionRotY)   = JointTorqueSet::AnkleExtension;
  gymTqIdx.at(ControlRightAnkleFlexionRotY)     = JointTorqueSet::AnkleFlexion;
  gymTqIdx.at(ControlMiddleTrunkExtensionRotY)  = JointTorqueSet::LumbarExtension;
  gymTqIdx.at(ControlMiddleTrunkFlexionRotY)    = JointTorqueSet::LumbarFlexion;
  gymTqIdx.at(ControlUpperTrunkExtensionRotY)   = JointTorqueSet::LumbarExtension;
  gymTqIdx.at(ControlUpperTrunkFlexionRotY)     = JointTorqueSet::LumbarFlexion;
  gymTqIdx.at(ControlHeadExtensionRotY)         = JointTorqueSet::HipExtension;
  gymTqIdx.at(ControlHeadFlexionRotY)           = JointTorqueSet::HipFlexion;
  gymTqIdx.at(ControlRightUpperArmExtensionRotY)= JointTorqueSet::ShoulderExtension;
  gymTqIdx.at(ControlRightUpperArmFlexionRotY)  = JointTorqueSet::ShoulderFlexion;
  gymTqIdx.at(ControlRightLowerArmExtensionRotY)= JointTorqueSet::ElbowExtension;
  gymTqIdx.at(ControlRightLowerArmFlexionRotY)  = JointTorqueSet::ElbowFlexion;
  gymTqIdx.at(ControlRightHandExtensionRotY)    = JointTorqueSet::WristUlnarDeviation;
  gymTqIdx.at(ControlRightHandFlexionRotY)      = JointTorqueSet::WristRadialDeviation;

  SubjectInformation subjectInfo;
  loadMetaData(filename,verbose);
  subjectInfo.gender          = GenderSet::Male;
  subjectInfo.ageGroup        = AgeGroupSet::Young18To25;
  subjectInfo.heightInMeters  = subjectMetaData.subject_height;
  subjectInfo.massInKg        = subjectMetaData.subject_weight;

  std::string muscleName("");
  double passiveTorqueScale = 1.0;
  double jointAngleOffset   = 0.0;
  double normDampingCoeff   = 0.1;

  double maxLumbarFlexionAngle = 57.0*M_PI/180.0;

  if(verbose){
    printf("\n\n==================================================\n");
    printf(" setupTorqueMuscles \n");
    printf("==================================================\n");
    printf("index,\ttauMax,\tomegaMax,\tname\n");
  }

  for (int j = 0;
           j < NumberOfMuscleControls;
           j++) {

    muscleName = Gymnast::JointTorqueNames[gymTqIdx.at(j)];
    tqMuscleModel.at(j) = Millard2016TorqueMuscle (  DataSet::Gymnast,
                                                     subjectInfo,
                                                     gymTqIdx.at(j),
                                                     jointAngleOffset,
                                                     angleSigns.at(j),
                                                     torqueSigns.at(j),
                                                     muscleName);

    tqMuscleModel.at(j).setPassiveTorqueScale(passiveTorqueScale);
    tqMuscleModel.at(j).setNormalizedDampingCoefficient(normDampingCoeff);


    if(j == ControlHeadExtensionRotY || j == ControlHeadFlexionRotY){
        tqMuscleModel.at(ControlHeadExtensionRotY)
          .setMaximumActiveIsometricTorque(50.0);
        tqMuscleModel.at(ControlHeadFlexionRotY)
          .setMaximumActiveIsometricTorque(50.0);
    }


    if(j == ControlHeadFlexionRotY || j == ControlHeadExtensionRotY){
      tqMuscleModel.at(j).setPassiveTorqueScale(0.0);
    }

    /*
      From the IKFit results we know that
      Total hip extension

        StateLeftHipRotY         -1.677730e+00
        LeftHipExtension          1.445407e+02
        LeftHipFlexion            7.318760e+01
                    Total:  71.3531  @ -1.677730e+00

        StateRightHipRotY        -1.803660e+00
        RightHipExtension         2.120004e+02
        RightHipFlexion           7.468660e+01
                    Total: 137.3138 @ -1.803660e+00

        Ideally I would fit the curve using these two points ... however 
        that makes a big assumption that both the hips are just passive.
        I'll make a more reasonable assumption and just assume that the right
        hip (which has the higher flexion angle) is passive.

             137.3138 Nm @ -1.803660e+00

    */
    if(gymTqIdx.at(j) == JointTorqueSet::HipExtension 
          && j != ControlHeadExtensionRotY
          && j != ControlHeadFlexionRotY){

      double angleTarget  = -2.124;
      double torqueTarget =  74.24*0.8;
      
      tqMuscleModel.at(j).setPassiveTorqueScale(1.0);
      tqMuscleModel.at(j).setPassiveCurveAngleOffset(0.0);
      tqMuscleModel.at(j).fitPassiveTorqueScale(angleTarget,
                                                torqueTarget);

      if(verbose){
        double test = tqMuscleModel.at(j).calcJointTorque(angleTarget,0,0);
        TorqueMuscleInfo tmi;
        tqMuscleModel.at(j).calcTorqueMuscleInfo(angleTarget,0,0,tmi);
        printf("\n\nHipExtension tp adj.: %i: angle %f target %f test %f"
               " passive stiffness %f\n\n",
                   j, angleTarget, torqueTarget, test, tmi.fiberStiffness);
      }
      double omegaMax =
          tqMuscleModel.at(j).getMaximumConcentricJointAngularVelocity();
      double omegaMaxUpd = 1.5*fabs(omegaMax);
      tqMuscleModel.at(j).setMaximumConcentricJointAngularVelocity(
                            omegaMaxUpd);

      //As determined using an inverse dynamics analysis to get tau, and
      //then solving for tauMax assuming a 75% activation.
      double tauMaxScaling = 1.284634044566721e+00;
      double tauMaxOriginal = 0.;
      tauMaxOriginal = tqMuscleModel.at(j).getMaximumActiveIsometricTorque();
      tqMuscleModel.at(j).setMaximumActiveIsometricTorque(
            tauMaxOriginal*tauMaxScaling);

      if(verbose){
        printf("\nHipExtension tauMax from %f to %f\n",
               tauMaxOriginal,
               tauMaxOriginal*tauMaxScaling);
      }      

    }

    if(gymTqIdx.at(j) == JointTorqueSet::ElbowFlexion){

      tqMuscleModel.at(j).fitPassiveTorqueScale(
        tqMuscleModel.at(j).getJointAngleAtOneNormalizedPassiveIsometricTorque(),
        tqMuscleModel.at(j).getMaximumActiveIsometricTorque()*0.025);
    }

    
    //Double the strength of the joint if this is a joint of the right 
    //leg or arm
    if(   j == ControlRightHipExtensionRotY      ||           
          j == ControlRightHipFlexionRotY        ||       
          j == ControlRightKneeExtensionRotY     ||             
          j == ControlRightKneeFlexionRotY       ||         
          j == ControlRightAnkleExtensionRotY    ||               
          j == ControlRightAnkleFlexionRotY      ||           
          j == ControlRightUpperArmExtensionRotY ||        
          j == ControlRightUpperArmFlexionRotY   ||                 
          j == ControlRightLowerArmExtensionRotY ||                  
          j == ControlRightLowerArmFlexionRotY   ||              
          j == ControlRightHandExtensionRotY     ||                   
          j == ControlRightHandFlexionRotY          )
    {
      tqMuscleModel.at(j).setMaximumActiveIsometricTorque(
        2.0*tqMuscleModel.at(j).getMaximumActiveIsometricTorque());
    }    


    //Populate the maximum active isometric torque within the range of 
    //motion.
    if(gymTqIdx.at(j) == JointTorqueSet::LumbarExtension){
      tqMuscleModel.at(j).setPassiveTorqueScale(1.0);

      tauMaxMuscles.at(j) = 
        tqMuscleModel.at(j).calcJointTorque(maxLumbarFlexionAngle, 0.0, 1.0)
      - tqMuscleModel.at(j).calcJointTorque(maxLumbarFlexionAngle, 0.0, 0.0);

    }else{
      tauMaxMuscles.at(j) = 
          tqMuscleModel.at(j).getMaximumActiveIsometricTorque()
          *tqMuscleModel.at(j).getJointTorqueSign();
    }


    if(verbose){
      printf("j: %i,\ttauMax: %f,\tomegaMax: %f, \t%s\n",
          j, 
          tauMaxMuscles.at(j), 
          tqMuscleModel.at(j).getMaximumConcentricJointAngularVelocity(),
          tqMuscleModel.at(j).getName().c_str());

      if(gymTqIdx.at(j) == JointTorqueSet::LumbarExtension){
          TorqueMuscleInfo tmi;
          tqMuscleModel.at(j).calcTorqueMuscleInfo(maxLumbarFlexionAngle,0,0,tmi);
          printf("\n\tpassive stiffness at max. flex %f\n",
                 j,tmi.fiberStiffness);

      }
    }
  }
}

void WholeBodyLifterModel::setupJointDamping(bool verbose){

  double normDamping = 0.2;

  if(verbose){
    printf("\n\n==================================================\n");
    printf(" setupJointDamping: using norm. damping of %0.3f \n",
             normDamping);
    printf("==================================================\n");
    printf("index,\tdamping,\ttauMax0,\twMax0,"
                            "\t\ttauMax1,\twMax1,\t\tname0,\t\tname1\n");
  }

  for(unsigned int i=0; i<nDof; ++i){
    jointDampingCoefficient(i) = 0.;
  }

  int k = 0;
  double tauMax0 = 0.;
  double tauMax1 = 0.;
  double omegaMax0 = 0.;
  double omegaMax1 = 0.;
  std::string name0, name1, pad;

  for(int i=StateHumanInternalPosFirst; i <= StatePosLast ; ++i){

    name0     = tqMuscleModel.at(k).getName();
    tauMax0   = abs(tauMaxMuscles.at(k));
    omegaMax0 = abs(tqMuscleModel.at(k)
                    .getMaximumConcentricJointAngularVelocity());
    if(abs(tauMaxMuscles.at(k)) < 0.){
      cerr << "tauMaxMuscles at " << k << " not set. "
           << "Call setupTorqueMuscles first.";
      abort();           
    }
    ++k;

    name1     = tqMuscleModel.at(k).getName();
    tauMax1   = abs(tauMaxMuscles.at(k));
    omegaMax1 = abs(tqMuscleModel.at(k)
                    .getMaximumConcentricJointAngularVelocity());  
    if(abs(tauMaxMuscles.at(k)) < 0.){
      cerr << "tauMaxMuscles at " << k << " not set. "
           << "Call setupTorqueMuscles first.";
      abort();           
    }
    ++k;

    jointDampingCoefficient(i) =
          normDamping*(tauMax1+tauMax0)/(omegaMax0+omegaMax1);
    
    if(i == StateRightUpperArmRotY || 
       i == StateRightLowerArmRotY ||
       i == StateRightHandRotY     )
    {
      jointDampingCoefficient(i) *= 2.0;
    }
    
    if(verbose){
      pad = "";
      if(name0.length() < 15){
        pad = "    ";
      }
      printf("%i,\t%f,\t%f,\t%f,\t%f,\t%f,\t%s,%s\t%s\n",
              i, 
              jointDampingCoefficient(i),
              tauMax0,
              omegaMax0,              
              tauMax1,              
              omegaMax1,
              name0.c_str(),              
              pad.c_str(),
              name1.c_str());
    }

  }

}
// =============================================================================
bool WholeBodyLifterModel::
loadInverseDynamicsDataFromFile(unsigned int stoopIdx,
                                bool updateBoxPickupPointLocations,
                                bool offsetBoxPickupPointLocations,
                                bool verbose)
{
  ostringstream stoopStreamName;
  stoopStreamName << "stoop" << stoopIdx;
  string stoopName = stoopStreamName.str();
  string timeFileName("../PRE/ID/2D/time_" + stoopName + ".csv");
  string qFileName(   "../PRE/ID/2D/q_"    + stoopName + ".csv");
  string qDotFileName("../PRE/ID/2D/qDot_" + stoopName + ".csv");
  string tauFileName("../PRE/ID/2D/tau_"   + stoopName + ".csv");
  string fpFileName( "../PRE/VUA2UHEI/2D/" + stoopName +
                     "_F1Cop1F2Cop2_2D.csv");


  readMatrixFromFile(timeFileName,0, timeExp);
  readMatrixFromFile(qFileName,   0, qExp);
  readMatrixFromFile(qDotFileName,0, qDotExp);
  readMatrixFromFile(tauFileName, 0, tauExp);
  readMatrixFromFile(fpFileName,  0, fpExp);

  unsigned int idxFzBox = 3;
  double fzBoxBias = 1e10;

  if(timeExp.size() != qExp.size()    ||
     timeExp.size() != qDotExp.size() ||
     timeExp.size() != tauExp.size()  ||
     timeExp.size() != fpExp.size()){
    cerr << "Sense of time between timeExp, "
         << "qExp, qDotExp, tauExp and fpExp"
         << " is inconsistent " << endl;
    assert(0);
    abort();
  }

  //The Kistler plate may have a bias in it. The smallest Fz force
  //should be 0.
  for(unsigned int i = 0; i < fpExp.size(); ++i){
    if(fpExp[i][idxFzBox] < fzBoxBias){
      fzBoxBias = fpExp[i][idxFzBox];
    }
  }

  //Remove the bias.
  for(unsigned int i = 0; i < fpExp.size(); ++i){
    fpExp[i][idxFzBox] = fpExp[i][idxFzBox]-fzBoxBias;
  }

  //The box starts on the plate.
  double fzBoxWeight = fpExp[0][idxFzBox];

  unsigned int idxBoxPickupExp = 0;
  for(unsigned int i = 0; i < fpExp.size(); ++i){
    if(idxBoxPickupExp == 0 && fpExp[i][idxFzBox] < 0.95*fzBoxWeight){
      idxBoxPickupExp = i;
    }
  }

  unsigned int idxBoxOffPlate = 0;
  for(unsigned int i = 0; i < fpExp.size(); ++i){
    if(fpExp[i][idxFzBox] > 0.05*fzBoxWeight){
      idxBoxOffPlate = i;
    }
  }


  for(unsigned int i = 0; i < idxBoxOffPlate; ++i){
    qExp[i][PosIDBoxRotY] = 0;
  }

  if(verbose){
    cout << "WholeBodyLifterModel::loadInverseDynamicsDataFromFile"
         << endl;
    cout << " Loaded: " << endl;
    cout << "  :" <<  timeFileName << endl;
    cout << "  :" <<  qDotFileName << endl;
    cout << "  :" <<  tauFileName  << endl;
    cout << "  :" <<  fpFileName  << endl;
    cout << " Identified pick up index and time as " << endl;
    cout << "  :" << idxBoxPickupExp << endl;
    cout << "  :" << timeExp[idxBoxPickupExp][0] << endl;
    cout << " Zeroed box angle up until " << endl;
    cout << "  :" << idxBoxOffPlate << endl;
    cout << "  :" << timeExp[idxBoxOffPlate][0] << endl;
  }

  if(updateBoxPickupPointLocations){
    VectorNd qPickup = VectorNd::Zero(model.dof_count);
    unsigned int j = PosBoxFirst;
    unsigned int k = PosIDMapBoxFirst;
    for(unsigned int i = 0; i < NumberOfBoxPositions; ++i){
      qPickup(j) = qExp[idxBoxPickupExp][MapIDToModel[k]];
      j++;
      k++;
    }

    j = PosHumanFirst;
    k = PosIDMapHumanFirst;
    for(unsigned int i = 0; i < NumberOfHumanPositions; ++i){
      qPickup(j) = qExp[idxBoxPickupExp][MapIDToModel[k]];
      j++;
      k++;
    }

   // for(unsigned int i=0; i<model.dof_count;++i ){
   //   qPickup(i) = qExp[idxBoxPickupExp][MapIDToModel[i]];
   // }

    //We just get the left hand information because we're
    //going to make the right hand identical.
    Vector3d zeroVec = Vector3d::Zero();
    Vector3d RightHandRootPos = CalcBodyToBaseCoordinates(
                  model,
                  qPickup,
                  model.GetBodyId("Hand_R"),
                  zeroVec,
                  true);

    Vector3d RightHandPos = CalcBodyToBaseCoordinates(
                  model,
                  qPickup,
                  model.GetBodyId("Hand_R"),
                  pointInfos[PointDistalMetacarpal_R].point_local,
                  false);

    Matrix3d ER0 = CalcBodyWorldOrientation(
                    model,
                    qPickup,
                    model.GetBodyId("Hand_R"),
                    false);

    Vector3d boxPos;
    boxPos(0)    = qExp[0][PosIDBoxPosX];
    boxPos(1)    = 0;
    boxPos(2)    = qExp[0][PosIDBoxPosZ];
    double b     = qExp[0][PosIDBoxRotY];
    double cb    = cos(b);
    double sb    = sin(b);

    Matrix3d E0b;

    E0b <<   cb, 0., sb,
             0., 1., 0.,
            -sb, 0., cb;

    Matrix3d eye;

    eye << 1, 0, 0,
           0, 1, 0,
           0, 0, 1;


    Vector3d rbRb = E0b.transpose()*(RightHandPos - boxPos);
    rbRb(0) = 0; //Put this above the CoM of the box

    Vector3d rhRh = ER0*((boxPos + E0b*rbRb)-RightHandRootPos);

    if(verbose){
      cout << "Updating box and hand point locations" << endl;
      printf("  :Old PointBoxAttach_R: (%.3f,%.3f,%.3f)\n",
               pointInfos[PointBoxAttach_R].point_local(0),
               pointInfos[PointBoxAttach_R].point_local(1),
               pointInfos[PointBoxAttach_R].point_local(2));
      printf("  :Old PointDistalMetacarpal_R: (%.3f,%.3f,%.3f)\n",
               pointInfos[PointDistalMetacarpal_R].point_local(0),
               pointInfos[PointDistalMetacarpal_R].point_local(1),
               pointInfos[PointDistalMetacarpal_R].point_local(2));
    }

    pointInfos[PointBoxAttach_R].point_local = rbRb;
    pointInfos[PointDistalMetacarpal_R].point_local = rhRh;

    if(verbose){
      printf("  :New PointBoxAttach_R: (%.3f,%.3f,%.3f)\n",
               pointInfos[PointBoxAttach_R].point_local(0),
               pointInfos[PointBoxAttach_R].point_local(1),
               pointInfos[PointBoxAttach_R].point_local(2));
      printf("  :New PointDistalMetacarpal_R: (%.3f,%.3f,%.3f)\n",
               pointInfos[PointDistalMetacarpal_R].point_local(0),
               pointInfos[PointDistalMetacarpal_R].point_local(1),
               pointInfos[PointDistalMetacarpal_R].point_local(2));
    }

    for(unsigned int i =0; i<constraint_sets.size();++i){
      for(unsigned int j=0; j < constraint_sets[i].name.size();++j){
        if(strcmp(constraint_sets[i].name[j].c_str(),"BoxRX") == 0 ||
           strcmp(constraint_sets[i].name[j].c_str(),"BoxRZ") == 0 ||
           strcmp(constraint_sets[i].name[j].c_str(),"BoxRY") == 0){
          constraint_sets[i].X_p[j].r = rbRb;
          constraint_sets[i].X_p[j].E = eye;
          constraint_sets[i].X_s[j].r =
              pointInfos[PointDistalMetacarpal_R].point_local;
        }        
      }
    }


  }


  return true;

}

// =============================================================================

bool WholeBodyLifterModel::loadMetaData (
		string filename,
		bool verbose) {

	LuaTable lua_table = LuaTable::fromFile (filename.c_str());
	int subject_count = lua_table["metadata"].length();

	if(subject_count != 1){
		cerr << "The lua file contains meta data for "
			<< subject_count
			<< " it should contain data for 1 subject"
			<< endl;
	}

	subjectMetaData = lua_table["metadata"][1];
	return true;
}

// =============================================================================

bool WholeBodyLifterModel::loadPoints (string filename, bool verbose) {
  LuaTable lua_table = LuaTable::fromFile (filename.c_str());
  int point_count = lua_table["points"].length();

  for (int pi = 1; pi <= point_count; pi++) {

    Point point = lua_table["points"][pi];
		PointId point_name = getPointIdFromString (point.name.c_str());

    if (point_name == PointNameLast) {
      continue;
    }
    point.body_id = model.GetBodyId (point.body_name.c_str());
    pointInfos[point_name] = point;

    if (verbose) {
      cout << "Point '"           << point.name
           << "' (PointName = "   << point_name << ")"    << endl;
      cout << "  body        = "  << point.body_name
           << " (id = "           << point.body_id << ")" << endl;
      cout << "  point_local = '" << point.point_local.transpose() << endl;
    }
  }

  // check whether we missed some points
  Point default_point;
  for (unsigned int i = 0; i < PointNameLast - 1; i++) {
    if (pointInfos[i].name == default_point.name) {
      cerr  << "Error: could not find point info for point '"
            << PointMap[i].name_str
            << "' in file "
            << filename << "." << endl;
      abort();
    }
  }

  return true;
}

// =============================================================================

bool WholeBodyLifterModel::loadConstraintSets (
		string filename,
		bool verbose) {
	LuaTable     lua_table = LuaTable::fromFile (filename.c_str());
	LuaTableNode ln_css    = lua_table["constraint_sets"];

	constraint_sets.resize(ConstraintSetNameLast);

	for (unsigned i = 0; i < ln_css.keys().size(); i++) {
		if (ln_css.keys()[i].type == LuaKey::String) {
			string cs_name = ln_css.keys()[i].string_value;
			CSId cs_id     = getConstraintSetIdFromString (cs_name.c_str());
			if (verbose) {
				cout << endl;
				cout << "Constraint " <<  cs_name << " with ID " << cs_id << " :" << endl;
				cout << endl;
			}
			if (cs_id < ConstraintSetNameLast) {
				LuaTableNode ln_cs = ln_css[cs_name.c_str()];
				cs_infos[cs_id].name = cs_name;
				cs_infos[cs_id].constraints.resize(ln_cs.length());
				for (unsigned j = 0; j < ln_cs.length(); j++) {
					ConstraintInfo ci = ln_cs[j + 1];
					switch (ci.type) {
					case ConstraintSet::ContactConstraint:
						ci.point_id = getPointIdFromString (ci.point_name.c_str());
						constraint_sets[cs_id].AddContactConstraint (
								pointInfos[ci.point_id].body_id,
								pointInfos[ci.point_id].point_local,
								ci.normal);
						break;

					case ConstraintSet::LoopConstraint:
						constraint_sets[cs_id].AddLoopConstraint (
									model.GetBodyId(ci.predecessor_body.c_str()),
									model.GetBodyId(ci.successor_body.c_str()),
									ci.predecessor_transform,
									ci.successor_transform,
									ci.axis,
									ci.baumgarte_enabled,
									ci.stabilization_coefficient,
									ci.name.c_str());
						break;

					case ConstraintSet::ConstraintTypeLast:
						break;
					}
					cs_infos[cs_id].constraints[j] = ci;
					if (verbose) {
						cout << setw(16) << cs_name << " " << setw(3) << j << endl;
						cout << "    type = " << ci.type << endl
								 << "    " << ci.point_id << endl
								 << "    " << ci.normal.transpose() << endl
								 << "    " << ci.name << endl
								 << "    " << ci.predecessor_body << endl
								 << "    " << ci.successor_body << endl
								 << "    " << ci.axis << endl
								 << "    " << ci.stabilization_coefficient << endl
								 << "    " << ci.predecessor_transform << endl
								 << "    " << ci.successor_transform << endl;
					}
				}
				constraint_sets[cs_id].Bind (model);
			} else {
				cerr << "Ignoring constraint set " << cs_id << " with name "
						 << cs_name << " : not specified in enum CSId." << endl;
			}
		} else {
			cerr << "Ignoring constraint set " << i
					 << " with invalid name, string expected." << endl;
		}
	}

  // check whether we missed some sets
  ConstraintSetInfo default_constraint_set;
  for (unsigned int i = 0; i < ConstraintSetNameLast; i++) {
		if (cs_infos[i].name != ConstraintSetMap[i].name_str) {
      cerr  << "Error: could not find ConstraintSet info for set '"
            << ConstraintSetMap[i].name_str << "' in file "
            << filename << "." << endl;
      abort();
    }
  }

  return true;
}
