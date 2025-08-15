#include <algorithm>
#include <cmath>
#include "def_usrmod.hpp"
#include <iostream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <vector>
#include <fstream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include "datfileutils.h"
#include "wholebodylifter2d_Model.h"


using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;
using namespace std;

int nmos   = -1;  /* Number of phases (MOdel Stages) */
int np     = -1;  /* Number of parameters */
int nrc    =  0;  /* Number of coupled constraints */
int nrce   =  0;  /* Number of coupled equality constraints */
int nxd    = -1;  /* Number of differential states */
int nxa    =  0;  /* Number of algebraic states */
int nu     = -1;  /* Number of controls */
int npr    =  0;  /* Number of local parameters */

const unsigned int stoopIdx = 2;

const unsigned int NoOfControls   = ControlNameLast;
const unsigned int NoOfStates     = StateNameLast;
const unsigned int NoOfPos        = PosNameLast;
const unsigned int NoOfParams     = ParamNameLast;
const double eta_tikhonov       = 1e-6;

WholeBodyLifterModel wholebodylifter;

//==============================================================================
// -------------------------------------------------------------------------- //
// Parallelization Stuff
// -------------------------------------------------------------------------- //
long nmos_m1 = 0;
vector<long> nmsn;
vector<WholeBodyLifterModel> model_clones;
// -------------------------------------------------------------------------- //

const char* const model_file = "DAT/vuaHuman2dRightyExo_15kgBox_BaumgarteEnabled.lua";
const string datfile_name = "DAT/wholebodylifter2d.dat";

bool verbose = false;

//==============================================================================

/* Model */
void LoadModelAndConstraints () {
  if (verbose){
    cout << "[LoadModelAndConstraints] Loading Model: '" << model_file
         << "'" << endl;
  }

  if (!wholebodylifter.loadFromFile (model_file, datfile_name, verbose)){
    abort();  
  }
  
  if (verbose){
    cout << "[LoadModelAndConstraints] Loading Points" <<  endl;
  }

  if (!wholebodylifter.loadPoints (model_file, verbose)){
    abort();
  }

  if (verbose){
    cout << "[LoadModelAndConstraints] Loading Constraint Sets" <<  endl;
  }

  if (!wholebodylifter.loadConstraintSets (model_file, verbose)){
    abort();
  }

  if (verbose){
    cout << "[LoadModelAndConstraints] Model and Constraint loading successful!"
         << endl;
  }

  if (!wholebodylifter.loadInverseDynamicsDataFromFile(
                              stoopIdx,true,true,true)){
    abort();
  }

  if (verbose){
    cout << "[loadInverseDynamicsDataFromFile] Inverse dynamics data for stoop"
         << "trial "<< stoopIdx << " loaded successfully"
         << endl;
  }
}

//==============================================================================

static void initialize_from_data (
  long   *imos,      /* index of model stage (I) */
  long   *imsn,      /* index of m.s. node on current model stage (I) */
  double *sd,        /* differential states at m.s. node (I/O) */
  double *sa,        /* algebraic states at m.s. node (I/O) */
  double *u,         /* controls at m.s. node (I/O) */
  double *udot,      /* control slopes at m.s. node (I/O) */
  double *ue,        /* controls at end of m.s. interval (I/O) */
  double *uedot,     /* control slopes at end of m.s. interval (I/O) */
  double *p,         /* global model parameters (I/O) */
  double *h,         /* model stage durations (I/O) */
  double *pr         /* local i.p.c. parameters (I/O) */
) {

  nmos_m1 = max(nmos_m1, *imos);


  if (nmsn.size() <= static_cast<unsigned>(nmos_m1)) {
    nmsn.resize(nmos_m1 + 1, 0);
  }
  nmsn[*imos] = max(nmsn[*imos], *imsn + 1);
  model_clones.resize(accumulate(nmsn.begin(), nmsn.end(), 0) + 1,
                      wholebodylifter);

#ifdef _POLISH_STATE_FROM_DAT_FILE_
  unsigned int phase = *imos;
  CSId constraintSetId = wholebodylifter.getConstraintSetId(phase);
  wholebodylifter.projectStateOntoMultibodyConstraintManifold(
                    sd,constraintSetId);

  for(unsigned int i=0; i<model_clones.size();++i){
    WholeBodyLifterModel & wholebodylifter = model_clones[i];
    wholebodylifter.updateState (sd, u, p, constraintSetId);
  }
#endif

}

//==============================================================================



//N.B. We do not need to distinguish between the different constraint
//sets as the muscle torques are only a function of the state and
//the controls. The constraints do not matter unless you are
//evaluating the state derivative.
void lfcn_minTauSqr(
		double *t,
		double *xd,
		double *xa,
		double *u,
		double *p,
		double *lval,
		double *rwh,
		long *iwh,
		InfoPtr *info) {
	WholeBodyLifterModel & wholebodylifter = model_clones[info->cnode];
	wholebodylifter.updateState (xd, u, p, CSBothFeetFlat2D);
	*lval = wholebodylifter.getSumNormalizedMuscleTorquesSqr();
  //*lval += eta_tikhonov * wholebodylifter.getSumActSqr();
  *lval *= 0.5;
}

//==============================================================================

void lfcn_minActSqr (
		double *t,
		double *xd,
		double *xa,
		double *u,
		double *p,
		double *lval,
		double *rwh,
		long *iwh,
		InfoPtr *info) {
	WholeBodyLifterModel & wholebodylifter = model_clones[info->cnode];
	wholebodylifter.updateState (xd, u, p, CSBothFeetFlat2D);
  *lval = wholebodylifter.getSumActSqr();
  //*lval += 0.001 * wholebodylifter.getSumNormalizedMuscleTorquesSqr();
  *lval *= 0.25;

}

void lfcn_cost(
    double *t,
    double *xd,
    double *xa,
    double *u,
    double *p,
    double *lval,
    double *rwh,
    long *iwh,
    InfoPtr *info) {

#ifdef _MIN_TAU_SQ_
  lfcn_minTauSqr(t,xd,xa,u,p,lval,rwh,iwh,info);
#endif
#ifdef _MIN_ACT_SQ_
  lfcn_minActSqr(t,xd,xa,u,p,lval,rwh,iwh,info);
#endif

}


//==============================================================================

// Forward Dynamics
void ffcn_bend (
		double *t,
		double *xd,
		double *xa,
		double *u,
		double *p,
		double *rhs,
		double *rwh,
		long *iwh,
		InfoPtr *info) {
	WholeBodyLifterModel & wholebodylifter = model_clones[info->cnode];
	wholebodylifter.updateState (xd, u, p, CSBothFeetFlat2D);
	wholebodylifter.calcForwardDynamicsRhs (rhs);


}

//==============================================================================

void ffcn_wait (
		double *t,
		double *xd,
		double *xa,
		double *u,
		double *p,
		double *rhs,
		double *rwh,
		long *iwh,
		InfoPtr *info) {
	WholeBodyLifterModel & wholebodylifter = model_clones[info->cnode];
	wholebodylifter.updateState (xd, u, p, CSBothFeetFlat2D);
	wholebodylifter.calcForwardDynamicsRhs (rhs);
}

//==============================================================================

void ffcn_transition (
    double *t,
    double *xd,
    double *xa,
    double *u,
    double *p,
    double *rhs,
    double *rwh,
    long *iwh,
    InfoPtr *info) {
  WholeBodyLifterModel & wholebodylifter = model_clones[info->cnode];
  wholebodylifter.updateState (xd, u, p, CSBothFeetFlat2DTransition);
  wholebodylifter.calcForwardDynamicsRhs (rhs);
}

//==============================================================================

void ffcn_lift (
		double *t,
		double *xd,
		double *xa,
		double *u,
		double *p,
		double *rhs,
		double *rwh,
		long *iwh,
		InfoPtr *info) {
	WholeBodyLifterModel & wholebodylifter = model_clones[info->cnode];
	wholebodylifter.updateState (xd, u, p, CSBothFeetFlat2DBoxAttach);
	wholebodylifter.calcForwardDynamicsRhs (rhs);
}

//==============================================================================
static int rdfcn_uprightPose_s_ne = 15+9+3+9+3;
static int rdfcn_uprightPose_s_n  =
            rdfcn_uprightPose_s_ne + 0;

void rdfcn_uprightPose_s (double *ts,
                          double *sd,
                          double *sa,
                          double *u,
                          double *p,
                          double *pr,
                          double *res,
                          long *dpnd,
                          InfoPtr *info)
{
	if (*dpnd) {
		*dpnd = RFCN_DPND(NULL, *sd, 0, *u, *p, 0);
		return;
	}

	WholeBodyLifterModel & wholebodylifter = model_clones[info->cnode];
	wholebodylifter.updateState (sd, u, p, CSBothFeetFlat2D);

  //----------------------------------------
  // Experimental Data: Get the box and human pose
  //----------------------------------------
  VectorNd qExpBox;
  VectorNd qExpHuman;
  qExpBox.resize(NumberOfBoxPositions);
  qExpHuman.resize(NumberOfHumanPositions);

  unsigned int y = PosIDMapBoxFirst;
  for(unsigned int i = 0; i < NumberOfBoxPositions;++i){
    qExpBox(i) = wholebodylifter.qExp[0][MapIDToModel[y]];
    y++;
  }
  y = PosIDMapHumanFirst;
  for(unsigned int i = 0; i < NumberOfHumanPositions;++i){
    qExpHuman(i) = wholebodylifter.qExp[0][MapIDToModel[y]];
    y++;
  }

  //----------------------------------------
  // Key Point locations and velocities
  //----------------------------------------

  Vector3d RightHeelVel = wholebodylifter.getPointVelocity (PointHeel_R);
  Vector3d RightToeVel  = wholebodylifter.getPointVelocity (PointToe_R);

  Vector3d ExoPelvisPos = wholebodylifter.getPointPosition(PointExoPelvis);
  Vector3d PelvisPos    = wholebodylifter.getPointPosition(PointPelvis);
  Vector3d ExoThighRPos = wholebodylifter.getPointPosition(PointExoThigh_R);
  Vector3d ThighRPos    = wholebodylifter.getPointPosition(PointThigh_R);
  Vector3d ExoTrunkRPos = wholebodylifter.getPointPosition(PointExoTrunk_R);
  Vector3d TrunkRPos    = wholebodylifter.getPointPosition(PointTrunk_R);

  Matrix3d ExoPelvisOri = wholebodylifter.getBodyWorldOrientation(PointExoPelvis);
  Matrix3d PelvisOri    = wholebodylifter.getBodyWorldOrientation(PointPelvis);
  Matrix3d ExoThighROri = wholebodylifter.getBodyWorldOrientation(PointExoThigh_R);
  Matrix3d ThighROri    = wholebodylifter.getBodyWorldOrientation(PointThigh_R);
  Matrix3d ExoTrunkROri = wholebodylifter.getBodyWorldOrientation(PointExoTrunk_R);
  Matrix3d TrunkROri    = wholebodylifter.getBodyWorldOrientation(PointTrunk_R);

  SpatialVector ExoPelvisVel  = wholebodylifter.getPointVelocity6D(PointExoPelvis);
  SpatialVector PelvisVel     = wholebodylifter.getPointVelocity6D(PointPelvis);
  SpatialVector ExoThighRVel  = wholebodylifter.getPointVelocity6D(PointExoThigh_R);
  SpatialVector ThighRVel     = wholebodylifter.getPointVelocity6D(PointThigh_R);
  SpatialVector ExoTrunkRVel  = wholebodylifter.getPointVelocity6D(PointExoTrunk_R);
  SpatialVector TrunkRVel     = wholebodylifter.getPointVelocity6D(PointTrunk_R);

  //15: The human and the box match the experimental positions
  int j = 0;
  unsigned int x = PosBoxFirst;
  for(unsigned int i = 0; i < NumberOfBoxPositions; ++i){
      res[j++] = sd[x]-qExpBox(i);
      x++;
  }
  x = PosHumanFirst;
  for(unsigned int i = 0; i < NumberOfHumanPositions; ++i){
      res[j++] = sd[x]-qExpHuman(i);
      x++;
  }

  //6: The x,z positions of the exo-suit contact points align with
  //   the human.
  res[j++] = ExoPelvisPos(0)-PelvisPos(0);
  res[j++] = ExoPelvisPos(2)-PelvisPos(2);
  res[j++] = ExoThighRPos(0)-ThighRPos(0);
  res[j++] = ExoThighRPos(2)-ThighRPos(2);
  res[j++] = ExoTrunkRPos(0)-TrunkRPos(0);
  res[j++] = ExoTrunkRPos(2)-TrunkRPos(2);

  //3: The orientations of the exo-body align: dot product of the
  //   exo's x vector and the body's z vector should be zero.
  res[j++] = ExoPelvisOri(0,0)*PelvisOri(2,0)
           + ExoPelvisOri(0,1)*PelvisOri(2,1)
           + ExoPelvisOri(0,2)*PelvisOri(2,2);
  res[j++] = ExoThighROri(0,0)*ThighROri(2,0)
           + ExoThighROri(0,1)*ThighROri(2,1)
           + ExoThighROri(0,2)*ThighROri(2,2);
  res[j++] = ExoTrunkROri(0,0)*TrunkROri(2,0)
           + ExoTrunkROri(0,1)*TrunkROri(2,1)
           + ExoTrunkROri(0,2)*TrunkROri(2,2);

  //3 The box must be at rest
  res[j++] = sd[StateBoxVelX];
  res[j++] = sd[StateBoxVelZ];
  res[j++] = sd[StateBoxRotVelY];

  //6: The vx,vz,wz velocities of the exo segments match that
  //   of the body they are joined to.
  res[j++] = ExoPelvisVel(1) - PelvisVel(1); //wy
  res[j++] = ExoPelvisVel(3) - PelvisVel(3); //vx
  res[j++] = ExoPelvisVel(5) - PelvisVel(5); //vz

  res[j++] = ExoThighRVel(1) - ThighRVel(1); //wy
  res[j++] = ExoThighRVel(3) - ThighRVel(3); //vx
  res[j++] = ExoThighRVel(5) - ThighRVel(5); //vz

  res[j++] = ExoTrunkRVel(1) - TrunkRVel(1); //wy
  res[j++] = ExoTrunkRVel(3) - TrunkRVel(3); //vx
  res[j++] = ExoTrunkRVel(5) - TrunkRVel(5); //vz

  //3 Foot ground contacts must be at zero velocity.
  res[j++] = RightHeelVel[0];
  res[j++] = RightHeelVel[2];
  res[j++] = RightToeVel[2];  

  assert(j == rdfcn_uprightPose_s_ne);

	assert(j == rdfcn_uprightPose_s_n);
}

//==============================================================================


static int rdfcn_maintainFootContacts_i_ne =
        wholebodylifter.eqCountlumbarVelocity;
static int rdfcn_maintainFootContacts_i_n =
    rdfcn_maintainFootContacts_i_ne
    + wholebodylifter.ieqCountFootGroundForce;

void rdfcn_maintainFootContacts_i ( double *ts,
                                    double *sd,
                                    double *sa,
                                    double *u,
                                    double *p,
                                    double *pr,
                                    double *res,
                                    long *dpnd,
                                    InfoPtr *info)
{
  if (*dpnd) {
    *dpnd = RFCN_DPND(NULL, *sd, 0, *u, *p, 0);
    return;
  }

  WholeBodyLifterModel & wholebodylifter = model_clones[info->cnode];
  wholebodylifter.updateState (sd, u, p, CSBothFeetFlat2D);

  int j = 0;

  // lumbar coupling contraint
  j = wholebodylifter.appendEqLumbarVelocityError(res,j);

  assert(j == rdfcn_maintainFootContacts_i_ne);

  j = wholebodylifter.appendIeqFootGroundForceError(res,j);
  assert(j == rdfcn_maintainFootContacts_i_n);
}


//==============================================================================

static int rdfcn_attachHands_s_ne =
      wholebodylifter.eqCountHandBoxPos
    + wholebodylifter.eqCountHandBoxVel
    + wholebodylifter.eqCountHandBoxRotY
    + wholebodylifter.eqCountHandBoxRotYVel
    + wholebodylifter.eqCountlumbarVelocity
    + 1;
static int rdfcn_attachHands_s_n =
    rdfcn_attachHands_s_ne
    + wholebodylifter.ieqCountFootGroundForce
    + wholebodylifter.ieqCountBoxLegPos;

void rdfcn_attachHands_s (double * ts,
                          double * sd,
                          double * sa,
                          double * u,
                          double * p,
                          double * pr,
                          double * res,
                          long * dpnd,
                          InfoPtr * info)
{
	if (*dpnd) {
		*dpnd = RFCN_DPND(*ts, *sd, 0, *u, *p, 0);
		return;
	}

	WholeBodyLifterModel & wholebodylifter = model_clones[info->cnode];
  wholebodylifter.updateState (sd, u, p, CSBothFeetFlat2DTransition);

  int j = 0;

  // Hand position to lift end point
  j = wholebodylifter.appendEqHandBoxPosError(res,j);
  j = wholebodylifter.appendEqHandBoxVelError(res,j);
  j = wholebodylifter.appendEqHandBoxRotYError(res,j);
  j = wholebodylifter.appendEqHandBoxRotYVelError(res,j);
  j = wholebodylifter.appendEqLumbarVelocityError(res,j);

  res[j++] = p[ParamTimeHandsOnBoxNoForce]-ts[0];

  assert(j == rdfcn_attachHands_s_ne);

  j = wholebodylifter.appendIeqFootGroundForceError(res,j);
  j = wholebodylifter.appendIeqBoxLegPosError(res,j);

  assert(j == rdfcn_attachHands_s_n);
}


//==============================================================================


static int rdfcn_attachHands_i_ne =
    wholebodylifter.eqCountlumbarVelocity   
    +3;
static int rdfcn_attachHands_i_n =
    rdfcn_attachHands_i_ne
    + wholebodylifter.ieqCountFootGroundForce
    + wholebodylifter.ieqCountBoxLegPos;

void rdfcn_attachHands_i (
    double * ts,
    double * sd,
    double * sa,
    double * u,
    double * p,
    double * pr,
    double * res,
    long * dpnd,
    InfoPtr * info) {
  if (*dpnd) {
    *dpnd = RFCN_DPND(NULL, *sd, 0, *u, *p, 0);
    return;
  }

  WholeBodyLifterModel & wholebodylifter = model_clones[info->cnode];

  wholebodylifter.updateState (sd, u, p, CSBothFeetFlat2DTransition);

  Vector3d RightHandForceA =
      wholebodylifter.getPointForce (PointDistalMetacarpal_R);
  Vector3d RightHandForceB =
      wholebodylifter.getPointForce (PointProximalMetacarpal_R);

  //Here we compute the desired vertical hand/box force as being a
  //ramp that goes from 0 to the weight of the box over the duration
  //of this phase.
  double t0 = p[ParamTimeHandsOnBoxNoForce];
  double t1 = p[ParamTimeHandsOnBoxFullForce];
  unsigned int boxId      = wholebodylifter.model.GetBodyId("Box");
  double boxMass          = wholebodylifter.model.mBodies[boxId].mMass;
  Vector3d gravityVec     = wholebodylifter.model.gravity;
  double handForceVec     = (boxMass*((ts[0]-t0)/(t1-t0)))*gravityVec(2);

  int j = 0;

  j = wholebodylifter.appendEqLumbarVelocityError(res,j);


  //Hand-box force ramp constraint: this ensures that there is a continuous
  //solution for the activations/joint torques to lift up the box.
  //
  //To do later: the x forces should be inequality constraints limited
  //             the coefficient of friction and also the force required
  //             to tip the box over.
  res[j++] = RightHandForceA(0);
  res[j++] = RightHandForceB(0);
  res[j++] = RightHandForceA(2) - handForceVec;

  assert(j == rdfcn_attachHands_i_ne);

  j = wholebodylifter.appendIeqFootGroundForceError(res,j);
  j = wholebodylifter.appendIeqBoxLegPosError(res,j);

  assert(j == rdfcn_attachHands_i_n);
}

//==============================================================================

static int rdfcn_attachHands_e_ne =
    + 1
    + wholebodylifter.eqCountlumbarVelocity
    + 3;

static int rdfcn_attachHands_e_n =
    rdfcn_attachHands_e_ne
    + wholebodylifter.ieqCountFootGroundForce
    + wholebodylifter.ieqCountBoxLegPos;

void rdfcn_attachHands_e (double * ts,
                          double * sd,
                          double * sa,
                          double * u,
                          double * p,
                          double * pr,
                          double * res,
                          long * dpnd,
                          InfoPtr * info)
{
  if (*dpnd) {
    *dpnd = RFCN_DPND(*ts, *sd, 0, *u, *p, 0);
    return;
  }
  WholeBodyLifterModel & wholebodylifter = model_clones[info->cnode];
  wholebodylifter.updateState (sd, u, p, CSBothFeetFlat2DBoxAttach);

  SpatialVector RightHandForce =
      wholebodylifter.getLoopConstraintForce ("Box","Hand_R");

  //Here we compute the desired vertical hand/box force as being a
  //ramp that goes from 0 to the weight of the box over the duration
  //of this phase.
  double t0 = p[ParamTimeHandsOnBoxNoForce];
  double t1 = p[ParamTimeHandsOnBoxFullForce];
  unsigned int boxId      = wholebodylifter.model.GetBodyId("Box");
  double boxMass          = wholebodylifter.model.mBodies[boxId].mMass;
  Vector3d gravityVec     = wholebodylifter.model.gravity;
  double handForceVec     = (boxMass*((ts[0]-t0)/(t1-t0)))*gravityVec(2);

  int j = 0;

  res[j++] = p[ParamTimeHandsOnBoxFullForce]-ts[0];  
  j = wholebodylifter.appendEqLumbarVelocityError(res,j);

  res[j++] = RightHandForce(3);
  res[j++] = RightHandForce(1);
  res[j++] = RightHandForce(5) - handForceVec;

  assert(j == rdfcn_attachHands_e_ne);

  j = wholebodylifter.appendIeqFootGroundForceError(res,j);
  j = wholebodylifter.appendIeqBoxLegPosError(res,j);

  assert(j == rdfcn_attachHands_e_n);
}

//==============================================================================

static int rdfcn_maintainLift_i_ne =
    wholebodylifter.eqCountlumbarVelocity;
static int rdfcn_maintainLift_i_n =
		rdfcn_maintainLift_i_ne
    + wholebodylifter.ieqCountFootGroundForce
    + wholebodylifter.ieqCountBoxLegPos;

void rdfcn_maintainLift_i ( double *ts,
                            double *sd,
                            double *sa,
                            double *u,
                            double *p,
                            double *pr,
                            double *res,
                            long *dpnd,
                            InfoPtr *info)
{
	if (*dpnd) {
		*dpnd = RFCN_DPND(NULL, *sd, 0, *u, *p, 0);
		return;
	}

	WholeBodyLifterModel & wholebodylifter = model_clones[info->cnode];
  wholebodylifter.updateState (sd, u, p, CSBothFeetFlat2DBoxAttach);


  int j = 0;
  j = wholebodylifter.appendEqLumbarVelocityError(res,j);


	assert(j == rdfcn_maintainLift_i_ne);

  j = wholebodylifter.appendIeqFootGroundForceError(res,j);
  j = wholebodylifter.appendIeqBoxLegPosError(res,j);

	assert(j == rdfcn_maintainLift_i_n);
}

//==============================================================================

static int rdfcn_endPose_s_ne =     
    + 3
    + 3
    + 3
    + wholebodylifter.eqCountlumbarVelocity;

static int rdfcn_endPose_s_n  =
    rdfcn_endPose_s_ne
    + wholebodylifter.ieqCountFootGroundForce
    + wholebodylifter.ieqCountBoxLegPos;
void rdfcn_endPose_s (double *ts,
                      double *sd,
                      double *sa,
                      double *u,
                      double *p,
                      double *pr,
                      double *res,
                      long * dpnd,
                      InfoPtr *info)
{
	if (*dpnd) {
		*dpnd = RFCN_DPND(NULL, *sd, 0, *u, *p, 0);
		return;
	}

	WholeBodyLifterModel & wholebodylifter = model_clones[info->cnode];
  wholebodylifter.updateState (sd, u, p, CSBothFeetFlat2DBoxAttach);

  unsigned int idxEndExp = wholebodylifter.qExp.size()-1;

  int j = 0;
    

  //MM: 9/2/2017
  //Careful: do not add constraints on both of the lumbar joints or 
  //         you will end up with an over constrained system since
  //         previous stages have constrained these two joints to 
  //         move together.
  //Note: You can add more velocity constraints, but the problem
  //      seems to run much shower and the final result is not (at this 
  //      preliminary stage) better.

  res[j++] = sd[StateRightKneeRotY] 
            -wholebodylifter.qExp[idxEndExp][PosIDRightKneeRotY];

  res[j++] = sd[StatePelvisRotY] 
            -wholebodylifter.qExp[idxEndExp][PosIDPelvisRotY];

  res[j++] = sd[StateMiddleTrunkRotY]
            -wholebodylifter.qExp[idxEndExp][PosIDMiddleTrunkRotY];

  res[j++] = sd[StatePelvisVelX];
  res[j++] = sd[StatePelvisVelZ];
  res[j++] = sd[StatePelvisRotVelY];

  //Box is at rest vertically.
  res[j++] = sd[StateBoxVelX];  
  res[j++] = sd[StateBoxVelZ];
  res[j++] = sd[StateBoxRotVelY];

  j = wholebodylifter.appendEqLumbarVelocityError(res,j);

  assert(j == rdfcn_endPose_s_ne);

  j = wholebodylifter.appendIeqFootGroundForceError(res,j);
  j = wholebodylifter.appendIeqBoxLegPosError(res,j);
  assert(j == rdfcn_endPose_s_n);

}

//==============================================================================
// \brief Entry point for the muscod application
//==============================================================================

void hires_data_out(double *t, double *sd, double *sa, double *u, double *p,
                    double *rwh, long *iwh, InfoPtr *info);
void write_ocp_output (long *imos, long *imsn, double *ts, double *te,
                      double *sd, double *sa,
                      double *u, double *udot, double *ue, double *uedot,
                      double *p, double *pr, double *ccxd, double *mul_ccxd,
                      #if defined(PRSQP) || defined(EXTPRSQP)
                        double *ares,
                        double *mul_ares,
                      #endif
                      double *rd, double *mul_rd, double *rc, double *mul_rc,
                      double *obj, double *rwh, long *iwh);

extern "C" void def_model(void);
void def_model(void) {

  //datfile_name = string("DAT/") + string("wholebodylifter2d") + string(".dat");

	if (verbose) {
		cout << "Using Datfile: " << datfile_name << endl;
	}

	LoadModelAndConstraints();

	nmos = StageNameLast;
	np   = ParamNameLast;
	nrc  = 0;
	nrce = 0;
	nxd  = StateNameLast;
	nxa  = 0;
	nu   = ControlNameLast;

	// Define OCP dimensions
	def_mdims (nmos, np, nrc, nrce);

  def_mstage(0, nxd, nxa,   nu,  NULL, lfcn_cost,
             0,   0,   0, NULL, ffcn_bend, NULL, NULL, NULL);

  def_mstage(1, nxd, nxa, nu, NULL, lfcn_cost,
             0,   0,   0, NULL, ffcn_transition, NULL, NULL, NULL);

  def_mstage(2, nxd, nxa, nu, NULL, lfcn_cost,
             0,   0,   0, NULL, ffcn_lift, NULL, NULL, NULL);


	def_mpc(0, "s", npr,
      rdfcn_uprightPose_s_n,
      rdfcn_uprightPose_s_ne,
      rdfcn_uprightPose_s,
			NULL);

	def_mpc(0, "i", npr,
			rdfcn_maintainFootContacts_i_n,
			rdfcn_maintainFootContacts_i_ne,
			rdfcn_maintainFootContacts_i,
			NULL);



  def_mpc(1, "s", npr,
      rdfcn_attachHands_s_n,
      rdfcn_attachHands_s_ne,
      rdfcn_attachHands_s,
      NULL);

  def_mpc(1, "i", npr,
      rdfcn_attachHands_i_n,
      rdfcn_attachHands_i_ne,
      rdfcn_attachHands_i,
      NULL);

  def_mpc(2, "s", npr,
      rdfcn_attachHands_e_n,
      rdfcn_attachHands_e_ne,
      rdfcn_attachHands_e,
      NULL);


  def_mpc(2, "i", npr,
			rdfcn_maintainLift_i_n,
			rdfcn_maintainLift_i_ne,
			rdfcn_maintainLift_i,
			NULL);

  def_mpc(2, "e", npr,
			rdfcn_endPose_s_n,
			rdfcn_endPose_s_ne,
			rdfcn_endPose_s,
			NULL);



	def_mio (initialize_from_data, write_ocp_output, hires_data_out);
}

//==============================================================================
// Hi-Res Data Output
//==============================================================================

vector<double> t_values;
vector<VectorNd> sd_values;
vector<VectorNd> u_values;
vector<VectorNd> r_hand_pos_values;
vector<VectorNd> active_torque_values;
vector<VectorNd> passive_torque_values;
vector<VectorNd> tau_values;
vector<VectorNd> feet_force_values;
vector<VectorNd> exo_human_wrench_values;
vector<int> currStage_value;

vector<VectorNd> feet_cop_values;
vector<VectorNd> motor_location_values;
vector<VectorNd> exo_human_contact_location_values;
vector<VectorNd> exo_human_contact_wrench_K0_values;

Addons::Muscle::TorqueMuscleInfo tqMuscleInfo;
bool debugBool;

void hires_data_out(double *t, double *sd, double *sa, double *u, double *p, 
                    double *rwh, long *iwh, InfoPtr *info) {

  WholeBodyLifterModel & wholebodylifter = model_clones[info->cnode];

  if (*t == 0.) {
  
    ofstream meshup_header_stream;
    ofstream meshup_data_stream;   
    ofstream meshup_grfForce_stream; 
    ofstream meshup_exoMotorForce_stream;     
    ofstream meshup_exoStrapForce_stream;         
    ofstream augmented_data_stream;
    ofstream augmented_header_stream;



    string meshup_file_name                 = "RES/wholebodylifter2d.csv";
    string meshup_grfForce_file_name        = "RES/wholebodylifter2d_grf.ff";
    string meshup_exoMotorForce_file_name   = "RES/wholebodylifter2d_exo.ff";
    string meshup_exoStrapForce_file_name   = "RES/wholebodylifter2d_strap.ff";    
    string meshup_header_file_name          = "RES/wholebodylifter2d_columnNames.txt";
    string augmented_data_file_name         = "RES/wholebodylifter2d_augmented.txt";
    string augmented_header_file_name       = "RES/wholebodylifter2d_augmented_columnNames.txt";

    meshup_data_stream.open(meshup_file_name.c_str(), ios_base::trunc);
    meshup_data_stream << std::fixed << std::setprecision(12) << std::setw(12);

    meshup_grfForce_stream.open(    meshup_grfForce_file_name.c_str(),
                                    ios_base::trunc);
    meshup_grfForce_stream << std::fixed << std::setprecision(12) << std::setw(12);

    meshup_exoMotorForce_stream.open(    meshup_exoMotorForce_file_name.c_str(), 
                                    ios_base::trunc);
    meshup_exoMotorForce_stream << std::fixed << std::setprecision(12) << std::setw(12);

    meshup_exoStrapForce_stream.open( meshup_exoStrapForce_file_name.c_str(),
                                    ios_base::trunc);
    meshup_exoStrapForce_stream << std::fixed << std::setprecision(12) << std::setw(12);

    meshup_header_stream.open(meshup_header_file_name.c_str(), ios_base::trunc);
    meshup_header_stream << std::fixed << std::setprecision(12) << std::setw(12);

    augmented_data_stream.open(augmented_data_file_name.c_str(), 
                               ios_base::trunc);
    augmented_data_stream << std::fixed << std::setprecision(12) << std::setw(12);

    augmented_header_stream.open(augmented_header_file_name.c_str(), 
                               ios_base::trunc);
    augmented_header_stream << std::fixed << std::setprecision(12) << std::setw(12);

    meshup_header_stream << "time";
    for(unsigned int i = 0; i < PosNameLast; ++i){
      meshup_header_stream << " " << JointNameMap[i].name_str;
    }
    meshup_header_stream.close();


    augmented_header_stream << "time ";
    for(unsigned int i =0; i < StateNameLast; ++i){
      augmented_header_stream << "State" << StateNameMap[i].name_str <<" ";      
    }
    for(unsigned int i =0; i < ControlNameLast; ++i){
      augmented_header_stream << "Control" << ActuationNameMap[i].name_str <<" ";      
    }
    for(unsigned int i =0; i < PosNameLast; ++i){
      augmented_header_stream << "Tau" << JointNameMap[i].name_str <<" ";      
    }
    for(unsigned int i =MuscleControlFirst; i <= MuscleControlLast; ++i){
      augmented_header_stream << "ActiveMuscle" << ActuationNameMap[i].name_str <<" ";      
    }
    for(unsigned int  i =MuscleControlFirst; i <= MuscleControlLast; ++i){
      augmented_header_stream << "PassiveMuscle" << ActuationNameMap[i].name_str <<" ";      
    }
    augmented_header_stream << "Heel_Fx "       << "Heel_Fy "       << "Heel_Fz "
                            << "Toe_Fx "        << "Toe_Fy "        << "Toe_Fz "
                            << "Hand_x "        << "Hand_y "        << "Hand_z "
                            << "Exo_Pelvis_Fx " << "Exo_Pelvis_Fz " << "Exo_Pelvis_Ty "
                            << "Exo_Thigh_Fx "  << "Exo_Thigh_Fz "  << "Exo_Thigh_Ty "
                            << "Exo_Torso_Fx "  << "Exo_Torso_Fz "  << "Exo_Torso_Ty "
                            << "Phase";
    augmented_header_stream.close();                        


    if (!meshup_data_stream 
        || !augmented_data_stream
        || !meshup_grfForce_stream 
        || !meshup_exoMotorForce_stream 
        || !meshup_exoStrapForce_stream) {
      cerr  << "Error opening file " 
            << meshup_file_name 
            << " or " << meshup_exoMotorForce_file_name 
            << " or " << meshup_grfForce_file_name
            << " or " << meshup_exoStrapForce_file_name            
            << " or " << augmented_data_file_name << endl;
      abort();
    }

    if (t_values.size() > 0) {
      for (unsigned int i = 0; i < t_values.size(); i ++) {
        augmented_data_stream   << t_values[i] << ", ";
        meshup_data_stream      << t_values[i] << ", ";
        meshup_grfForce_stream        << t_values[i] << ", ";
        meshup_exoMotorForce_stream   << t_values[i] << ", ";
        meshup_exoStrapForce_stream   << t_values[i] << ", ";                

        //Write entries to the grf force file
        for(unsigned int j=0; j < 3; ++j){
          meshup_grfForce_stream << feet_cop_values[i][j] << ", ";
        }
        for(unsigned int j=0; j < 3; ++j){
          meshup_grfForce_stream << feet_force_values[i][j]
                                   +feet_force_values[i][j+3]
                                 << ", ";
        }
        meshup_grfForce_stream << 0. << ", "<< 0. << ", "<< 0. << ", ";
        meshup_grfForce_stream << endl;


        //Write entries to the exoMotor force
        for(unsigned int k=0; k < 2; ++k){

          for(unsigned int j=0; j < 3; ++j){
            meshup_exoMotorForce_stream 
              << motor_location_values[i][j+k*3] << ", ";
          }
          for(unsigned int j=0; j < 3; ++j){
            meshup_exoMotorForce_stream 
              << 0. << ", ";
          }
          for(unsigned int j=0; j < 3; ++j){
            if(k==0 && j==1){
              meshup_exoMotorForce_stream 
                << tau_values[i][PosExoThighBarRotY] << ", ";
            }else if(k==1 && j == 1){
              meshup_exoMotorForce_stream 
                << tau_values[i][PosExoTorsoBarRotY] << ", ";
            }else{
              meshup_exoMotorForce_stream 
                << 0. << ", ";
            }
          }
        }
        meshup_exoMotorForce_stream << endl;

        //Write entries to the exoStrap force
        for(unsigned int k=0; k<3;++k){
          //loc
          for(unsigned int j=0; j<3;++j){
            meshup_exoStrapForce_stream << 
              exo_human_contact_location_values[i][j+k*3]  << ", ";
          }
          meshup_exoStrapForce_stream
              << exo_human_contact_wrench_K0_values[i][0+k*3] << ", ";
          meshup_exoStrapForce_stream << 0. << ", ";
          meshup_exoStrapForce_stream
              << exo_human_contact_wrench_K0_values[i][1+k*3] << ", ";
          meshup_exoStrapForce_stream << 0. << ", ";
          meshup_exoStrapForce_stream
              << exo_human_contact_wrench_K0_values[i][2+k*3] << ", ";
          meshup_exoStrapForce_stream << 0. << ", ";
        }
        meshup_exoStrapForce_stream << endl;

        // Write q values to CSV
        for (unsigned int j = 0; j < NoOfPos; j++) {
          meshup_data_stream << sd_values[i][j];
          if (j < sd_values[i].size() -1 ){
            meshup_data_stream << ", ";
          }
        }
        meshup_data_stream << endl;
        
        // Write rest of states (q_vel and activations) to Augmented data file
        for (unsigned int j = 0; j < sd_values[i].size(); j++) {
          augmented_data_stream << sd_values[i][j];
          if (j < sd_values[i].size() -1 ){
            augmented_data_stream << ", ";
          }
        }
        augmented_data_stream << ", ";
        
        // Write controls (excitations) to Augmented data file
        for (unsigned int j = 0; j < NoOfControls; j++) {
          augmented_data_stream << u_values[i][j];
          if (j < u_values[i].size() -1 ){
            augmented_data_stream << ", ";
          }
         }
        augmented_data_stream << ", ";

        //write net joint torque values to file (these include joint damping)
        for (unsigned int j = 0; j < PosNameLast; j++) {
          augmented_data_stream << tau_values[i][j];
          if (j < tau_values[i].size() -1 )
            augmented_data_stream << ", ";
        }
        augmented_data_stream << ", ";

        // Write muscle passive and active torques to Augmented data file
        for (unsigned int j = 0; j < NumberOfMuscleControls; j++) {
          augmented_data_stream << active_torque_values[i][j];
          if (j < active_torque_values[i].size() -1 )
            augmented_data_stream << ", ";
        }
        augmented_data_stream << ", ";

        for (unsigned int j = 0; j < NumberOfMuscleControls; j++) {
          augmented_data_stream << passive_torque_values[i][j];
          if (j < passive_torque_values[i].size() -1 )
            augmented_data_stream << ", ";
        }
        augmented_data_stream << ", ";


        // Write feet force values
        for (unsigned int j = 0; j < 6; j++) {
          augmented_data_stream << feet_force_values[i][j];
          if (j < feet_force_values[i].size() -1 )
            augmented_data_stream << ", ";
        }
        augmented_data_stream << ", ";

        // Write R and L hand positions
        for (unsigned int j = 0; j < 3; j++) {
          augmented_data_stream << r_hand_pos_values[i][j];
          if (j < r_hand_pos_values[i].size() -1 ){
            augmented_data_stream << ", ";
                }
        }
        augmented_data_stream << ", ";

        // Write the exo-human interaction forces/torques to file
        for (unsigned int j = 0; j < 9; j++) {
          augmented_data_stream << exo_human_wrench_values[i][j];
          if (j < exo_human_wrench_values[i].size() -1 ){
            augmented_data_stream << ", ";
                }
        }

        augmented_data_stream << ", " << currStage_value[i] << endl;
      }
    }
  
    t_values.clear();
    sd_values.clear();
    u_values.clear();
        active_torque_values.clear();
        passive_torque_values.clear();
        tau_values.clear();
        feet_force_values.clear();
    r_hand_pos_values.clear();
    exo_human_wrench_values.clear();
    currStage_value.clear();
    
    feet_cop_values.clear();
    motor_location_values.clear();
    exo_human_contact_location_values.clear();

    meshup_data_stream.close();
    augmented_data_stream.close();
    meshup_grfForce_stream.close();
    meshup_exoMotorForce_stream.close();
    meshup_exoStrapForce_stream.close();

  }  
    
    t_values.push_back (*t);
    
    const long currStage = info->cimos;
    currStage_value.push_back(currStage);

    VectorNd sd_vec (NoOfStates);
    for (unsigned i = 0; i < NoOfStates; i++){
      sd_vec[i] = sd[i];
    }
    sd_values.push_back (sd_vec);

    VectorNd u_vec (NoOfControls);
    for (unsigned i = 0; i < NoOfControls; i++){
      u_vec[i] = u[i];
    }
    u_values.push_back (u_vec);

    // Get hand R and L positions
    RigidBodyDynamics::Math::VectorNd q_curr;
    q_curr = VectorNd::Zero (wholebodylifter.model.dof_count);
    for (unsigned int j = 0; j < NoOfPos; j++){
      q_curr[j] = sd_vec[j];
    }

    // Recompute and write Muscle Total Forces and Muscle Passive Forces
    int sdIdx  = StateHumanInternalPosFirst; //joint counter
    VectorNd active_torque_vec (NumberOfMuscleControls);
    VectorNd passive_torque_vec (NumberOfMuscleControls);
    double tqSign = 1.0;    

    unsigned sdCtr = 0;
    for (unsigned i = 0; i < NumberOfMuscleControls; ++i) {

      //If this is a lumbar joint scale q and qdot so that the lumbar
      //develops forces according to the total bend: the MTG for the lumbar
      //is for L5-L1, rather than for an individual joint.
      if ( k == ControlMiddleTrunkExtensionRotY ||
         k == ControlMiddleTrunkFlexionRotY   ||
         k == ControlUpperTrunkExtensionRotY  ||
         k == ControlUpperTrunkFlexionRotY ) {

        wholebodylifter.tqMuscleModel.at(i).calcTorqueMuscleInfo(
                sd[sdIdx]*2,
                sd[sdIdx+PosNameLast]*2,
                u[i],
                tqMuscleInfo);
      }else{
        wholebodylifter.tqMuscleModel.at(i).calcTorqueMuscleInfo(
                sd[sdIdx],
                sd[sdIdx+PosNameLast],
                u[i],
                tqMuscleInfo);
      }
      tqSign = wholebodylifter.tqMuscleModel.at(i).getJointTorqueSign();
      active_torque_vec[i]  = tqSign*tqMuscleInfo.fiberActiveTorque;
      passive_torque_vec[i] = tqSign*tqMuscleInfo.fiberPassiveTorque;      
      
      if(sdCtr==1){
        sdCtr = 0;
        ++sdIdx;
      }else{
        ++sdCtr;
      }

    }

    /*for (unsigned i = 0; i < NumberOfMuscleControls; i+=2) {

        wholebodylifter.tqMuscleModel.at(i).calcTorqueMuscleInfo(
              sd[sdIdx],
              sd[sdIdx+PosNameLast],
              u[i],
              tqMuscleInfo);

        tqSign = wholebodylifter.tqMuscleModel.at(i).getJointTorqueSign();

        active_torque_vec[i]  = tqSign*tqMuscleInfo.fiberActiveTorque;
        passive_torque_vec[i] = tqSign*tqMuscleInfo.fiberPassiveTorque;

        wholebodylifter.tqMuscleModel.at(i+1).calcTorqueMuscleInfo(
              sd[sdIdx],
              sd[sdIdx+PosNameLast],
              u[i+1],
              tqMuscleInfo);

        tqSign = wholebodylifter.tqMuscleModel.at(i+1).getJointTorqueSign();

        active_torque_vec[i+1]  = tqSign*tqMuscleInfo.fiberActiveTorque;
        passive_torque_vec[i+1] = tqSign*tqMuscleInfo.fiberPassiveTorque;

        sdIdx++;
    }*/
    active_torque_values.push_back(active_torque_vec);
    passive_torque_values.push_back(passive_torque_vec);

    VectorNd feet_cop_vec(3);
    VectorNd motor_location_vec(6);
    VectorNd exo_human_contact_location_vec(9);
    VectorNd exo_human_contact_wrench_K0_vec(9);

    VectorNd feet_force_vec (6);
    VectorNd tau_vec;
    tau_vec.resize(PosNameLast);

    Vector3d RightHeelPos(0.,0.,0.);
    Vector3d RightToePos(0.,0.,0.);

    Vector3d MotorPosHip(0.,0.,0.);
    Vector3d MotorPosBack(0.,0.,0.);

    Vector3d ExoStrapPelvis(0.,0.,0.);
    Vector3d ExoStrapBack(0.,0.,0.);
    Vector3d ExoStrapThigh(0.,0.,0.);

    Vector3d RightHeelForce(0.0,0.0,0.0);
    Vector3d RightToeForce(0.0,0.0,0.0);
    SpatialVector exoTrunk,exoPelvis,exoThigh,exoTrunkK0,exoPelvisK0,exoThighK0;
    VectorNd exo_human_wrench_vec(9);
    for(unsigned int z =0; z<6;z++){
      exoTrunk(z)   = 0.;
      exoPelvis(z)  = 0.;
      exoThigh(z)   = 0.;
    }

    RightHeelPos    = wholebodylifter.getPointPosition(PointHeel_R);
    RightToePos     = wholebodylifter.getPointPosition(PointToe_R);

    MotorPosHip     = RigidBodyDynamics::CalcBodyToBaseCoordinates (
                      wholebodylifter.model,
                      q_curr,
                      wholebodylifter.model.GetBodyId("Exo6Pt_Thigh_R_Bar"),
                      Vector3d::Zero(),
                      true); 

    MotorPosBack    = RigidBodyDynamics::CalcBodyToBaseCoordinates (
                      wholebodylifter.model,
                      q_curr,
                      wholebodylifter.model.GetBodyId("Exo6Pt_Torso_Bar"),
                      Vector3d::Zero(),
                      true);


    ExoStrapPelvis  = wholebodylifter.getPointPosition(PointExoPelvis); 
    ExoStrapThigh   = wholebodylifter.getPointPosition(PointExoThigh_R);
    ExoStrapBack    = wholebodylifter.getPointPosition(PointExoTrunk_R);



    switch (currStage) {
    case 0:     // Bend
			wholebodylifter.updateState (sd, u, p, CSBothFeetFlat2D);
      RightHeelForce  = wholebodylifter.getPointForce (PointHeel_R);
      RightToeForce   = wholebodylifter.getPointForce (PointToe_R);
      exoThigh        = wholebodylifter.getLoopConstraintForce(
                        "Thigh_R",    "Exo6Pt_Thigh_R_Module");
      exoPelvis       = wholebodylifter.getLoopConstraintForce(
                        "Pelvis",     "Exo6Pt_PelvisModule");
      exoTrunk       = wholebodylifter.getLoopConstraintForce(
                        "UpperTrunk", "Exo6Pt_UpperTrunkModule");

      exoThighK0      = wholebodylifter.getLoopConstraintForceInBaseCoordinates(
                        "Thigh_R",    "Exo6Pt_Thigh_R_Module");
      exoPelvisK0     = wholebodylifter.getLoopConstraintForceInBaseCoordinates(
                        "Pelvis",     "Exo6Pt_PelvisModule");
      exoTrunkK0     = wholebodylifter.getLoopConstraintForceInBaseCoordinates(
                        "UpperTrunk", "Exo6Pt_UpperTrunkModule");

      break;

    case 1:     // Wait
      wholebodylifter.updateState (sd, u, p, CSBothFeetFlat2DTransition);
      RightHeelForce  = wholebodylifter.getPointForce (PointHeel_R);
      RightToeForce   = wholebodylifter.getPointForce (PointToe_R);
      exoThigh        = wholebodylifter.getLoopConstraintForce(
                        "Thigh_R",    "Exo6Pt_Thigh_R_Module");
      exoPelvis       = wholebodylifter.getLoopConstraintForce(
                        "Pelvis",     "Exo6Pt_PelvisModule");
      exoTrunk       = wholebodylifter.getLoopConstraintForce(
                        "UpperTrunk", "Exo6Pt_UpperTrunkModule");

      exoThighK0      = wholebodylifter.getLoopConstraintForceInBaseCoordinates(
                        "Thigh_R",    "Exo6Pt_Thigh_R_Module");
      exoPelvisK0     = wholebodylifter.getLoopConstraintForceInBaseCoordinates(
                        "Pelvis",     "Exo6Pt_PelvisModule");
      exoTrunkK0     = wholebodylifter.getLoopConstraintForceInBaseCoordinates(
                        "UpperTrunk", "Exo6Pt_UpperTrunkModule");
      break;

    case 2:     // Lift
      wholebodylifter.updateState (sd, u, p, CSBothFeetFlat2DBoxAttach);
      RightHeelForce  = wholebodylifter.getPointForce (PointHeel_R);
      RightToeForce   = wholebodylifter.getPointForce (PointToe_R);
      exoThigh        = wholebodylifter.getLoopConstraintForce(
                        "Thigh_R",    "Exo6Pt_Thigh_R_Module");
      exoPelvis       = wholebodylifter.getLoopConstraintForce(
                        "Pelvis",     "Exo6Pt_PelvisModule");
      exoTrunk       = wholebodylifter.getLoopConstraintForce(
                        "UpperTrunk", "Exo6Pt_UpperTrunkModule");

      exoThighK0      = wholebodylifter.getLoopConstraintForceInBaseCoordinates(
                        "Thigh_R",    "Exo6Pt_Thigh_R_Module");
      exoPelvisK0     = wholebodylifter.getLoopConstraintForceInBaseCoordinates(
                        "Pelvis",     "Exo6Pt_PelvisModule");
      exoTrunkK0     = wholebodylifter.getLoopConstraintForceInBaseCoordinates(
                        "UpperTrunk", "Exo6Pt_UpperTrunkModule");
      break;
    }

    for(unsigned int j=0; j<PosNameLast;++j){
      tau_vec[j] = wholebodylifter.tau[j];
    }
    tau_values.push_back(tau_vec);

    feet_cop_vec = (RightHeelPos*RightHeelForce[2]+RightToePos*RightToeForce[2])
                  /(RightHeelForce[2]+RightToeForce[2]);
    feet_cop_values.push_back(feet_cop_vec);

    feet_force_vec[0]   = RightHeelForce[0]; 
    feet_force_vec[1]   = RightHeelForce[1]; 
    feet_force_vec[2]   = RightHeelForce[2];
    feet_force_vec[3]   = RightToeForce[0]; 
    feet_force_vec[4]   = RightToeForce[1]; 
    feet_force_vec[5]   = RightToeForce[2];
    feet_force_values.push_back(feet_force_vec);

    motor_location_vec[0] = MotorPosHip[0];
    motor_location_vec[1] = MotorPosHip[1];
    motor_location_vec[2] = MotorPosHip[2];
    motor_location_vec[3] = MotorPosBack[0];
    motor_location_vec[4] = MotorPosBack[1];
    motor_location_vec[5] = MotorPosBack[2];
    motor_location_values.push_back(motor_location_vec);

    exo_human_contact_location_vec[0] = ExoStrapPelvis[0];
    exo_human_contact_location_vec[1] = ExoStrapPelvis[1];
    exo_human_contact_location_vec[2] = ExoStrapPelvis[2];
    exo_human_contact_location_vec[3] = ExoStrapThigh[0];
    exo_human_contact_location_vec[4] = ExoStrapThigh[1];
    exo_human_contact_location_vec[5] = ExoStrapThigh[2];
    exo_human_contact_location_vec[6] = ExoStrapBack[0];
    exo_human_contact_location_vec[7] = ExoStrapBack[1];
    exo_human_contact_location_vec[8] = ExoStrapBack[2];
    exo_human_contact_location_values.push_back(exo_human_contact_location_vec);

    RigidBodyDynamics::Math::Vector3d RightHandPos;
    RightHandPos = RigidBodyDynamics::CalcBodyToBaseCoordinates (
			wholebodylifter.model,
			q_curr,
			wholebodylifter.model.GetBodyId("Hand_R"),
			wholebodylifter.pointInfos[PointDistalMetacarpal_R].point_local,
			true);
    
    VectorNd r_hand_pos_vec (3);
    r_hand_pos_vec[0] = RightHandPos[0]; 
    r_hand_pos_vec[1] = RightHandPos[1]; 
    r_hand_pos_vec[2] = RightHandPos[2];
    r_hand_pos_values.push_back(r_hand_pos_vec);

    exo_human_wrench_vec[0] = exoPelvis[3]; //Fx
    exo_human_wrench_vec[1] = exoPelvis[5]; //Fz
    exo_human_wrench_vec[2] = exoPelvis[1]; //Ty
    
    exo_human_wrench_vec[3] = exoThigh[3]; //Fx
    exo_human_wrench_vec[4] = exoThigh[5]; //Fz
    exo_human_wrench_vec[5] = exoThigh[1]; //Ty

    exo_human_wrench_vec[6] = exoTrunk[3]; //Fx
    exo_human_wrench_vec[7] = exoTrunk[5]; //Fz
    exo_human_wrench_vec[8] = exoTrunk[1]; //Ty
    exo_human_wrench_values.push_back(exo_human_wrench_vec);


    exo_human_contact_wrench_K0_vec[0] = -exoPelvisK0[3]; //Fx
    exo_human_contact_wrench_K0_vec[1] = -exoPelvisK0[5]; //Fz
    exo_human_contact_wrench_K0_vec[2] = -exoPelvisK0[1]; //Ty

    exo_human_contact_wrench_K0_vec[3] = -exoThighK0[3]; //Fx
    exo_human_contact_wrench_K0_vec[4] = -exoThighK0[5]; //Fz
    exo_human_contact_wrench_K0_vec[5] = -exoThighK0[1]; //Ty

    exo_human_contact_wrench_K0_vec[6] = -exoTrunkK0[3]; //Fx
    exo_human_contact_wrench_K0_vec[7] = -exoTrunkK0[5]; //Fz
    exo_human_contact_wrench_K0_vec[8] = -exoTrunkK0[1]; //Ty
    exo_human_contact_wrench_K0_values.push_back(exo_human_contact_wrench_K0_vec);
}


void write_ocp_output (long *imos, long *imsn, double *ts, double *te,
                      double *sd, double *sa,
                      double *u, double *udot, double *ue, double *uedot,
                      double *p, double *pr, double *ccxd, double *mul_ccxd,
                      #if defined(PRSQP) || defined(EXTPRSQP)
                        double *ares,
                        double *mul_ares,
                      #endif
                      double *rd, double *mul_rd, double *rc, double *mul_rc,
                      double *obj, double *rwh, long *iwh) {

  InfoPtr info (0, *imos, *imsn);
  hires_data_out( ts, sd, sa, u, p, rwh, iwh, &info);
}
