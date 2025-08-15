#ifndef _WHOLEBODYLIFTER2D_MODEL_H
#define _WHOLEBODYLIFTER2D_MODEL_H

#include <limits>
#include <vector>

#include <rbdl/rbdl.h>
#include <rbdl/addons/muscle/muscle.h>

#include "wholebodylifter2d_Enums.h"
#include "wholebodylifter2d_EnumMap.h"


struct Point {
  Point() :
    name ("unknown"),
    body_id (-1),
    body_name (""),
    point_local (
      std::numeric_limits<double>::signaling_NaN(),
      std::numeric_limits<double>::signaling_NaN(),
      std::numeric_limits<double>::signaling_NaN()
      )
  { }

  std::string name;
  unsigned int body_id;
  std::string body_name;
  RigidBodyDynamics::Math::Vector3d point_local;
};

//==============================================================================

struct MetaData {
  MetaData() :
    scaling_used("nan"),
    subject_age(std::numeric_limits<double>::signaling_NaN()),
    subject_height(std::numeric_limits<double>::signaling_NaN()),
    subject_weight(std::numeric_limits<double>::signaling_NaN()),
    subject_gender(std::numeric_limits<double>::signaling_NaN()),
    subject_pelvisWidth(std::numeric_limits<double>::signaling_NaN()),
    subject_hipCenterWidth(std::numeric_limits<double>::signaling_NaN())
  { }

  std::string   scaling_used;
  double        subject_age;
  double        subject_height;
  double        subject_weight;
  double        subject_gender;
  double        subject_pelvisWidth;
  double        subject_hipCenterWidth;
};

/** Data of a single constraint */
struct ConstraintInfo {
	ConstraintInfo() :
		point_id (PointNameLast),
		type (RigidBodyDynamics::ConstraintSet::ConstraintTypeLast),
		point_name (""),
    normal (
      std::numeric_limits<double>::signaling_NaN(),
      std::numeric_limits<double>::signaling_NaN(),
      std::numeric_limits<double>::signaling_NaN()
			),
		name(""),
		predecessor_body(""),
		successor_body(""),
		predecessor_transform(RigidBodyDynamics::Math::SpatialTransform()),
		successor_transform(RigidBodyDynamics::Math::SpatialTransform()),
		axis(RigidBodyDynamics::Math::SpatialVector::Zero()),
		baumgarte_enabled(false),
		stabilization_coefficient(0.0) {
	}
	unsigned int point_id;
	RigidBodyDynamics::ConstraintSet::ConstraintType type;

	// case type == ContactConstraint
	std::string point_name;
	RigidBodyDynamics::Math::Vector3d normal;

	// case type == LoopConstraint
	std::string name;
	std::string predecessor_body;
	std::string successor_body;
	RigidBodyDynamics::Math::SpatialTransform predecessor_transform;
	RigidBodyDynamics::Math::SpatialTransform successor_transform;
	RigidBodyDynamics::Math::SpatialVector axis;
	bool   baumgarte_enabled;
	double stabilization_coefficient;
};

/** Structure that holds data of a complete constraint set */
struct ConstraintSetInfo {
  ConstraintSetInfo() :
    name ("undefined") {
  }
  std::vector<ConstraintInfo> constraints;
  std::string name;
};

struct WholeBodyLifterModel {
  WholeBodyLifterModel();

  unsigned int activeConstraintSet;
  unsigned int nDof;
  unsigned int nActuatedDof;

  bool dynamicsComputed;
  bool kinematicsUpdated;

  RigidBodyDynamics::Math::VectorNd q;              //multibody gen. coord.
  RigidBodyDynamics::Math::VectorNd qdot;           //multibody gen. vel
  RigidBodyDynamics::Math::VectorNd qddot;          //multibody gen. acc
  RigidBodyDynamics::Math::VectorNd tau;            //net joint torques
  RigidBodyDynamics::Math::VectorNd p;      //parameters

  std::vector< std::vector < double > > timeExp;
  std::vector< std::vector < double > > qExp;
  std::vector< std::vector < double > > qDotExp;
  std::vector< std::vector < double > > tauExp;
  std::vector< std::vector < double > > fpExp;
  unsigned int idxBoxPickupExp;

  double handOffset;
  double hipAngleOffset;

  double coefficientOfFriction;
  double boxHalfWidth;
  RigidBodyDynamics::Math::SpatialVector maxHandBoxForces;
  RigidBodyDynamics::Math::SpatialVector minHandBoxForces;
  //The first index holds the extension torque of the joint while the
  //second index hods the flexion torque of each joint. The sum of
  //the flexion and extension torques yields the net torque.
  std::vector < double > tauMuscles;
  std::vector < double > tauMaxMuscles;
  RigidBodyDynamics::Math::VectorNd jointDampingCoefficient;
  //RigidBodyDynamics::Math::VectorNd highestJointPassiveStiffness;
  //RigidBodyDynamics::Math::VectorNd smallestInertiaOnJoint;
  //RigidBodyDynamics::Math::VectorNd jointTimeConstant;
  RigidBodyDynamics::Addons::Muscle::TorqueMuscleInfo tmi;

  /**Activation state of the muscles :
      index 0: accesses the joint. Orded to be consistent with the controls
      index 1: 0: extensor activation
               1: flexor activation
 */
  std::vector< double > act;
  std::vector< double > actDot;
  double activationTimeConstant;

  std::vector< double > exoTauNorm;
  std::vector< double > exoTauDotNorm;
  std::vector< double > exoControl;

  std::vector< double > exoActuatorTauMax;
  std::vector< double > exoSpringStiffness;
  std::vector< double > exoDampingCoefficients;


  /**
    Vector of muscle vectors. This vector of vectors is arranged to make
    accessing an individual muscle as easy as accessing a control:

    index 1: Used to access a joint. Here the ordering is identical to
             the ordering used for the controls - you can use the ControlName
             to access the joint that you're interested in.
    index 2: 0 - accesses the extensor muscle. Extension is defined using
                 the anatomical definitions. Be sure to check these: some of
                 them (esp. shoulders) are counter intuitive.
             1 - accesses the flexor muscle.
  */
  std::vector < RigidBodyDynamics::Addons::Muscle::Millard2016TorqueMuscle > 
              tqMuscleModel;

  RigidBodyDynamics::Model model;

  MetaData subjectMetaData;

  Point pointInfos[PointNameLast];
  /// Information of the constraint sets (mostly used when parsing Lua file)
	ConstraintSetInfo cs_infos[ConstraintSetNameLast];

  /** RDBL constraint sets that are used during forward dynamics and collision
  computations */
	std::vector<RigidBodyDynamics::ConstraintSet> constraint_sets;

  CSId getConstraintSetId(unsigned int phase);

  void projectStateOntoMultibodyConstraintManifold(
      double *sd,
      const CSId cs_id);

  /** Copies state information from MUSCOD to the model and switches to the
      given constraint set.
  */
	void updateState (
			const double *sd,
			const double *u,
			const double *p,
			const CSId cs_id);

  /** Updates the kinematics of the RDBL model (is called automatically when
  required) */
  void updateKinematics ();

  /** Computes the forward dynamics for the model and active constraint set */
  void calcForwardDynamicsRhs (double *res);

  void zeroTau();

  void addMuscleTorqueToTau();
  void addJointDampingToTau();
  void addExoTorqueToTau();

	double getSumActSqr ();

	double getSumNormalizedMuscleTorquesSqr ();

  RigidBodyDynamics::Math::Matrix3d getBodyWorldOrientation(
      const PointId &point_name);
  RigidBodyDynamics::Math::Vector3d getPointPosition (
			const PointId &point_name);
  RigidBodyDynamics::Math::Vector3d getPointVelocity (
			const PointId &point_name);
  RigidBodyDynamics::Math::SpatialVector getPointVelocity6D (
      const PointId &point_name);
  RigidBodyDynamics::Math::Vector3d getPointForce (
			const PointId &point_name);

  RigidBodyDynamics::Math::SpatialVector getLoopConstraintForce (
      const std::string& body_predecessor,
      const std::string& body_successor);

  RigidBodyDynamics::Math::SpatialVector getLoopConstraintForceInBaseCoordinates (
      const std::string& body_predecessor,
      const std::string& body_successor);

  bool loadInverseDynamicsDataFromFile(
      unsigned int stoopIdx,
      bool updateBoxPickupPointLocations,
      bool offsetBoxPickupPointLocations,
      bool verbose);

  bool loadFromFile (std::string modelfilename,
      std::string datfilename, bool verbose=false);




  void setupTorqueMuscles (const char* filename,
                           bool verbose=false);
  void setupJointDamping(bool verbose=false);

  //Muscle strength fitting functions
  /*MM: 28 Feb 2017
   *These functions need the following upgrades to work
   * with the exo:
   *
   * 1. Q and Qdot of the exo need to be polished to
   *    enforce the position constraints between the exo
   *    and the human.
   * 2. A decision needs to be made about what q and qdot
   *    are given as arguments to these functions. Normally
   *    this would just be the box and the human. If this is
   *    the case the Exo would need to be added internally
   *    before any RBDL functions are called.
   * 3. Either the user needs to provide the Exo parameters
   *    and controls ... or they need to be set in the function
   *    (perhaps to zero).
   *
   * And this may or may not be useful: the muscles fo the
   * human model should be fitted so that the human is strong
   * enough to perform the motion without the Exo.
   *
  void calcMuscleActivations(
      const RigidBodyDynamics::Math::VectorNd &qIn,
      const RigidBodyDynamics::Math::VectorNd &qDotIn,
      const RigidBodyDynamics::Math::VectorNd &tauIn,
      bool addJointDamping,
      RigidBodyDynamics::Math::VectorNd &aOut,
      RigidBodyDynamics::Math::VectorNd &taOut,
      RigidBodyDynamics::Math::VectorNd &tvOut,
      RigidBodyDynamics::Math::VectorNd &tpOut) const ;

  void calcMaximumActiveIsometricTorqueScaleFactor(
      const RigidBodyDynamics::Math::VectorNd &qIn,
      const RigidBodyDynamics::Math::VectorNd &qDotIn,
      const RigidBodyDynamics::Math::VectorNd &tauIn,
      const RigidBodyDynamics::Math::VectorNd &activationIn,
      bool addJointDamping,
      RigidBodyDynamics::Math::VectorNd &sfOut) const ;
  */
  int appendIeqFootGroundForceError(double *res, int j);
  const static int ieqCountFootGroundForce = 4;

  int appendEqLumbarVelocityError(double *res, int j);
  const static int eqCountlumbarVelocity = 1;

  int appendEqHandBoxPosError(double *res, int j);
  const static int eqCountHandBoxPos = 2;

  int appendEqHandBoxVelError(double *res, int j);
  const static int eqCountHandBoxVel = 2;

  int appendEqHandBoxRotYError(double *res, int j);
  const static int eqCountHandBoxRotY = 1;

  int appendEqHandBoxRotYVelError(double *res, int j);
  const static int eqCountHandBoxRotYVel = 1;

  int appendIeqBoxLegPosError(double *res, int j);
  const static int ieqCountBoxLegPos = 2;

  int appendEqBoxParallelToLegPosError(double *res, int j);
  const static int eqBoxParallelToLegPos = 1;

  int appendIeqHandBoxForceError( double *res, int j);
  const static int ieqCountHandBoxForce = 1;

  bool loadPoints (std::string filename,
                   bool verbose=false);

	bool loadConstraintSets (
			std::string filename,
			bool verbose = false);

	bool loadMetaData (
			std::string filename,
			bool verbose = false);

	RigidBodyDynamics::Math::VectorNd ikState0;
};

#endif
