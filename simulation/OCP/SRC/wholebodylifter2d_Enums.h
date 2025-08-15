#ifndef _WHOLEBODYLIFTER2D_ENUMS_H
#define _WHOLEBODYLIFTER2D_ENUMS_H
//==============================================================================
//MM 16 Feb 2017
//To add activation dynamics
// 1. Uncomment the line  #define _ACTIVATION_DYNAMICS_
// 2. Run the CreateDATFile with  flag_addActivationDynamics = 1
// 3. Post-processing code needs a small update:
//    a. Update the indexes defined in getAugmentedFileColumnIndices2
//       to include activation dynamics
//    b. Update the plotting functions that access these indices.
//#define _ACTIVATION_DYNAMICS_

//MM 6 March 2017
//Uncomment this if you want the q's coming from the dat file to
//be polished such that the multibody constraints are satisfied.
//Note
// 1. This works well if the exo is weak. If its supporting 30% of the L5/S1
//    torque Muscod will not be able to get through the first few iterations
//    with good initial states alone.
// 2. Unless you want to also solve for the physically consistent control
//    signal to go with the initial states (which I haven't done) you will
//    need to enable Baumgarte stabilization (currently done by changing the
//    the Lua model file in wholebodylifter2d.cc) for the first 10 or so
//    iterations. After KKT hits around 5e-3 its safe to switch over to the
//    Baumgarte disabled model if you wish.
//#define _POLISH_STATE_FROM_DAT_FILE_

//MM 16 Feb 2017
//To change cost functions, just comment out the lines below except for the
//cost function that you would like.
//#define _MIN_TAU_SQ_
#define _MIN_ACT_SQ_


enum StageName {
  StageBend = 0,
  StagePause,
  StageLift,
  StageNameLast
};

enum PosName {
  PosExoPosX             = 0,
  PosExoPosZ,
  PosExoRotY,
  PosExoThighBarRotY,
  PosExoThighModulePosZ,
  PosExoThighModuleRotY,
  PosExoTorsoBarRotY,
  PosExoTorsoModulePosZ,
  PosExoTorsoModuleRotY,
  PosBoxPosX,
  PosBoxPosZ,
  PosBoxRotY,                
  PosPelvisPosX,             
  PosPelvisPosZ,             
  PosPelvisRotY,             
  PosRightHipRotY,           
  PosRightKneeRotY,          
  PosRightAnkleRotY,         
  PosMiddleTrunkRotY,        
  PosUpperTrunkRotY,         
  PosHeadRotY,               
  PosRightUpperArmRotY,      
  PosRightLowerArmRotY,      
  PosRightHandRotY,          
  PosNameLast                
};

unsigned int const PosExoActuated[2]={
  PosExoThighBarRotY,
  PosExoTorsoBarRotY,
};

static unsigned int PosExoFirst   = PosExoPosX;
static unsigned int PosExoLast    = PosExoTorsoModuleRotY;
static unsigned int PosBoxFirst   = PosBoxPosX;
static unsigned int PosBoxLast    = PosBoxRotY;
static unsigned int PosHumanFirst = PosPelvisPosX;
static unsigned int PosHumanLast  = PosRightHandRotY;
static unsigned int PosHumanInternalFirst = PosRightHipRotY;


static unsigned int NumberOfBoxPositions = PosBoxLast
                                          -PosBoxFirst
                                          +1;
static unsigned int NumberOfExoPositions = PosExoLast
                                          -PosExoFirst
                                          +1;
static unsigned int NumberOfHumanPositions = PosHumanLast
                                            -PosHumanFirst
                                            +1;

enum PosIDName {
  PosIDBoxPosX = 0,
  PosIDBoxPosZ,
  PosIDBoxRotY,
  PosIDPelvisPosX,
  PosIDPelvisPosZ,
  PosIDPelvisRotY,
  PosIDRightHipRotY,
  PosIDRightKneeRotY,
  PosIDRightAnkleRotY,
  PosIDLeftHipRotY,
  PosIDLeftKneeRotY,
  PosIDLeftAnkleRotY,
  PosIDMiddleTrunkRotY,
  PosIDUpperTrunkRotY,
  PosIDHeadRotY,
  PosIDRightUpperArmRotY,
  PosIDRightLowerArmRotY,
  PosIDRightHandRotY,
  PosIDLeftUpperArmRotY,
  PosIDLeftLowerArmRotY,
  PosIDLeftHandRotY,
  PosIDNameLast
};


unsigned int const MapIDToModel[15] = { PosIDBoxPosX,
                                        PosIDBoxPosZ,
                                        PosIDBoxRotY,
                                        PosIDPelvisPosX,
                                        PosIDPelvisPosZ,
                                        PosIDPelvisRotY,
                                        PosIDRightHipRotY,
                                        PosIDRightKneeRotY,
                                        PosIDRightAnkleRotY,
                                        PosIDMiddleTrunkRotY,
                                        PosIDUpperTrunkRotY,
                                        PosIDHeadRotY,
                                        PosIDRightUpperArmRotY,
                                        PosIDRightLowerArmRotY,
                                        PosIDRightHandRotY};

static unsigned int PosIDMapBoxFirst = 0;
static unsigned int PosIDMapBoxLast  = 2;
static unsigned int PosIDMapHumanFirst = 3;
static unsigned int PosIDMapHumanLast  = 14;



#ifdef _ACTIVATION_DYNAMICS_

  enum StateName {
    StateExoPosX             = 0,
    StateExoPosZ,
    StateExoRotY,
    StateExoThighBarRotY,
    StateExoThighModulePosZ,
    StateExoThighModuleRotY,
    StateExoTorsoBarRotY,
    StateExoTorsoModulePosZ,
    StateExoTorsoModuleRotY,
    StateBoxPosX,
    StateBoxPosZ,
    StateBoxRotY,
    StatePelvisPosX,
    StatePelvisPosZ,
    StatePelvisRotY,
    StateRightHipRotY,
    StateRightKneeRotY,
    StateRightAnkleRotY,
    StateMiddleTrunkRotY,
    StateUpperTrunkRotY,
    StateHeadRotY,
    StateRightUpperArmRotY,
    StateRightLowerArmRotY,
    StateRightHandRotY,
    StateExoVelX,
    StateExoVelZ,
    StateExoRotVelY,
    StateExoThighBarRotVelY,
    StateExoThighModuleVelZ,
    StateExoThighModuleRotVelY,
    StateExoTorsoBarRotVelY,
    StateExoTorsoModuleVelZ,
    StateExoTorsoModuleRotVelY,
    StateBoxVelX,
    StateBoxVelZ,
    StateBoxRotVelY,
    StatePelvisVelX,
    StatePelvisVelZ,
    StatePelvisRotVelY,
    StateRightHipRotVelY,
    StateRightKneeRotVelY,
    StateRightAnkleRotVelY,
    StateMiddleTrunkRotVelY,
    StateUpperTrunkRotVelY,
    StateHeadRotVelY,
    StateRightUpperArmRotVelY,
    StateRightLowerArmRotVelY,
    StateRightHandRotVelY,
    StateActRightHipExtensionRotY,
    StateActRightHipFlexionRotY,
    StateActRightKneeExtensionRotY,
    StateActRightKneeFlexionRotY,
    StateActRightAnkleExtensionRotY,
    StateActRightAnkleFlexionRotY,
    StateActMiddleTrunkExtensionRotY,
    StateActMiddleTrunkFlexionRotY,
    StateActUpperTrunkExtensionRotY,
    StateActUpperTrunkFlexionRotY,
    StateActHeadExtensionRotY,
    StateActHeadFlexionRotY,
    StateActRightUpperArmExtensionRotY,
    StateActRightUpperArmFlexionRotY,
    StateActRightLowerArmExtensionRotY,
    StateActRightLowerArmFlexionRotY,
    StateActRightHandExtensionRotY,
    StateActRightHandFlexionRotY,
    StateNameLast
  };

  // StateExoThighBarTauRotY,
  // StateExoTorsoBarTauRotY,
#else

  enum StateName {
    StateExoPosX             = 0,
    StateExoPosZ,
    StateExoRotY,
    StateExoThighBarRotY,
    StateExoThighModulePosZ,
    StateExoThighModuleRotY,
    StateExoTorsoBarRotY,
    StateExoTorsoModulePosZ,
    StateExoTorsoModuleRotY,
    StateBoxPosX,
    StateBoxPosZ,
    StateBoxRotY,
    StatePelvisPosX,
    StatePelvisPosZ,
    StatePelvisRotY,
    StateRightHipRotY,
    StateRightKneeRotY,
    StateRightAnkleRotY,
    StateMiddleTrunkRotY,
    StateUpperTrunkRotY,
    StateHeadRotY,
    StateRightUpperArmRotY,
    StateRightLowerArmRotY,
    StateRightHandRotY,
    StateExoVelX,
    StateExoVelZ,
    StateExoRotVelY,
    StateExoThighBarRotVelY,
    StateExoThighModuleVelZ,
    StateExoThighModuleRotVelY,
    StateExoTorsoBarRotVelY,
    StateExoTorsoModuleVelZ,
    StateExoTorsoModuleRotVelY,
    StateBoxVelX,
    StateBoxVelZ,
    StateBoxRotVelY,
    StatePelvisVelX,
    StatePelvisVelZ,
    StatePelvisRotVelY,
    StateRightHipRotVelY,
    StateRightKneeRotVelY,
    StateRightAnkleRotVelY,
    StateMiddleTrunkRotVelY,
    StateUpperTrunkRotVelY,
    StateHeadRotVelY,
    StateRightUpperArmRotVelY,
    StateRightLowerArmRotVelY,
    StateRightHandRotVelY,
    StateNameLast
  };
#endif

static unsigned int StateExoPosFirst   = StateExoPosX;
static unsigned int StateExoPosLast    = StateExoTorsoModuleRotY;
static unsigned int StateBoxPosFirst   = StateBoxPosX;
static unsigned int StateBoxPosLast    = StateBoxRotY;
static unsigned int StateHumanPosFirst = StatePelvisPosX;
static unsigned int StateHumanPosLast  = StateRightHandRotY;
static unsigned int StateHumanInternalPosFirst = StateRightHipRotY;

static unsigned int StateExoVelFirst   = StateExoVelX;
static unsigned int StateExoVelLast    = StateExoTorsoModuleRotVelY;
static unsigned int StateBoxVelFirst   = StateBoxVelX;
static unsigned int StateBoxVelLast    = StateBoxRotVelY;
static unsigned int StateHumanVelFirst = StatePelvisVelX;
static unsigned int StateHumanVelLast  = StateRightHandRotVelY;

#ifdef _ACTIVATION_DYNAMICS_
static unsigned int StateMuscleActFirst = StateActRightHipExtensionRotY;
static unsigned int StateMuscleActLast  = StateActRightHandFlexionRotY;
//static unsigned int StateExoTauFirst    = StateExoThighBarTauRotY;
//static unsigned int StateExoTauLast     = StateExoTorsoBarTauRotY;
#endif

static unsigned int StatePosFirst = StateExoPosX;
static unsigned int StatePosLast  = StateRightHandRotY;
static unsigned int StateVelFirst = StateExoVelX;
static unsigned int StateVelLast  = StateRightHandRotVelY;

//MM: Careful: the function setupTorqueMsucles in
//             wholebodylifter2d_Model code assumes
//             that the muscle controls come first.
enum ControlName {
  ControlRightHipExtensionRotY = 0,
	ControlRightHipFlexionRotY,
	ControlRightKneeExtensionRotY,
	ControlRightKneeFlexionRotY,
	ControlRightAnkleExtensionRotY,
	ControlRightAnkleFlexionRotY,
	ControlMiddleTrunkExtensionRotY,
	ControlMiddleTrunkFlexionRotY,
	ControlUpperTrunkExtensionRotY,
	ControlUpperTrunkFlexionRotY,
	ControlHeadExtensionRotY,
	ControlHeadFlexionRotY,
	ControlRightUpperArmExtensionRotY,
	ControlRightUpperArmFlexionRotY,
	ControlRightLowerArmExtensionRotY,
	ControlRightLowerArmFlexionRotY,
	ControlRightHandExtensionRotY,
	ControlRightHandFlexionRotY,
  ControlExoRightHipRotY,
  ControlExoTorsoBarRotY,
	ControlNameLast
};





static unsigned int MuscleControlFirst = ControlRightHipExtensionRotY;
static unsigned int MuscleControlLast  = ControlRightHandFlexionRotY;



static unsigned int NumberOfMuscleControls = ControlRightHandFlexionRotY
                                            -ControlRightHipExtensionRotY
                                            + 1;

static int NumberOfExoControls    = ControlExoTorsoBarRotY
                                   -ControlExoRightHipRotY
                                   +1;
static unsigned int ExoControlFirst = ControlExoRightHipRotY;
static unsigned int ExoControlLast  = ControlExoTorsoBarRotY;

enum ParamName { 
  ParamTimeHandsOnBoxNoForce = 0,
  ParamTimeHandsOnBoxFullForce,
	ParamNameLast
};

enum CSId {
	CSBothFeetFlat2D = 0,
  CSBothFeetFlat2DTransition,
	CSBothFeetFlat2DBoxAttach,
	CSHandRightFlat,
	ConstraintSetNameLast
};

enum PointId {
  PointExoPelvis = 0,
  PointExoThigh_R,
  PointExoTrunk_R,
  PointPelvis,
  PointThigh_R,
  PointTrunk_R,
  PointHeel_R,
  PointToe_R,
  PointProximalMetacarpal_R,
  PointDistalMetacarpal_R,
  PointBoxAttach_R,
  PointBoxCoM,
  PointBoxBottom,
  PointBoxTop,
  PointBoxBottomLeft,
  PointBoxBottomRight,
  PointNameLast
};

#endif
