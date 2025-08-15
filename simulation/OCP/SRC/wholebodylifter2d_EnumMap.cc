#include "wholebodylifter2d_EnumMap.h"

#include <iostream>
#include <string>
#include <cstdlib>

using namespace std;

PointMapEntry PointMap[PointNameLast + 1] = {
  { PointExoPelvis,            "Exo6Pt_VuaPelvis_R"    },
  { PointExoThigh_R,           "Exo6Pt_Thigh_R"        },
  { PointExoTrunk_R,           "Exo6Pt_VuaUpperTrunk_R"},
  { PointPelvis,               "VuaPelvis_R"           },
  { PointThigh_R,              "VuaThigh_R"            },
  { PointTrunk_R,              "VuaUpperTrunk_R"       },
  { PointHeel_R,               "Heel_R"                },
  { PointToe_R,                "Toe_R"                 },
  { PointProximalMetacarpal_R, "ProximalMetacarpal_R"  },
  { PointDistalMetacarpal_R,   "DistalMetacarpal_R"    },
  { PointBoxAttach_R,          "BoxAttach_R"           },
  { PointBoxCoM,               "BoxCoM"                },
  { PointBoxBottom,            "BoxBottom"             },
  { PointBoxTop,               "BoxTop"                },
  { PointBoxBottomLeft,        "BoxBottomLeft"         },
  { PointBoxBottomRight,       "BoxBottomRight"        },
  { PointNameLast,             "PointNameLast"         }
};

PointId getPointIdFromString (const char* point_name_str) {
	unsigned int map_index = 0;

	while (map_index != PointNameLast) {
		if (string(PointMap[map_index].name_str) == string(point_name_str)) {
			break;
		}
		map_index++;
	}

	if (map_index == PointNameLast) {
		cerr << "Warning: could not find mapping for point '" << point_name_str << "'." << endl;
		return PointNameLast;
	}

	return static_cast<PointId>(map_index);
}

ConstraintSetMapEntry ConstraintSetMap[ConstraintSetNameLast + 1] = {
  { CSBothFeetFlat2D,           "BothFeetFlat2D"            },
  { CSBothFeetFlat2DTransition, "BothFeetFlat2DTransition"  },
  { CSBothFeetFlat2DBoxAttach,  "BothFeetFlat2DBoxAttach"   },
  { CSHandRightFlat,            "HandRightFlat"             },
  { ConstraintSetNameLast,      "ConstraintSetNameLast"     }
};

CSId getConstraintSetIdFromString (const char* constraint_set_name_str) {
	unsigned int map_index = 0;

	while (map_index != ConstraintSetNameLast) {
		if (string(ConstraintSetMap[map_index].name_str) == string(constraint_set_name_str))
			break;

		map_index++;
	}

	if (map_index == ConstraintSetNameLast) {
		cerr << "Warning: could not find mapping for constraint set '" << constraint_set_name_str << "'." << endl;
		return ConstraintSetNameLast;
	}

	return static_cast<CSId>(map_index);
}


JointMapEntry JointNameMap[PosNameLast+1] = {
  {PosExoPosX,             "PosExoPosX"           },
  {PosExoPosZ,             "PosExoPosZ"           },
  {PosExoRotY,             "PosExoRotY"           },
  {PosExoThighBarRotY,     "PosExoThighBarRotY"   },
  {PosExoThighModulePosZ,  "PosExoThighModulePosZ"},
  {PosExoThighModuleRotY,  "PosExoThighModuleRotY"},
  {PosExoTorsoBarRotY,     "PosExoTorsoBarRotY"   },
  {PosExoTorsoModulePosZ,  "PosExoTorsoModulePosZ"},
  {PosExoTorsoModuleRotY,  "PosExoTorsoModuleRotY"},
  {PosBoxPosX,             "BoxPosX"              },
  {PosBoxPosZ,             "BoxPosZ"              },
  {PosBoxRotY,             "BoxRotY"              },
  {PosPelvisPosX,          "PelvisPosX"           },
  {PosPelvisPosZ,          "PelvisPosZ"           },
  {PosPelvisRotY,          "PelvisRotY"           },
  {PosRightHipRotY,        "RightHipRotY"         },
  {PosRightKneeRotY,       "RightKneeRotY"        },
  {PosRightAnkleRotY,      "RightAnkleRotY"       },
  {PosMiddleTrunkRotY,     "MiddleTrunkRotY"      },
  {PosUpperTrunkRotY,      "UpperTrunkRotY"       },
  {PosHeadRotY,            "HeadRotY"             },
  {PosRightUpperArmRotY,   "RightUpperArmRotY"    },
  {PosRightLowerArmRotY,   "RightLowerArmRotY"    },
  {PosRightHandRotY,       "RightHandRotY"        },
  {PosNameLast,            "NameLast"             },
};


#ifdef _ACTIVATION_DYNAMICS_

StateMapEntry StateNameMap[StateNameLast+1] = {
    {StateExoPosX                       , "StateExoPosX"                  },
    {StateExoPosZ                       , "StateExoPosZ"                  },
    {StateExoRotY                       , "StateExoRotY"                  },
    {StateExoThighBarRotY               , "StateExoThighBarRotY"          },
    {StateExoThighModulePosZ            , "StateExoThighModulePosZ"       },
    {StateExoThighModuleRotY            , "StateExoThighModuleRotY"       },
    {StateExoTorsoBarRotY               , "StateExoTorsoBarRotY"          },
    {StateExoTorsoModulePosZ            , "StateExoTorsoModulePosZ"       },
    {StateExoTorsoModuleRotY            , "StateExoTorsoModuleRotY"       },
    {StateBoxPosX                       , "BoxPosX"                       },
    {StateBoxPosZ                       , "BoxPosZ"                       },
    {StateBoxRotY                       , "BoxRotY"                       },
    {StatePelvisPosX                    , "PelvisPosX"                    },
    {StatePelvisPosZ                    , "PelvisPosZ"                    },
    {StatePelvisRotY                    , "PelvisRotY"                    },
    {StateRightHipRotY                  , "RightHipRotY"                  },
    {StateRightKneeRotY                 , "RightKneeRotY"                 },
    {StateRightAnkleRotY                , "RightAnkleRotY"                },
    {StateMiddleTrunkRotY               , "MiddleTrunkRotY"               },
    {StateUpperTrunkRotY                , "UpperTrunkRotY"                },
    {StateHeadRotY                      , "HeadRotY"                      },
    {StateRightUpperArmRotY             , "RightUpperArmRotY"             },
    {StateRightLowerArmRotY             , "RightLowerArmRotY"             },
    {StateRightHandRotY                 , "RightHandRotY"                 },
    {StateExoVelX                       , "StateExoVelX"                  },
    {StateExoVelZ                       , "StateExoVelZ"                  },
    {StateExoRotVelY                    , "StateExoRotVelY"               },
    {StateExoThighBarRotVelY            , "StateExoThighBarRotVelY"       },
    {StateExoThighModuleVelZ            , "StateExoThighModuleVelZ"       },
    {StateExoThighModuleRotVelY         , "StateExoThighModuleRotVelY"    },
    {StateExoTorsoBarRotVelY            , "StateExoTorsoBarRotVelY"       },
    {StateExoTorsoModuleVelZ            , "StateExoTorsoModuleVelZ"       },
    {StateExoTorsoModuleRotVelY         , "StateExoTorsoModuleRotVelY"    },
    {StateBoxVelX                       , "BoxVelX"                       },
    {StateBoxVelZ                       , "BoxVelZ"                       },
    {StateBoxRotVelY                    , "BoxRotVelY"                    },
    {StatePelvisVelX                    , "PelvisVelX"                    },
    {StatePelvisVelZ                    , "PelvisVelZ"                    },
    {StatePelvisRotVelY                 , "PelvisRotVelY"                 },
    {StateRightHipRotVelY               , "RightHipRotVelY"               },
    {StateRightKneeRotVelY              , "RightKneeRotVelY"              },
    {StateRightAnkleRotVelY             , "RightAnkleRotVelY"             },
    {StateMiddleTrunkRotVelY            , "MiddleTrunkRotVelY"            },
    {StateUpperTrunkRotVelY             , "UpperTrunkRotVelY"             },
    {StateHeadRotVelY                   , "HeadRotVelY"                   },
    {StateRightUpperArmRotVelY          , "RightUpperArmRotVelY"          },
    {StateRightLowerArmRotVelY          , "RightLowerArmRotVelY"          },
    {StateRightHandRotVelY              , "RightHandRotVelY"              },
    {StateActRightHipExtensionRotY      , "ActRightHipExtensionRotY"      },
    {StateActRightHipFlexionRotY        , "ActRightHipFlexionRotY"        },
    {StateActRightKneeExtensionRotY     , "ActRightKneeExtensionRotY"     },
    {StateActRightKneeFlexionRotY       , "ActRightKneeFlexionRotY"       },
    {StateActRightAnkleExtensionRotY    , "ActRightAnkleExtensionRotY"    },
    {StateActRightAnkleFlexionRotY      , "ActRightAnkleFlexionRotY"      },
    {StateActMiddleTrunkExtensionRotY   , "ActMiddleTrunkExtensionRotY"   },
    {StateActMiddleTrunkFlexionRotY     , "ActMiddleTrunkFlexionRotY"     },
    {StateActUpperTrunkExtensionRotY    , "ActUpperTrunkExtensionRotY"    },
    {StateActUpperTrunkFlexionRotY      , "ActUpperTrunkFlexionRotY"      },
    {StateActHeadExtensionRotY          , "ActHeadExtensionRotY"          },
    {StateActHeadFlexionRotY            , "ActHeadFlexionRotY"            },
    {StateActRightUpperArmExtensionRotY , "ActRightUpperArmExtensionRotY" },
    {StateActRightUpperArmFlexionRotY   , "ActRightUpperArmFlexionRotY"   },
    {StateActRightLowerArmExtensionRotY , "ActRightLowerArmExtensionRotY" },
    {StateActRightLowerArmFlexionRotY   , "ActRightLowerArmFlexionRotY"   },
    {StateActRightHandExtensionRotY     , "ActRightHandExtensionRotY"     },
    {StateActRightHandFlexionRotY       , "ActRightHandFlexionRotY"       },
    {StateNameLast                      , "NameLast"                      },
};

#else

StateMapEntry StateNameMap[StateNameLast+1] = {
    {StateExoPosX                       , "StateExoPosX"                  },
    {StateExoPosZ                       , "StateExoPosZ"                  },
    {StateExoRotY                       , "StateExoRotY"                  },
    {StateExoThighBarRotY               , "StateExoThighBarRotY"          },
    {StateExoThighModulePosZ            , "StateExoThighModulePosZ"       },
    {StateExoThighModuleRotY            , "StateExoThighModuleRotY"       },
    {StateExoTorsoBarRotY               , "StateExoTorsoBarRotY"          },
    {StateExoTorsoModulePosZ            , "StateExoTorsoModulePosZ"       },
    {StateExoTorsoModuleRotY            , "StateExoTorsoModuleRotY"       },
    {StateBoxPosX                       , "BoxPosX"                       },
    {StateBoxPosZ                       , "BoxPosZ"                       },
    {StateBoxRotY                       , "BoxRotY"                       },
    {StatePelvisPosX                    , "PelvisPosX"                    },
    {StatePelvisPosZ                    , "PelvisPosZ"                    },
    {StatePelvisRotY                    , "PelvisRotY"                    },
    {StateRightHipRotY                  , "RightHipRotY"                  },
    {StateRightKneeRotY                 , "RightKneeRotY"                 },
    {StateRightAnkleRotY                , "RightAnkleRotY"                },
    {StateMiddleTrunkRotY               , "MiddleTrunkRotY"               },
    {StateUpperTrunkRotY                , "UpperTrunkRotY"                },
    {StateHeadRotY                      , "HeadRotY"                      },
    {StateRightUpperArmRotY             , "RightUpperArmRotY"             },
    {StateRightLowerArmRotY             , "RightLowerArmRotY"             },
    {StateRightHandRotY                 , "RightHandRotY"                 },
    {StateExoVelX                       , "StateExoVelX"                  },
    {StateExoVelZ                       , "StateExoVelZ"                  },
    {StateExoRotVelY                    , "StateExoRotVelY"               },
    {StateExoThighBarRotVelY            , "StateExoThighBarRotVelY"       },
    {StateExoThighModuleVelZ            , "StateExoThighModuleVelZ"       },
    {StateExoThighModuleRotVelY         , "StateExoThighModuleRotVelY"    },
    {StateExoTorsoBarRotVelY            , "StateExoTorsoBarRotVelY"       },
    {StateExoTorsoModuleVelZ            , "StateExoTorsoModuleVelZ"       },
    {StateExoTorsoModuleRotVelY         , "StateExoTorsoModuleRotVelY"    },
    {StateBoxVelX                       , "BoxVelX"                       },
    {StateBoxVelZ                       , "BoxVelZ"                       },
    {StateBoxRotVelY                    , "BoxRotVelY"                    },
    {StatePelvisVelX                    , "PelvisVelX"                    },
    {StatePelvisVelZ                    , "PelvisVelZ"                    },
    {StatePelvisRotVelY                 , "PelvisRotVelY"                 },
    {StateRightHipRotVelY               , "RightHipRotVelY"               },
    {StateRightKneeRotVelY              , "RightKneeRotVelY"              },
    {StateRightAnkleRotVelY             , "RightAnkleRotVelY"             },
    {StateMiddleTrunkRotVelY            , "MiddleTrunkRotVelY"            },
    {StateUpperTrunkRotVelY             , "UpperTrunkRotVelY"             },
    {StateHeadRotVelY                   , "HeadRotVelY"                   },
    {StateRightUpperArmRotVelY          , "RightUpperArmRotVelY"          },
    {StateRightLowerArmRotVelY          , "RightLowerArmRotVelY"          },
    {StateRightHandRotVelY              , "RightHandRotVelY"              },
    {StateNameLast                      , "NameLast"                      },
};

#endif
ActuationMapEntry ActuationNameMap[ControlNameLast+1] = {
  {ControlRightHipExtensionRotY     , "RightHipExtensionRotY"},
  {ControlRightHipFlexionRotY       , "RightHipFlexionRotY"},
  {ControlRightKneeExtensionRotY    , "RightKneeExtensionRotY"},
  {ControlRightKneeFlexionRotY      , "RightKneeFlexionRotY"},
  {ControlRightAnkleExtensionRotY   , "RightAnkleExtensionRotY"},
  {ControlRightAnkleFlexionRotY     , "RightAnkleFlexionRotY"},
  {ControlMiddleTrunkExtensionRotY  , "MiddleTrunkExtensionRotY"},
  {ControlMiddleTrunkFlexionRotY    , "MiddleTrunkFlexionRotY"},
  {ControlUpperTrunkExtensionRotY   , "UpperTrunkExtensionRotY"},
  {ControlUpperTrunkFlexionRotY     , "UpperTrunkFlexionRotY"},
  {ControlHeadExtensionRotY         , "HeadExtensionRotY"},
  {ControlHeadFlexionRotY           , "HeadFlexionRotY"},
  {ControlRightUpperArmExtensionRotY, "RightUpperArmExtensionRotY"},
  {ControlRightUpperArmFlexionRotY  , "RightUpperArmFlexionRotY"},
  {ControlRightLowerArmExtensionRotY, "RightLowerArmExtensionRotY"},
  {ControlRightLowerArmFlexionRotY  , "RightLowerArmFlexionRotY"},
  {ControlRightHandExtensionRotY    , "RightHandExtensionRotY"},
  {ControlRightHandFlexionRotY      , "RightHandFlexionRotY"},
  {ControlExoRightHipRotY           , "ControlExoRightHipRotY"},
  {ControlExoTorsoBarRotY           , "ControlExoTorsoBarRotY"},
  {ControlNameLast                  , "NameLast"},
};
