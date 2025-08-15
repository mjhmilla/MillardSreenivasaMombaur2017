#ifndef _WHOLEBODYLIFTER2D_ENUM_MAP_H
#define _WHOLEBODYLIFTER2D_ENUM_MAP_H

#include "wholebodylifter2d_Enums.h"

struct PointMapEntry {
	PointId name_enum;
	const char* name_str;
};

extern PointMapEntry PointMap[PointNameLast + 1];
PointId getPointIdFromString (const char* point_name_str);

struct ConstraintSetMapEntry {
	CSId name_enum;
	const char* name_str;
};

extern ConstraintSetMapEntry ConstraintSetMap[ConstraintSetNameLast + 1];
CSId getConstraintSetIdFromString (const char* point_name_str);




struct JointMapEntry {
  PosName name_enum;
  const char* name_str;
};
extern JointMapEntry JointNameMap[PosNameLast+1];

struct StateMapEntry {
  StateName name_enum;
  const char* name_str;
};
extern StateMapEntry StateNameMap[StateNameLast+1];

struct ActuationMapEntry {
  ControlName name_enum;
  const char* name_str;
};
extern ActuationMapEntry ActuationNameMap[ControlNameLast+1];


#endif
