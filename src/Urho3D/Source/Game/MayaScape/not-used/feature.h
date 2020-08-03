/*
*/
/** @file
 *  Definitions for the feature structures.
 */

#pragma once

#include "MayaScape/objectdef.h"

/* The statistics for the features */
extern FEATURE_STATS	*asFeatureStats;
extern UDWORD			numFeatureStats;

//Value is stored for easy access to this feature in destroyDroid()/destroyStruct()
extern FEATURE_STATS *oilResFeature;

/* Load the feature stats */
bool loadFeatureStats(WzConfig &ini);

/* Release the feature stats memory */
void featureStatsShutDown();

/* Create a feature on the map */
FEATURE *buildFeature(FEATURE_STATS *psStats, UDWORD x, UDWORD y, bool FromSave);

/* Update routine for features */
void featureUpdate(FEATURE *psFeat);

// free up a feature with no visual effects
bool removeFeature(FEATURE *psDel);

/* Remove a Feature and free it's memory */
bool destroyFeature(FEATURE *psDel, unsigned impactTime);

/* get a feature stat id from its name */
SDWORD getFeatureStatFromName(const WzString &name);

int32_t featureDamage(FEATURE *psFeature, unsigned damage, WEAPON_CLASS weaponClass, WEAPON_SUBCLASS weaponSubClass, unsigned impactTime, bool isDamagePerSecond, int minDamage);

void featureInitVars();

StructureBounds getStructureBounds(FEATURE const *object);
StructureBounds getStructureBounds(FEATURE_STATS const *stats, Vector2i pos);

#define syncDebugFeature(psFeature, ch) _syncDebugFeature(__FUNCTION__, psFeature, ch)
void _syncDebugFeature(const char *function, FEATURE const *psFeature, char ch);


// True iff object is a feature.
static inline bool isFeature(SIMPLE_OBJECT const *psObject)
{
	return psObject != nullptr && psObject->type == OBJ_FEATURE;
}
// Returns FEATURE * if feature or NULL if not.
static inline FEATURE *castFeature(SIMPLE_OBJECT *psObject)
{
	return isFeature(psObject) ? (FEATURE *)psObject : (FEATURE *)nullptr;
}
// Returns FEATURE const * if feature or NULL if not.
static inline FEATURE const *castFeature(SIMPLE_OBJECT const *psObject)
{
	return isFeature(psObject) ? (FEATURE const *)psObject : (FEATURE const *)nullptr;
}

