/*
*/
/** \file
 *  Definitions for features.
 */

#pragma once

#include "MayaScape/basedef.h"
#include "statsdef.h"

enum FEATURE_TYPE
{
	FEAT_TANK = 2, // hack to keep enums the same value
	FEAT_GEN_ARTE,
	FEAT_OIL_RESOURCE,
	FEAT_BOULDER,
	FEAT_VEHICLE,
	FEAT_BUILDING,
	FEAT_UNUSED,
	FEAT_LOS_OBJ,
	FEAT_OIL_DRUM,
	FEAT_TREE,
	FEAT_SKYSCRAPER,
	FEAT_COUNT
};

/* Stats for a feature */
struct FEATURE_STATS : public BASE_STATS
{
	FEATURE_STATS(int idx = 0) : BASE_STATS(idx) {}

	FEATURE_TYPE    subType = FEAT_COUNT;   ///< type of feature

	iIMDShape      *psImd = nullptr;        ///< Graphic for the feature
	UWORD           baseWidth = 0;          ///< The width of the base in tiles
	UWORD           baseBreadth = 0;        ///< The breadth of the base in tiles

	bool            tileDraw = false;       ///< Whether the tile needs to be drawn
	bool            allowLOS = false;       ///< Whether the feature allows the LOS. true = can see through the feature
	bool            visibleAtStart = false; ///< Whether the feature is visible at the start of the mission
	bool            damageable = false;     ///< Whether the feature can be destroyed
	UDWORD		body = 0;               ///< Number of body points
	UDWORD          armourValue = 0;        ///< Feature armour

	inline Vector2i size() const { return Vector2i(baseWidth, baseBreadth); }
};

struct FEATURE : public BASE_OBJECT
{
	FEATURE(uint32_t id, FEATURE_STATS const *psStats);
	~FEATURE();

	FEATURE_STATS const *psStats;

	inline Vector2i size() const { return psStats->size(); }
};
