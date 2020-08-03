/*
*/
/** \file
 *  Definitions for movement tracking.
 */

#pragma once

#include <vector>
#include "shared_libs.h"

using namespace Urho3D;

enum MOVE_STATUS
{
	MOVEINACTIVE,
	MOVENAVIGATE,
	MOVETURN,
	MOVEPAUSE,
	MOVEPOINTTOPOINT,
	MOVETURNTOTARGET,
	MOVEHOVER,
	MOVEWAITROUTE,
	MOVESHUFFLE,
};

struct MOVE_CONTROL
{
	MOVE_STATUS Status = MOVEINACTIVE;    ///< Inactive, Navigating or moving point to point status
	int pathIndex = 0;                    ///< Position in asPath
	std::vector<IntVector2> asPath;         ///< Pointer to list of block X,Y map coordinates.

    IntVector2 destination = IntVector2(0, 0);                 ///< World coordinates of movement destination
	IntVector2 src = IntVector2(0, 0);
	IntVector2 target = IntVector2(0, 0);
	int speed = 0;                        ///< Speed of motion

	uint16_t moveDir = 0;                 ///< Direction of motion (not the direction the droid is facing)
	uint16_t bumpDir = 0;                 ///< Direction at last bump
	unsigned bumpTime = 0;                ///< Time of first bump with something
	uint16_t lastBump = 0;                ///< Time of last bump with a droid - relative to bumpTime
	uint16_t pauseTime = 0;               ///< When MOVEPAUSE started - relative to bumpTime
	IntVector3 bumpPos = IntVector3(0, 0, 0); ///< Position of last bump

	unsigned shuffleStart = 0;            ///< When a shuffle started

	int iVertSpeed = 0;                   ///< VTOL movement
};
