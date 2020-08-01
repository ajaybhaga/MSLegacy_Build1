/*
*/
/** \file
 *  Definitions for movement tracking.
 */

#pragma once

#include <vector>

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
	std::vector<Vector2i> asPath;         ///< Pointer to list of block X,Y map coordinates.

	Vector2i destination = Vector2i(0, 0);                 ///< World coordinates of movement destination
	Vector2i src = Vector2i(0, 0);
	Vector2i target = Vector2i(0, 0);
	int speed = 0;                        ///< Speed of motion

	uint16_t moveDir = 0;                 ///< Direction of motion (not the direction the droid is facing)
	uint16_t bumpDir = 0;                 ///< Direction at last bump
	unsigned bumpTime = 0;                ///< Time of first bump with something
	uint16_t lastBump = 0;                ///< Time of last bump with a droid - relative to bumpTime
	uint16_t pauseTime = 0;               ///< When MOVEPAUSE started - relative to bumpTime
	Position bumpPos = Position(0, 0, 0); ///< Position of last bump

	unsigned shuffleStart = 0;            ///< When a shuffle started

	int iVertSpeed = 0;                   ///< VTOL movement
};
