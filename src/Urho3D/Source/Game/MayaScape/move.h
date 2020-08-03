/*
*/
/** @file
 *  Interface for the unit movement system
 */

#pragma once

#include "objectdef.h"
#include "fpath.h"

/* Set a target location for a droid to move to  - returns a bool based on if there is a path to the destination (true if there is a path)*/
//bool moveDroidTo(DROID *psDroid, UDWORD x, UDWORD y, FPATH_MOVETYPE moveType = FMT_MOVE);

/* Set a target location for a droid to move to  - returns a bool based on if there is a path to the destination (true if there is a path)*/
// the droid will not join a formation when it gets to the location
//bool moveDroidToNoFormation(DROID *psDroid, UDWORD x, UDWORD y, FPATH_MOVETYPE moveType = FMT_MOVE);

// move a droid directly to a location (used by vtols only)
//void moveDroidToDirect(DROID *psDroid, UDWORD x, UDWORD y);

// Get a droid to turn towards a locaton
//void moveTurnDroid(DROID *psDroid, UDWORD x, UDWORD y);

/* Stop a droid */
//void moveStopDroid(DROID *psDroid);

/*Stops a droid dead in its tracks - doesn't allow for any little skidding bits*/
//void moveReallyStopDroid(DROID *psDroid);

/* Get a droid to do a frame's worth of moving */
//void moveUpdateDroid(DROID *psDroid);

//SDWORD moveCalcDroidSpeed(DROID *psDroid);

/* Frame update for the movement of a tracked droid */
//void moveUpdateTracked(DROID *psDroid);

/* update body and turret to local slope */
//void updateDroidOrientation(DROID *psDroid);

/* audio callback used to kill movement sounds */
//bool moveCheckDroidMovingAndVisible(void *psObj);

// set a vtol to be hovering in the air
//void moveMakeVtolHover(DROID *psDroid);

const char *moveDescription(MOVE_STATUS status);

