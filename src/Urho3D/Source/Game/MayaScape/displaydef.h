/*
*/
/** \file
 *  Display structures.
 */

#include "types.h"

#pragma once

struct iIMDShape;

// for mouse scrolling. (how many pixels from the edge before pointer scrolls the screen in a direction)
#define	BOUNDARY_X	(2)
#define	BOUNDARY_Y	(2)

struct SCREEN_DISP_DATA
{
	iIMDShape	*imd;
	UDWORD		frameNumber;		// last frame it was drawn
	UDWORD		screenX, screenY;
	UDWORD		screenR;
};
