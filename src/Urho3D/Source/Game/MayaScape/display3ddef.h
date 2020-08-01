/*
*/
#pragma once

#ifndef __INCLUDED_SRC_DISPLAY3DDEF_H__
#define __INCLUDED_SRC_DISPLAY3DDEF_H__

#define TILE_WIDTH 128
#define TILE_HEIGHT 128
#define TILE_SIZE (TILE_WIDTH*TILE_HEIGHT)

// Amount of visible terrain tiles in x/y direction
#define VISIBLE_XTILES 64
#define VISIBLE_YTILES 64

#define	RADTLX		(OBJ_BACKX + OBJ_BACKWIDTH + BASE_GAP + 1 +D_W)	// Paul's settings (492+12)
#define	RADTLY		(RET_Y + 1)									// Paul's settings (332-17)
#define	RADWIDTH	128
#define RADHEIGHT	128

#define SKY_MULT	1
#define SKY_SHIMMY_BASE	((DEG(1)*SKY_MULT)/2)
#define SKY_SHIMMY (SKY_SHIMMY_BASE - (rand()%(2*SKY_SHIMMY_BASE)))
