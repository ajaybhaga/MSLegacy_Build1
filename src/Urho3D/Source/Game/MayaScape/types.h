#pragma once

#include <limits.h>
#include <ctype.h>
#include <stdint.h>

#define GAME_TICKS_PER_SEC 1000
uint32_t selectedPlayer = 0;  /**< Current player */

/* Basic numeric types */
typedef uint8_t  UBYTE;
typedef int8_t   SBYTE;
typedef uint16_t UWORD;
typedef int16_t  SWORD;
typedef uint32_t UDWORD;
typedef int32_t  SDWORD;

/* Numeric size defines */
#define UBYTE_MAX	UINT8_MAX
#define SBYTE_MIN	INT8_MIN
#define SBYTE_MAX	INT8_MAX
#define UWORD_MAX	UINT16_MAX
#define SWORD_MIN	INT16_MIN
#define SWORD_MAX	INT16_MAX
#define UDWORD_MAX	UINT32_MAX
#define SDWORD_MIN	INT32_MIN
#define SDWORD_MAX	INT32_MAX


#define MAX_PLAYERS         11                 ///< Maximum number of players in the game.

#if MAX_PLAYERS <= 8
typedef uint8_t PlayerMask;
#elif MAX_PLAYERS <= 16
typedef uint16_t PlayerMask;
#endif

/***************************************************************************/
/*
 *	Global Definitions (CONSTANTS)
 */
/***************************************************************************/
#define LONG_WAY			(1<<15)
#define BUTTON_DEPTH		2000 // will be stretched to 16000

#define OLD_TEXTURE_SIZE_FIX 256.0f

//Render style flags for all pie draw functions
#define pie_ECM                 0x1
#define pie_TRANSLUCENT         0x2
#define pie_ADDITIVE            0x4
#define pie_FORCE_FOG           0x8
#define pie_HEIGHT_SCALED       0x10
#define pie_RAISE               0x20
#define pie_BUTTON              0x40
#define pie_SHADOW              0x80
#define pie_STATIC_SHADOW       0x100
#define pie_PREMULTIPLIED       0x200

#define pie_RAISE_SCALE			256



/*#define BASE_COORDS_X	(640)
#define BASE_COORDS_Y	(460)
#define E_W (pie_GetVideoBufferWidth() - BASE_COORDS_X)
#define E_H (pie_GetVideoBufferHeight() - BASE_COORDS_Y)
#define D_W	((pie_GetVideoBufferWidth() - BASE_COORDS_X)/2)
#define D_H ((pie_GetVideoBufferHeight() - BASE_COORDS_Y)/2)
*/
// Reticule position.
#define E_H                  0
#define RET_X				6
#define RET_Y				(324+E_H)
#define RET_FORMWIDTH		132
#define RET_FORMHEIGHT		132


enum LIGHTING_TYPE
{
    LIGHT_EMISSIVE,
    LIGHT_AMBIENT,
    LIGHT_DIFFUSE,
    LIGHT_SPECULAR,
    LIGHT_MAX
};

enum REND_MODE
{
    REND_ALPHA,
    REND_ADDITIVE,
    REND_OPAQUE,
    REND_MULTIPLICATIVE,
    REND_PREMULTIPLIED,
    REND_TEXT,
};

enum DEPTH_MODE
{
    DEPTH_CMP_LEQ_WRT_ON,
    DEPTH_CMP_ALWAYS_WRT_ON,
    DEPTH_CMP_ALWAYS_WRT_OFF
};

enum TEXPAGE_TYPE
{
    TEXPAGE_NONE = -1,
    TEXPAGE_EXTERN = -2
};

enum SHADER_MODE
{
    SHADER_NONE,
    SHADER_COMPONENT,
    SHADER_BUTTON,
    SHADER_NOLIGHT,
    SHADER_TERRAIN,
    SHADER_TERRAIN_DEPTH,
    SHADER_DECALS,
    SHADER_WATER,
    SHADER_RECT,
    SHADER_TEXRECT,
    SHADER_GFX_COLOUR,
    SHADER_GFX_TEXT,
    SHADER_GENERIC_COLOR,
    SHADER_LINE,
    SHADER_TEXT,
    SHADER_MAX
};

//*************************************************************************
//
// Simple derived types
//
//*************************************************************************

struct iV_Image
{
    unsigned int width, height, depth;
    unsigned char *bmp;
};

struct PIELIGHTBYTES
{
    uint8_t r, g, b, a;
};

/** Our basic colour type. Use whenever you want to define a colour.
 *  Set bytes separetely, and do not assume a byte order between the components. */
union PIELIGHT
{
    PIELIGHTBYTES byte;
    UDWORD rgba;
    UBYTE vector[4];
};

