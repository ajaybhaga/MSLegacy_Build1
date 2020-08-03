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
