/*
*/

#pragma once

#include "display.h"
#include "display3ddef.h"	// This should be the only place including this file
#include "objectdef.h"
#include "MayaScape/not-used/message.h"

#define HEIGHT_TRACK_INCREMENTS (50)

/*!
 * Special tile types
 */
enum TILE_ID
{
	RIVERBED_TILE = 5, //! Underwater ground
	WATER_TILE = 17, //! Water surface
	RUBBLE_TILE = 54, //! You can drive over these
	BLOCKING_RUBBLE_TILE = 67 //! You cannot drive over these
};

enum ENERGY_BAR
{
	BAR_SELECTED,
	BAR_DROIDS,
	BAR_DROIDS_AND_STRUCTURES,
	BAR_LAST
};

struct iView
{
	Vector3i p = Vector3i(0, 0, 0);
	Vector3i r = Vector3i(0, 0, 0);
};

extern bool showFPS;
extern bool showUNITCOUNT;
extern bool showSAMPLES;
extern bool showORDERS;

float getViewDistance();
void setViewDistance(float dist);
extern bool	radarOnScreen;
extern bool	radarPermitted;
bool radarVisible();

extern bool rangeOnScreen; // Added to get sensor/gun range on screen.  -Q 5-10-05
void setViewPos(UDWORD x, UDWORD y, bool Pan);
Vector2i    getPlayerPos();
void setPlayerPos(SDWORD x, SDWORD y);
void disp3d_setView(iView *newView);
void disp3d_oldView(); // for save games <= 10
void disp3d_getView(iView *newView);

void draw3DScene();
//void renderStructure(STRUCTURE *psStructure, const glm::mat4 &viewMatrix);
//void renderFeature(FEATURE *psFeature, const glm::mat4 &viewMatrix);
//void renderProximityMsg(PROXIMITY_DISPLAY	*psProxDisp, const glm::mat4 &viewMatrix);
//void renderProjectile(PROJECTILE *psCurr, const glm::mat4 &viewMatrix);
//void renderDeliveryPoint(FLAG_POSITION *psPosition, bool blueprint, const glm::mat4 &viewMatrix);
void debugToggleSensorDisplay();

void calcScreenCoords(DROID *psDroid, const glm::mat4 &viewMatrix);
ENERGY_BAR toggleEnergyBars();

bool doWeDrawProximitys();
void setProximityDraw(bool val);

bool	clipXY(SDWORD x, SDWORD y);
inline bool clipShapeOnScreen(const iIMDShape *pIMD, const glm::mat4& viewModelMatrix, int overdrawScreenPoints = 10);
bool clipDroidOnScreen(DROID *psDroid, const glm::mat4& viewModelMatrix, int overdrawScreenPoints = 25);
bool clipStructureOnScreen(STRUCTURE *psStructure, const glm::mat4 &viewModelMatrix, int overdrawScreenPoints = 0);

bool init3DView();
extern iView player;
extern bool selectAttempt;

extern SDWORD scrollSpeed;
void assignSensorTarget(BASE_OBJECT *psObj);
void assignDestTarget();
UDWORD getWaterTileNum();
void setUnderwaterTile(UDWORD num);
UDWORD getRubbleTileNum();
void setRubbleTile(UDWORD num);

STRUCTURE *getTileBlueprintStructure(int mapX, int mapY);  ///< Gets the blueprint at those coordinates, if any. Previous return value becomes invalid.
STRUCTURE_STATS const *getTileBlueprintStats(int mapX, int mapY);  ///< Gets the structure stats of the blueprint at those coordinates, if any.
bool anyBlueprintTooClose(STRUCTURE_STATS const *stats, Vector2i pos, uint16_t dir);  ///< Checks if any blueprint is too close to the given structure.
void clearBlueprints();

void display3dScreenSizeDidChange(unsigned int oldWidth, unsigned int oldHeight, unsigned int newWidth, unsigned int newHeight);

extern SDWORD mouseTileX, mouseTileY;
extern Vector2i mousePos;

extern bool bRender3DOnly;
extern bool showGateways;
extern bool showPath;
extern const Vector2i visibleTiles;

/*returns the graphic ID for a droid rank*/
UDWORD  getDroidRankGraphic(DROID *psDroid);

/* Visualize radius at position */
void showRangeAtPos(SDWORD centerX, SDWORD centerY, SDWORD radius);

void setSkyBox(const char *page, float mywind, float myscale);

#define	BASE_MUZZLE_FLASH_DURATION	(GAME_TICKS_PER_SEC/10)
#define	EFFECT_MUZZLE_ADDITIVE		128

extern UWORD barMode;

extern bool CauseCrash;

extern bool tuiTargetOrigin;

/// Draws using the animation systems. Usually want to use in a while loop to get all model levels.
bool drawShape(BASE_OBJECT *psObj, iIMDShape *strImd, int colour, PIELIGHT buildingBrightness, int pieFlag, int pieFlagData, const glm::mat4& viewMatrix);
