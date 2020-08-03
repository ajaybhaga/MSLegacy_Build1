/*
*/

#pragma once

#include "types.h"
#include <glm/fwd.hpp>

bool initTerrain();
void shutdownTerrain();

void drawTerrain(const glm::mat4 &ModelViewProjection);
void drawWater(const glm::mat4 &viewMatrix);

PIELIGHT getTileColour(int x, int y);
void setTileColour(int x, int y, PIELIGHT colour);

void markTileDirty(int i, int j);
