/*
*/
/** @file
 *  Allows querying which objects are within a given radius of a given location.
 */

#pragma once

typedef std::vector<BASE_OBJECT *> GridList;
typedef GridList::const_iterator GridIterator;

// initialise the grid system
bool gridInitialise();

// shutdown the grid system
void gridShutDown();

// Reset the grid system. Called once per update.
// Resets seenThisTick[] to false.
void gridReset();

/// Find all objects within radius.
GridList const &gridStartIterate(int32_t x, int32_t y, uint32_t radius);

/// Find all objects within radius.
GridList const &gridStartIterateArea(int32_t x, int32_t y, uint32_t x2, uint32_t y2);

/// Find all objects within radius where object->type == OBJ_DROID && object->player == player.
GridList const &gridStartIterateDroidsByPlayer(int32_t x, int32_t y, uint32_t radius, int player);

// Used for visibility.
/// Find all objects within radius where object->seenThisTick[player] != 255.
GridList const &gridStartIterateUnseen(int32_t x, int32_t y, uint32_t radius, int player);
