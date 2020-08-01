/*
*/
/** @file
 *  Raycaster functions
 */

#pragma once


/*!
 * The raycast intersection callback.
 * \param pos Current position
 * \param dist Current distance from start
 * \param data Payload (store intermediate results here)
 * \return true if ore points are required, false otherwise
 */
typedef bool (*RAY_CALLBACK)(Vector2i pos, int32_t dist, void *data);


/*!
 * Cast a ray from a position into a certain direction
 * \param src Position to cast from
 * \param dst Position to cast to (casts to end of map, if dst is off the map)
 * \param callback Callback to call for each passed tile
 * \param data Data to pass through to the callback
 */
void rayCast(Vector2i src, Vector2i dst, RAY_CALLBACK callback, void *data);


// Calculates the maximum height and distance found along a line from any
// point to the edge of the grid
void getBestPitchToEdgeOfGrid(UDWORD x, UDWORD y, uint16_t direction, uint16_t *pitch);

