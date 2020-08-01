/*
*/
/** \file
 *  Definitions for projectiles.
 */

#pragma once

#include "basedef.h"

#include <vector>


enum PROJ_STATE
{
	PROJ_INFLIGHT,
	PROJ_IMPACT,
	PROJ_POSTIMPACT,
	PROJ_INACTIVE,
};

struct PROJECTILE : public SIMPLE_OBJECT
{
	PROJECTILE(uint32_t id, unsigned player) : SIMPLE_OBJECT(OBJ_PROJECTILE, id, player) {}

	void            update();
	bool            deleteIfDead()
	{
		if (died == 0 || died >= gameTime - deltaGameTime)
		{
			return false;
		}
		delete this;
		return true;
	}

	UBYTE           state;                  ///< current projectile state
	UBYTE           bVisible;               ///< whether the selected player should see the projectile
	WEAPON_STATS   *psWStats;               ///< firing weapon stats
	BASE_OBJECT    *psSource;               ///< what fired the projectile
	BASE_OBJECT    *psDest;                 ///< target of this projectile
	std::vector<BASE_OBJECT *> psDamaged;   ///< the targets that have already been dealt damage to (don't damage the same target twice)

	Vector3i        src = Vector3i(0, 0, 0); ///< Where projectile started
	Vector3i        dst = Vector3i(0, 0, 0); ///< The target coordinates
	SDWORD          vXY, vZ;                ///< axis velocities
	Spacetime       prevSpacetime;          ///< Location of projectile in previous tick.
	UDWORD          expectedDamageCaused;   ///< Expected damage that this projectile will cause to the target.
	int             partVisible;            ///< how much of target was visible on shooting (important for homing)
};

typedef std::vector<PROJECTILE *>::const_iterator ProjectileIterator;


/// True iff object is a projectile.
static inline bool isProjectile(SIMPLE_OBJECT const *psObject)
{
	return psObject != nullptr && psObject->type == OBJ_PROJECTILE;
}
/// Returns PROJECTILE * if projectile or NULL if not.
static inline PROJECTILE *castProjectile(SIMPLE_OBJECT *psObject)
{
	return isProjectile(psObject) ? (PROJECTILE *)psObject : (PROJECTILE *)nullptr;
}
/// Returns PROJECTILE const * if projectile or NULL if not.
static inline PROJECTILE const *castProjectile(SIMPLE_OBJECT const *psObject)
{
	return isProjectile(psObject) ? (PROJECTILE const *)psObject : (PROJECTILE const *)nullptr;
}
