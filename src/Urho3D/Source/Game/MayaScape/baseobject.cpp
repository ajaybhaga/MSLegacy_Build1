/*
*/

#include "baseobject.h"
//#include "droid.h"
//#include "projectile.h"
//#include "structure.h"
//#include "feature.h"
//#include "intdisplay.h"
#include "map.h"

using namespace Urho3D;

/// Returns the given angle, wrapped to the range [-180°; 180°) = [-32768; 32767].
static inline int32_t angleDelta(int32_t a)
{
return (int16_t)a;  // Cast wrapping intended.
}


static inline uint16_t interpolateAngle(uint16_t v1, uint16_t v2, uint32_t t1, uint32_t t2, uint32_t t)
{
	const int numer = t - t1, denom = t2 - t1;

	return v1 + angleDelta(v2 - v1) * numer / denom;
}

static IntVector3 interpolatePos(IntVector3 p1, IntVector3 p2, uint32_t t1, uint32_t t2, uint32_t t)
{
	const int numer = t - t1, denom = t2 - t1;
	return p1 + (p2 - p1) * numer / denom;
}

Quaternion interpolateRot(Quaternion v1, Quaternion v2, uint32_t t1, uint32_t t2, uint32_t t)
{
	//return v1 + (v2 - v1) * (t - t1) / (t2 - t1);
	return Quaternion(interpolateAngle(v1.YawAngle(), v2.YawAngle(), t1, t2, t),
	                interpolateAngle(v1.PitchAngle(),     v2.PitchAngle(),     t1, t2, t),
	                interpolateAngle(v1.RollAngle(),      v2.RollAngle(),      t1, t2, t)
	               );
}

static Spacetime interpolateSpacetime(Spacetime st1, Spacetime st2, uint32_t t)
{
	// Cyp says this should never happen, #3037 and #3238 say it does though.
	//ASSERT_OR_RETURN(st1, st1.time != st2.time, "Spacetime overlap!");
	return Spacetime(interpolatePos(st1.pos, st2.pos, st1.time, st2.time, t), interpolateRot(st1.rot, st2.rot, st1.time, st2.time, t), t);
}

Spacetime interpolateObjectSpacetime(const SIMPLE_OBJECT *obj, uint32_t t)
{
	switch (obj->type)
	{
	default:
		return getSpacetime(obj);
	case OBJ_DROID:
//		return interpolateSpacetime(castDroid(obj)->prevSpacetime, getSpacetime(obj), t);
        return getSpacetime(obj);
	case OBJ_PROJECTILE:
//		return interpolateSpacetime(castProjectile(obj)->prevSpacetime, getSpacetime(obj), t);
        return getSpacetime(obj);
    }
}

SIMPLE_OBJECT::SIMPLE_OBJECT(OBJECT_TYPE type, uint32_t id, unsigned player, float gameTime)
	: type(type)
	, id(id)
	, pos(0, 0, 0)
	, rot(0, 0, 0)
	, player(player)
	, born(gameTime)
	, died(0)
	, time(0)
{}

SIMPLE_OBJECT::~SIMPLE_OBJECT()
{
	// Make sure to get rid of some final references in the sound code to this object first
	//audio_RemoveObj(this);

	const_cast<OBJECT_TYPE volatile &>(type) = (OBJECT_TYPE)(type + 1000000000);  // Hopefully this will trigger an assert              if someone uses the freed object.
	const_cast<UBYTE volatile &>(player) += 100;                                  // Hopefully this will trigger an assert and/or crash if someone uses the freed object.
}

BASE_OBJECT::BASE_OBJECT(OBJECT_TYPE type, uint32_t id, unsigned player, float gameTime)
	: SIMPLE_OBJECT(type, id, player, gameTime)
	, selected(false)
	, lastEmission(0)
//	, lastHitWeapon(WSC_NUM_WEAPON_SUBCLASSES)  // No such weapon.
	, timeLastHit(UDWORD_MAX)
	, body(0)
	, periodicalDamageStart(0)
	, periodicalDamage(0)
	, timeAnimationStarted(0)
//	, animationEvent(ANIM_EVENT_NONE)
{
	memset(visible, 0, sizeof(visible));
	/*sDisplay.imd = nullptr;
	sDisplay.frameNumber = 0;
	sDisplay.screenX = 0;
	sDisplay.screenY = 0;
	sDisplay.screenR = 0;*/
}

BASE_OBJECT::~BASE_OBJECT()
{
	//visRemoveVisibility(this);

#ifdef DEBUG
	psNext = this;                                                       // Hopefully this will trigger an infinite loop       if someone uses the freed object.
	psNextFunc = this;                                                   // Hopefully this will trigger an infinite loop       if someone uses the freed object.
#endif //DEBUG
}

void checkObject(const SIMPLE_OBJECT *psObject, const char *const location_description, const char *function, const int recurse)
{
	if (recurse < 0)
	{
		return;
	}

	//ASSERT(psObject != nullptr, "NULL pointer");

	switch (psObject->type)
	{
	case OBJ_DROID:
	//	checkDroid((const DROID *)psObject, location_description, function, recurse - 1);
		break;

	case OBJ_STRUCTURE:
//		checkStructure((const STRUCTURE *)psObject, location_description, function, recurse - 1);
		break;

	case OBJ_PROJECTILE:
//		checkProjectile((const PROJECTILE *)psObject, location_description, function, recurse - 1);
		break;

	case OBJ_FEATURE:
	case OBJ_TARGET:
		break;

	default:
//		ASSERT_HELPER(!"invalid object type", location_description, function, "CHECK_OBJECT: Invalid object type (type num %u)", (unsigned int)psObject->type);
		break;
	}
}

void _syncDebugObject(const char *function, SIMPLE_OBJECT const *psObject, char ch)
{
	switch (psObject->type)
	{
	    case 0: break;
//	case OBJ_DROID:      _syncDebugDroid(function, (const DROID *)     psObject, ch); break;
//	case OBJ_STRUCTURE:  _syncDebugStructure(function, (const STRUCTURE *) psObject, ch); break;
//	case OBJ_FEATURE:    _syncDebugFeature(function, (const FEATURE *)   psObject, ch); break;
//	case OBJ_PROJECTILE: _syncDebugProjectile(function, (const PROJECTILE *)psObject, ch); break;
	default:             //_syncDebug(function, "%c unidentified_object%d = p%d;objectType%d", ch, psObject->id, psObject->player, psObject->type);
//		ASSERT_HELPER(!"invalid object type", "_syncDebugObject", function, "syncDebug: Invalid object type (type num %u)", (unsigned int)psObject->type);
		break;
	}
}

IntVector2 getStatsSize(BASE_STATS const *pType, uint16_t direction)
{
    /*
	if (StatIsStructure(pType))
	{
		return static_cast<STRUCTURE_STATS const *>(pType)->size(direction);
	}
	else if (StatIsFeature(pType))
	{
		return static_cast<FEATURE_STATS const *>(pType)->size();
	}*/
	return IntVector2(1, 1);
}

StructureBounds getStructureBounds(BASE_OBJECT const *object)
{
    /*
	STRUCTURE const *psStructure = castStructure(object);
	FEATURE const *psFeature = castFeature(object);

	if (psStructure != nullptr)
	{
		return getStructureBounds(psStructure);
	}
	else if (psFeature != nullptr)
	{
		return getStructureBounds(psFeature);
	}
*/
	return StructureBounds(IntVector2(32767, 32767), IntVector2(-65535, -65535));  // Default to an invalid area.
}

StructureBounds getStructureBounds(BASE_STATS const *stats, IntVector2 pos, uint16_t direction)
{
    /*
	if (StatIsStructure(stats))
	{
		return getStructureBounds(static_cast<STRUCTURE_STATS const *>(stats), pos, direction);
	}
	else if (StatIsFeature(stats))
	{
		return getStructureBounds(static_cast<FEATURE_STATS const *>(stats), pos);
	}`
*/
	return StructureBounds(map_coord(pos), IntVector2(1, 1));  // Default to a 1×1 tile.
}
