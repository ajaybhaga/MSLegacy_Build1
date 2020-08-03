/*

/*
 * Objects.c
 *
 * The object system.
 *
 */


#include "objects.h"

/* Initialise the object system */
bool objInitialise()
{
	if (!objmemInitialise())
	{
		return false;
	}

	return true;
}


/* Shutdown the object system */
bool objShutdown()
{
	objmemShutdown();

	return true;
}


/*goes thru' the list passed in reversing the order so the first entry becomes
the last and the last entry becomes the first!*/
void reverseObjectList(BASE_OBJECT **ppsList)
{
	BASE_OBJECT *psPrev = nullptr;
	BASE_OBJECT *psCurrent = *ppsList;

	while (psCurrent != nullptr)
	{
		BASE_OBJECT *psNext = psCurrent->psNext;
		psCurrent->psNext = psPrev;
		psPrev = psCurrent;
		psCurrent = psNext;
	}
	//set the list passed in to point to the new top
	*ppsList = psPrev;
}

const char *objInfo(const BASE_OBJECT *psObj)
{
	static char	info[PATH_MAX];

	if (!psObj)
	{
		return "null";
	}

	switch (psObj->type)
	{
/*
	case OBJ_DROID:
		{
			const DROID *psDroid = (const DROID *)psObj;
			return droidGetName(psDroid);
		}
	case OBJ_STRUCTURE:
		{
			const STRUCTURE *psStruct = (const STRUCTURE *)psObj;
			sstrcpy(info, getName(psStruct->pStructureType));
			break;
		}
	case OBJ_FEATURE:
		{
			const FEATURE *psFeat = (const FEATURE *)psObj;
			sstrcpy(info, getName(psFeat->psStats));
			break;
		}
	case OBJ_PROJECTILE:
		sstrcpy(info, "Projectile");	// TODO
		break;*/
	case OBJ_TARGET:
	//	sstrcpy(info, "Target");	// TODO
		break;
	default:
	//	sstrcpy(info, "Unknown object type");
		break;
	}
	return info;
}
