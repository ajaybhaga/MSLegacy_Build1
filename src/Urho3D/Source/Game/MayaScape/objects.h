/*
*/
/** @file
 *  A header file that groups together all the object header files
 */

#pragma once

#include "objectdef.h"
#include "droid.h"
//#include "structure.h"
//#include "feature.h"
#include "objmem.h"

/* Initialise the object system */
bool objInitialise();

/* Shutdown the object system */
bool objShutdown();

/// Goes through the list passed in reversing the order so the first entry becomes the last and the last entry becomes the first!
void reverseObjectList(BASE_OBJECT **ppsList);

template <typename OBJECT>
void reverseObjectList(OBJECT **ppsList)
{
	BASE_OBJECT *baseList = *ppsList;
	reverseObjectList(&baseList);
	*ppsList = static_cast<OBJECT *>(baseList);
}

/** Output an informative string about this object. For debugging. */
const char *objInfo(const BASE_OBJECT *psObj);
