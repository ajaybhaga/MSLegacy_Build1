/*
*/
/** @file
 *  Definition for position objects.
 */

#pragma once

enum POSITION_TYPE
{
	POS_DELIVERY,		//Delivery Points NOT wayPoints
	POS_PROXDATA,	//proximity messages that are data generated
	POS_PROXOBJ,	//proximity messages that are in game generated
	POS_TEMPDELIVERY //SAVE ONLY delivery point for factory currently assigned to commander
};

struct OBJECT_POSITION
{
	POSITION_TYPE   type;                   ///< the type of position obj - FlagPos or ProxDisp
	UDWORD          frameNumber;            ///< when the Position was last drawn
	UDWORD          screenX;                ///< screen coords and radius of Position imd
	UDWORD          screenY;
	UDWORD          screenR;
	UDWORD          player;                 ///< which player the Position belongs to
	bool            selected;               ///< flag to indicate whether the Position is to be highlighted
};
