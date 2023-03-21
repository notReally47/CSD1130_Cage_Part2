/******************************************************************************/
/*!
\file		GameStateList.h
\author 	Ian Chua
\par    	email: i.chua@digipen.edu
\date   	March 21, 2023
\brief

Copyright (C) 2023 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the
prior written consent of DigiPen Institute of Technology is prohibited.
 */
 /******************************************************************************/

#ifndef CSD1130_GAME_STATE_LIST_H_
#define CSD1130_GAME_STATE_LIST_H_

// ---------------------------------------------------------------------------
// game state list

enum class GS_STATE
{
	// list of all game states 
	GS_CAGE = 0, 
	
	// special game state. Do not change
	GS_RESTART,
	GS_QUIT, 
	GS_NUM
};

// ---------------------------------------------------------------------------

#endif // CSD1130_GAME_STATE_LIST_H_