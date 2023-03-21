/******************************************************************************/
/*!
\file		GameState_Cage.h
\author 	Ian Chua
\par    	email: i.chua@digipen.edu
\date   	March 21, 2023
\brief

Copyright (C) 2023 DigiPen Institute of Technology.
Reproduction or disclosure of this file or its contents without the
prior written consent of DigiPen Institute of Technology is prohibited.
 */
 /******************************************************************************/

#ifndef CSD1130_GAME_STATE_PLAY_H_
#define CSD1130_GAME_STATE_PLAY_H_


// ---------------------------------------------------------------------------

void GameStateCageLoad(void);
void GameStateCageInit(void);
void GameStateCageUpdate(void);
void GameStateCageDraw(void);
void GameStateCageFree(void);
void GameStateCageUnload(void);

// ---------------------------------------------------------------------------

#endif // CSD1130_GAME_STATE_PLAY_H_