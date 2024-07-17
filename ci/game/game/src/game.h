#pragma once


#ifdef _WIN32
  #define GAME_EXPORT __declspec(dllexport)
#else
  #define GAME_EXPORT
#endif

GAME_EXPORT void game();
