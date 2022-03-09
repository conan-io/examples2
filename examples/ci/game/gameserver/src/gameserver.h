#pragma once


#ifdef _WIN32
  #define GAMESERVER_EXPORT __declspec(dllexport)
#else
  #define GAMESERVER_EXPORT
#endif

GAMESERVER_EXPORT void gameserver();
