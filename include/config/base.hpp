#pragma once

#if BUILD_TYPE == 1 // infantry
#define TYPE_INFANTRY
#endif

#if BUILD_TYPE == 2 //hero
#define TYPE_HERO
#endif

#if BUILD_TYPE == 3 // sentry
#define TYPE_SENTRY
#endif