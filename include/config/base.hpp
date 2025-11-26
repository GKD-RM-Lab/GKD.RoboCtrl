#pragma once

#if BUILD_TYPE == 1 // infantry
#define TYPE_INFANTRY
#define TYPE_STR "infantry"
#endif

#if BUILD_TYPE == 2 //hero
#define TYPE_HERO
#define TYPE_STR "hero"
#endif

#if BUILD_TYPE == 3 // sentry
#define TYPE_SENTRY
#define TYPE_STR "sentry"
#endif

#if BUILD_TYPE == 4 // project
#define TYPE_PROJECT
#define TYPE_STR "project"
#endif

#ifndef TYPE_STR
#error "No type specified"
#endif