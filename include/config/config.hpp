#pragma once
#include "config/base.hpp"

#ifdef TYPE_INFANTRY // infantry
#include "config/config.infantry.hpp"
#endif

#ifdef TYPE_HERO
#include "config/config.hero.hpp"
#endif

#ifdef TYPE_SENTRY
#include "config/config.sentry.hpp"
#endif