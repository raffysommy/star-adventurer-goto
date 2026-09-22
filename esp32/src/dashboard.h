#pragma once
#include <WebServer.h>

// Mount dashboard (port of lx200.py's web UI) and its JSON API:
//   GET  /                 dashboard
//   GET  /api/status       mount state
//   POST /api/settings     dec_axis_reversed, flip_ra_guiding, lat, lon (form fields)
//   POST /api/time         unix (seconds UTC), tz_minutes (browser getTimezoneOffset)
//   POST /api/home         position register := home
//   POST /api/stop         same as :Q
//   GET  /api/lx200?c=...  run one LX200 command, returns the raw reply
void dashboardBegin(WebServer &web);
