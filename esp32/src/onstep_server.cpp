#include "onstep_server.h"

#include <WiFi.h>
#include <astro.h>

#include "clock.h"
#include "dec_axis.h"
#include "gps.h"
#include "mount.h"
#include "netlog.h"
#include "ra_axis.h"
#include "settings.h"

namespace onstep {

static const int MAX_CLIENTS = 3;
static const uint16_t ONSTEP_PORT = 9999;  // OnStep's usual Wi-Fi command port

// OnStepX :GR/:GD precision, toggled by :U# (generic LX200 drivers use it)
static bool lowPrecision = false;
static volatile int nClients = 0;
static SemaphoreHandle_t lock;  // process() is called from the server task and the web UI

int clients() { return nClients; }

// ---------------------------------------------------------------- replies

static String hms(double deg, bool low) {
  char b[24];
  if (low) {
    long t = lround(astro::wrap360(deg) / 15.0 * 600.0) % (24 * 600);  // tenths of a minute
    snprintf(b, sizeof(b), "%02ld:%02ld.%01ld", t / 600, t / 10 % 60, t % 10);
  } else {
    astro::formatRa(deg, b, sizeof(b));
  }
  return b;
}

static String dms(double deg, bool low) {
  char b[24];
  if (low) {
    astro::formatSite(deg, 2, false, b, sizeof(b));
  } else {
    astro::formatDec(deg, b, sizeof(b));
  }
  return b;
}

static String raHigh(double deg) {
  char b[24];
  astro::formatRaHigh(deg, b, sizeof(b));
  return b;
}

static String decHigh(double deg) {
  char b[24];
  astro::formatDecHigh(deg, b, sizeof(b));
  return b;
}

static String site(double deg, int degDigits, bool high) {
  char b[24];
  astro::formatSite(deg, degDigits, high, b, sizeof(b));
  return b;
}

static String localTime(const char *fmt) {
  time_t t = (time_t)(clockNow() + settings.utcOffset * 3600);
  struct tm tm;
  gmtime_r(&t, &tm);
  char b[16];
  strftime(b, sizeof(b), fmt, &tm);
  return b;
}

// :GU# status letters (OnStepX Status.command.cpp)
static String statusLetters() {
  ra::State r = ra::state();
  mount::State m = mount::state();
  String s;
  if (!r.tracking) s += 'n';                 // [n]ot tracking
  if (!r.slewing && !dec::slewing()) s += 'N';  // [N]o goto
  s += 'p';                                  // not [p]arked (no park support)
  if (gps::timeSynced()) s += 'S';           // PPS/GPS [S]ync
  if (r.guideEast || r.guideWest || m.guideNorth || m.guideSouth) s += 'G';  // pulse [G]uide active
  ra::PecInfo pi = ra::pecInfo();
  if (pi.recorded) s += 'R';                 // PEC data [R]ecorded
  s += "/,~;^"[(int)pi.state];               // PEC (/)gnore, ready (,)lay, (~)laying, ready (;)ecord, (^)ecording
  if (settings.refraction) s += "rs";       // [r]efraction, [s]ingle axis
  else {
    double hz = ra::trackRateHz();
    if (fabs(hz - ra::LUNAR_HZ) < 0.001) s += '(';
    else if (fabs(hz - ra::SOLAR_HZ) < 0.001) s += 'O';
    else if (fabs(hz - ra::KING_HZ) < 0.001) s += 'k';
  }
  s += 'E';                                  // GEM
  s += mount::pierSide() == 'W' ? 'W' : 'T'; // pier side eas[T] / [W]est
  s += '2';                                  // pulse-guide rate index (0.5x)
  s += (char)('0' + mount::moveRate());     // manual-move rate index (:Rn)
  s += r.connected ? '0' : '7';              // error code: 7 = hardware fault (mount not connected)
  return s;
}

// Minutes past the meridian allowed, OnStep's meridian limits (:GXE9# east, :GXEA# west):
// on the normal branch (pier east) the window reaches -eastLimit before the meridian; on
// the flipped branch tracking may go on until ra::trackMax().
static long minutesPastMeridianE() { return lround(-settings.raEastLimit * 4.0); }
static long minutesPastMeridianW() { return lround((ra::trackMax() - ra::offset() - 180.0) * 4.0); }

// Horizontal coordinates for :GA# / :GZ#
static void altAz(double &alt, double &az) {
  double ha = astro::wrap360(ra::lst() - mount::reportedRaDeg()) * DEG_TO_RAD;
  double dec = mount::reportedDecDeg() * DEG_TO_RAD, lat = settings.lat * DEG_TO_RAD;
  double sinAlt = sin(dec) * sin(lat) + cos(dec) * cos(lat) * cos(ha);
  alt = asin(sinAlt) * RAD_TO_DEG;
  az = atan2(-cos(dec) * sin(ha), sin(dec) * cos(lat) - cos(dec) * sin(lat) * cos(ha)) * RAD_TO_DEG;
  az = astro::wrap360(az);
}

// ---------------------------------------------------------------- commands

static String get(const String &cmd, const char *c) {
  if (cmd == ":GR#") return hms(mount::reportedRaDeg(), lowPrecision) + "#";
  if (cmd == ":GRH#" || cmd == ":GRa#") return raHigh(mount::reportedRaDeg()) + "#";
  if (cmd == ":GD#") return dms(mount::reportedDecDeg(), lowPrecision) + "#";
  if (cmd == ":GDH#" || cmd == ":GDe#") return decHigh(mount::reportedDecDeg()) + "#";
  if (cmd == ":Gr#") return hms(mount::targetRaDeg(), lowPrecision) + "#";
  if (cmd == ":Gd#") return dms(mount::targetDecDeg(), lowPrecision) + "#";
  if (cmd == ":GA#" || cmd == ":GZ#") {
    double alt, az;
    altAz(alt, az);
    if (lowPrecision) return cmd == ":GA#" ? site(alt, 2, false) + "#" : site(az, 3, false).substring(1) + "#";
    // sDD*MM:SS# / DDD*MM:SS#
    long t = lround((cmd == ":GA#" ? fabs(alt) : az) * 3600.0);
    char b[20];
    if (cmd == ":GA#") snprintf(b, sizeof(b), "%c%02ld*%02ld:%02ld#", alt < 0 ? '-' : '+', t / 3600, t / 60 % 60, t % 60);
    else snprintf(b, sizeof(b), "%03ld*%02ld:%02ld#", t / 3600 % 360, t / 60 % 60, t % 60);
    return b;
  }
  if (cmd == ":GU#") return statusLetters() + "#";
  if (cmd == ":Gm#") return String(mount::pierSide()) + "#";
  if (cmd == ":GS#") return hms(ra::lst(), false) + "#";
  if (cmd == ":GT#") return ra::state().tracking ? String(ra::trackRateHz(), 5) + "#" : "0#";
  if (cmd == ":GVP#") return "On-Step#";
  if (cmd == ":GVN#") return "10.26a#";
  if (cmd == ":GVD#") return String(__DATE__) + "#";
  if (cmd == ":GVT#") return String(__TIME__) + "#";
  if (cmd == ":Gc#") return "24#";
  if (cmd == ":GM#" || cmd == ":GN#" || cmd == ":GO#" || cmd == ":GP#") return "Site " + String(c[2] - 'M' + 1) + "#";
  if (cmd == ":Gt#") return site(settings.lat, 2, false) + "#";
  if (cmd == ":GtH#") return site(settings.lat, 2, true) + "#";
  if (cmd == ":Gg#") return site(-settings.lonEast, 3, false) + "#";  // Meade: west positive
  if (cmd == ":GgH#") return site(-settings.lonEast, 3, true) + "#";
  if (cmd == ":GG#") {
    // local + value = UTC
    char b[16];
    double o = -settings.utcOffset;
    long m = lround(fabs(o) * 60);
    snprintf(b, sizeof(b), "%c%02ld:%02ld#", o < 0 ? '-' : '+', m / 60, m % 60);
    return b;
  }
  if (cmd == ":GL#" || cmd == ":Ga#") return localTime("%H:%M:%S") + "#";
  if (cmd == ":GC#") return localTime("%m/%d/%y") + "#";
  // backlash in arc-seconds
  if (cmd == ":%BD#") return String(lround(settings.decBacklash * 360.0 * 3600.0 / astro::DEC_STEPS_PER_REV)) + "#";
  if (cmd == ":%BR#") return "0#";
  if (cmd == ":GX90#") return String(settings.guideRate, 2) + "#";
  // axis limits, degrees: RA = the register window in axis hour angle, DEC = mechanical
  if (cmd == ":GXEe#") return String(lround(settings.raEastLimit)) + "#";
  if (cmd == ":GXEw#") return String(lround(ra::trackMax() - ra::offset())) + "#";
  if (cmd == ":GXEB#") return String(lround((ra::trackMax() - ra::offset()) / 15.0)) + "#";
  if (cmd == ":GXEC#") return String(lround(settings.decAxisMin)) + "#";
  if (cmd == ":GXED#") return String(lround(settings.decAxisMax)) + "#";
  if (cmd == ":GX95#") return "0#";  // no automatic meridian flip: the client re-slews
  if (cmd == ":GX96#") return "B#";  // preferred pier side: best (the GoTo window decides)
  if (cmd == ":GXE9#") return String(minutesPastMeridianE()) + "#";
  if (cmd == ":GXEA#") return String(minutesPastMeridianW()) + "#";
  if (cmd == ":Gh#") return String(settings.horizonLimit < 0 ? "-" : "+") + (abs(settings.horizonLimit) < 10 ? "0" : "") +
                            String(abs(settings.horizonLimit)) + "*#";
  if (cmd == ":Go#") return String(settings.overheadLimit) + "*#";
  // PEC: worm geometry (register counts are OnStep's "steps")
  if (cmd == ":GXE6#") return String(ra::pecInfo().countsPerSecond, 6) + "#";  // steps per sidereal second
  if (cmd == ":GXE7#") return String(ra::pecInfo().wormCounts) + "#";          // steps per worm turn
  if (cmd == ":GXE8#") return String(ra::pecInfo().segments) + "#";            // buffer size in seconds
  if (cmd == ":GX98#") return "N#";   // no rotator
  return "0";
}

static String set(const String &cmd, const char *c) {
  double v;
  int a, b, y;
  const char *p = c + 3;
  if (c[2] == 'X' && c[3] == 'E' && strchr("ewCD", c[4]) && c[5] == ',') {
    // :SXEe,n# :SXEw,n# RA axis limits (axis hour angle, deg); :SXEC,n# :SXED,n# DEC axis
    double n = atof(c + 6);
    switch (c[4]) {
      case 'e':
        if (n < -90 || n > 0) return "0";
        ra::setEastLimit(n);
        return "1";
      case 'w':
        if (n < 90 || n > 270) return "0";
        settings.raWestMinutes = constrain(lround((n - 180) * 4), 0L, 240L);
        break;
      case 'C':
        if (n < -180 || n >= settings.decAxisMax) return "0";
        settings.decAxisMin = n;
        break;
      case 'D':
        if (n > 180 || n <= settings.decAxisMin) return "0";
        settings.decAxisMax = n;
        break;
    }
    settingsSave();
    mount::applyDecSettings();
    return "1";
  }
  if (c[2] == 'X' && c[3] == '9' && c[4] == '0' && c[5] == ',') {  // :SX90,n.n#  guide rate (x sidereal)
    double r = atof(c + 6);
    if (r < 0.1 || r > 0.9) return "0";
    settings.guideRate = r;
    settingsSave();
    mount::applyDecSettings();
    ra::ratesChanged();
    return "1";
  }
  if (c[2] == 'T') {  // :ST[H.H]#  tracking rate in Hz (60.16427 = sidereal)
    double hz = atof(p);
    if (hz < 50 || hz > 70) return "0";
    ra::setTrackRate(hz);
    return "1";
  }
  if (c[2] == 'X' && c[3] == 'E' && (c[4] == '9' || c[4] == 'A') && c[5] == ',') {
    // :SXE9,n# / :SXEA,n#  minutes past the meridian, east / west
    long n = atol(c + 6);
    if (n < 0 || n > 360) return "0";
    if (c[4] == '9') ra::setEastLimit(-n / 4.0);  // shifts the register window (saved)
    else {
      settings.raWestMinutes = min(n, 240L);  // physical: camera ~40 deg below horizontal at 4 h
      settingsSave();
    }
    return "1";
  }
  switch (c[2]) {
    case 'h': {  // :ShsDD#  horizon limit
      int d = atoi(p);
      if (*p != '+' && *p != '-' && (*p < '0' || *p > '9')) return "0";
      if (d < -30 || d > 30) return "0";
      settings.horizonLimit = d;
      settingsSave();
      return "1";
    }
    case 'o': {  // :SoDD#  overhead limit
      int d = atoi(p);
      if (d < 60 || d > 90) return "0";
      settings.overheadLimit = d;
      settingsSave();
      return "1";
    }
    case 'r':
      if (!astro::parseOnStepRa(p, v)) return "0";
      mount::setTargetRa(v);
      return "1";
    case 'd':
      if (!astro::parseOnStepDec(p, v)) return "0";
      mount::setTargetDec(v);
      return "1";
    case 't':
      if (!astro::parseOnStepLat(p, v)) return "0";
      settings.lat = v;
      settingsSave();
      return "1";
    case 'g':
      if (!astro::parseOnStepLon(p, v)) return "0";
      settings.lonEast = v;
      settingsSave();
      return "1";
    case 'G':
      if (!astro::parseOnStepUtcOffset(p, v)) return "0";
      settings.utcOffset = -v;
      settingsSave();
      return "1";
    case 'L':
      if (!astro::parseOnStepTime(p, a, b, y)) return "0";
      clockLx200Time(a, b, y);
      return "1";
    case 'C':
      if (!astro::parseOnStepDate(p, a, b, y)) return "0";
      clockLx200Date(a, b, y);
      return "1";
  }
  return "0";
}

String process(const String &cmd) {
  const char *c = cmd.c_str();
  if (cmd.length() < 3 || c[0] != ':' || !cmd.endsWith("#")) return "0";
  xSemaphoreTake(lock, portMAX_DELAY);
  String r = "0";  // OnStepX: unknown / unsupported

  switch (c[1]) {
    case 'G':
    case '%':
      r = get(cmd, c);
      break;
    case 'S':
      r = set(cmd, c);
      break;
    case 'M':
      if (cmd == ":MS#") {
        r = String(mount::gotoTarget());  // 0 started, 1 below horizon, 2 above overhead, 9 no target
      } else if (c[2] == 'g' && strchr("nsew", c[3])) {
        mount::pulse(c[3], atoi(c + 4));  // :Mgn0500#
        r = "";
      } else if (strchr("nsew", c[2]) && c[3] == '#') {
        mount::move(c[2]);
        r = "";
      }
      break;
    case 'Q':
      if (c[2] == '#') mount::stop();
      else mount::stopAxis(c[2]);
      r = "";
      break;
    case 'C':
      if (cmd == ":CM#") r = mount::syncTarget() ? "N/A#" : "E6#";  // E6 = outside limits
      if (cmd == ":CS#") {
        mount::syncTarget();
        r = "";
      }
      break;
    case 'D':
      if (cmd == ":D#") r = mount::busy() ? String((char)0x7f) + "#" : "#";
      break;
    case '$':
      if (c[2] == 'B' && (c[3] == 'D' || c[3] == 'R')) {  // :$BD[n]# / :$BR[n]#  backlash in arcsec
        long arcsec = atol(c + 4);
        if (c[3] == 'R') {
          r = arcsec == 0 ? "1" : "0";  // the Star Adventurer's RA needs none
        } else if (arcsec >= 0 && arcsec <= 3600) {
          settings.decBacklash = lround(arcsec * astro::DEC_STEPS_PER_REV / (360.0 * 3600.0));
          settingsSave();
          dec::setBacklash(settings.decBacklash);
          r = "1";
        }
        break;
      }
      if (c[2] == 'Q' && c[3] == 'Z' && c[5] == '#') {
        if (c[4] == '?') {
          ra::PecInfo pi = ra::pecInfo();
          r = String("IpPrR"[(int)pi.state]) + (pi.indexKnown ? "." : "") + "#";
        } else if (strchr("+-/Z!", c[4])) {
          ra::pecCommand(c[4]);
          r = "";
        }
      }
      break;
    case 'T':
      if (cmd == ":TQ#" || cmd == ":TL#" || cmd == ":TS#" || cmd == ":TK#") {
        ra::setTrackRate(c[2] == 'Q' ? ra::SIDEREAL_HZ : c[2] == 'L' ? ra::LUNAR_HZ : c[2] == 'S' ? ra::SOLAR_HZ : ra::KING_HZ);
        r = "1";
        break;
      }
      if (cmd == ":Tr#" || cmd == ":Tn#") {  // refraction compensation on / off
        settings.refraction = c[2] == 'r';
        settingsSave();
        ra::ratesChanged();
        r = "1";
        break;
      }
      if (cmd == ":T1#") r = "1";  // single axis: the only mode
      // :T2# dual axis, :To# full model: not supported -> "0"
      if (cmd == ":Te#") {
        mount::setTracking(true);
        r = "1";
      } else if (cmd == ":Td#") {
        mount::setTracking(false);
        r = "1";
      }
      break;
    case 'U':
      if (cmd == ":U#") {
        lowPrecision = !lowPrecision;
        r = "";
      }
      break;
    case 'R':  // :R0#..:R9# manual-move rate; Meade :RG :RC :RM :RS
      if (c[2] >= '0' && c[2] <= '9' && c[3] == '#') mount::setMoveRate(c[2] - '0');
      else if (cmd == ":RG#") mount::setMoveRate(2);
      else if (cmd == ":RC#") mount::setMoveRate(5);
      else if (cmd == ":RM#") mount::setMoveRate(6);
      else if (cmd == ":RS#") mount::setMoveRate(9);
      r = "";
      break;
    case 'V': {
      ra::PecInfo pi = ra::pecInfo();
      if (cmd == ":VW#") r = String(pi.wormCounts) + "#";
      else if (cmd == ":VS#") r = String(pi.countsPerSecond, 3) + "#";
      else if (cmd == ":VH#") r = "0#";  // index = segment 0
      else if (cmd == ":VR#") r = String(lround(ra::pecEntry(pi.segment))) + "," + String(pi.segment) + "#";
      else if (c[2] == 'R') r = String(lround(ra::pecEntry(atoi(c + 3)))) + "#";  // :VR[n]#
      break;
    }
    case 'W':
      if (c[2] == 'R') {  // :WR[n,sn]#  write a table entry (counts)
        const char *comma = strchr(c, ',');
        if (comma) {
          ra::setPecEntry(atoi(c + 3), atof(comma + 1));
          r = "";
        }
      }
      break;
    case 'h':
      if (cmd == ":hR#") r = "1";  // unpark: never parked
      // :hP# park, :hQ# set park, :hF#/:hC# home: not supported -> "0"
      break;
  }
  xSemaphoreGive(lock);
  return r;
}

// ---------------------------------------------------------------- TCP servers

static void serverTask(void *) {
  WiFiServer servers[2] = {WiFiServer(settings.lx200Port), WiFiServer(ONSTEP_PORT)};
  for (auto &s : servers) {
    s.begin();
    s.setNoDelay(true);
  }
  logf("onstep: listening on ports %u and %u", settings.lx200Port, ONSTEP_PORT);
  WiFiClient clients[MAX_CLIENTS];
  String buf[MAX_CLIENTS];

  while (true) {
    for (auto &s : servers) {
      if (!s.hasClient()) continue;
      WiFiClient c = s.accept();
      int slot = -1;
      for (int i = 0; i < MAX_CLIENTS; i++)
        if (!clients[i].connected()) {
          slot = i;
          break;
        }
      if (slot < 0) {
        c.stop();
      } else {
        clients[slot] = c;
        clients[slot].setNoDelay(true);
        buf[slot] = "";
        logf("onstep: client %s connected", c.remoteIP().toString().c_str());
      }
    }
    int n = 0;
    for (int i = 0; i < MAX_CLIENTS; i++) {
      if (!clients[i].connected()) continue;
      n++;
      while (clients[i].available()) {
        char ch = clients[i].read();
        if (ch == 0x06) {  // ACK: mount type -> "G" (German equatorial)
          clients[i].write('G');
          continue;
        }
        // Commands start with ':' but also contain it (":Sr05:35:17#"): only start
        // collecting at a ':' when idle, and drop any stray bytes before it
        if (!buf[i].length() && ch != ':') continue;
        buf[i] += ch;
        if (ch == '#') {
          String reply = process(buf[i]);
          if (reply.length()) clients[i].write(reply.c_str(), reply.length());
          buf[i] = "";
        } else if (buf[i].length() > 64) {
          buf[i] = "";
        }
      }
    }
    nClients = n;
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void begin() {
  lock = xSemaphoreCreateMutex();
  xTaskCreatePinnedToCore(serverTask, "onstep", 6144, nullptr, 4, nullptr, 1);
}

}  // namespace onstep
