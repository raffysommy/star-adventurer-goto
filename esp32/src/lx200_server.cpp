#include "lx200_server.h"

#include <WiFi.h>
#include <astro.h>
#include <esp_timer.h>

#include "clock.h"
#include "dec_axis.h"
#include "netlog.h"
#include "ra_axis.h"
#include "settings.h"

namespace lx200 {

static const int MAX_CLIENTS = 2;

static SemaphoreHandle_t lock;  // process() can be called from the server task and the web UI
static State st = {false, 0, false, false, 0};
static bool targetValid = false;  // no :MS / :CM before a successful :Sr
static esp_timer_handle_t decGuideTimer;

// ---------------------------------------------------------------- helpers

static String fmtRa(double deg) {
  char b[16];
  astro::formatRa(deg, b, sizeof(b));
  return b;
}

String reportedRa() {
  ra::State r = ra::state();
  return fmtRa(astro::reportedRa(r.axisRa, st.meridianFlipped, ra::lst(), ra::OFFSET));
}

static bool reverseDec() { return st.meridianFlipped ^ settings.decAxisReversed; }

// get_dec() / steps_to_coord()
String reportedDec() {
  char b[16];
  astro::formatDec(astro::stepsToDec(dec::position(), reverseDec()), b, sizeof(b));
  return b;
}

// End of a pulse's duration: the step move itself ends on its own
static void decGuideEnd(void *) { st.guideNorth = st.guideSouth = false; }

static String localTime(const char *fmt) {
  time_t t = (time_t)(clockNow() + settings.utcOffset * 3600);
  struct tm tm;
  gmtime_r(&t, &tm);
  char b[16];
  strftime(b, sizeof(b), fmt, &tm);
  return b;
}

static String fmtSite(double deg) {  // "+DD*MM" as lx200.py's :Gt/:Gg
  int d = (int)deg;
  char b[16];
  snprintf(b, sizeof(b), "%+03d*%02.0f", d, fabs(fmod(deg, 1.0) * 60));
  return b;
}

// slow_move(): :Mn :Ms :Me :Mw, or pulse guiding :Mgn1000 etc.
static String slowMove(const String &cmd) {
  String dir = cmd.substring(2);
  bool pulse = dir.startsWith("g");
  if (pulse) dir = dir.substring(1);
  int ms = pulse ? dir.substring(1).toInt() : 0;
  char d = dir.length() ? dir[0] : 0;
  if (d == 'n' || d == 's') {
    st.guideNorth = d == 'n';
    st.guideSouth = d == 's';
    // Beyond the pole (or with DEC_AXIS_REVERSED) north and south swap on the motor
    bool north = (d == 'n') != reverseDec();
    esp_timer_stop(decGuideTimer);
    if (ms > 0) {
      dec::guidePulse(north ? +1 : -1, ms);
      esp_timer_start_once(decGuideTimer, (uint64_t)ms * 1000);  // only clears the :D flags
    } else {
      dec::guide(north ? +1 : -1);  // manual move until :Q
    }
  } else if (d == 'e' || d == 'w') {
    bool swap = st.meridianFlipped && settings.flipRaGuiding;
    char rd = swap ? (d == 'e' ? 'w' : 'e') : d;
    ra::guide(rd, ms);
  }
  return "0";
}

// ---------------------------------------------------------------- commands

String process(const String &cmd) {
  const char *c = cmd.c_str();
  xSemaphoreTake(lock, portMAX_DELAY);
  String r;
  double v;
  int a, b, y;

  if (cmd.startsWith(":GR")) {
    r = reportedRa() + "#";
  } else if (cmd.startsWith(":GD")) {
    r = reportedDec() + "#";
  } else if (cmd.startsWith(":GVP")) {
    r = "Proxino#";
  } else if (cmd.startsWith(":GVN")) {
    r = "1.0#";
  } else if (cmd.startsWith(":GVR")) {
    r = "001012023#";
  } else if (cmd.startsWith(":GVD")) {
    r = "01.1#";
  } else if (cmd.startsWith(":GVT")) {
    r = localTime("%H:%M:%S") + "#";
  } else if (cmd.startsWith(":GVF")) {
    r = "11#";
  } else if (cmd.startsWith(":Sr")) {
    // set_ra(): pick the meridian-flipped target now, as lx200.py does
    if (astro::parseSr(c, v)) {
      astro::RaTarget t = astro::selectTarget(v, ra::lst(), ra::OFFSET);
      st.meridianFlipped = t.flipped;
      st.raTarget = t.ra;
      targetValid = true;
      logf("lx200: target RA %.4f -> axis RA %.4f%s", v, t.ra, t.flipped ? " (meridian flipped)" : "");
      r = "#";
    }
  } else if (cmd.startsWith(":Sd")) {
    // set_dec(): beyond the pole the axis goes to 180 - dec
    if (astro::parseSd(c, v)) {
      long steps = astro::decToSteps(astro::decForSteps(v, reverseDec()));
      dec::setTarget(steps);
      logf("lx200: target DEC %.4f -> %ld steps%s", v, steps, reverseDec() ? " (reversed)" : "");
      r = "#";
    }
  } else if (cmd.startsWith(":MS")) {
    if (!targetValid) {
      logf("lx200: :MS without a target, ignored");
      r = "1No target#";
    } else {
      ra::gotoRa(st.raTarget);
      r = "0";
    }
    dec::slew();
  } else if (cmd.startsWith(":M")) {
    r = slowMove(cmd);
  } else if (cmd.startsWith(":Q")) {
    ra::stop();
    esp_timer_stop(decGuideTimer);
    st.guideNorth = st.guideSouth = false;
    dec::stop();
    r = "";
  } else if (cmd.startsWith(":CM")) {
    if (targetValid) ra::sync(st.raTarget);
    else logf("lx200: :CM without an RA target, RA not synced");
    dec::syncToTarget();
    r = ":Coordinates matched #";
  } else if (cmd.startsWith(":D")) {
    ra::State rs = ra::state();
    bool busy = rs.slewing || rs.guideEast || rs.guideWest || st.guideNorth || st.guideSouth || dec::slewing();
    r = busy ? String((char)127) + "#" : "#";
  } else if (cmd.startsWith(":GM")) {
    r = "Site1Name#";
  } else if (cmd.startsWith(":GT")) {
    r = "60.0#";
  } else if (cmd.startsWith(":Gt")) {
    r = fmtSite(settings.lat) + "#";
  } else if (cmd.startsWith(":Gg")) {
    r = fmtSite(-settings.lonEast) + "#";  // Meade: east negative
  } else if (cmd.startsWith(":Sg")) {
    if (astro::parseSg(c, v)) {
      settings.lonEast = v;
      settingsSave();
      r = "1";
    } else {
      r = "0";
    }
  } else if (cmd.startsWith(":St")) {
    if (astro::parseSt(c, v)) {
      settings.lat = v;
      settingsSave();
      r = "1";
    } else {
      r = "0";
    }
  } else if (cmd.startsWith(":So") || cmd.startsWith(":Sh") || cmd.startsWith(":SG") || cmd.startsWith(":SL") ||
             cmd.startsWith(":SC")) {
    // lx200.py ignored these; the ESP has no other clock, so time commands are applied
    if (astro::parseSG(c, v)) {
      settings.utcOffset = -v;
      settingsSave();
    } else if (astro::parseSL(c, a, b, y)) {
      clockLx200Time(a, b, y);
    } else if (astro::parseSC(c, a, b, y)) {
      clockLx200Date(a, b, y);
    }
    r = "1";
  } else if (cmd.startsWith(":R")) {
    r = "";
  } else if (cmd.startsWith(":GG")) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%+.1f#", -settings.utcOffset);
    r = buf;
  } else if (cmd.startsWith(":Gc")) {
    r = "24#";
  } else if (cmd.startsWith(":GL")) {
    r = localTime("%H:%M:%S") + "#";
  } else if (cmd.startsWith(":GC")) {
    r = localTime("%m/%d/%y") + "#";
  } else {
    r = "Invalid Command";
  }
  xSemaphoreGive(lock);
  return r;
}

State state() {
  xSemaphoreTake(lock, portMAX_DELAY);
  State s = st;
  xSemaphoreGive(lock);
  return s;
}

// ---------------------------------------------------------------- TCP server

static void serverTask(void *) {
  WiFiServer server(settings.lx200Port);
  server.begin();
  server.setNoDelay(true);
  logf("lx200: listening on port %u", settings.lx200Port);
  WiFiClient clients[MAX_CLIENTS];
  String buf[MAX_CLIENTS];

  while (true) {
    if (server.hasClient()) {
      WiFiClient c = server.accept();
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
        logf("lx200: client %s connected", c.remoteIP().toString().c_str());
      }
    }
    int n = 0;
    for (int i = 0; i < MAX_CLIENTS; i++) {
      if (!clients[i].connected()) continue;
      n++;
      while (clients[i].available()) {
        char ch = clients[i].read();
        if (ch == 0x06) {  // ACK: alignment query -> "P" (polar)
          clients[i].write('P');
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
    st.clients = n;
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

void begin() {
  lock = xSemaphoreCreateMutex();
  esp_timer_create_args_t args = {};
  args.callback = decGuideEnd;
  args.name = "dec_guide";
  esp_timer_create(&args, &decGuideTimer);
  xTaskCreatePinnedToCore(serverTask, "lx200", 6144, nullptr, 4, nullptr, 1);
}

}  // namespace lx200
