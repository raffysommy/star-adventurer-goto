#include "dashboard.h"

#include <astro.h>

#include "clock.h"
#include "dec_axis.h"
#include "lx200_server.h"
#include "ra_axis.h"
#include "settings.h"

static WebServer *srv;

static const char DASHBOARD_HTML[] PROGMEM = R"HTML(<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Star Adventurer</title>
<style>
body{font-family:sans-serif;max-width:900px;margin:30px auto;padding:0 15px;background:#111;color:#eee}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(180px,1fr));gap:12px}
.card{background:#222;padding:16px;border-radius:8px}
.value{margin-top:6px;font-family:monospace;font-size:1.3em;overflow-wrap:anywhere}
label{display:block;margin:15px 0}button{padding:10px 18px;cursor:pointer;margin:4px 4px 4px 0}
.true{color:#ffb74d}.false{color:#81c784}.warn{color:#ff7070}a{color:#8cf}
input[type=number]{width:9em}
</style></head><body>
<h1>Star Adventurer</h1>
<p><a href="/sys">system</a> | <a href="/log">log</a> | <a href="/update">firmware</a></p>
<div class="grid">
<div class="card"><div>Current RA</div><div class="value" id="ra">...</div></div>
<div class="card"><div>Current DEC</div><div class="value" id="dec">...</div></div>
<div class="card"><div>DEC steps / target</div><div class="value" id="decsteps">...</div></div>
<div class="card"><div>Mount state</div><div class="value" id="state">...</div></div>
<div class="card"><div>Meridian flipped</div><div class="value" id="flipped">...</div></div>
<div class="card"><div>Effective DEC flip</div><div class="value" id="decflip">...</div></div>
<div class="card"><div>Axis HA / register</div><div class="value" id="ha">...</div></div>
<div class="card"><div>LST</div><div class="value" id="lst">...</div></div>
<div class="card"><div>Clock (UTC)</div><div class="value" id="clock">...</div></div>
<div class="card"><div>Stalls / kicks / keep-alives</div><div class="value" id="stalls">...</div></div>
</div>
<h2>Controls</h2>
<div class="card">
<button onclick="post('/api/stop')">Stop (:Q)</button>
<button onclick="post('/api/time',timeBody())">Set clock from this device</button>
<button onclick="if(confirm('Mount at home (counterweight down, pointing at the pole)?'))post('/api/home')">Set home</button>
<span id="msg"></span>
</div>
<h2>DEC motor test</h2>
<div class="card">
<button onclick="dec('guide',{dir:'n',ms:2000})">Guide N 2 s</button>
<button onclick="dec('guide',{dir:'s',ms:2000})">Guide S 2 s</button>
<button onclick="dec('move',{steps:813})">+1&deg;</button>
<button onclick="dec('move',{steps:-813})">-1&deg;</button>
<button onclick="dec('move',{steps:8133})">+10&deg;</button>
<button onclick="dec('move',{steps:-8133})">-10&deg;</button>
<button onclick="dec('stop',{})">Stop DEC</button>
</div>
<h2>Runtime settings</h2>
<div class="card">
<label><input type="checkbox" id="dec_axis_reversed"> DEC_AXIS_REVERSED</label>
<label><input type="checkbox" id="flip_ra_guiding"> FLIP_RA_GUIDING_ON_MERIDIAN</label>
<label><input type="checkbox" id="pier_east"> PIER_EAST_SIDE (inverts DEC motor)</label>
<label>DEC backlash (steps) <input type="number" step="1" min="0" id="dec_backlash"></label>
<label>RA east limit (deg, 0 = flip at the meridian, -30 = flip 2 h before; check lens clearance first) <input type="number" step="1" min="-90" max="0" id="ra_east_limit"></label>
<label>Latitude <input type="number" step="0.0001" id="lat"> Longitude (east +) <input type="number" step="0.0001" id="lon"></label>
<button onclick="saveSettings()">Apply settings</button>
</div>
<script>
let init=false,autoTime=false;
const $=id=>document.getElementById(id);
function bool(id,v){$(id).textContent=v?"TRUE":"FALSE";$(id).className="value "+(v?"true":"false")}
function timeBody(){return new URLSearchParams({unix:(Date.now()/1000).toFixed(3),tz_minutes:new Date().getTimezoneOffset()})}
async function post(url,body){try{const r=await fetch(url,{method:"POST",body});$("msg").textContent=r.ok?"OK":"Error";}
catch(e){$("msg").textContent="Error"}setTimeout(()=>$("msg").textContent="",2000);refresh()}
async function refresh(){try{
const s=await(await fetch("/api/status",{cache:"no-store"})).json();
$("ra").textContent=s.ra;$("dec").textContent=s.dec;
$("decsteps").textContent=s.dec_steps+" / "+s.dec_target+(s.dec_moving?" (moving)":"");$("state").textContent=s.state;
bool("flipped",s.meridian_flipped);bool("decflip",s.effective_dec_flip);
$("ha").textContent=s.axis_ha.toFixed(4)+"° / "+s.counts;
$("lst").textContent=s.lst_hms+" ("+s.lst.toFixed(3)+"°)";
$("clock").textContent=(s.clock_valid?new Date(s.clock*1000).toISOString().substr(11,8):"NOT SET")+" ["+s.clock_source+"]";
$("clock").className="value"+(s.clock_valid?"":" warn");
$("stalls").textContent=s.stalls+" / "+s.kicks+" / "+s.keep_alives;
if(!s.clock_valid&&!autoTime){autoTime=true;post("/api/time",timeBody())}
if(!init){$("dec_axis_reversed").checked=s.dec_axis_reversed;$("flip_ra_guiding").checked=s.flip_ra_guiding;$("pier_east").checked=s.pier_east;
$("lat").value=s.lat;$("lon").value=s.lon;$("dec_backlash").value=s.dec_backlash;$("ra_east_limit").value=s.ra_east_limit;init=true}
}catch(e){$("state").textContent="Web connection error"}}
function dec(action,args){post("/api/dec",new URLSearchParams({action,...args}))}
async function saveSettings(){await post("/api/settings",new URLSearchParams({
dec_axis_reversed:$("dec_axis_reversed").checked?"1":"0",flip_ra_guiding:$("flip_ra_guiding").checked?"1":"0",pier_east:$("pier_east").checked?"1":"0",dec_backlash:$("dec_backlash").value,ra_east_limit:$("ra_east_limit").value,
lat:$("lat").value,lon:$("lon").value}));init=false}
refresh();setInterval(refresh,1000);
</script></body></html>)HTML";

// dashboard_mount_state()
static String mountState(const ra::State &r, const lx200::State &l) {
  if (!r.connected && !dec::moving()) return "RA MOUNT NOT CONNECTED";
  if (!strcmp(r.phase, "limit")) return "STOPPED AT RA LIMIT";
  if (r.slewing) return strcmp(r.phase, "approach") ? "SLEWING" : "SLEWING (approach)";
  if (dec::slewing()) return "SLEWING (DEC)";
  String dirs;
  if (l.guideNorth) dirs += "N/";
  if (l.guideSouth) dirs += "S/";
  if (r.guideEast) dirs += "E/";
  if (r.guideWest) dirs += "W/";
  if (dirs.length()) return "GUIDING " + dirs.substring(0, dirs.length() - 1);
  return "TRACKING";
}

static void sendStatus() {
  ra::State r = ra::state();
  lx200::State l = lx200::state();
  double lst = ra::lst();
  char lstHms[16];
  astro::formatRa(lst, lstHms, sizeof(lstHms));
  char buf[1100];
  snprintf(buf, sizeof(buf),
           "{\"ra\":\"%s\",\"dec\":\"%s\",\"state\":\"%s\",\"phase\":\"%s\",\"meridian_flipped\":%s,"
           "\"dec_axis_reversed\":%s,\"flip_ra_guiding\":%s,\"effective_dec_flip\":%s,"
           "\"axis_ra\":%.6f,\"axis_ha\":%.6f,\"counts\":%ld,\"ra_target\":%.6f,\"lst\":%.6f,\"lst_hms\":\"%s\","
           "\"clock\":%.3f,\"clock_valid\":%s,\"clock_source\":\"%s\",\"utc_offset\":%.2f,\"lat\":%.4f,\"lon\":%.4f,"
           "\"stalls\":%lu,\"kicks\":%lu,\"keep_alives\":%lu,\"lx200_clients\":%d,"
           "\"dec_steps\":%ld,\"dec_target\":%ld,\"dec_motor\":%ld,\"dec_backlash\":%ld,\"ra_east_limit\":%.1f,\"dec_moving\":%s,\"pier_east\":%s}",
           lx200::reportedRa().c_str(), lx200::reportedDec().c_str(), mountState(r, l).c_str(), r.phase,
           l.meridianFlipped ? "true" : "false", settings.decAxisReversed ? "true" : "false",
           settings.flipRaGuiding ? "true" : "false", (l.meridianFlipped ^ settings.decAxisReversed) ? "true" : "false",
           r.axisRa, r.axisHa, r.counts, l.raTarget, lst, lstHms, clockNow(), clockValid() ? "true" : "false",
           clockSource(), settings.utcOffset, settings.lat, settings.lonEast, (unsigned long)r.stalls,
           (unsigned long)r.kicks, (unsigned long)r.keepAlives, l.clients, dec::position(), dec::target(),
           dec::motorPosition(), (long)settings.decBacklash, settings.raEastLimit, dec::moving() ? "true" : "false", settings.pierEast ? "true" : "false");
  srv->send(200, "application/json", buf);
}

static void postSettings() {
  settings.decAxisReversed = srv->arg("dec_axis_reversed") == "1";
  settings.flipRaGuiding = srv->arg("flip_ra_guiding") == "1";
  if (srv->hasArg("pier_east")) {
    settings.pierEast = srv->arg("pier_east") == "1";
    dec::setInverted(settings.pierEast);
  }
  if (srv->hasArg("dec_backlash") && srv->arg("dec_backlash").length()) {
    settings.decBacklash = srv->arg("dec_backlash").toInt();
    dec::setBacklash(settings.decBacklash);
  }
  if (srv->hasArg("ra_east_limit") && srv->arg("ra_east_limit").length() &&
      srv->arg("ra_east_limit").toDouble() != settings.raEastLimit)
    ra::setEastLimit(srv->arg("ra_east_limit").toDouble());  // saves the setting itself
  if (srv->hasArg("lat") && srv->arg("lat").length()) settings.lat = srv->arg("lat").toDouble();
  if (srv->hasArg("lon") && srv->arg("lon").length()) settings.lonEast = srv->arg("lon").toDouble();
  settingsSave();
  srv->send(200, "application/json", "{\"ok\":true}");
}

static void postTime() {
  double unix = srv->arg("unix").toDouble();
  if (unix < 1700000000) {
    srv->send(400, "text/plain", "bad time\n");
    return;
  }
  if (srv->hasArg("tz_minutes")) {  // JS getTimezoneOffset(): UTC - local, in minutes
    settings.utcOffset = -srv->arg("tz_minutes").toInt() / 60.0;
    settingsSave();
  }
  bool applied = clockSet(unix, "browser");
  srv->send(200, "application/json", applied ? "{\"applied\":true}" : "{\"applied\":false}");
}

void dashboardBegin(WebServer &web) {
  srv = &web;
  web.on("/", [] { srv->send_P(200, "text/html", DASHBOARD_HTML); });
  web.on("/api/status", sendStatus);
  web.on("/api/settings", HTTP_POST, postSettings);
  web.on("/api/time", HTTP_POST, postTime);
  web.on("/api/home", HTTP_POST, [] {
    ra::setHome();
    srv->send(200, "application/json", "{\"ok\":true}");
  });
  web.on("/api/stop", HTTP_POST, [] {
    lx200::process(":Q#");
    srv->send(200, "application/json", "{\"ok\":true}");
  });
  web.on("/api/register", HTTP_POST, [] {
    ra::setRegister(srv->arg("deg").toDouble());
    srv->send(200, "application/json", "{\"ok\":true}");
  });
  web.on("/api/goto_ha", HTTP_POST, [] {
    ra::gotoHa(srv->arg("ha").toDouble());
    srv->send(200, "application/json", "{\"ok\":true}");
  });
  web.on("/api/dec", HTTP_POST, [] {
    String a = srv->arg("action");
    if (a == "move") {
      dec::setTarget(dec::position() + srv->arg("steps").toInt());
      dec::slew();
    } else if (a == "goto") {
      dec::setTarget(astro::decToSteps(srv->arg("deg").toDouble()));
      dec::slew();
    } else if (a == "guide") {
      String c = ":Mg" + srv->arg("dir") + srv->arg("ms") + "#";
      lx200::process(c);
    } else if (a == "stop") {
      dec::stop();
    } else {
      srv->send(400, "text/plain", "action: move&steps= | goto&deg= | guide&dir=n|s&ms= | stop\n");
      return;
    }
    srv->send(200, "application/json", "{\"ok\":true}");
  });
  web.on("/api/lx200", [] {
    String c = srv->arg("c");
    if (!c.endsWith("#")) c += "#";
    srv->send(200, "text/plain", lx200::process(c));
  });
}
