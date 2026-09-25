#include "dashboard.h"

#include <astro.h>

#include "clock.h"
#include "dec_axis.h"
#include "gps.h"
#include "mount.h"
#include "onstep_server.h"
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
<div class="card"><div>PEC</div><div class="value" id="pec">...</div></div>
<div class="card"><div>GPS</div><div class="value" id="gps">...</div></div>
</div>
<h2>Controls</h2>
<div class="card">
<button onclick="post('/api/stop')">Stop (:Q)</button>
<button onclick="post('/api/time',timeBody())">Set clock from this device</button>
<button onclick="if(confirm('Mount at home (counterweight down, pointing at the pole)?'))post('/api/home')">Set home</button>
<span id="msg"></span>
</div>
<h2>PEC</h2>
<div class="card">
<button onclick="post('/api/lx200',new URLSearchParams({c:':$QZ+#'}))">Play</button>
<button onclick="post('/api/lx200',new URLSearchParams({c:':$QZ-#'}))">Stop</button>
<button onclick="if(confirm('Record one worm turn (~10 min) from the RA guide pulses? Turn PHD2 Predictive PEC off first.'))post('/api/lx200',new URLSearchParams({c:':$QZ/#'}))">Record</button>
<button onclick="if(confirm('Clear the PEC table?'))post('/api/lx200',new URLSearchParams({c:':$QZZ#'}))">Clear</button>
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
<label>RA west tracking limit (minutes past the meridian on the flipped branch) <input type="number" step="1" min="0" max="360" id="ra_west_minutes"></label>
<label>Horizon limit (deg, -30..30) <input type="number" step="1" min="-30" max="30" id="horizon_limit"> Overhead limit (deg, 60..90) <input type="number" step="1" min="60" max="90" id="overhead_limit"></label>
<label><input type="checkbox" id="refraction"> Refraction-compensated tracking</label>
<label><input type="checkbox" id="pec_strict"> PEC starts only before guiding (strict; otherwise after 60 s without pulses)</label>
<label>Guide rate (x sidereal, 0.1..0.9) <input type="number" step="0.05" min="0.1" max="0.9" id="guide_rate"></label>
<label>DEC axis limits (deg, -180..180 = none) <input type="number" step="1" min="-180" max="180" id="dec_axis_min"> .. <input type="number" step="1" min="-180" max="180" id="dec_axis_max"></label>
<label><input type="checkbox" id="gps_enabled"> GPS on UART1 (RX 17 / TX 18, needs a reboot)</label>
<label><input type="checkbox" id="gps_sets_site"> GPS fix sets latitude/longitude</label>
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
$("stalls").textContent=s.stalls+" / "+s.kicks+" / "+s.keep_alives+"  |  "+s.track_hz.toFixed(3)+" Hz, refraction x"+s.refraction_factor.toFixed(5);
$("pec").textContent=["off","ready to play","playing","ready to record","recording"][s.pec.state]+(s.pec.recorded?", recorded":", not recorded")+(s.pec.index?", seg "+s.pec.segment+"/"+s.pec.segments:", phase unknown");
$("gps").textContent=!s.gps.enabled?"disabled":!s.gps.receiving?"no data":(s.gps.pos?s.gps.sats+" sats, "+s.gps.lat.toFixed(4)+" "+s.gps.lon.toFixed(4):"no fix")+(s.gps.time_sync_age>=0?", time "+s.gps.time_sync_age+" s ago":"");
if(!s.clock_valid&&!autoTime){autoTime=true;post("/api/time",timeBody())}
if(!init){$("dec_axis_reversed").checked=s.dec_axis_reversed;$("flip_ra_guiding").checked=s.flip_ra_guiding;$("pier_east").checked=s.pier_east;
$("lat").value=s.lat;$("lon").value=s.lon;$("dec_backlash").value=s.dec_backlash;$("ra_east_limit").value=s.ra_east_limit;$("ra_west_minutes").value=s.ra_west_minutes;$("horizon_limit").value=s.horizon_limit;$("overhead_limit").value=s.overhead_limit;$("gps_enabled").checked=s.gps.enabled;$("gps_sets_site").checked=s.gps_sets_site;$("refraction").checked=s.refraction;$("pec_strict").checked=s.pec_strict;$("guide_rate").value=s.guide_rate;$("dec_axis_min").value=s.dec_axis_min;$("dec_axis_max").value=s.dec_axis_max;init=true}
}catch(e){$("state").textContent="Web connection error"}}
function dec(action,args){post("/api/dec",new URLSearchParams({action,...args}))}
async function saveSettings(){await post("/api/settings",new URLSearchParams({
dec_axis_reversed:$("dec_axis_reversed").checked?"1":"0",flip_ra_guiding:$("flip_ra_guiding").checked?"1":"0",pier_east:$("pier_east").checked?"1":"0",dec_backlash:$("dec_backlash").value,ra_east_limit:$("ra_east_limit").value,ra_west_minutes:$("ra_west_minutes").value,horizon_limit:$("horizon_limit").value,overhead_limit:$("overhead_limit").value,gps_enabled:$("gps_enabled").checked?"1":"0",refraction:$("refraction").checked?"1":"0",pec_strict:$("pec_strict").checked?"1":"0",guide_rate:$("guide_rate").value,dec_axis_min:$("dec_axis_min").value,dec_axis_max:$("dec_axis_max").value,gps_sets_site:$("gps_sets_site").checked?"1":"0",
lat:$("lat").value,lon:$("lon").value}));init=false}
refresh();setInterval(refresh,1000);
</script></body></html>)HTML";

// dashboard_mount_state()
static String mountState(const ra::State &r, const mount::State &l) {
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
  mount::State l = mount::state();
  double lst = ra::lst();
  char lstHms[16];
  astro::formatRa(lst, lstHms, sizeof(lstHms));
  ra::PecInfo pi = ra::pecInfo();
  gps::State g = gps::state();
  char buf[1600];
  snprintf(buf, sizeof(buf),
           "{\"ra\":\"%s\",\"dec\":\"%s\",\"state\":\"%s\",\"phase\":\"%s\",\"meridian_flipped\":%s,"
           "\"dec_axis_reversed\":%s,\"flip_ra_guiding\":%s,\"effective_dec_flip\":%s,"
           "\"axis_ra\":%.6f,\"axis_ha\":%.6f,\"counts\":%ld,\"ra_target\":%.6f,\"lst\":%.6f,\"lst_hms\":\"%s\","
           "\"clock\":%.3f,\"clock_valid\":%s,\"clock_source\":\"%s\",\"utc_offset\":%.2f,\"lat\":%.4f,\"lon\":%.4f,"
           "\"stalls\":%lu,\"kicks\":%lu,\"keep_alives\":%lu,\"lx200_clients\":%d,"
           "\"dec_steps\":%ld,\"dec_target\":%ld,\"dec_motor\":%ld,\"dec_backlash\":%ld,\"ra_east_limit\":%.1f,\"ra_west_minutes\":%ld,\"track_max\":%.2f,\"horizon_limit\":%ld,\"overhead_limit\":%ld,\"gps_sets_site\":%s,\"refraction\":%s,\"refraction_factor\":%.6f,\"track_hz\":%.5f,\"pec_strict\":%s,\"guide_rate\":%.2f,\"dec_axis_min\":%.1f,\"dec_axis_max\":%.1f,\"pec\":{\"state\":%d,\"recorded\":%s,\"index\":%s,\"segment\":%d,\"segments\":%d},\"gps\":{\"enabled\":%s,\"receiving\":%s,\"time\":%s,\"pos\":%s,\"sats\":%d,\"lat\":%.6f,\"lon\":%.6f,\"alt\":%.1f,\"hdop\":%.1f,\"sentences\":%lu,\"bad\":%lu,\"time_sync_age\":%ld},\"dec_moving\":%s,\"pier_east\":%s}",
           mount::reportedRa().c_str(), mount::reportedDec().c_str(), mountState(r, l).c_str(), r.phase,
           l.meridianFlipped ? "true" : "false", settings.decAxisReversed ? "true" : "false",
           settings.flipRaGuiding ? "true" : "false", (l.meridianFlipped ^ settings.decAxisReversed) ? "true" : "false",
           r.axisRa, r.axisHa, r.counts, l.raTarget, lst, lstHms, clockNow(), clockValid() ? "true" : "false",
           clockSource(), settings.utcOffset, settings.lat, settings.lonEast, (unsigned long)r.stalls,
           (unsigned long)r.kicks, (unsigned long)r.keepAlives, onstep::clients(), dec::position(), dec::target(),
           dec::motorPosition(), (long)settings.decBacklash, settings.raEastLimit, (long)settings.raWestMinutes, ra::trackMax(),
           (long)settings.horizonLimit, (long)settings.overheadLimit, settings.gpsSetsSite ? "true" : "false", settings.refraction ? "true" : "false", ra::refractionFactor(),
           ra::trackRateHz(), settings.pecStrict ? "true" : "false", settings.guideRate, settings.decAxisMin, settings.decAxisMax, (int)pi.state,
           pi.recorded ? "true" : "false", pi.indexKnown ? "true" : "false", pi.segment, pi.segments, g.enabled ? "true" : "false",
           g.receiving ? "true" : "false", g.timeValid ? "true" : "false", g.posValid ? "true" : "false", g.sats, g.lat, g.lon,
           g.altM, g.hdop, (unsigned long)g.sentences, (unsigned long)g.badChecksums, (long)g.lastTimeSyncAgeS, dec::moving() ? "true" : "false", settings.pierEast ? "true" : "false");
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
  if (srv->hasArg("ra_west_minutes") && srv->arg("ra_west_minutes").length())
    settings.raWestMinutes = constrain(srv->arg("ra_west_minutes").toInt(), 0, 360);
  if (srv->hasArg("horizon_limit") && srv->arg("horizon_limit").length())
    settings.horizonLimit = constrain(srv->arg("horizon_limit").toInt(), -30, 30);
  if (srv->hasArg("overhead_limit") && srv->arg("overhead_limit").length())
    settings.overheadLimit = constrain(srv->arg("overhead_limit").toInt(), 60, 90);
  if (srv->hasArg("gps_enabled")) settings.gpsEnabled = srv->arg("gps_enabled") == "1";
  if (srv->hasArg("refraction")) settings.refraction = srv->arg("refraction") == "1";
  if (srv->hasArg("pec_strict")) settings.pecStrict = srv->arg("pec_strict") == "1";
  if (srv->hasArg("guide_rate") && srv->arg("guide_rate").length())
    settings.guideRate = constrain(srv->arg("guide_rate").toDouble(), 0.1, 0.9);
  if (srv->hasArg("dec_axis_min") && srv->hasArg("dec_axis_max") && srv->arg("dec_axis_min").length() &&
      srv->arg("dec_axis_min").toDouble() < srv->arg("dec_axis_max").toDouble()) {
    settings.decAxisMin = constrain(srv->arg("dec_axis_min").toDouble(), -180.0, 180.0);
    settings.decAxisMax = constrain(srv->arg("dec_axis_max").toDouble(), -180.0, 180.0);
  }
  mount::applyDecSettings();
  ra::ratesChanged();
  if (srv->hasArg("gps_sets_site")) settings.gpsSetsSite = srv->arg("gps_sets_site") == "1";
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
    onstep::process(":Q#");
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
      onstep::process(c);
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
    srv->send(200, "text/plain", onstep::process(c));
  });
}
