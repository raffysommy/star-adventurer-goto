#pragma once
#include <Arduino.h>

// SkyWatcher motor-controller protocol on top of mountCmd() (":<cmd><axis><data>\r").
// Motion-mode digits follow the user's fix in pysynscan motors.py: first digit
// 1=tracking slow, 3=tracking fast (2/0 = goto, not used); second 0=CW, 1=CCW.
namespace sw {

struct Params {
  long cpr = 0;      // counts per revolution (:a)
  long timerHz = 0;  // T1 timer interrupt frequency (:b)
  long highSpeedRatio = 0;
  bool valid = false;
};

struct Status {
  bool running, ccw, fast, tracking;
};

// Raw command; payload receives the data after '=' (may be null). False on '!' or timeout.
bool cmd(char c, int axis, const char *data = "", String *payload = nullptr);

bool readParams(int axis, Params &p);
bool getPos(int axis, long &counts);  // 0x800000 offset removed
bool setPos(int axis, long counts);   // :E, motor must be stopped
bool status(int axis, Status &s);
bool stopSoft(int axis);              // :K
bool stopHard(int axis);              // :L
bool waitStopped(int axis, uint32_t timeoutMs = 3000);
bool setMode(int axis, bool fast, bool ccw);
bool setT1(int axis, uint32_t t1);
bool start(int axis);                 // :J
bool initDone(int axis);              // :F

// Slow-mode step period for a rate in degrees/second (pysynscan _degreesPerSecond2T1preset)
uint32_t t1ForRate(const Params &p, double degPerSec);
double countsToDeg(const Params &p, long counts);
long degToCounts(const Params &p, double deg);

}  // namespace sw
