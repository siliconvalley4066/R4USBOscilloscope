#include "pwm.h"

#define PWMPin 10
PwmOut pwm(PWMPin);

byte duty = 128;  // duty ratio = duty/256
byte p_range = 0;
unsigned short count;
const long range_min[1] PROGMEM = {1};
const long range_div[1] PROGMEM = {1};

double pulse_frq(void) {          // 31Hz <= freq <= 1MHz
  long divide = range_div[p_range];
  return(2000000.0d / ((double)((long)count + 1) * (double)divide));
}

void set_pulse_frq(float freq) {
  if (freq > 1000000.0f) freq = 1000000.0f;
  float fduty = duty*100.0/256.0;
  pwm.end();
  pwm.begin(freq, fduty);
}

void pulse_init() {
  long divide;
  p_range = 0;  // constrain(p_range, 0, 16);
  divide = range_div[p_range];
  float fduty = duty*100.0/256.0;
  pwm.begin((float)pulse_frq(), fduty);
}

void update_frq(int diff) {
  int fast;
  long newCount;

  if (abs(diff) > 3) {
    fast = 512;
  } else if (abs(diff) > 2) {
    fast = 128;
  } else if (abs(diff) > 1) {
    fast = 25;
  } else {
    fast = 1;
  }
  newCount = (long)count + fast * diff;

  if (newCount < range_min[p_range]) {
    if (p_range < 1) {
      newCount = 1;
    } else {
      --p_range;
      newCount = 65535;
    }
  } else if (newCount > 65535) {
    if (p_range < 0) {
      ++p_range;
      newCount = range_min[p_range];
    } else {
      newCount = 65535;
    }
  }
  count = newCount;
  float fduty = duty*100.0/256.0;
  pwm.end();
  pwm.begin((float)pulse_frq(), fduty);
}

void pulse_start(void) {
  pulse_init();
}

void pulse_close(void) {
  pwm.end();
  pinMode(PWMPin, INPUT);
}
