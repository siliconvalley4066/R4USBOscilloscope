#include "pwm.h"

#define PWMPin 10
PwmOut pwm(PWMPin);

byte duty = 128;  // duty ratio = duty/256
byte p_range = 0;
unsigned short count;
const long range_min[6] PROGMEM = {1, 16383, 16383, 16383, 16383, 16383};
const long range_div[6] PROGMEM = {1, 4, 16, 64, 256, 1024};
const timer_source_div_t source_div[6] PROGMEM = {TIMER_SOURCE_DIV_1, TIMER_SOURCE_DIV_4,
  TIMER_SOURCE_DIV_16, TIMER_SOURCE_DIV_64, TIMER_SOURCE_DIV_256, TIMER_SOURCE_DIV_1024};

double pulse_frq(void) {          // 0.715Hz <= freq <= 24MHz
  long divide = range_div[p_range];
  return(sys_clk / ((double)((long)count + 1) * (double)divide));
}

void set_pulse_frq(float freq) {
  if (freq > (float)(sys_clk / 2)) freq = sys_clk / 2;
  p_range = constrain(5 - int(6.0 - log(sys_clk / 16384.0 / freq)/log(4)), 0, 5);
  long divide = range_div[p_range];
  count = (float)sys_clk/freq/(float)divide - 1;
  uint32_t pulse = map((unsigned short)duty, 0, 255, 0, count);
  pwm.end();
  pwm.begin(count, pulse, true, source_div[p_range]);
//  pwm.begin(freq, duty*100.0/256.0);
}

uint32_t pulsew(void) {
  if (count < 2)
    return 1;
  else
    return map((unsigned short)duty, 0, 255, 0, count);
}

void pulse_init() {
  long divide;
  p_range = constrain(p_range, 0, 5);
  divide = range_div[p_range];
//  float fduty = duty*100.0/256.0;
//  pwm.begin((float)pulse_frq(), fduty);
  pwm.begin(count + 1, pulsew(), true, source_div[p_range]);
}

void update_frq(int diff) {
  int fast;
  long divide, newCount;

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
    if (p_range < 5) {
      ++p_range;
      newCount = range_min[p_range];
    } else {
      newCount = 65535;
    }
  }
  divide = range_div[p_range];
  count = newCount;
  pwm.end();
//  float fduty = duty*100.0/256.0;
//  pwm.begin((float)pulse_frq(), fduty);
  pwm.begin(count + 1, pulsew(), true, source_div[p_range]);
}

void pulse_start(void) {
  pulse_init();
}

void pulse_close(void) {
  pwm.end();
  pinMode(PWMPin, INPUT);
}
