#pragma once

#include <math.h>

// Magnus formula, Sonntag 1990 coefficients (valid ~ -45..+60 C, RH 1..100 %)
//   gamma = ln(RH/100) + (b*T) / (c + T)
//   Td    = (c * gamma) / (b - gamma)
static inline float dew_point_magnus_c(float temp_c, float rh_pct) {
  if (rh_pct < 1.0f)   rh_pct = 1.0f;     // log domain guard
  if (rh_pct > 100.0f) rh_pct = 100.0f;

  const float b = 17.625f;
  const float c = 243.04f;

  float gamma = logf(rh_pct / 100.0f) + (b * temp_c) / (c + temp_c);
  return (c * gamma) / (b - gamma);
}
