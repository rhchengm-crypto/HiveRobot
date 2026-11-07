#pragma once

// Voltage-based estimate for a single 18650, not a coulomb-counting fuel gauge.
namespace Battery {
void begin();
void tick();
bool valid();
float voltage();
int percent();
bool low();
void report();
}
