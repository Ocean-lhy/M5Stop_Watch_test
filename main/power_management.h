#ifndef POWER_MANAGEMENT_H
#define POWER_MANAGEMENT_H

#include "setting.h"
#include "tools.h"

#include "m5_stamp_pm1.h"
#include "m5_stamp_pm1_class.h"

#include "m5_io_py32ioexpander.h"

extern m5_io_py32ioexpander io_expander;

void pmic_init();

void py32_io_expander_init();

void stop_watch_power_mode_L0();
void stop_watch_power_mode_L1();
void stop_watch_power_mode_L2();
void stop_watch_power_mode_L3A();
void stop_watch_power_mode_L3B();

#endif // POWER_MANAGEMENT_H