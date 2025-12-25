#ifndef POWER_MANAGEMENT_H
#define POWER_MANAGEMENT_H

#include "setting.h"
#include "tools.h"

#include "m5_stamp_pm1.h"
#include "m5_stamp_pm1_class.h"

#include "m5_io_py32ioexpander.h"

extern m5_io_py32ioexpander io_expander;

enum class WakeupMode
{
    WAKEUP_DEEPSLEEP,
    WAKEUP_SHUTDOWN,
};

enum class WakeupDevice
{
    RTC_WAKEUP,
    IMU_WAKEUP,
    PORT_WAKEUP,
};

void pmic_init();

void py32_io_expander_init();

void stop_watch_power_mode_L0();
void stop_watch_power_mode_L1();
void stop_watch_power_mode_L2();
void stop_watch_power_mode_L3A();
void stop_watch_power_mode_L3B();

void stop_watch_speaker_set(bool enable);

void wakeup_test(WakeupDevice wakeup_device, WakeupMode wakeup_mode);

#endif // POWER_MANAGEMENT_H