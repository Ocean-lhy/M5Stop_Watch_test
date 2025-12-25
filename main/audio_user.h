#ifndef AUDIO_USER_H
#define AUDIO_USER_H

#include "driver/i2s_std.h"
// #include "driver/i2s_tdm.h"
#include "soc/soc_caps.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"

#include "i2c_bus.h"

#include "setting.h"

#ifndef CHECK
#define CHECK(r) if (!(r)) return -1;
#endif
#ifndef TEST_ESP_ERR
#define TEST_ESP_ERR(rc, res) CHECK((rc) == (res))
#endif
#ifndef TEST_ESP_OK
#define TEST_ESP_OK(rc) CHECK((rc) == ESP_OK)
#endif


extern void es8311_init();

#endif // AUDIO_USER_H