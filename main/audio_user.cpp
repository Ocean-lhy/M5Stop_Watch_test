#include "audio_user.h"

#include "esp_idf_version.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <stdexcept>

#include "power_management.h"

// demo.pcm
extern "C" const uint8_t _binary_demo_pcm_start[] asm("_binary_demo_pcm_start");
extern "C" const uint8_t _binary_demo_pcm_end[] asm("_binary_demo_pcm_end");

static i2c_master_bus_handle_t es_i2c_bus_handle;
#define I2S_MAX_KEEP SOC_I2S_NUM

static const char *TAG = "audio_user";

typedef struct {
    i2s_chan_handle_t tx_handle;
    i2s_chan_handle_t rx_handle;
} i2s_keep_t;

static i2s_comm_mode_t i2s_in_mode  = I2S_COMM_MODE_STD;
static i2s_comm_mode_t i2s_out_mode = I2S_COMM_MODE_STD;
static i2s_keep_t *i2s_keep[I2S_MAX_KEEP];

static int ut_i2c_init(uint8_t port)
{
    i2c_master_bus_config_t i2c_bus_config = {0};
    i2c_bus_config.clk_source = (i2c_clock_source_t)I2C_FREQ;
    i2c_bus_config.i2c_port = port;
    i2c_bus_config.scl_io_num = I2C_SCL_PIN;
    i2c_bus_config.sda_io_num = I2C_SDA_PIN;
    i2c_bus_config.glitch_ignore_cnt = 7;
    i2c_bus_config.flags.enable_internal_pullup = true;
    return i2c_new_master_bus(&i2c_bus_config, &es_i2c_bus_handle);
}

static int ut_i2c_deinit(uint8_t port)
{
   if (es_i2c_bus_handle) {
       i2c_del_master_bus(es_i2c_bus_handle);
   }
   es_i2c_bus_handle = NULL;
   return 0;
}

static void ut_set_i2s_mode(i2s_comm_mode_t out_mode, i2s_comm_mode_t in_mode)
{
    i2s_in_mode  = in_mode;
    i2s_out_mode = out_mode;
}

static void ut_clr_i2s_mode(void)
{
    i2s_in_mode  = I2S_COMM_MODE_STD;
    i2s_out_mode = I2S_COMM_MODE_STD;
}

static int ut_i2s_init(uint8_t port)
{
    if (port >= I2S_MAX_KEEP) {
        ESP_LOGE(TAG, "I2S port %d is not supported", port);
        return -1;
    }
    // Already installed
    if (i2s_keep[port]) {
        ESP_LOGI(TAG, "I2S already installed");
        return 0;
    }
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    i2s_std_config_t std_cfg   = {
          .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(44100),
          .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
          .gpio_cfg =
            {
                  .mclk = I2S_MCLK_PIN,
                  .bclk = I2S_BCLK_PIN,
                  .ws   = I2S_LRCK_PIN,
                  .dout = I2S_DDAC_OUT_PIN,
                  .din  = I2S_DADC_IN_PIN,
            },
    };
    i2s_keep[port] = (i2s_keep_t *)calloc(1, sizeof(i2s_keep_t));
    if (i2s_keep[port] == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for I2S keep");
        return -1;
    }

    int ret = i2s_new_channel(&chan_cfg, &i2s_keep[port]->tx_handle, &i2s_keep[port]->rx_handle);
    TEST_ESP_OK(ret);
    if (i2s_out_mode == I2S_COMM_MODE_STD) {
        ret = i2s_channel_init_std_mode(i2s_keep[port]->tx_handle, &std_cfg);
        if(ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize I2S TX channel in standard mode");
            return ret;
        }
    }
    TEST_ESP_OK(ret);

    if (i2s_in_mode == I2S_COMM_MODE_STD) {
        ret = i2s_channel_init_std_mode(i2s_keep[port]->rx_handle, &std_cfg);
        if(ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize I2S RX channel in standard mode");
            return ret;
        }
    }
    TEST_ESP_OK(ret);

    // For tx master using duplex mode
    ret = i2s_channel_enable(i2s_keep[port]->tx_handle);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2S TX channel");
        return ret;
    }

    ret = i2s_channel_enable(i2s_keep[port]->rx_handle);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable I2S RX channel");
        return ret;
    }

    return ret;
}

static int ut_i2s_deinit(uint8_t port)
{
    if (port >= I2S_MAX_KEEP) {
        return -1;
    }
    // already installed
    if (i2s_keep[port] == NULL) {
        return 0;
    }
    // i2s_channel_disable(i2s_keep[port]->tx_handle);
    // i2s_channel_disable(i2s_keep[port]->rx_handle);
    i2s_del_channel(i2s_keep[port]->tx_handle);
    i2s_del_channel(i2s_keep[port]->rx_handle);
    free(i2s_keep[port]);
    i2s_keep[port] = NULL;
    return 0;
}

static void codec_max_sample(uint8_t *data, int size, int *max_value, int *min_value)
{
    int16_t *s = (int16_t *)data;
    size >>= 1;
    int i = 1, max, min;
    max = min = s[0];
    while (i < size) {
        if (s[i] > max) {
            max = s[i];
        } else if (s[i] < min) {
            min = s[i];
        }
        i++;
    }
    *max_value = max;
    *min_value = min;
}

void play_demo_file(esp_codec_dev_handle_t codec_dev)
{
    ESP_LOGI(TAG, "Playing demo file...");
    // demo.pcm 开始与结束指针
    const uint8_t *pcm_start = _binary_demo_pcm_start;
    size_t pcm_len           = _binary_demo_pcm_end - _binary_demo_pcm_start;

    // 假设 play_dev 已正常打开并设置了格式
    esp_codec_dev_write(codec_dev, (uint8_t*)pcm_start, pcm_len);
    ESP_LOGI(TAG, "Finished playing demo file.");
}

// es8311 init
void es8311_init()
{
    // Need install driver (i2c and i2s) firstly
    // int ret = ut_i2c_init(1);
    // TEST_ESP_OK((esp_err_t)ret);
    int ret = ut_i2s_init(0);
    // TEST_ESP_OK((esp_err_t)ret);


    // Do initialize of related interface: data_if, ctrl_if and gpio_if
    audio_codec_i2s_cfg_t i2s_cfg = {
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
        .rx_handle = i2s_keep[0]->rx_handle,
        .tx_handle = i2s_keep[0]->tx_handle,
#endif
    };
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_cfg);
    // TEST_ASSERT_NOT_NULL(data_if);

    audio_codec_i2c_cfg_t i2c_cfg = {.addr = ES8311_CODEC_DEFAULT_ADDR};
    i2c_cfg.bus_handle = i2c_bus_get_internal_bus_handle(g_i2c_bus);
    const audio_codec_ctrl_if_t *out_ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);
    // TEST_ASSERT_NOT_NULL(out_ctrl_if);

    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();
    // TEST_ASSERT_NOT_NULL(gpio_if);

    // New output codec interface
    es8311_codec_cfg_t es8311_cfg = {
        .ctrl_if    = out_ctrl_if,
        .gpio_if    = gpio_if,
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_BOTH,
        .pa_pin     = GPIO_NUM_NC,
        .pa_reverted = false,
        .use_mclk   = true,
    };
    const audio_codec_if_t *out_codec_if = es8311_codec_new(&es8311_cfg);
    // TEST_ASSERT_NOT_NULL(out_codec_if);

    // New output codec device
    esp_codec_dev_cfg_t dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN_OUT,
        .codec_if = out_codec_if,
        .data_if  = data_if,
    };
    esp_codec_dev_handle_t codec_dev = esp_codec_dev_new(&dev_cfg);
    // TEST_ASSERT_NOT_NULL(codec_dev);

    ret = esp_codec_dev_set_out_vol(codec_dev, 50.0);
    // TEST_ESP_OK(ret);
    ret = esp_codec_dev_set_in_gain(codec_dev, 30.0);
    // TEST_ESP_OK(ret);

    esp_codec_dev_sample_info_t fs = {
        .bits_per_sample = 16,
        .channel         = 1,
        .sample_rate     = 44100,
    };
    ret = esp_codec_dev_open(codec_dev, &fs);
    // TEST_ESP_OK(ret);

    stop_watch_speaker_set(true);
    vTaskDelay(10 / portTICK_PERIOD_MS);

    play_demo_file(codec_dev);

    stop_watch_speaker_set(false);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    
    while (true)
    {
        // record 3 seconds and play
        uint8_t *data = (uint8_t *)malloc(fs.sample_rate * fs.channel * (fs.bits_per_sample >> 3) * 5); // 5 seconds recording buffer
        if (data == NULL) {
            ESP_LOGE(TAG, "Failed to allocate memory for recording");
            break;
        }
        
        int buffer_size = fs.sample_rate * fs.channel * (fs.bits_per_sample >> 3) * 5; // 5 seconds recording total bytes
        
        ESP_LOGI(TAG, "Start recording 5 seconds...");
        // read 3 seconds data at once
        ret = esp_codec_dev_read(codec_dev, data, buffer_size);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Record failed: %d", ret);
            break;
        }
        stop_watch_speaker_set(true);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "Record completed, start playing...");
        ret = esp_codec_dev_write(codec_dev, data, buffer_size);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Playback failed: %d", ret);
        }
        ESP_LOGI(TAG, "Playback completed");
        stop_watch_speaker_set(false);
        vTaskDelay(10 / portTICK_PERIOD_MS);
        
        free(data);
        break;
    }

    ret = esp_codec_dev_close(codec_dev);
    // TEST_ESP_OK(ret);

    esp_codec_dev_delete(codec_dev);

    // Delete codec interface
    audio_codec_delete_codec_if(out_codec_if);
    // Delete codec control interface
    audio_codec_delete_ctrl_if(out_ctrl_if);
    audio_codec_delete_gpio_if(gpio_if);
    // Delete codec data interface
    audio_codec_delete_data_if(data_if);

    // ut_i2c_deinit(1);
    ut_i2s_deinit(0);
}