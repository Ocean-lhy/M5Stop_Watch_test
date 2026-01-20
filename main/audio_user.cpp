#include "audio_user.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_http_server.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "power_management.h"
#include "tools.h"
#include <math.h>

// 音频1
extern "C" const uint8_t _binary_demo_pcm_start[] asm("_binary_demo_pcm_start");
extern "C" const uint8_t _binary_demo_pcm_end[] asm("_binary_demo_pcm_end");

// 音频2
extern "C" const uint8_t _binary_tone_wav_start[] asm("_binary_tone_wav_start");
extern "C" const uint8_t _binary_tone_wav_end[] asm("_binary_tone_wav_end");

// 音频3
extern "C" const uint8_t _binary_hotel_california_wav_start[] asm("_binary_hotel_california_wav_start");
extern "C" const uint8_t _binary_hotel_california_wav_end[] asm("_binary_hotel_california_wav_end");

// 音频4
extern "C" const uint8_t _binary_dear_1_wav_start[] asm("_binary_DEAR_1_wav_start");
extern "C" const uint8_t _binary_dear_1_wav_end[] asm("_binary_DEAR_1_wav_end");

// 音频5
extern "C" const uint8_t _binary_dear_2_wav_start[] asm("_binary_DEAR_2_wav_start");
extern "C" const uint8_t _binary_dear_2_wav_end[] asm("_binary_DEAR_2_wav_end");

// 音效
extern "C" const uint8_t voice_start_start[] asm("_binary_voice_start_wav_start");
extern "C" const uint8_t voice_start_end[] asm("_binary_voice_start_wav_end");
extern "C" const uint8_t voice_button_start[] asm("_binary_voice_button_wav_start");
extern "C" const uint8_t voice_button_end[] asm("_binary_voice_button_wav_end");
extern "C" const uint8_t voice_touch_start[] asm("_binary_voice_touch_wav_start");
extern "C" const uint8_t voice_touch_end[] asm("_binary_voice_touch_wav_end");

static const char *TAG = "audio_user";

// --- 全局变量管理 ---
static i2s_chan_handle_t tx_handle            = NULL;
static i2s_chan_handle_t rx_handle            = NULL;
static esp_codec_dev_handle_t g_codec_dev     = NULL;
static const audio_codec_data_if_t *g_data_if = NULL;
static const audio_codec_ctrl_if_t *g_ctrl_if = NULL;
static const audio_codec_gpio_if_t *g_gpio_if = NULL;
static const audio_codec_if_t *g_codec_if     = NULL;

// 录音相关
static uint8_t *g_record_buffer          = NULL;
static size_t g_record_max_size          = 0;
static size_t g_record_actual_size       = 0;
static volatile bool g_is_recording      = false;
static TaskHandle_t g_record_task_handle = NULL;

// 播放相关
static int g_current_volume            = 50;
static TaskHandle_t g_play_task_handle = NULL;
static const uint8_t *g_play_ptr       = NULL;
static size_t g_play_total_len         = 0;
static volatile size_t g_play_cursor   = 0;
static volatile bool g_is_playing      = false;

// HTTP Server 相关
static httpd_handle_t g_server = NULL;

typedef struct {
    const uint8_t* data;
    size_t size;
    uint32_t sample_rate;
    uint8_t channels;
    bool interrupt;
    bool need_fft;
} audio_msg_t;

static QueueHandle_t xAudioQueue = NULL;

static uint32_t g_active_sample_rate = 44100;
static uint8_t g_active_channels     = 1;

// I2S 配置
#define I2S_PORT I2S_NUM_0

#define VISUALIZATION_BUFF_SIZE 2048
static int16_t g_vis_raw_buff[VISUALIZATION_BUFF_SIZE];
static int g_vis_raw_ptr = 0;

// ================= 内部辅助函数 =================

static void update_visualization_buffer(const uint8_t* data, size_t len) {
    const int16_t* pcm = (const int16_t*)data;
    int samples = len / 2; // 16bit = 2 bytes per sample
    
    for (int i = 0; i < samples; i++) {
        g_vis_raw_buff[g_vis_raw_ptr] = pcm[i];
        g_vis_raw_ptr++;
        if (g_vis_raw_ptr >= VISUALIZATION_BUFF_SIZE) {
            g_vis_raw_ptr = 0;
        }
    }
}

void audio_get_fft_input(float *out_buff, int count) {
    if (count > VISUALIZATION_BUFF_SIZE) count = VISUALIZATION_BUFF_SIZE;
    
    int read_ptr = g_vis_raw_ptr - count;
    if (read_ptr < 0) read_ptr += VISUALIZATION_BUFF_SIZE;
    
    for (int i = 0; i < count; i++) {
        int16_t raw = g_vis_raw_buff[read_ptr];
        // 归一化到 -1.0 ~ 1.0 范围
        out_buff[i] = (float)raw / 32768.0f;
        
        read_ptr++;
        if (read_ptr >= VISUALIZATION_BUFF_SIZE) read_ptr = 0;
    }
}

static void audio_main_task(void *arg) {
    audio_msg_t msg;
    const size_t chunk_size = 1024;
    const size_t silence_size = 44100 * 2 * 0.1; 
    uint8_t *silence_buf = (uint8_t *)calloc(1, silence_size); // 预分配静音数据
    uint32_t current_rate = 0;
    uint8_t current_ch = 0;

    while (1) {
        if (xQueueReceive(xAudioQueue, &msg, portMAX_DELAY) == pdPASS) {
            // 1. 更新当前参数，供 UI 计算时长
            bool need_reconfig = (msg.sample_rate != current_rate || msg.channels != current_ch);

            // 2. 配置 Codec
            if (need_reconfig) {
                esp_codec_dev_close(g_codec_dev);
                esp_codec_dev_sample_info_t fs = {
                    .bits_per_sample = 16,
                    .channel         = msg.channels,
                    .sample_rate     = msg.sample_rate,
                };
                esp_codec_dev_open(g_codec_dev, &fs);
                current_rate = msg.sample_rate;
                current_ch = msg.channels;
            }

            g_active_sample_rate = msg.sample_rate;
            g_active_channels    = msg.channels;
            g_play_total_len     = msg.size;
            g_play_cursor        = 0;
            g_is_playing         = true;
            
            // ESP_LOGI(TAG, "Playing: Rate=%ld, Ch=%d", g_active_sample_rate, g_active_channels);

            // audio_speaker_enable(true);

            // 3. 播放循环
            while (g_play_cursor < g_play_total_len && g_is_playing) {
                // 检查是否有新音频请求（实现打断）
                if (uxQueueMessagesWaiting(xAudioQueue) > 0) {
                    esp_codec_dev_write(g_codec_dev, silence_buf, silence_size);
                    break; 
                }

                size_t remain    = g_play_total_len - g_play_cursor;
                size_t write_len = (remain > chunk_size) ? chunk_size : remain;
                
                if (msg.need_fft) {
                    update_visualization_buffer((const uint8_t*)(msg.data + g_play_cursor), write_len);
                }

                esp_codec_dev_write(g_codec_dev, (void *)(msg.data + g_play_cursor), write_len);
                g_play_cursor += write_len;
            }

            // 4. 消除杂音
            esp_codec_dev_write(g_codec_dev, silence_buf, silence_size);
            // audio_speaker_enable(false);
            g_is_playing = false;
            g_play_cursor = 0;
        }
    }
}

void audio_get_active_info(uint32_t *rate, uint8_t *ch) {
    if (rate) *rate = g_active_sample_rate;
    if (ch) *ch = g_active_channels;
}

void audio_get_record_buffer(uint8_t **out_data, size_t *out_size)
{
    if (out_data) *out_data = g_record_buffer;
    if (out_size) *out_size = g_record_actual_size;
}

int audio_play_raw_async(const uint8_t* data, size_t size, uint32_t rate, uint8_t ch) {
    if (data == NULL || size == 0) return -1;
    if (!g_codec_dev || !xAudioQueue) return -1;
    
    audio_speaker_enable(true); 

    audio_msg_t msg = {
        .data = data,
        .size = size,
        .sample_rate = rate,
        .channels = ch,
        .interrupt = true,
        .need_fft = true,
    };
    g_is_playing = true;
    xQueueOverwrite(xAudioQueue, &msg);
    return 0;
}

static int ut_i2s_init()
{
    if (tx_handle != NULL) return 0;  // Already initialized

    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_PORT, I2S_ROLE_MASTER);
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

    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
    return 0;
}

static void ut_i2s_deinit()
{
    if (tx_handle) {
        i2s_channel_disable(tx_handle);
        i2s_del_channel(tx_handle);
        tx_handle = NULL;
    }
    if (rx_handle) {
        i2s_channel_disable(rx_handle);
        i2s_del_channel(rx_handle);
        rx_handle = NULL;
    }
}

// WAV Header 结构体
struct wav_header_t {
    char chunkId[4];         // "RIFF"
    uint32_t chunkSize;      // file size - 8
    char format[4];          // "WAVE"
    char subchunk1Id[4];     // "fmt "
    uint32_t subchunk1Size;  // 16 for PCM
    uint16_t audioFormat;    // 1 for PCM
    uint16_t numChannels;    // 1
    uint32_t sampleRate;     // 44100
    uint32_t byteRate;       // 44100 * 1 * 2
    uint16_t blockAlign;     // 2
    uint16_t bitsPerSample;  // 16
    char subchunk2Id[4];     // "data"
    uint32_t subchunk2Size;  // data size
};

// HTTP GET Handler (生成 WAV 下载)
static esp_err_t record_download_get_handler(httpd_req_t *req)
{
    if (!g_record_buffer || g_record_actual_size == 0) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    // 设置 Header
    httpd_resp_set_type(req, "audio/wav");
    httpd_resp_set_hdr(req, "Content-Disposition", "attachment; filename=\"record.wav\"");

    // 构造 WAV 头
    struct wav_header_t wav_head;
    memcpy(wav_head.chunkId, "RIFF", 4);
    memcpy(wav_head.format, "WAVE", 4);
    memcpy(wav_head.subchunk1Id, "fmt ", 4);
    wav_head.subchunk1Size = 16;
    wav_head.audioFormat   = 1;
    wav_head.numChannels   = 1;
    wav_head.sampleRate    = 44100;
    wav_head.bitsPerSample = 16;
    wav_head.byteRate      = 44100 * 1 * 2;
    wav_head.blockAlign    = 2;
    memcpy(wav_head.subchunk2Id, "data", 4);
    wav_head.subchunk2Size = g_record_actual_size;
    wav_head.chunkSize     = 36 + wav_head.subchunk2Size;

    // 发送 WAV 头
    httpd_resp_send_chunk(req, (const char *)&wav_head, sizeof(wav_head));

    // 发送 PCM 数据
    httpd_resp_send_chunk(req, (const char *)g_record_buffer, g_record_actual_size);

    // 结束发送
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

static const httpd_uri_t download_uri = {
    .uri = "/", .method = HTTP_GET, .handler = record_download_get_handler, .user_ctx = NULL};

// 后台录音任务
static void record_task_entry(void *arg)
{
    const size_t chunk_size = 2048;
    uint8_t *chunk_buf = (uint8_t *)malloc(chunk_size);

    if (!chunk_buf) {
        ESP_LOGE(TAG, "Failed to alloc chunk buffer");
        g_is_recording = false;
        vTaskDelete(NULL);
        return;
    }

    g_record_actual_size = 0;

    while (g_is_recording && (g_record_actual_size + chunk_size < g_record_max_size)) {
        // 读取音频
        esp_codec_dev_read(g_codec_dev, chunk_buf, chunk_size);

        update_visualization_buffer(chunk_buf, chunk_size);

        memcpy(g_record_buffer + g_record_actual_size, chunk_buf, chunk_size);
        g_record_actual_size += chunk_size;

        // task watchdog
        // vTaskDelay(1);
    }

    free(chunk_buf);
    g_is_recording       = false;
    g_record_task_handle = NULL;
    vTaskDelete(NULL);
}

int audio_init()
{
    if (g_codec_dev) return 0;  // Already init

    ut_i2s_init();

    // 1. Data Interface
    audio_codec_i2s_cfg_t i2s_cfg = {
        .rx_handle = rx_handle,
        .tx_handle = tx_handle,
    };
    g_data_if = audio_codec_new_i2s_data(&i2s_cfg);

    // 2. Control Interface (I2C)
    audio_codec_i2c_cfg_t i2c_cfg = {.addr = ES8311_CODEC_DEFAULT_ADDR};
    i2c_cfg.bus_handle            = i2c_bus_get_internal_bus_handle(g_i2c_bus);
    g_ctrl_if                     = audio_codec_new_i2c_ctrl(&i2c_cfg);

    // 3. GPIO Interface
    g_gpio_if = audio_codec_new_gpio();

    // 4. Codec Interface
    es8311_codec_cfg_t es8311_cfg = {
        .ctrl_if     = g_ctrl_if,
        .gpio_if     = g_gpio_if,
        .codec_mode  = ESP_CODEC_DEV_WORK_MODE_BOTH,
        .pa_pin      = GPIO_NUM_NC,  // 手动控制 PA
        .pa_reverted = false,
        .use_mclk    = true,
    };
    g_codec_if = es8311_codec_new(&es8311_cfg);

    // 5. Codec Device
    esp_codec_dev_cfg_t dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN_OUT,
        .codec_if = g_codec_if,
        .data_if  = g_data_if,
    };
    g_codec_dev = esp_codec_dev_new(&dev_cfg);

    // 默认配置
    esp_codec_dev_set_out_vol(g_codec_dev, g_current_volume);
    esp_codec_dev_set_in_gain(g_codec_dev, 30.0);

    esp_codec_dev_sample_info_t fs = {
        .bits_per_sample = 16,
        .channel         = 1,
        .sample_rate     = 44100,
    };
    esp_codec_dev_open(g_codec_dev, &fs);

    audio_speaker_enable(true);

    if (xAudioQueue == NULL) {
        xAudioQueue = xQueueCreate(1, sizeof(audio_msg_t));
        xTaskCreatePinnedToCore(audio_main_task, "audio_task", 4096, NULL, 5, NULL, 1);
    }

    return 0;
}

void audio_deinit()
{
    // 1. 停止服务
    audio_stop_web_server();

    // 2. 停止录音任务
    if (g_is_recording) {
        g_is_recording = false;
        while (g_record_task_handle != NULL) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }

    // 3. 停止播放任务
    if (g_is_playing) {
        g_is_playing = false;
        while (g_play_task_handle != NULL) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }

    // 4. 清空音频队列，防止残留消息
    if (xAudioQueue) {
        xQueueReset(xAudioQueue);
    }

    // 5. 等待 audio_main_task 回到空闲状态
    vTaskDelay(200 / portTICK_PERIOD_MS);

    // 6. 释放录音内存
    if (g_record_buffer) {
        free(g_record_buffer);
        g_record_buffer = NULL;
    }

    // 7. 销毁 Codec Device
    if (g_codec_dev) {
        esp_codec_dev_close(g_codec_dev);
        esp_codec_dev_delete(g_codec_dev);
        g_codec_dev = NULL;
    }

    // 8. 销毁 Interfaces
    if (g_codec_if) audio_codec_delete_codec_if(g_codec_if);
    if (g_ctrl_if) audio_codec_delete_ctrl_if(g_ctrl_if);
    if (g_gpio_if) audio_codec_delete_gpio_if(g_gpio_if);
    if (g_data_if) audio_codec_delete_data_if(g_data_if);

    g_codec_if = NULL;
    g_ctrl_if = NULL;
    g_gpio_if = NULL;
    g_data_if = NULL;

    // 9. 销毁 I2S
    ut_i2s_deinit();
}

// 写入静音数据以刷新缓冲区
static void flush_audio_buffer()
{
    if (!g_codec_dev) return;
    // 写入约 100ms 的静音数据
    size_t silence_size = 44100 * 2 * 0.1;
    uint8_t *silence    = (uint8_t *)calloc(1, silence_size);
    if (silence) {
        esp_codec_dev_write(g_codec_dev, silence, silence_size);
        free(silence);
    }
}

// 播放任务
static void playback_task(void *arg)
{
    const size_t chunk_size = 2048;
    g_is_playing            = true;
    g_play_cursor           = 0;

    ESP_LOGI(TAG, "Playback started, total: %d", g_play_total_len);

    while (g_is_playing && g_play_cursor < g_play_total_len) {
        size_t remain    = g_play_total_len - g_play_cursor;
        size_t write_len = (remain > chunk_size) ? chunk_size : remain;

        update_visualization_buffer((const uint8_t*)(g_play_ptr + g_play_cursor), write_len);

        esp_err_t ret = esp_codec_dev_write(g_codec_dev, (void *)(g_play_ptr + g_play_cursor), write_len);

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Codec write failed");
            break;
        }

        g_play_cursor += write_len;
    }

    flush_audio_buffer();

    g_is_playing       = false;
    g_play_task_handle = NULL;
    ESP_LOGI(TAG, "Playback finished/stopped");
    vTaskDelete(NULL);
}

int audio_play_start(const uint8_t *data, size_t size)
{
    if (!g_codec_dev) return -1;

    if (g_is_playing) {
        audio_play_stop();
    }

    g_play_ptr       = data;
    g_play_total_len = size;
    g_play_cursor    = 0;

    BaseType_t ret = xTaskCreatePinnedToCore(playback_task, "audio_play", 4096, NULL, 5, &g_play_task_handle, 1);
    return (ret == pdPASS) ? 0 : -1;
}

void audio_play_stop()
{
    if (!g_codec_dev) return;
    if (g_is_playing) {
        g_is_playing = false;  // 通知任务退出循环
        // 等待任务彻底结束
        while (g_play_task_handle != NULL) {
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
}

bool audio_is_playing()
{
    return g_is_playing;
}

void audio_get_progress(size_t *out_current, size_t *out_total)
{
    if (out_current) *out_current = g_play_cursor;
    if (out_total) *out_total = g_play_total_len;
}

int audio_set_volume(int volume)
{
    if (!g_codec_dev) return -1;
    g_current_volume = volume;
    return esp_codec_dev_set_out_vol(g_codec_dev, (float)volume);
}

void audio_speaker_enable(bool enable)
{
    stop_watch_speaker_set(enable);
}

void audio_enable(bool enable)
{
    if (enable == false) {
        audio_deinit();
    }
    stop_watch_audio_enable(enable);
    if (enable == true) {
        audio_init();
    }
}

int audio_start_record()
{
    if (!g_codec_dev) return -1;
    if (g_is_recording) return 0;  // 已经在录音

    if (g_active_sample_rate != 44100) {
        esp_codec_dev_close(g_codec_dev);
        esp_codec_dev_sample_info_t fs = {
           .bits_per_sample = 16,
           .channel         = 1,
           .sample_rate     = 44100,
        };
        esp_codec_dev_open(g_codec_dev, &fs);
        g_active_sample_rate = 44100;
        g_active_channels = 1;
   }

    // 分配内存：30秒 * 44100 * 2 bytes = ~2.6MB
    size_t max_seconds = 30;
    g_record_max_size  = 44100 * 1 * 2 * max_seconds;

    // 如果之前有内存，先释放
    if (g_record_buffer) free(g_record_buffer);

    g_record_buffer = (uint8_t *)heap_caps_malloc(g_record_max_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!g_record_buffer) {
        ESP_LOGE(TAG, "Failed to allocate %d bytes for record from PSRAM", g_record_max_size);
        ESP_LOGE(TAG, "Available PSRAM: %d bytes", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        return -1;
    }

    ESP_LOGI(TAG, "Successfully allocated %d bytes from PSRAM", g_record_max_size);
    g_is_recording = true;
    xTaskCreatePinnedToCore(record_task_entry, "rec_task", 4096, NULL, 5, &g_record_task_handle, 1);

    return 0;
}

int audio_stop_record(uint8_t **out_data, size_t *out_size)
{
    if (!g_is_recording && g_record_buffer == NULL) return -1;

    g_is_recording = false;

    // 等待任务结束
    int timeout = 100;
    while (g_record_task_handle != NULL && timeout > 0) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        timeout--;
    }

    if (out_data) *out_data = g_record_buffer;
    if (out_size) *out_size = g_record_actual_size;

    return 0;
}

bool audio_is_recording()
{
    return g_is_recording;
}

int audio_play_data(uint8_t *data, size_t size)
{
    if (!g_codec_dev || !data || size == 0) return -1;

    return esp_codec_dev_write(g_codec_dev, data, size);
}

int audio_play_async(const uint8_t* data, size_t size, uint32_t rate, uint8_t ch, bool need_fft) {
    if (!g_codec_dev || !xAudioQueue) {
        return -1;
    }
    
    audio_msg_t msg = {
        .data = data,
        .size = size,
        .sample_rate = rate,
        .channels = ch,
        .interrupt = true,
        .need_fft = need_fft,
    };
    g_is_playing = true; 
    xQueueOverwrite(xAudioQueue, &msg);
    return 0;
}

int audio_play_demo(audio_type_t demo_type)
{
    switch (demo_type) {
        case AUDIO_DEMO_PIANO:
        {
            size_t len = _binary_demo_pcm_end - _binary_demo_pcm_start;
            return audio_play_async(_binary_demo_pcm_start, len, 44100, 1, true);
        }
        break;
        case AUDIO_DEMO_TONE:
        {
            size_t len = _binary_tone_wav_end - _binary_tone_wav_start - 44;
            return audio_play_async(_binary_tone_wav_start + 44, len, 44100, 2, true);
        }
        break;
        case AUDIO_DEMO_HOTEL_CALIFORNIA:
        {
            size_t len = _binary_hotel_california_wav_end - _binary_hotel_california_wav_start - 44;
            return audio_play_async(_binary_hotel_california_wav_start + 44, len, 44100, 1, true);
        }
        break;
        case AUDIO_DEMO_DEAR_1:
        {
            size_t len = _binary_dear_1_wav_end - _binary_dear_1_wav_start - 44;
            return audio_play_async(_binary_dear_1_wav_start + 44, len, 44100, 1, true);
        }
        break;
        case AUDIO_DEMO_DEAR_2:
        {
            size_t len = _binary_dear_2_wav_end - _binary_dear_2_wav_start - 44;
            return audio_play_async(_binary_dear_2_wav_start + 44, len, 44100, 1, true);
        }
        break;
        case AUDIO_VOICE_START:
        {
            size_t len = voice_start_end - voice_start_start - 1000;
            return audio_play_async(voice_start_start + 44, len, 44100, 1, false);
        }
        break;
        case AUDIO_VOICE_SWITCH:
        {
            audio_set_volume(60);
            size_t len = voice_touch_end - voice_touch_start - 500;
            return audio_play_async(voice_touch_start, len, 44100, 1, false);
        }
        break;
        case AUDIO_VOICE_CHECK:
        {
            audio_set_volume(60);
            size_t len = voice_button_end - voice_button_start;
            return audio_play_async(voice_button_start, len, 44100, 1, false);
        }
        break;
        default:
        {
            ESP_LOGE(TAG, "Invalid audio type: %d", demo_type);
            return -1;
        }
        break;
    }
}

void audio_start_web_server()
{
    if (g_server) return;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.uri_match_fn   = httpd_uri_match_wildcard;

    if (httpd_start(&g_server, &config) == ESP_OK) {
        httpd_register_uri_handler(g_server, &download_uri);
        ESP_LOGI(TAG, "Web Server Started");
    } else {
        ESP_LOGE(TAG, "Failed to start Web Server");
    }
}

void audio_stop_web_server()
{
    if (g_server) {
        httpd_stop(g_server);
        g_server = NULL;
    }
}