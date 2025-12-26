#ifndef AUDIO_USER_H
#define AUDIO_USER_H

#include "driver/i2s_std.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "i2c_bus.h"
#include "setting.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// 初始化音频驱动 (I2S, Codec)
int audio_init();

// 反初始化 (释放所有资源, 停止服务)
void audio_deinit();

// 设置音量 (0-100)
int audio_set_volume(int volume);

// 开始后台录音
// 返回 0 成功
int audio_start_record();

// 停止录音
// out_data: 指向录音数据的指针
// out_size: 实际录到的字节数
int audio_stop_record(uint8_t **out_data, size_t *out_size);

// 播放内存中的 PCM 数据
int audio_play_data(uint8_t *data, size_t size);

// 播放内置 Demo
int audio_play_demo(uint8_t demo_type = 0);

// 控制功放开关
void audio_speaker_enable(bool enable);

// 查询是否正在录音
bool audio_is_recording();

// 启动 HTTP 文件下载服务 (下载最近一次的录音)
void audio_start_web_server();

// 停止 HTTP 服务
void audio_stop_web_server();

/**
 * @brief 异步播放内存中的 PCM 数据
 * @param data 音频数据指针
 * @param size 数据长度
 * @return 0 成功, -1 失败
 */
int audio_play_start(const uint8_t *data, size_t size);

/**
 * @brief 停止播放 (中断)
 */
void audio_play_stop();

/**
 * @brief 检查是否正在播放
 */
bool audio_is_playing();

/**
 * @brief 获取播放进度
 * @param out_current 当前播放字节数
 * @param out_total 总字节数
 */
void audio_get_progress(size_t *out_current, size_t *out_total);

#ifdef __cplusplus
}
#endif

#endif // AUDIO_USER_H