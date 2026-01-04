#include "ui.h"

#include <math.h>

#include <vector>

#include "bmi270_tools.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "tools.h"

#include "circle_images.h"
#include "esp_flash.h"
#include "esp_chip_info.h"
#include "spi_flash_chip_driver.h"
#include "esp_partition.h"

#include "fft.h"

extern const uint8_t stopwatch_jpg_start[] asm("_binary_StopWATCH_jpg_start");
extern const uint8_t stopwatch_jpg_end[] asm("_binary_StopWATCH_jpg_end");

static const char *TAG = "UI";
StopWatchApp app;

// title
#define MAIN_TITLE "StopWatch CES2026"

#define CYLINDER_POINTS 16  // 圆周采样点数

struct Point3D {
    float x, y, z;
};
struct Point2D {
    float x, y;
};

Point3D stopwatch_vertices[CYLINDER_POINTS * 2];

void init_stopwatch_geometry()
{
    float radius      = 1.0f;
    float half_height = 0.26f;

    for (int i = 0; i < CYLINDER_POINTS; i++) {
        float angle = i * 2.0f * 3.1415926f / CYLINDER_POINTS;
        float x     = -sin(angle) * radius;
        float y     = cos(angle) * radius;

        // 顶面
        stopwatch_vertices[i] = {x, y, -half_height};
        // 底面
        stopwatch_vertices[i + CYLINDER_POINTS] = {x, y, half_height};
    }
}

// 测试列表定义
std::vector<TestItem> testList = {{"MAIN PAGE", StopWatchApp::test_main_page},
                                  {"IMU", StopWatchApp::test_imu_data},
                                  {"TOUCH", StopWatchApp::test_touch_draw},
                                  {"VIRBRATION MOTOR", StopWatchApp::test_vibration},
                                  {"RECORD", StopWatchApp::test_audio_record},
                                  {"PLAY PIANO DEMO", StopWatchApp::test_audio_play_0},
                                  {"PLAY TONE(NOISE)", StopWatchApp::test_audio_play_1},
                                  {"PLAY DEAR~", StopWatchApp::test_audio_play_2}};

void StopWatchApp::init()
{
    gpio_reset_pin(USER_BUTTON1_PIN);
    gpio_set_direction(USER_BUTTON1_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(USER_BUTTON1_PIN, GPIO_PULLUP_ONLY);

    gpio_reset_pin(USER_BUTTON2_PIN);
    gpio_set_direction(USER_BUTTON2_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode(USER_BUTTON2_PIN, GPIO_PULLUP_ONLY);
}

unsigned long getMillis()
{
    return (unsigned long)(esp_timer_get_time() / 1000);
}
// 静态变量用于锁存坐标
static int _lastValidX = 0;
static int _lastValidY = 0;
void StopWatchApp::updateInputs(bool updateTouch)
{
    unsigned long now = getMillis();

    // 1. 初始化点击状态
    _inputs.btnLeftClicked  = false;
    _inputs.btnRightClicked = false;
    _inputs.touchClicked    = false;

    // 2. 处理左键 (Button2)
    int stateL = gpio_get_level(USER_BUTTON2_PIN);
    if (_trackL.lastState == 1 && stateL == 0) {
        _inputs.btnLeftClicked  = true;
        _trackL.lastTriggerTime = now;
    } else if (stateL == 0 && (now - _trackL.lastTriggerTime >= 200)) {
        _inputs.btnLeftClicked  = true;
        _trackL.lastTriggerTime = now;
    }
    _trackL.lastState = stateL;

    // 3. 处理右键 (Button1)
    int stateR = gpio_get_level(USER_BUTTON1_PIN);
    if (_trackR.lastState == 1 && stateR == 0) {
        _inputs.btnRightClicked = true;
        _trackR.lastTriggerTime = now;
    } else if (stateR == 0 && (now - _trackR.lastTriggerTime >= 500)) {
        _inputs.btnRightClicked = true;
        _trackR.lastTriggerTime = now;
    }
    _trackR.lastState = stateR;

    // 4. 处理触摸
    if (updateTouch) {
        cst820_loop();

        // 判断当前帧是否有有效触摸
        bool isContacting = (cst820_status == 0 || cst820_status == 2) && (cst820_x > 0 || cst820_y > 0);

        if (isContacting) {
            // --- 手指按压中 ---

            // 实时锁存有效坐标
            _lastValidX = cst820_x;
            _lastValidY = cst820_y;

            // 实时更新 inputs (主要用于滑块/拖动)
            _inputs.touchX = cst820_x;
            _inputs.touchY = cst820_y;

            if (!_isTouching) {
                // 刚按下的第一帧 (Touch Down)
                _isTouching        = true;
                _touchStartY       = cst820_y;
                _touchStartX       = cst820_x;  // 记录X以便可能的横向判断
                _startScrollOffset = scroll_offset;
                _isDragging        = false;
                // ESP_LOGI(TAG, "Touch Down: %d, %d", cst820_x, cst820_y);
            } else {
                // 按压持续中 (Dragging check)
                int diffY = abs(cst820_y - _touchStartY);
                int diffX = abs(cst820_x - _touchStartX);

                // 增加一点死区，防止手指轻微抖动误判为滑动
                // 只有位移超过 15 像素才视为拖动
                if (diffY > 15 || diffX > 15) {
                    _isDragging = true;
                }
            }
        } else {
            // --- 手指已抬起 (Touch Up) ---

            if (_isTouching) {
                // 状态翻转瞬间
                if (!_isDragging) {
                    // 判定为点击
                    _inputs.touchClicked = true;

                    // 强制使用最后一次有效的坐标，防止抬起瞬间坐标漂移或归零
                    _inputs.touchX = _lastValidX;
                    _inputs.touchY = _lastValidY;

                    // ESP_LOGI(TAG, "Click Detected at: %d, %d", _inputs.touchX, _inputs.touchY);
                }
                _isTouching = false;
                _isDragging = false;
            }
        }
    }
}

// 区域判断函数
bool StopWatchApp::checkTouchRegion(int x1, int y1, int x2, int y2)
{
    return _inputs.touchClicked && _inputs.touchX >= x1 && _inputs.touchX <= x2 && _inputs.touchY >= y1 &&
           _inputs.touchY <= y2;
}

void StopWatchApp::loop()
{
    // 1. 处理输入
    updateInputs(true);
    int item_h      = 30;  // 每一行的高度
    int total_items = testList.size();

    if (_isDragging) {
        // 1. 滑动中：scroll_offset 实时跟随手指位置
        float diffY   = _inputs.touchY - _touchStartY;
        scroll_offset = _startScrollOffset - (diffY / (float)item_h);

        // 计算当前选中的菜单索引（四舍五入到最近的项）
        menu_index = (int)round(scroll_offset);
    } else {
        // 2. 未滑动（或松手后）：执行平滑吸附动画
        // 按钮点击切换
        if (checkTouchRegion(0, 50, 466, 180)) menu_index--;
        if (_inputs.btnLeftClicked || checkTouchRegion(0, 280, 466, 410)) menu_index++;

        // 弹性追踪：scroll_offset 向整数的 menu_index 靠近
        scroll_offset += (menu_index - scroll_offset) * spring_k;
    }

    // 3. 处理循环逻辑，防止 scroll_offset 变成极大的正数或负数
    if (scroll_offset < 0) {
        scroll_offset += total_items;
        menu_index += total_items;
        _startScrollOffset += total_items;  // 如果在滑动中跳变，需同步更新起始偏移
    } else if (scroll_offset >= total_items) {
        scroll_offset -= total_items;
        menu_index -= total_items;
        _startScrollOffset -= total_items;
    }

    if (menu_index != _last_menu_index) {
        _last_menu_index = menu_index;
        trigger_motor(30, 60);
        audio_play_demo(AUDIO_VOICE_SWITCH);
    }

    // 4. 渲染
    canvas.fillScreen(TFT_BLACK);
    drawHeader();
    drawFooter();
    drawMenu();  // 内部使用 scroll_offset

    // 5. 确认进入
    if (_inputs.btnRightClicked || (!_isDragging && checkTouchRegion(0, 181, 466, 279))) {
        audio_play_demo(AUDIO_VOICE_CHECK);
        int final_idx = (menu_index % total_items + total_items) % total_items;
        if (testList[final_idx].func) {
            canvas.fillScreen(TFT_BLACK);
            testList[final_idx].func();
            canvas.fillScreen(TFT_BLACK);
        }
    }

    canvas.pushSprite(&gfx, 0, 0);
    // vTaskDelay(1 / portTICK_PERIOD_MS);  // 降低延时让动画更流畅
}

void StopWatchApp::drawHeader()
{
    canvas.setTextSize(1);
    canvas.setTextColor(TFT_WHITE, TFT_BLACK);
    canvas.setTextDatum(top_center);
    canvas.drawString(MAIN_TITLE, 233, 30);
    canvas.drawLine(50, 50, 416, 50, TFT_DARKGRAY);
}

void StopWatchApp::drawFooter()
{
    static uint16_t vbat                  = 0;
    // static uint16_t input_voltage         = 0;
    // static uint16_t _5vinout_voltage      = 0;
    static pm1_gpio_in_state_t gpio_state = PM1_GPIO_IN_STATE_LOW;
    static uint32_t vbat_last_read        = 0;
    if (lgfx::v1::millis() - vbat_last_read > 1000) {
        pm1_vbat_read(&vbat);
        // pm1_vin_read(&input_voltage);
        // pm1_5vinout_read(&_5vinout_voltage);
        pm1_gpio_get_in_state(PMG2_CHG_STAT, &gpio_state);
        vbat_last_read = lgfx::v1::millis();
    }

    canvas.setTextSize(1);
    canvas.setTextDatum(bottom_center);
    canvas.fillRect(0, 430, 466, 30, TFT_BLACK);

    char buf[48];
    if (gpio_state == PM1_GPIO_IN_STATE_HIGH) {
        canvas.setTextColor(TFT_RED, TFT_BLACK);
    } else {
        canvas.setTextColor(TFT_GREEN, TFT_BLACK);
    }
    sprintf(buf, "BAT: %dmV", vbat);
    canvas.drawString(buf, 233, 460);
    canvas.setTextColor(TFT_WHITE, TFT_BLACK);
    // sprintf(buf, "VIN: %dmV, 5VIO: %dmV", input_voltage, _5vinout_voltage);
    sprintf(buf, "LEFT:next RIGHT:enter");
    canvas.drawString(buf, 233, 440);
    canvas.drawLine(50, 440, 416, 440, TFT_DARKGRAY);
}

// 辅助函数：计算弧形偏移
int get_curved_offset(float deltaY)
{
    const float radius     = 400.0f;  // 弧线半径
    const float max_offset = 100.0f;  // 最大向右挤出的距离

    float y = abs(deltaY);
    if (y > radius) y = radius;

    // 圆方程: x = r - sqrt(r^2 - y^2)
    float x = radius - sqrtf(radius * radius - y * y);
    return (int)(std::min(x, max_offset));
}

void StopWatchApp::drawMenu()
{
    int center_y = 233;  // 屏幕中心
    int item_h   = 30;   // 每行间距

    canvas.setTextDatum(middle_left);  // 改为左对齐，方便配合弧形

    // 渲染范围：当前索引前后 5 个
    for (int i = -6; i <= 6; i++) {
        // 算出真实索引（处理循环）
        int idx = (menu_index + i) % (int)testList.size();
        if (idx < 0) idx += testList.size();

        // 这样当 menu_index 变化时，所有项会平滑移动
        float relative_pos = (menu_index + i) - scroll_offset;
        int y_pos          = center_y + (relative_pos * item_h);

        // 只有在屏幕范围内的才画
        if (y_pos > 70 && y_pos < 410) {
            // 计算弧形 X 偏移
            int x_offset = get_curved_offset(y_pos - center_y);
            int base_x   = 80 + x_offset;  // 基础左边距 + 弧形偏移

            // 区分选中态和非选中态
            if (i == 0) {
                // 选中项加个高亮框
                canvas.fillRoundRect(base_x - 10, y_pos - 20, 300, 40, 8, TFT_SKYBLUE);
                canvas.setTextColor(TFT_WHITE);
                canvas.setTextSize(2);
                canvas.drawString(testList[idx].name, base_x, y_pos);
            } else {
                // 非选中项渐变透明（颜色变暗）
                int alpha = 255 - (abs(i) * 30);  // 越远越暗
                if (alpha < 50) alpha = 50;
                uint16_t color = canvas.color565(alpha, alpha, alpha);

                canvas.setTextColor(color);
                canvas.setTextSize(1);
                canvas.drawString(testList[idx].name, base_x, y_pos > 233 ? y_pos + 5 : y_pos - 5);
            }
        }
    }

    // 画一个红色的固定指示器（参考代码中的 Selector）
    canvas.drawRoundRect(60, center_y - 25, 340, 50, 10, TFT_RED);
}

void StopWatchApp::drawTitle(const char *title)
{
    canvas.setTextDatum(top_center);
    canvas.setTextColor(TFT_YELLOW, TFT_BLACK);
    canvas.setTextSize(1);
    canvas.drawString(title, 233, 30);
    canvas.drawLine(0, 50, 466, 50, TFT_WHITE);
    canvas.setTextSize(2);
}

// ================= 具体测试实现 =================

// Main Page
void StopWatchApp::test_main_page()
{
    gfx.fillScreen(TFT_BLACK);
    size_t jpg_size = stopwatch_jpg_end - stopwatch_jpg_start;
    gfx.drawJpg(stopwatch_jpg_start, jpg_size, 0, 0);
    while (1) {
        app.updateInputs(true);
        if (app._inputs.btnRightClicked || app.checkTouchRegion(0, 0, 466, 466)) return;
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

// Touch Test
void StopWatchApp::test_touch_draw()
{
    canvas.fillScreen(TFT_BLACK);
    app.drawTitle(testList[2].name);
    canvas.fillRect(0, 55, 466, 345, TFT_BLACK);
    canvas.drawString("L：Clear | R：Exit", 233, 60);

    static int last_x             = -1;
    static int last_y             = -1;
    static uint16_t current_color = TFT_WHITE;

    gpio_set_direction((gpio_num_t)TP_INT_IRQ_PIN, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)TP_INT_IRQ_PIN, GPIO_PULLUP_ONLY);

    while (1) {
        app.updateInputs(true);

        // 左键清屏
        if (app._inputs.btnLeftClicked) {
            canvas.fillScreen(TFT_BLACK);
        }

        // 右键退出
        if (app._inputs.btnRightClicked) return;

        // 触摸逻辑处理
        if (cst820_status == 0) {
            // 刚按下：生成一个新颜色，并记录起点
            current_color = lgfx::color565(rand() % 256, rand() % 256, rand() % 256);
            last_x        = cst820_x;
            last_y        = cst820_y;
            canvas.fillCircle(cst820_x, cst820_y, 2, current_color);
        } else if (cst820_status == 2) {
            // 移动中：如果上次坐标有效，则画线连接
            if (last_x != -1 && last_y != -1) {
                canvas.drawLine(last_x, last_y, cst820_x, cst820_y, current_color);
            }
            last_x = cst820_x;
            last_y = cst820_y;
        } else if (cst820_status == 1) {
            // 抬起：重置坐标，断开连接
            last_x = -1;
            last_y = -1;
        }

        // 调试信息显示
        int irq = gpio_get_level((gpio_num_t)TP_INT_IRQ_PIN);
        char buf[64];
        sprintf(buf, "X:%3d Y:%3d S:%d G13:%d", cst820_x, cst820_y, cst820_status, irq);

        // 在底部显示坐标，背景涂黑防止重叠
        canvas.setTextColor(TFT_WHITE, TFT_BLACK);
        canvas.setTextDatum(bottom_center);
        canvas.setTextSize(1);
        canvas.drawString(buf, 233, 233);

        canvas.pushSprite(&gfx, 0, 0);
        vTaskDelay(1);
    }
}

// IMU
void StopWatchApp::test_imu_data()
{
    bmi270_tools b_tools;
    static i2c_bus_device_handle_t dev_h = i2c_bus_device_create(g_i2c_bus, BMI270_ADDR, 100000);
    b_tools.init(g_i2c_bus, &dev_h, true, bmi270_tools::MODE_CONTEXT);
    b_tools.enable_default_sensors();

    init_stopwatch_geometry();

    Point2D projected[CYLINDER_POINTS * 2];
    float scale = 120.0;
    int centerX = 233;
    int centerY = 260;

    while (1) {
        app.updateInputs(false);
        if (app._inputs.btnRightClicked) return;

        bmi270_tools::sensor_data_t data;
        b_tools.get_sensor_data(data);

        float pitch = atan2(-data.acc_x, data.acc_z);
        float roll  = atan2(data.acc_y, sqrt(data.acc_x * data.acc_x + data.acc_z * data.acc_z));

        canvas.fillSprite(TFT_BLACK);

        app.drawTitle(testList[1].name);
        canvas.setTextSize(1);
        canvas.setTextColor(TFT_WHITE);
        canvas.setTextDatum(top_center);
        char buf[40];
        sprintf(buf, "ACC: %.2f, %.2f, %.2f", data.acc_x, data.acc_y, data.acc_z);
        canvas.drawString(buf, 233, 60);
        sprintf(buf, "GYR: %.2f, %.2f, %.2f", data.gyr_x, data.gyr_y, data.gyr_z);
        canvas.drawString(buf, 233, 85);

        for (int i = 0; i < CYLINDER_POINTS * 2; i++) {
            Point3D p = stopwatch_vertices[i];

            // 1. 绕 X 轴旋转 (Pitch)
            float y1 = p.y * cos(pitch) - p.z * sin(pitch);
            float z1 = p.y * sin(pitch) + p.z * cos(pitch);
            float x1 = p.x;

            // 2. 绕 Y 轴旋转 (Roll)
            float x2 = x1 * cos(roll) + z1 * sin(roll);
            float z2 = -x1 * sin(roll) + z1 * cos(roll);
            float y2 = y1;

            // 3. 投影到屏幕 (添加透视)
            float perspective = 400.0 / (400.0 + z2);
            projected[i].x    = centerX + x2 * scale * perspective;
            projected[i].y    = centerY + y2 * scale * perspective;
        }

        // --- 绘制部分 ---
        // 1. 先画底面 (Depth buffer 模拟：先画后面的线)
        for (int i = 0; i < CYLINDER_POINTS; i++) {
            int next = (i + 1) % CYLINDER_POINTS;
            canvas.drawLine(projected[i + CYLINDER_POINTS].x, projected[i + CYLINDER_POINTS].y,
                            projected[next + CYLINDER_POINTS].x, projected[next + CYLINDER_POINTS].y,
                            0x52AA);  // 深灰蓝
        }

        // 2. 画侧边连线
        for (int i = 0; i < CYLINDER_POINTS; i++) {
            if (i % 2 == 0) {  // 减少线条密度，看起来更清晰
                canvas.drawLine(projected[i].x, projected[i].y, projected[i + CYLINDER_POINTS].x,
                                projected[i + CYLINDER_POINTS].y, TFT_DARKGREY);
            }
        }

        // 3. 画顶面 (最上面，用亮色)
        for (int i = 0; i < CYLINDER_POINTS; i++) {
            int next = (i + 1) % CYLINDER_POINTS;
            canvas.drawLine(projected[i].x, projected[i].y, projected[next].x, projected[next].y, TFT_CYAN);
        }

        // 在顶面圆心画一个标记，确认方向
        // 顶面圆心其实就是投影后的原点偏移（或者计算所有顶面点的平均值）
        int step = CYLINDER_POINTS / 4;
        for (int i = 0; i < 4; i++) {
            int index = i * step;
            canvas.fillCircle(projected[index].x, projected[index].y, 4, TFT_RED);
        }

        canvas.pushSprite(&gfx, 0, 0);
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

// Vibration
void StopWatchApp::test_vibration()
{
    int strength = 0;
    // Motor on PY32_IO_EXPANDER, Pin 9
    io_expander.pinMode(PY32_MOTOR_EN_PIN, OUTPUT);
    io_expander.setPwmFrequency(5000);
    app.drawTitle(testList[3].name);
    canvas.fillRect(0, 55, 466, 345, TFT_BLACK);

    while (1) {
        char buf[32];
        sprintf(buf, "Intensity: %d%%", strength);
        canvas.fillRect(0, 55, 466, 345, TFT_BLACK);
        canvas.setTextDatum(top_center);
        canvas.drawString(buf, 233, 220);
        canvas.drawString("LEFT：Increase | RIGHT：Exit", 233, 260);
        canvas.pushSprite(&gfx, 0, 0);

        app.updateInputs(false);
        if (app._inputs.btnLeftClicked) {
            strength = (strength + 10);
            if (strength > 100) strength = 0;
            // Map 0-100 to PWM duty
            io_expander.setPwmDuty(PY32_MOTOR_PWM_CHANNEL, strength, false, true);
        }
        if (app._inputs.btnRightClicked) {
            io_expander.setPwmDuty(PY32_MOTOR_PWM_CHANNEL, 0, false, true);  // Off
            return;
        }
        vTaskDelay(10);
    }
}

uint16_t hsvToRGB565(uint8_t h, uint8_t s, uint8_t v)
{
    float fH = h * 360.0 / 255.0;
    float fS = s / 255.0;
    float fV = v / 255.0;

    float c = fV * fS;
    float x = c * (1 - fabs(fmod(fH / 60.0, 2) - 1));
    float m = fV - c;
    float r = 0, g = 0, b = 0;

    if (fH >= 0 && fH < 60) {
        r = c;
        g = x;
        b = 0;
    } else if (fH >= 60 && fH < 120) {
        r = x;
        g = c;
        b = 0;
    } else if (fH >= 120 && fH < 180) {
        r = 0;
        g = c;
        b = x;
    } else if (fH >= 180 && fH < 240) {
        r = 0;
        g = x;
        b = c;
    } else if (fH >= 240 && fH < 300) {
        r = x;
        g = 0;
        b = c;
    } else {
        r = c;
        g = 0;
        b = x;
    }

    uint8_t R = (r + m) * 255;
    uint8_t G = (g + m) * 255;
    uint8_t B = (b + m) * 255;

    // 转换为 16位 RGB565 (5位红，6位绿，5位蓝)
    return ((R & 0xF8) << 8) | ((G & 0xFC) << 3) | (B >> 3);
}

void StopWatchApp::test_audio_record()
{
    test_audio_unified(MODE_RECORDER);
}

void StopWatchApp::test_audio_play_0()
{
    test_audio_unified(MODE_PLAY_PIANO);
}

void StopWatchApp::test_audio_play_1()
{
    test_audio_unified(MODE_PLAY_TONE);
}

void StopWatchApp::test_audio_play_2()
{
    test_audio_unified(MODE_PLAY_DEAR);
}

// 辅助函数：将毫秒转为 MM:SS 字符串
void format_time(char *buf, size_t seconds)
{
    sprintf(buf, "%02d:%02d", (int)(seconds / 60), (int)(seconds % 60));
}

// Audio Unified
void StopWatchApp::test_audio_unified(int current_mode_idx)
{
    enum PlayerState { STATE_IDLE, STATE_PLAYING, STATE_PAUSED, STATE_RECORDING, STATE_REC_FINISHED };

    PlayerState current_state    = STATE_IDLE;
    int volume                   = 60;
    unsigned long rec_start_time = 0;

    // --- 频谱动画变量 ---
    const int BARS_COUNT             = 11;   // 奇数，方便有一个中间的主柱子
    float bars_current_h[BARS_COUNT] = {0};  // 当前高度

    // --- FFT 初始化 ---
    const int FFT_SIZE     = 512;  // 512点FFT足够且速度快
    fft_config_t *fft_plan = fft_init(FFT_SIZE, FFT_REAL, FFT_FORWARD, NULL, NULL);
    // 频段映射表：将 FFT_SIZE/2 个频点映射到 BARS_COUNT 个柱子
    // 这里的映射方式稍微偏重低频，因为音乐能量主要在低频
    // 512点FFT -> 256个有效频点。44.1kHz采样率下，每点约 86Hz。
    // 我们取前 128 个点 (约 0-11kHz) 进行映射
    int bin_groups[11] = {1, 2, 3, 5, 8, 12, 16, 20, 25, 30, 35};  // 每个柱子包含的FFT bin数量

    // 初始化
    audio_init();
    audio_set_volume(volume);
    audio_speaker_enable(true);
    audio_play_stop();

    auto get_mode_name = [&](int idx) -> const char * {
        switch (idx) {
            case MODE_PLAY_PIANO:
                return "Piano Demo";
            case MODE_PLAY_DEAR:
                return "Dear~";
            case MODE_PLAY_TONE:
                return "Tone Test";
            case MODE_RECORDER:
                return "Voice Recorder";
            default:
                return "Unknown";
        }
    };

    auto start_play = [&](int idx) {
        if (idx == MODE_RECORDER) {
            uint8_t *rec_data = NULL;
            size_t rec_size   = 0;
            // 必须先获取 buffer
            audio_get_record_buffer(&rec_data, &rec_size);

            if (rec_data != NULL && rec_size > 0) {
                // 播放录音
                audio_play_raw_async(rec_data, rec_size, 44100, 1);
                current_state = STATE_PLAYING;
            } else {
                // 没有录音数据，无法播放
                current_state = STATE_IDLE;
            }
        } else {
            // 播放预置音频
            audio_type_t type = AUDIO_DEMO_PIANO;
            if (idx == MODE_PLAY_PIANO) type = AUDIO_DEMO_PIANO;
            if (idx == MODE_PLAY_DEAR) type = AUDIO_DEMO_DEAR;
            if (idx == MODE_PLAY_TONE) type = AUDIO_DEMO_TONE;

            audio_play_demo(type);
            current_state = STATE_PLAYING;
        }
    };

    unsigned long last_touch_action_time = 0;
    while (1) {
        app.updateInputs(true);

        // 1. 右键退出
        if (app._inputs.btnRightClicked) {
            audio_play_stop();
            if (current_state == STATE_RECORDING) audio_stop_record(NULL, NULL);
            fft_destroy(fft_plan);
            return;
        }

        // 2. 左键功能键 (播放/暂停/录音)
        if (app._inputs.btnLeftClicked) {
            if (current_mode_idx == MODE_RECORDER) {
                // 录音模式下的左键
                if (current_state == STATE_RECORDING) {
                    // 停止录音
                    audio_stop_record(NULL, NULL);
                    current_state = STATE_REC_FINISHED;
                    audio_speaker_enable(true);  // 恢复功放
                } else {
                    // 开始录音 (无论是 Idle 还是 Finished 还是 Playing 都可以重新录)
                    audio_play_stop();
                    if (audio_start_record() == 0) {
                        current_state  = STATE_RECORDING;
                        rec_start_time = getMillis();
                        audio_speaker_enable(false);  // 录音关闭功放
                    }
                }
            } else {
                // 播放器模式下的左键
                if (current_state == STATE_PLAYING) {
                    audio_play_stop();
                    current_state = STATE_PAUSED;
                } else {
                    start_play(current_mode_idx);
                }
            }
        }

        // 3. 触摸控制
        if (app._inputs.touchClicked && (getMillis() - last_touch_action_time > 200)) {
            // 记录操作时间
            last_touch_action_time = getMillis();

            int tx = app._inputs.touchX;
            int ty = app._inputs.touchY;

            // 调试日志：如果还有问题，打开这个看坐标是否正确
            // ESP_LOGI("TOUCH", "Click at %d, %d", tx, ty);

            // 下方区域：调音量
            if (ty > 320) {
                int vol_bar_w   = 200;
                int vol_start_x = (466 - vol_bar_w) / 2;
                // 扩大一点触摸判定范围 (+/- 20px)，提高容错率
                if (tx > (vol_start_x - 20) && tx < (vol_start_x + vol_bar_w + 20)) {
                    int rel_x = tx - vol_start_x;
                    int v     = (rel_x * 100) / vol_bar_w;
                    if (v < 0) v = 0;
                    if (v > 100) v = 100;
                    volume = v;
                    audio_set_volume(volume);
                }
            }
            // 上方左侧：上一曲 (扩大判定区域)
            else if (tx < 160 && ty < 300) {
                current_mode_idx--;
                if (current_mode_idx < 0) current_mode_idx = MODE_COUNT - 1;
                audio_play_stop();
                if (current_state == STATE_RECORDING) audio_stop_record(NULL, NULL);
                current_state = STATE_IDLE;
                audio_speaker_enable(true);
            }
            // 上方右侧：下一曲 (扩大判定区域)
            else if (tx > 306 && ty < 300) {
                current_mode_idx++;
                if (current_mode_idx >= MODE_COUNT) current_mode_idx = 0;
                audio_play_stop();
                if (current_state == STATE_RECORDING) audio_stop_record(NULL, NULL);
                current_state = STATE_IDLE;
                audio_speaker_enable(true);
            }
            // 中间点击：播放/暂停
            else {
                // 只有在非音量、非切换区域点击才算中间
                if (ty < 320) {
                    if (current_mode_idx != MODE_RECORDER) {
                        if (current_state == STATE_PLAYING) {
                            audio_play_stop();
                            current_state = STATE_PAUSED;
                        } else {
                            start_play(current_mode_idx);
                        }
                    } else if (current_state == STATE_REC_FINISHED) {
                        start_play(current_mode_idx);
                    }
                }
            }
        }

        // 滑动调音量
        if (app._isDragging && app._inputs.touchY > 300) {
            int vol_bar_w   = 200;
            int vol_start_x = (466 - vol_bar_w) / 2;
            int rel_x       = app._inputs.touchX - vol_start_x;
            int v           = (rel_x * 100) / vol_bar_w;
            if (v < 0) v = 0;
            if (v > 100) v = 100;
            volume = v;
            audio_set_volume(volume);
        }

        // 自动状态维护：播放结束处理
        if (current_state == STATE_PLAYING && !audio_is_playing()) {
            if (current_mode_idx == MODE_RECORDER) {
                current_state = STATE_REC_FINISHED;
            } else {
                start_play(current_mode_idx);
            }
        }

        // =================== 绘制界面 ===================
        canvas.fillScreen(TFT_BLACK);

        // 1. 顶部信息
        app.drawTitle("Music & Record");

        canvas.setTextDatum(top_center);
        // 显示当前模式编号和总数，辅助定位
        char title_buf[64];
        sprintf(title_buf, "[ %d / %d ]", current_mode_idx + 1, MODE_COUNT);
        canvas.setTextColor(TFT_DARKGREY, TFT_BLACK);
        canvas.setTextSize(1);
        canvas.drawString(title_buf, 233, 65);

        // 显示歌曲/模式名称
        canvas.setTextColor(TFT_CYAN, TFT_BLACK);
        canvas.setTextSize(2);
        canvas.drawString(get_mode_name(current_mode_idx), 233, 85);

        // 2. 左右箭头
        canvas.setTextColor(0x4208, TFT_BLACK);  // 暗灰色
        canvas.setTextSize(3);
        canvas.drawString("<", 40, 233);
        canvas.drawString(">", 426, 233);

        // 3. 核心特效：FFT 频谱
        int center_x        = 233;
        int spectrum_y_base = 240;  // 频谱底部基线

        // 方块参数
        int block_w     = 18;  // 方块宽度
        int block_h     = 6;   // 方块高度
        int block_gap_v = 3;   // 方块垂直间距
        int block_gap_h = 6;   // 柱子水平间距
        int max_blocks  = 14;  // 最大显示多少个方块高度

        // 获取音频数据并执行 FFT
        if (current_state == STATE_PLAYING || current_state == STATE_RECORDING) {
            audio_get_fft_input(fft_plan->input, FFT_SIZE);
            fft_execute(fft_plan);

            int bin_idx = 2;
            for (int i = 0; i < BARS_COUNT; i++) {
                float mag_sum = 0;
                int count     = bin_groups[i];

                for (int k = 0; k < count; k++) {
                    if (bin_idx < FFT_SIZE / 2) {
                        float r  = fft_plan->output[2 * bin_idx];
                        float im = fft_plan->output[2 * bin_idx + 1];
                        mag_sum += sqrtf(r * r + im * im);
                        bin_idx++;
                    }
                }

                float val = mag_sum / count;

                // 对数缩放以获得更好的动态范围
                // 添加小偏移避免 log(0)
                val = log10f(val + 1.0f);

                // 映射到方块数量，使用更合理的缩放系数
                float scale         = (current_state == STATE_RECORDING) ? 12.0f : 8.0f;
                float target_blocks = val * scale;

                if (target_blocks > max_blocks) target_blocks = max_blocks;
                if (target_blocks < 0) target_blocks = 0;

                // 下落物理效果
                if (target_blocks > bars_current_h[i]) {
                    // 上升快
                    bars_current_h[i] = bars_current_h[i] * 0.3f + target_blocks * 0.7f;
                } else {
                    // 下落稍慢
                    bars_current_h[i] -= 0.5f;
                    if (bars_current_h[i] < 0) bars_current_h[i] = 0;
                }
            }
        } else {
            // 静音归零
            for (int i = 0; i < BARS_COUNT; i++) {
                bars_current_h[i] -= 0.5f;
                if (bars_current_h[i] < 0) bars_current_h[i] = 0;
            }
        }

        // [修改] 绘制方块循环
        int total_w = BARS_COUNT * block_w + (BARS_COUNT - 1) * block_gap_h;
        int start_x = center_x - total_w / 2;

        for (int i = 0; i < BARS_COUNT; i++) {
            // 获取当前高度对应的方块数量 (转为整数)
            int visible_blocks = (int)bars_current_h[i];

            // 计算当前列的 X 坐标
            int x = start_x + i * (block_w + block_gap_h);

            // 循环绘制每一个小方块 (从下往上)
            for (int b = 0; b < max_blocks; b++) {
                // 计算 Y 坐标：基线 - (块高+间距)*第几个块
                int y = spectrum_y_base - (b * (block_h + block_gap_v));

                // 颜色逻辑
                uint16_t color;

                if (b < visible_blocks) {
                    // === 这是一个 "点亮" 的方块 ===
                    if (current_state == STATE_RECORDING) {
                        // 录音模式：红色渐变
                        // 底部深红，顶部亮红
                        int bright = 100 + (b * 155 / max_blocks);
                        color      = canvas.color565(bright, 0, 0);
                    } else {
                        // 播放模式：参考示例的颜色逻辑 (底部黄/绿，顶部红)
                        // b=0 (底) -> b=max (顶)
                        // if (b < max_blocks / 2) {
                        //     // 下半部分：黄色/绿色
                        //     color = canvas.color565(0xFF, 0x9C, 0x00);  // 类似参考的橙黄色
                        // } else {
                        //     // 上半部分：红色/亮色
                        //     color = canvas.color565(0xFF, 0x30, 0x30);  // 红色
                        // }

                        // 或者使用彩虹渐变：
                        int hue = (i * 10) + (b * 10); // 基于列和高度变化色相
                        color = hsvToRGB565(hue % 255, 255, 255);
                    }
                } else {
                    // === 这是一个 "熄灭" 的方块 (背景槽) ===
                    // 画一个很暗的灰色，作为方块的轨道背景，看起来更有科技感
                    // 如果不想显示背景槽，直接 continue 即可
                    color = canvas.color565(20, 20, 20);
                }

                // 绘制方块 (留一点边距当间隙)
                canvas.fillRect(x, y - block_h, block_w, block_h, color);
            }

            // 底部装饰点 (保持不变)
            canvas.fillCircle(x + block_w / 2, spectrum_y_base + 8, 2, 0x4208);
        }

        // 4. 进度条 (胶囊型，带时间)
        size_t curr_bytes = 0, total_bytes = 1;
        uint32_t rate = 44100;
        uint8_t ch    = 1;

        if (current_state == STATE_PLAYING || current_state == STATE_PAUSED) {
            audio_get_progress(&curr_bytes, &total_bytes);
            audio_get_active_info(&rate, &ch);
        } else if (current_state == STATE_RECORDING) {
            curr_bytes  = (getMillis() - rec_start_time) * (44100 * 2 / 1000);
            total_bytes = 30 * 44100 * 2;  // 30s max
        }

        if (total_bytes == 0) total_bytes = 1;
        float progress = (float)curr_bytes / total_bytes;
        if (progress > 1.0f) progress = 1.0f;

        // 计算时间
        uint32_t bytes_per_sec = rate * ch * 2;
        if (bytes_per_sec == 0) bytes_per_sec = 88200;
        int curr_sec  = curr_bytes / bytes_per_sec;
        int total_sec = total_bytes / bytes_per_sec;

        int bar_w = 260;
        int bar_h = 24;
        int bar_x = (466 - bar_w) / 2;
        int bar_y = 280;  // 放在频谱图下方

        // 背景槽
        canvas.fillRoundRect(bar_x, bar_y, bar_w, bar_h, 12, 0x18E3);  // 深灰
        // 进度条
        int fill_w = (int)(bar_w * progress);
        if (fill_w < 12) fill_w = 12;  // 最小宽度保持圆角
        if (fill_w > bar_w) fill_w = bar_w;

        uint32_t bar_color = (current_state == STATE_RECORDING) ? TFT_RED : TFT_GREEN;
        canvas.fillRoundRect(bar_x, bar_y, fill_w, bar_h, 12, bar_color);

        // 时间文字
        char time_buf[32];
        char s_curr[10], s_tot[10];
        format_time(s_curr, curr_sec);
        format_time(s_tot, total_sec);
        sprintf(time_buf, "%s / %s", s_curr, s_tot);

        canvas.setTextDatum(top_center);
        canvas.setTextColor(TFT_WHITE, TFT_BLACK);
        canvas.setTextSize(1);
        canvas.drawString(time_buf, 233, bar_y + bar_h + 8);

        // 5. 音量条 (上移，短条，百分比)
        int vol_bar_w = 200;
        int vol_bar_h = 10;
        int vol_bar_x = (466 - vol_bar_w) / 2;
        int vol_bar_y = 360;

        canvas.fillRoundRect(vol_bar_x, vol_bar_y, vol_bar_w, vol_bar_h, 5, TFT_DARKGREY);
        canvas.fillRoundRect(vol_bar_x, vol_bar_y, (vol_bar_w * volume) / 100, vol_bar_h, 5, TFT_YELLOW);

        char vol_buf[16];
        sprintf(vol_buf, "VOL %d%%", volume);
        canvas.setTextDatum(bottom_center);
        canvas.setTextColor(TFT_YELLOW, TFT_BLACK);
        canvas.drawString(vol_buf, 233, vol_bar_y - 5);

        // 6. 底部状态栏
        canvas.setTextDatum(bottom_center);
        canvas.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
        char status_msg[64];
        if (current_mode_idx == MODE_RECORDER) {
            if (current_state == STATE_RECORDING)
                strcpy(status_msg, "LEFT: Stop Record");
            else if (current_state == STATE_REC_FINISHED)
                strcpy(status_msg, "LEFT: Re-Record  TAP: Play");
            else
                strcpy(status_msg, "LEFT: Start Record");
        } else {
            if (current_state == STATE_PLAYING)
                strcpy(status_msg, "LEFT: Pause   TAP: Pause");
            else
                strcpy(status_msg, "LEFT: Play    TAP: Play");
        }
        canvas.drawString(status_msg, 233, 440);

        canvas.pushSprite(&gfx, 0, 0);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}