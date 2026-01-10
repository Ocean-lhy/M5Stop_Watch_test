#include "ui.h"

#include <math.h>

#include <vector>

#include "bmi270_tools.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "tools.h"

#include "wifi_manager.h"
#include "lwip/sockets.h"
#include <arpa/inet.h>

#include "circle_images.h"
#include "esp_flash.h"
#include "esp_chip_info.h"
#include "spi_flash_chip_driver.h"
#include "esp_partition.h"
#include "fft.h"

static const char *TAG = "UI";
StopWatchApp app;

// WiFi 联网配置
#define WIFI_SSID "M5Stack"
#define WIFI_PASS "m5office888"
// WiFi 拉距测试配置
#define WIFI_TEST_SSID  "M5-WiFi-Test"
#define WIFI_TEST_PASS  "M5NETWORKTEST"
#define UDP_TARGET_IP   "192.168.3.5"
#define UDP_TARGET_PORT 12345
// title
#define MAIN_TITLE "StopWatch Test v0.3"

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
std::vector<TestItem> testList = {
    {"1. 设置I2C速率", StopWatchApp::test_i2c_rate},
    {"2. 设置Grove 5V", StopWatchApp::test_grove_5v},
    {"3. 设置充电", StopWatchApp::test_charge_switch},
    {"4. 状态信息", StopWatchApp::test_status_show},
    {"5. 显示测试", StopWatchApp::test_display_color},
    {"6. 触摸测试", StopWatchApp::test_touch_draw},
    {"7. 录音测试", StopWatchApp::test_audio_record},
    {"8. 播放测试", StopWatchApp::test_audio_play},
    {"9. IMU测试", StopWatchApp::test_imu_data},
    {"10. RTC时间", StopWatchApp::test_rtc_show},
    {"11. 振动测试", StopWatchApp::test_vibration},
    {"12. Grove IO测试", StopWatchApp::test_grove_io},
    {"13. 底部IO测试", StopWatchApp::test_bottom_io},
    {"14. CH442E 设置", StopWatchApp::test_ch442e},
    {"15. WiFi 扫描", StopWatchApp::test_wifi_scan},
    {"16. WiFi 拉距", StopWatchApp::test_wifi_distance},
    {"17. Flash 测试", StopWatchApp::test_flash},
    {"18. L0 关机", StopWatchApp::test_l0_mode},
    {"19. L1 待机", StopWatchApp::test_l1_mode},
    {"20. L2 睡眠", StopWatchApp::test_l2_mode},
    {"21. IMU 唤醒 L2", StopWatchApp::test_imu_wake},
    {"22. IMU 唤醒 L1", StopWatchApp::test_imu_shutdown_wake},
    {"23. RTC 唤醒 L2", StopWatchApp::test_rtc_wake},
    {"24. RTC 唤醒 L1", StopWatchApp::test_rtc_shutdown_wake},
    {"25. 底座插入唤醒", StopWatchApp::test_base_wake},
    {"26. 满载测试", StopWatchApp::test_full_load},
    {"27. 老化测试", StopWatchApp::test_aging},
    {"28. PMIC定时关机", StopWatchApp::test_pmic_timer_shutdown},
    {"29. PMIC定时开机", StopWatchApp::test_pmic_timer_wake},
    {"30. PMIC定时重启", StopWatchApp::test_pmic_timer_restart},
};

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

        // 判断当前帧是否有有效触摸 (0=按下, 2=移动/接触)
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
    static uint16_t input_voltage         = 0;
    static uint16_t _5vinout_voltage      = 0;
    static pm1_gpio_in_state_t gpio_state = PM1_GPIO_IN_STATE_LOW;
    static uint32_t vbat_last_read        = 0;
    if (lgfx::v1::millis() - vbat_last_read > 1000) {
        pm1_vbat_read(&vbat);
        pm1_vin_read(&input_voltage);
        pm1_5vinout_read(&_5vinout_voltage);
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
    sprintf(buf, "VIN: %dmV, 5VIO: %dmV", input_voltage, _5vinout_voltage);
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

void format_time(char *buf, size_t seconds)
{
    sprintf(buf, "%02d:%02d", (int)(seconds / 60), (int)(seconds % 60));
}

// ================= 具体测试实现 =================

// 1. I2C Rate
void StopWatchApp::test_i2c_rate()
{
    int selection = 0;  // 0: 400k, 1: 100k
    while (1) {
        app.drawTitle(testList[0].name);
        canvas.fillRect(0, 55, 466, 345, TFT_BLACK);
        canvas.drawString("左键切换 右键确定", 233, 60);
        canvas.drawString("设置PMIC PY32 I2C速率", 233, 100);
        canvas.setTextDatum(middle_center);

        canvas.setTextColor(selection == 0 ? TFT_GREEN : TFT_WHITE, TFT_BLACK);
        canvas.drawString(selection == 0 ? "> 400 KHz <" : "  400 KHz  ", 233, 200);

        canvas.setTextColor(selection == 1 ? TFT_GREEN : TFT_WHITE, TFT_BLACK);
        canvas.drawString(selection == 1 ? "> 100 KHz <" : "  100 KHz  ", 233, 260);

        canvas.pushSprite(&gfx, 0, 0);

        app.updateInputs(false);
        if (app._inputs.btnLeftClicked) selection = !selection;
        if (app._inputs.btnRightClicked) {
            stop_watch_set_i2c_speed(selection == 0 ? true : false);
            return;
        }
        vTaskDelay(10);
    }
}

// 2. Grove 5V
void StopWatchApp::test_grove_5v()
{
    int mode = 0;  // 0: INPUT, 1: OUTPUT
    while (1) {
        app.drawTitle(testList[1].name);
        canvas.drawString("左键切换 右键确定", 233, 60);
        canvas.setTextDatum(middle_center);
        canvas.fillRect(0, 55, 466, 345, TFT_BLACK);
        canvas.drawString(mode == 0 ? "> INPUT <" : "  INPUT  ", 233, 200);
        canvas.drawString(mode == 1 ? "> OUTPUT <" : "  OUTPUT  ", 233, 260);
        canvas.pushSprite(&gfx, 0, 0);

        app.updateInputs(false);
        if (app._inputs.btnLeftClicked) mode = !mode;
        if (app._inputs.btnRightClicked) {
            pm1_pwr_set_cfg(PM1_PWR_CFG_5V_INOUT, mode == 1 ? PM1_PWR_CFG_5V_INOUT : 0, NULL);
            return;
        }
        vTaskDelay(10);
    }
}

// 3. Charge Switch
void StopWatchApp::test_charge_switch()
{
    int opt            = 0;
    const char *opts[] = {"关闭充电", "输出高电平,慢充", "输出高电平,快充"};
    while (1) {
        app.drawTitle(testList[2].name);
        canvas.fillRect(0, 55, 466, 345, TFT_BLACK);
        canvas.drawString("左键切换 右键确定", 233, 60);
        for (int i = 0; i < 3; i++) {
            canvas.setTextColor(opt == i ? TFT_GREEN : TFT_WHITE, TFT_BLACK);
            canvas.drawString(opt == i ? ("> " + std::string(opts[i]) + "<").c_str() : opts[i], 233, 200 + i * 40);
        }
        canvas.pushSprite(&gfx, 0, 0);

        app.updateInputs(false);
        if (app._inputs.btnLeftClicked) opt = (opt + 1) % 3;
        if (app._inputs.btnRightClicked) {
            if (opt == 0)
                pm1_pwr_set_cfg(PM1_PWR_CFG_CHG_EN, 0, NULL);
            else {
                pm1_pwr_set_cfg(PM1_PWR_CFG_CHG_EN, PM1_PWR_CFG_CHG_EN, NULL);
                // Fast/Slow logic using PMG3_CHG_PROG
                if (opt == 2) {
                    pm1_gpio_set(PMG3_CHG_PROG, PM1_GPIO_MODE_OUTPUT, PM1_GPIO_OUTPUT_HIGH, PM1_GPIO_PUPD_NC,
                                 PM1_GPIO_DRV_PUSH_PULL);
                } else {
                    pm1_gpio_set(PMG3_CHG_PROG, PM1_GPIO_MODE_OUTPUT, PM1_GPIO_OUTPUT_LOW, PM1_GPIO_PUPD_NC,
                                 PM1_GPIO_DRV_PUSH_PULL);
                }
            }
            return;
        }
        vTaskDelay(10);
    }
}

// 4. Status Display
void StopWatchApp::test_status_show()
{
    app.drawTitle(testList[3].name);
    canvas.fillRect(0, 55, 466, 345, TFT_BLACK);
    canvas.drawString("右键退出", 233, 60);
    canvas.setTextSize(1.5);
    while (1) {
        app.updateInputs(false);
        if (app._inputs.btnRightClicked) return;

        uint16_t vbat, vin, v5v, vref;
        pm1_vbat_read(&vbat);
        pm1_vin_read(&vin);
        pm1_5vinout_read(&v5v);
        pm1_vref_read(&vref);

        canvas.setTextDatum(top_left);
        canvas.setCursor(100, 100);
        canvas.printf("VBAT: %d mV   ", vbat);
        canvas.setCursor(100, 160);
        canvas.printf("VIN : %d mV   ", vin);
        canvas.setCursor(100, 220);
        canvas.printf("5VIO: %d mV   ", v5v);
        canvas.setCursor(100, 280);
        canvas.printf("VREF: %d mV   ", vref);

        canvas.setTextDatum(bottom_center);
        canvas.drawString("Press Right to Exit", 233, 400);
        canvas.pushSprite(&gfx, 0, 0);
        vTaskDelay(10);
    }
}

// 5. Display Color
void StopWatchApp::test_display_color()
{
    int colors[] = {TFT_WHITE, TFT_RED, TFT_GREEN, TFT_BLUE, TFT_BLACK, TFT_WHITE};
    int c_idx    = 0;
    gfx.fillScreen(colors[c_idx]);

    while (1) {
        app.updateInputs(false);
        if (app._inputs.btnLeftClicked) {
            c_idx = (c_idx + 1) % 6;
            if (c_idx == 4) {
                gfx.pushImage(0, 0, 466, 466, circle_red_white);
            } else if (c_idx == 5) {
                gfx.pushImage(0, 0, 466, 466, circle_white_black);
            } else {
                gfx.fillScreen(colors[c_idx]);
            }
        }
        if (app._inputs.btnRightClicked) return;

        vTaskDelay(10);
    }
}

// 6. Touch Test
void StopWatchApp::test_touch_draw()
{
    canvas.fillScreen(TFT_BLACK);
    app.drawTitle(testList[5].name);
    canvas.fillRect(0, 55, 466, 345, TFT_BLACK);
    canvas.drawString("左键清屏 右键退出", 233, 60);

    // 用于记录上一次的坐标
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

// 7. Audio Record
void StopWatchApp::test_audio_record()
{
    enum State { IDLE, RECORDING, FINISHED };
    State state                     = IDLE;
    unsigned long record_start_time = 0;
    uint8_t *rec_data               = NULL;
    size_t rec_size                 = 0;

    // 初始化音频驱动
    audio_init();
    audio_speaker_enable(false);  // 录音时关闭 PA 防止啸叫

    while (1) {
        app.updateInputs(false);

        // 界面绘制
        canvas.fillScreen(TFT_BLACK);
        app.drawTitle(testList[6].name);  // "7. 录音测试"
        canvas.setTextDatum(middle_center);

        if (state == IDLE) {
            canvas.setTextColor(TFT_WHITE, TFT_BLACK);
            canvas.setTextSize(2);
            canvas.drawString("按左键开始录音", 233, 200);
            canvas.setTextSize(1.5);
            canvas.drawString("按右键退出", 233, 280);

            // 逻辑处理
            if (app._inputs.btnLeftClicked) {
                if (audio_start_record() == 0) {
                    state             = RECORDING;
                    record_start_time = getMillis();
                } else {
                    canvas.drawString("内存不足!", 233, 233);
                    canvas.pushSprite(&gfx, 0, 0);
                    vTaskDelay(1000);
                }
            }
            if (app._inputs.btnRightClicked) {
                break;  // 退出
            }

        } else if (state == RECORDING) {
            unsigned long duration = (getMillis() - record_start_time) / 1000;

            // UI
            canvas.setTextColor(TFT_RED, TFT_BLACK);
            canvas.setTextSize(2);
            canvas.drawString("正在录音...", 233, 180);
            char buf[32];
            sprintf(buf, "%ld / 30 s", duration);
            canvas.drawString(buf, 233, 220);
            canvas.setTextSize(1.5);
            canvas.setTextColor(TFT_WHITE, TFT_BLACK);
            canvas.drawString("再次按左键停止", 233, 300);

            // 逻辑: 超时或按键停止
            if (duration >= 30 || app._inputs.btnLeftClicked) {
                audio_stop_record(&rec_data, &rec_size);
                state = FINISHED;

                // 自动连接 WiFi 并启动 Server
                if (!wifi_manager_is_connected()) {
                    canvas.fillRect(0, 55, 466, 345, TFT_BLACK);
                    canvas.setTextColor(TFT_YELLOW, TFT_BLACK);
                    canvas.drawString("正在连接WiFi...", 233, 200);
                    canvas.pushSprite(&gfx, 0, 0);
                    wifi_manager_connect(WIFI_SSID, WIFI_PASS, 10000);
                }

                if (wifi_manager_is_connected()) {
                    audio_start_web_server();
                }
            }

        } else if (state == FINISHED) {
            // UI
            canvas.setTextSize(1.5);
            canvas.setTextColor(TFT_GREEN, TFT_BLACK);
            canvas.drawString("录音完成!", 233, 120);

            // WiFi 信息
            if (wifi_manager_is_connected()) {
                char ip[32];
                wifi_manager_get_ip(ip, sizeof(ip));
                canvas.setTextSize(1);
                canvas.setTextColor(TFT_CYAN, TFT_BLACK);
                char url[64];
                sprintf(url, "WIFI M5Stack下载: http://%s/", ip);
                canvas.drawString(url, 233, 160);
            } else {
                canvas.setTextColor(TFT_RED, TFT_BLACK);
                canvas.drawString("WiFi 连接失败", 233, 160);
            }

            canvas.setTextColor(TFT_WHITE, TFT_BLACK);
            canvas.setTextSize(1.5);
            canvas.drawString("左键: 本地播放", 233, 250);
            canvas.drawString("右键: 退出并清理", 233, 300);

            if (app._inputs.btnLeftClicked) {
                canvas.setTextColor(TFT_YELLOW, TFT_BLACK);
                canvas.drawString("正在播放...", 233, 350);
                canvas.pushSprite(&gfx, 0, 0);

                audio_speaker_enable(true);
                audio_set_volume(100);
                audio_play_data(rec_data, rec_size);
                audio_speaker_enable(false);
            }
            if (app._inputs.btnRightClicked) {
                break;  // 退出
            }
        }

        canvas.pushSprite(&gfx, 0, 0);
        vTaskDelay(20);
    }

    audio_deinit();
    audio_init();
}

// 8. Audio Play
void StopWatchApp::test_audio_play()
{
    enum PlayerState { STATE_IDLE, STATE_PLAYING, STATE_PAUSED, STATE_RECORDING, STATE_REC_FINISHED };

    int current_mode_idx         = MODE_PLAY_PIANO;
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
            case MODE_PLAY_DEAR_1:
                return "Dear~1";
            case MODE_PLAY_DEAR_2:
                return "Dear~2";
            case MODE_PLAY_TONE:
                return "Tone Test";
            case MODE_PLAY_HOTEL_CALIFORNIA:
                return "Hotel California";
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
            if (idx == MODE_PLAY_DEAR_1) type = AUDIO_DEMO_DEAR_1;
            if (idx == MODE_PLAY_DEAR_2) type = AUDIO_DEMO_DEAR_2;
            if (idx == MODE_PLAY_HOTEL_CALIFORNIA) type = AUDIO_DEMO_HOTEL_CALIFORNIA;
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
                        int hue = (i * 10) + (b * 10);  // 基于列和高度变化色相
                        color   = hsvToRGB565(hue % 255, 255, 255);
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

// 9. IMU
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

        app.drawTitle(testList[8].name);
        canvas.setTextSize(1);
        canvas.setTextColor(TFT_DARKGREY);
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
        canvas.fillCircle(projected[0].x, projected[0].y, 4,
                          TFT_RED);  // 12点钟方向标记

        canvas.pushSprite(&gfx, 0, 0);
        vTaskDelay(20 / portTICK_PERIOD_MS);
    }
}

// 10. RTC
void StopWatchApp::test_rtc_show()
{
    rx8130.begin(g_i2c_bus, RX8130_ADDR);
    app.drawTitle(testList[9].name);

    bool sntp_synced             = false;
    static bool sntp_initialized = false;

    while (1) {
        canvas.fillRect(0, 55, 466, 345, TFT_BLACK);

        struct tm rtc_time;
        rx8130.getTime(&rtc_time);

        canvas.setTextDatum(middle_center);
        canvas.setTextSize(3);
        canvas.setTextColor(TFT_WHITE, TFT_BLACK);

        char buf[64];
        sprintf(buf, "%04d-%02d-%02d", rtc_time.tm_year + 1900, rtc_time.tm_mon + 1, rtc_time.tm_mday);
        canvas.drawString(buf, 233, 160);

        sprintf(buf, "%02d:%02d:%02d", rtc_time.tm_hour, rtc_time.tm_min, rtc_time.tm_sec);
        canvas.drawString(buf, 233, 210);

        canvas.setTextSize(1);
        canvas.setTextDatum(bottom_center);

        if (wifi_manager_is_connected()) {
            canvas.setTextColor(TFT_GREEN, TFT_BLACK);
            canvas.drawString("WiFi: 已连接", 233, 300);
        } else {
            canvas.setTextColor(TFT_RED, TFT_BLACK);
            canvas.drawString("WiFi: 未连接", 233, 300);
        }

        canvas.setTextColor(TFT_WHITE, TFT_BLACK);
        canvas.drawString("左键：同步网络时间", 233, 350);
        canvas.drawString("右键：退出", 233, 380);

        canvas.pushSprite(&gfx, 0, 0);

        app.updateInputs(false);

        if (app._inputs.btnLeftClicked) {
            canvas.fillRect(0, 55, 466, 345, TFT_BLACK);
            canvas.setTextDatum(middle_center);
            canvas.setTextSize(1.5);

            if (!wifi_manager_is_connected()) {
                canvas.setTextColor(TFT_YELLOW, TFT_BLACK);
                canvas.drawString("正在连接WiFi...", 233, 200);
                canvas.pushSprite(&gfx, 0, 0);

                wifi_status_t status = wifi_manager_connect(WIFI_SSID, WIFI_PASS, 15000);

                if (status != WIFI_STATUS_CONNECTED) {
                    canvas.fillRect(0, 55, 466, 345, TFT_BLACK);
                    canvas.setTextColor(TFT_RED, TFT_BLACK);
                    canvas.drawString("WiFi连接失败", 233, 233);
                    canvas.pushSprite(&gfx, 0, 0);
                    vTaskDelay(2000);
                    continue;
                }
            }

            canvas.fillRect(0, 55, 466, 345, TFT_BLACK);
            canvas.setTextColor(TFT_GREEN, TFT_BLACK);
            canvas.drawString("WiFi已连接", 233, 180);
            canvas.setTextColor(TFT_YELLOW, TFT_BLACK);
            canvas.drawString("正在同步时间...", 233, 220);
            canvas.pushSprite(&gfx, 0, 0);

            if (!sntp_initialized) {
                esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
                esp_sntp_setservername(0, "ntp.aliyun.com");
                esp_sntp_setservername(1, "ntp1.aliyun.com");
                esp_sntp_setservername(2, "pool.ntp.org");
                esp_sntp_setservername(3, "time.apple.com");
                sntp_set_sync_mode(SNTP_SYNC_MODE_IMMED);
                esp_sntp_init();
                sntp_initialized = true;
            }

            setenv("TZ", "CST-8", 1);
            tzset();

            int retry                      = 0;
            sntp_synced                    = false;
            sntp_sync_status_t sync_status = sntp_get_sync_status();
            while (sync_status == SNTP_SYNC_STATUS_RESET && retry < 100) {
                vTaskDelay(100);
                retry++;
                sync_status = sntp_get_sync_status();
            }

            if (sync_status == SNTP_SYNC_STATUS_COMPLETED) {
                sntp_synced = true;

                time_t now;
                struct tm timeinfo;
                time(&now);
                localtime_r(&now, &timeinfo);

                rx8130.setTime(&timeinfo);

                canvas.fillRect(0, 55, 466, 345, TFT_BLACK);
                canvas.setTextColor(TFT_GREEN, TFT_BLACK);
                canvas.drawString("时间同步成功！", 233, 200);

                canvas.setTextSize(2);
                sprintf(buf, "%d-%d-%d", timeinfo.tm_year + 2000, timeinfo.tm_mon + 1, timeinfo.tm_mday);
                canvas.drawString(buf, 233, 250);
                sprintf(buf, "%d:%d:%d", timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
                canvas.drawString(buf, 233, 290);

                canvas.pushSprite(&gfx, 0, 0);
                vTaskDelay(2000);
            } else {
                time_t now;
                struct tm info;
                time(&now);
                localtime_r(&now, &info);
                ESP_LOGI(TAG, "Sync failed, current time: %d-%d-%d %d:%d:%d", info.tm_year + 1900, info.tm_mon + 1,
                         info.tm_mday, info.tm_hour, info.tm_min, info.tm_sec);
                canvas.fillRect(0, 55, 466, 345, TFT_BLACK);
                canvas.setTextColor(TFT_RED, TFT_BLACK);
                canvas.drawString("时间同步失败", 233, 233);
                canvas.pushSprite(&gfx, 0, 0);
                vTaskDelay(2000);
            }
        }

        if (app._inputs.btnRightClicked) {
            return;
        }

        vTaskDelay(100);
    }
}

// 11. Vibration
void StopWatchApp::test_vibration()
{
    int strength = 0;
    // Motor on PY32_IO_EXPANDER, Pin 9
    io_expander.pinMode(PY32_MOTOR_EN_PIN, OUTPUT);
    io_expander.setPwmFrequency(5000);
    app.drawTitle(testList[10].name);
    canvas.fillRect(0, 55, 466, 345, TFT_BLACK);

    while (1) {
        char buf[32];
        sprintf(buf, "强度: %d%%", strength);
        canvas.fillRect(0, 55, 466, 345, TFT_BLACK);
        canvas.setTextDatum(top_center);
        canvas.drawString(buf, 233, 220);
        canvas.drawString("左键：增加强度 右键：退出", 233, 260);
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

// 12. Grove IO
void StopWatchApp::test_grove_io()
{
    app.drawTitle(testList[11].name);
    canvas.drawString("右键退出", 233, 60);
    gpio_reset_pin(GROVE_3_PIN);
    gpio_reset_pin(GROVE_4_PIN);
    gpio_set_direction(GROVE_3_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(GROVE_4_PIN, GPIO_MODE_OUTPUT);
    unsigned long last_time = 0;
    uint8_t gpio_3_level    = 1;
    uint8_t gpio_4_level    = 0;
    gpio_set_level(GROVE_3_PIN, gpio_3_level);
    gpio_set_level(GROVE_4_PIN, gpio_4_level);

    while (1) {
        app.updateInputs(false);
        if (app._inputs.btnRightClicked) return;

        if (getMillis() - last_time > 500) {
            gpio_3_level = !gpio_3_level;
            gpio_4_level = !gpio_4_level;
            gpio_set_level(GROVE_3_PIN, gpio_3_level);
            gpio_set_level(GROVE_4_PIN, gpio_4_level);
            last_time = getMillis();
        }
        char buf[32];
        sprintf(buf, "GPIO3: %d  GPIO4: %d", gpio_3_level, gpio_4_level);
        canvas.drawString(buf, 233, 233);
        canvas.pushSprite(&gfx, 0, 0);
        vTaskDelay(10);
    }
}

// 13. Bottom IO
void StopWatchApp::test_bottom_io()
{
    app.drawTitle(testList[12].name);
    canvas.drawString("右键退出", 233, 60);
    unsigned long last_time = 0;
    int idx                 = 3;  // 3-9
    while (1) {
        app.updateInputs(false);
        if (app._inputs.btnRightClicked) return;
        if (getMillis() - last_time > 500) {
            gpio_set_level((gpio_num_t)idx, 0);
            idx++;
            if (idx > 9) idx = 3;
            gpio_set_level((gpio_num_t)idx, 1);
            last_time = getMillis();
        }
        char buf[32];
        sprintf(buf, "GPIO %d HIGH", idx);
        canvas.drawString(buf, 233, 233);
        canvas.pushSprite(&gfx, 0, 0);
        vTaskDelay(10);
    }
}

// 14. CH442E
void StopWatchApp::test_ch442e()
{
    app.drawTitle(testList[13].name);
    canvas.drawString("左键切换 右键退出", 233, 60);
    int lvl = 0;
    while (1) {
        canvas.drawString(lvl ? " Level: HIGH " : " Level: LOW ", 233, 233);
        canvas.pushSprite(&gfx, 0, 0);
        app.updateInputs(false);
        if (app._inputs.btnLeftClicked) {
            lvl = !lvl;
            io_expander.digitalWrite(PY32_MUX_CTR_PIN, lvl);
        }
        if (app._inputs.btnRightClicked) return;
        vTaskDelay(50);
    }
}

// 15. WiFi Scan
void StopWatchApp::test_wifi_scan()
{
    app.drawTitle(testList[14].name);

    wifi_manager_init();

    unsigned long last_time = 0;
    canvas.drawString("扫描WIFI中", 233, 233);
    canvas.pushSprite(&gfx, 0, 0);

    while (1) {
        app.updateInputs(false);
        if (app._inputs.btnRightClicked) {
            return;
        }

        if (getMillis() - last_time > 3000) {
            wifi_scan_config_t scan_config = {0};
            esp_wifi_scan_start(&scan_config, true);

            uint16_t ap_num = 0;
            esp_wifi_scan_get_ap_num(&ap_num);
            wifi_ap_record_t *ap_list = (wifi_ap_record_t *)malloc(sizeof(wifi_ap_record_t) * ap_num);
            esp_wifi_scan_get_ap_records(&ap_num, ap_list);

            canvas.fillScreen(TFT_BLACK);
            app.drawTitle(testList[14].name);
            canvas.setTextSize(1.5);
            canvas.setTextDatum(top_center);
            int y = 60;
            for (int i = 0; i < ap_num && i < 10; i++) {
                char buf[64];
                sprintf(buf, "%s (%d)", ap_list[i].ssid, ap_list[i].rssi);
                canvas.drawString(buf, 233, y);
                y += 30;
            }
            free(ap_list);
            canvas.pushSprite(&gfx, 0, 0);
            last_time = getMillis();
        }
        vTaskDelay(10);
    }
}

// 16. WiFi 拉距测试
void StopWatchApp::test_wifi_distance()
{
    wifi_manager_init();

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(UDP_TARGET_IP);
    dest_addr.sin_family      = AF_INET;
    dest_addr.sin_port        = htons(UDP_TARGET_PORT);

    int sock              = -1;
    char rx_buffer[128]   = {0};
    const char *send_data = "Hello";

    int rssi            = -100;
    int tx_count        = 0;
    int ok_count        = 0;
    bool socket_created = false;

    uint32_t last_send_time     = 0;
    uint32_t last_rssi_time     = 0;
    uint32_t last_connect_retry = 0;
    uint32_t last_ok_time       = 0;

    canvas.fillScreen(TFT_BLACK);
    canvas.setTextDatum(middle_center);
    canvas.setTextColor(TFT_YELLOW, TFT_BLACK);
    canvas.drawString("正在连接 WiFi...", 233, 233);
    canvas.pushSprite(&gfx, 0, 0);

    if (!wifi_manager_is_connected()) {
        wifi_manager_connect(WIFI_TEST_SSID, WIFI_TEST_PASS, 5000);
    }

    while (1) {
        app.updateInputs(false);
        bool is_connected = wifi_manager_is_connected();

        char my_ip[32] = "0.0.0.0";
        wifi_manager_get_ip(my_ip, sizeof(my_ip));

        // A. 自动重连
        if (!is_connected) {
            if (getMillis() - last_connect_retry > 5000) {
                wifi_manager_connect(WIFI_TEST_SSID, WIFI_TEST_PASS, 100);
                last_connect_retry = getMillis();
            }
        }

        // B. Socket 管理
        if (is_connected && !socket_created) {
            if (strcmp(my_ip, "0.0.0.0") != 0) {
                sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
                if (sock >= 0) {
                    // 设置超时
                    struct timeval timeout;
                    timeout.tv_sec  = 0;
                    timeout.tv_usec = 50000;  // 50ms
                    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

                    // 绑定本地端口
                    struct sockaddr_in local_addr;
                    local_addr.sin_family      = AF_INET;
                    local_addr.sin_addr.s_addr = htonl(INADDR_ANY);
                    local_addr.sin_port        = htons(UDP_TARGET_PORT);

                    if (bind(sock, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
                        close(sock);
                    } else {
                        socket_created = true;
                    }
                }
            }
        } else if (!is_connected && socket_created) {
            close(sock);
            socket_created = false;
        }

        // C. 获取 RSSI
        // if (is_connected && (getMillis() - last_rssi_time > 1000)) {
        //     wifi_ap_record_t ap_info;
        //     if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        //         rssi = ap_info.rssi;
        //     }
        //     last_rssi_time = getMillis();
        // }

        // D. 发送数据 (每 1000ms 发送一次 "Hello")
        if (socket_created && (getMillis() - last_send_time > 1000)) {
            // 发送 5 字节: 'H','e','l','l','o'
            int err = sendto(sock, send_data, 5, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err > 0) {
                tx_count++;
            }
            last_send_time = getMillis();
        }

        // E. 接收数据 (检查 "OK")
        if (socket_created) {
            struct sockaddr_in source_addr;
            socklen_t socklen = sizeof(source_addr);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);

            if (len > 0) {
                rx_buffer[len] = 0;
                // 检查是否包含 "OK"
                if (len >= 2 && rx_buffer[0] == 'O' && rx_buffer[1] == 'K') {
                    ok_count++;
                    last_ok_time = getMillis();
                    // 收到 OK 时刷新一下 RSSI
                    wifi_ap_record_t ap_info;
                    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
                        rssi = ap_info.rssi;
                    }
                }
            }
        }

        // --- 界面绘制 ---
        canvas.fillScreen(TFT_BLACK);
        app.drawTitle(testList[15].name);

        // 1. 状态与 IP
        canvas.setTextDatum(top_center);
        canvas.setTextSize(1.5);
        if (is_connected) {
            canvas.setTextColor(TFT_GREEN, TFT_BLACK);
            char ip_buf[64];
            sprintf(ip_buf, "IP: %s", my_ip);
            canvas.drawString(ip_buf, 233, 70);
        } else {
            canvas.setTextColor(TFT_YELLOW, TFT_BLACK);
            canvas.drawString("连接 WiFi 中...", 233, 70);
        }

        // 2. RSSI 信号大字
        canvas.setTextDatum(middle_center);
        if (is_connected) {
            // 颜色逻辑：最近收到过 OK 显示绿色，否则虽然有 RSSI 但通信不通显示黄色
            bool link_alive     = (getMillis() - last_ok_time < 2000);
            uint16_t main_color = link_alive ? TFT_GREEN : TFT_YELLOW;

            if (rssi < -85) main_color = TFT_RED;

            canvas.setTextColor(main_color, TFT_BLACK);
            canvas.setTextSize(4);
            char rssi_str[16];
            sprintf(rssi_str, "%d dBm", rssi);
            canvas.drawString(rssi_str, 233, 160);

            // 信号条
            int bar_w = 200;
            int level = (rssi + 100) * 2;
            if (level < 0) level = 0;
            if (level > 100) level = 100;
            canvas.drawRect(133, 210, bar_w, 12, TFT_WHITE);
            canvas.fillRect(133 + 1, 210 + 1, (int)(level * 2), 10, main_color);

        } else {
            canvas.setTextColor(TFT_DARKGREY, TFT_BLACK);
            canvas.setTextSize(3);
            canvas.drawString("No Signal", 233, 160);
        }

        // 3. 统计数据
        canvas.setTextSize(1);
        canvas.setTextDatum(middle_left);
        canvas.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
        int info_y = 260;
        char buf[128];

        sprintf(buf, "Target: %s:%d", UDP_TARGET_IP, UDP_TARGET_PORT);
        canvas.drawString(buf, 40, info_y);

        // 发送统计
        canvas.setTextColor(TFT_CYAN, TFT_BLACK);
        sprintf(buf, "TX (Hello): %d", tx_count);
        canvas.drawString(buf, 40, info_y + 30);

        // 接收统计
        if (ok_count > 0) {
            // 计算丢包率或通信质量提示
            canvas.setTextColor(TFT_GREEN, TFT_BLACK);
            sprintf(buf, "RX (OK): %d  [Pass]", ok_count);
        } else {
            if (tx_count > 3) {
                canvas.setTextColor(TFT_RED, TFT_BLACK);
                sprintf(buf, "RX: 0 (No Response)");
            } else {
                canvas.setTextColor(TFT_ORANGE, TFT_BLACK);
                sprintf(buf, "RX: Waiting...");
            }
        }
        canvas.drawString(buf, 40, info_y + 60);

        // 提示信息
        canvas.setTextDatum(bottom_center);
        canvas.setTextColor(TFT_WHITE, TFT_BLACK);
        canvas.drawString("右键：退出", 233, 400);

        canvas.pushSprite(&gfx, 0, 0);

        // 退出
        if (app._inputs.btnRightClicked) {
            break;
        }

        vTaskDelay(20);
    }

    // --- 退出清理 ---
    if (socket_created) close(sock);
    wifi_manager_disconnect();
}

// Power Modes wrappers
void StopWatchApp::test_l0_mode()
{
    char buf[64];
    canvas.setTextDatum(bottom_center);
    canvas.setTextColor(TFT_WHITE, TFT_BLACK);
    canvas.setTextSize(1);
    for (int i = 0; i < 3; i++) {
        sprintf(buf, "3秒后设备进入L0模式,关闭所有外设(%d)", i + 1);
        canvas.drawString(buf, 233, 233);
        canvas.pushSprite(&gfx, 0, 0);
        vTaskDelay(1000);
    }
    stop_watch_power_mode_L0();
}
void StopWatchApp::test_l1_mode()
{
    char buf[64];
    canvas.setTextDatum(bottom_center);
    canvas.setTextColor(TFT_WHITE, TFT_BLACK);
    canvas.setTextSize(1);
    for (int i = 0; i < 3; i++) {
        sprintf(buf, "3秒后设备进入L1模式,保持IMU和RTC(%d)", i + 1);
        canvas.drawString(buf, 233, 233);
        canvas.pushSprite(&gfx, 0, 0);
        vTaskDelay(1000);
    }
    stop_watch_power_mode_L1();
}
void StopWatchApp::test_l2_mode()
{
    char buf[64];
    canvas.setTextDatum(bottom_center);
    canvas.setTextColor(TFT_WHITE, TFT_BLACK);
    canvas.setTextSize(1);
    for (int i = 0; i < 3; i++) {
        sprintf(buf, "3秒后设备进入L2低功耗休眠模式(%d)", i + 1);
        canvas.drawString(buf, 233, 233);
        canvas.pushSprite(&gfx, 0, 0);
        vTaskDelay(1000);
    }
    stop_watch_power_mode_L2();
}

void StopWatchApp::test_imu_wake()
{
    char buf[64];
    canvas.setTextDatum(bottom_center);
    canvas.setTextColor(TFT_WHITE, TFT_BLACK);
    canvas.setTextSize(1);
    for (int i = 0; i < 3; i++) {
        sprintf(buf, "3秒后esp32s3进入deepsleep,晃动设备唤醒(%d)", i + 1);
        canvas.drawString(buf, 233, 233);
        canvas.pushSprite(&gfx, 0, 0);
        vTaskDelay(1000);
    }
    wakeup_test(WakeupDevice::IMU_WAKEUP, WakeupMode::WAKEUP_DEEPSLEEP);
}
void StopWatchApp::test_imu_shutdown_wake()
{
    char buf[64];
    canvas.setTextDatum(bottom_center);
    canvas.setTextColor(TFT_WHITE, TFT_BLACK);
    canvas.setTextSize(1);
    for (int i = 0; i < 3; i++) {
        sprintf(buf, "3秒后设备关机,晃动设备唤醒(%d)", i + 1);
        canvas.drawString(buf, 233, 233);
        canvas.pushSprite(&gfx, 0, 0);
        vTaskDelay(1000);
    }
    wakeup_test(WakeupDevice::IMU_WAKEUP, WakeupMode::WAKEUP_SHUTDOWN);
}
void StopWatchApp::test_rtc_wake()
{
    char buf[64];
    canvas.setTextDatum(bottom_center);
    canvas.setTextColor(TFT_WHITE, TFT_BLACK);
    canvas.setTextSize(1);
    for (int i = 0; i < 3; i++) {
        sprintf(buf, "3秒后esp32s3进入deepsleep, 10秒后唤醒(%d)", i + 1);
        canvas.drawString(buf, 233, 233);
        canvas.pushSprite(&gfx, 0, 0);
        vTaskDelay(1000);
    }
    wakeup_test(WakeupDevice::RTC_WAKEUP, WakeupMode::WAKEUP_DEEPSLEEP);
}
void StopWatchApp::test_rtc_shutdown_wake()
{
    char buf[64];
    canvas.setTextDatum(bottom_center);
    canvas.setTextColor(TFT_WHITE, TFT_BLACK);
    canvas.setTextSize(1);
    for (int i = 0; i < 3; i++) {
        sprintf(buf, "3秒后设备关机, 10秒后唤醒(%d)", i + 1);
        canvas.drawString(buf, 233, 233);
        canvas.pushSprite(&gfx, 0, 0);
        vTaskDelay(1000);
    }
    wakeup_test(WakeupDevice::RTC_WAKEUP, WakeupMode::WAKEUP_SHUTDOWN);
}
void StopWatchApp::test_base_wake()
{
    char buf[64];
    canvas.setTextDatum(bottom_center);
    canvas.setTextColor(TFT_WHITE, TFT_BLACK);
    canvas.setTextSize(1);
    for (int i = 0; i < 3; i++) {
        sprintf(buf, "3秒后设备关机, 底座插入5V唤醒(%d)", i + 1);
        canvas.drawString(buf, 233, 233);
        canvas.pushSprite(&gfx, 0, 0);
        vTaskDelay(1000);
    }
    wakeup_test(WakeupDevice::PORT_WAKEUP, WakeupMode::WAKEUP_SHUTDOWN);
}

// 17. Flash Test
void StopWatchApp::test_flash()
{
    app.drawTitle(testList[16].name);
    canvas.fillRect(0, 55, 466, 345, TFT_BLACK);
    canvas.setTextDatum(top_center);
    canvas.setTextSize(1);
    canvas.setTextColor(TFT_WHITE, TFT_BLACK);

    int y_pos  = 70;
    int line_h = 25;

    canvas.drawString("正在读取Flash信息...", 233, y_pos);
    canvas.pushSprite(&gfx, 0, 0);

    // 获取芯片信息
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);

    // 获取Flash大小
    uint32_t flash_size = 0;
    esp_flash_get_size(NULL, &flash_size);

    // 获取Flash ID
    uint32_t flash_id = 0;
    esp_flash_read_id(NULL, &flash_id);

    // 获取Flash芯片名称
    const char *flash_name = "Unknown";
    if (esp_flash_default_chip && esp_flash_default_chip->chip_drv) {
        flash_name = esp_flash_default_chip->chip_drv->name;
    }

    // 显示Flash信息
    canvas.fillRect(0, 55, 466, 355, TFT_BLACK);
    y_pos = 70;

    canvas.setTextColor(TFT_CYAN, TFT_BLACK);
    canvas.drawString("=== Flash 信息 ===", 233, y_pos);
    y_pos += line_h + 5;

    canvas.setTextColor(TFT_WHITE, TFT_BLACK);
    char buf[64];

    sprintf(buf, "芯片: %s", CONFIG_IDF_TARGET);
    canvas.drawString(buf, 233, y_pos);
    y_pos += line_h;

    sprintf(buf, "Flash 大小: %lu MB", flash_size / (1024 * 1024));
    canvas.drawString(buf, 233, y_pos);
    y_pos += line_h;

    sprintf(buf, "Flash ID: 0x%08lX", flash_id);
    canvas.drawString(buf, 233, y_pos);
    y_pos += line_h;

    sprintf(buf, "制造商: 0x%02lX", flash_id & 0xFF);
    canvas.drawString(buf, 233, y_pos);
    y_pos += line_h;

    sprintf(buf, "Flash 型号: %s", flash_name);
    canvas.drawString(buf, 233, y_pos);
    y_pos += line_h;

    sprintf(buf, "类型: %s", (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "Embedded" : "External");
    canvas.drawString(buf, 233, y_pos);
    y_pos += line_h;

    sprintf(buf, "模式: %s", CONFIG_ESPTOOLPY_FLASHMODE);
    canvas.drawString(buf, 233, y_pos);
    y_pos += line_h;

    sprintf(buf, "速度: %s Hz", CONFIG_ESPTOOLPY_FLASHFREQ);
    canvas.drawString(buf, 233, y_pos);
    y_pos += line_h;

    sprintf(buf, "PSRAM: %.2f KB / %.2f KB", (double)heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024,
            (double)heap_caps_get_total_size(MALLOC_CAP_SPIRAM) / 1024);
    canvas.drawString(buf, 233, y_pos);
    y_pos += line_h;

    sprintf(buf, "Internal RAM: %.2f KB / %.2f KB", (double)heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024,
            (double)heap_caps_get_total_size(MALLOC_CAP_INTERNAL) / 1024);
    canvas.drawString(buf, 233, y_pos);
    y_pos += (line_h + 10);

    // Flash读写测试选项
    canvas.setTextColor(TFT_YELLOW, TFT_BLACK);
    canvas.drawString("左键: 读写速度测试", 233, y_pos);
    y_pos += line_h;

    canvas.setTextColor(TFT_YELLOW, TFT_BLACK);
    canvas.drawString("右键: 退出", 233, y_pos);

    canvas.pushSprite(&gfx, 0, 0);

    // 等待按键
    while (1) {
        app.updateInputs(false);

        if (app._inputs.btnLeftClicked) {
            // 查找测试分区
            const esp_partition_t *test_partition =
                esp_partition_find_first(ESP_PARTITION_TYPE_DATA, (esp_partition_subtype_t)0xFF, "test");

            if (!test_partition) {
                canvas.fillRect(0, 55, 466, 355, TFT_BLACK);
                canvas.setTextColor(TFT_RED, TFT_BLACK);
                canvas.setTextDatum(middle_center);
                canvas.drawString("未找到测试分区!", 233, 200);
                canvas.drawString("请检查 partitions.csv", 233, 230);
                canvas.pushSprite(&gfx, 0, 0);
                vTaskDelay(3000);
                continue;
            }

            // 执行读写速度测试
            canvas.fillRect(0, 55, 466, 355, TFT_BLACK);
            y_pos = 70;
            canvas.setTextDatum(top_center);

            canvas.setTextColor(TFT_CYAN, TFT_BLACK);
            canvas.drawString("=== Flash 读写测试 ===", 233, y_pos);
            y_pos += line_h + 5;

            canvas.setTextColor(TFT_YELLOW, TFT_BLACK);
            sprintf(buf, "测试分区: 0x%lX", test_partition->address);
            canvas.drawString(buf, 233, y_pos);
            y_pos += line_h;

            sprintf(buf, "分区大小: %lu KB", test_partition->size / 1024);
            canvas.drawString(buf, 233, y_pos);
            y_pos += line_h + 5;

            canvas.drawString("擦除分区...", 233, y_pos);
            canvas.pushSprite(&gfx, 0, 0);

            // 擦除测试分区
            uint32_t erase_start = esp_timer_get_time();
            esp_err_t err        = esp_partition_erase_range(test_partition, 0, test_partition->size);
            uint32_t erase_time  = esp_timer_get_time() - erase_start;

            if (err != ESP_OK) {
                canvas.fillRect(0, 55, 466, 355, TFT_BLACK);
                canvas.setTextColor(TFT_RED, TFT_BLACK);
                canvas.drawString("擦除失败!", 233, 200);
                canvas.pushSprite(&gfx, 0, 0);
                vTaskDelay(2000);
                continue;
            }

            // 准备测试数据
            const size_t test_size = 4096;  // 4KB
            const int test_count   = 100;
            uint8_t *write_buf     = (uint8_t *)malloc(test_size);
            uint8_t *read_buf      = (uint8_t *)malloc(test_size);

            if (write_buf && read_buf) {
                // 填充测试数据
                for (int i = 0; i < test_size; i++) {
                    write_buf[i] = (i & 0xFF) ^ 0xAA;
                }

                canvas.fillRect(0, 55, 466, 355, TFT_BLACK);
                y_pos = 70;
                canvas.setTextColor(TFT_YELLOW, TFT_BLACK);
                canvas.drawString("写入测试中...", 233, y_pos);
                canvas.pushSprite(&gfx, 0, 0);

                // 写入速度测试
                uint32_t write_start = esp_timer_get_time();
                for (int i = 0; i < test_count; i++) {
                    esp_partition_write(test_partition, i * test_size, write_buf, test_size);
                }
                uint32_t write_time = esp_timer_get_time() - write_start;

                canvas.drawString("读取测试中...", 233, y_pos + line_h);
                canvas.pushSprite(&gfx, 0, 0);

                // 读取速度测试
                uint32_t read_start = esp_timer_get_time();
                for (int i = 0; i < test_count; i++) {
                    esp_partition_read(test_partition, i * test_size, read_buf, test_size);
                }
                uint32_t read_time = esp_timer_get_time() - read_start;

                // 数据验证
                bool verify_ok = true;
                esp_partition_read(test_partition, 0, read_buf, test_size);
                for (int i = 0; i < test_size; i++) {
                    if (read_buf[i] != write_buf[i]) {
                        verify_ok = false;
                        break;
                    }
                }

                // 计算速度
                float write_speed = (test_size * test_count / 1024.0f) / (write_time / 1000000.0f);  // KB/s
                float read_speed  = (test_size * test_count / 1024.0f) / (read_time / 1000000.0f);   // KB/s
                float erase_speed = (test_partition->size / 1024.0f) / (erase_time / 1000000.0f);    // KB/s

                // 显示测试结果
                canvas.fillRect(0, 55, 466, 345, TFT_BLACK);
                y_pos = 70;

                canvas.setTextColor(TFT_CYAN, TFT_BLACK);
                canvas.drawString("=== 测试结果 ===", 233, y_pos);
                y_pos += line_h + 5;

                canvas.setTextColor(TFT_WHITE, TFT_BLACK);
                sprintf(buf, "测试块: %d 字节 x %d", test_size, test_count);
                canvas.drawString(buf, 233, y_pos);
                y_pos += line_h + 5;

                // 擦除结果
                canvas.setTextColor(TFT_ORANGE, TFT_BLACK);
                sprintf(buf, "擦除: %lu ms", erase_time / 1000);
                canvas.drawString(buf, 233, y_pos);
                y_pos += line_h;
                sprintf(buf, "速度: %.2f KB/s", erase_speed);
                canvas.drawString(buf, 233, y_pos);
                y_pos += line_h + 5;

                // 写入结果
                canvas.setTextColor(TFT_YELLOW, TFT_BLACK);
                sprintf(buf, "写入: %lu ms", write_time / 1000);
                canvas.drawString(buf, 233, y_pos);
                y_pos += line_h;
                sprintf(buf, "速度: %.2f KB/s (%.2f MB/s)", write_speed, write_speed / 1024.0f);
                canvas.drawString(buf, 233, y_pos);
                y_pos += line_h + 5;

                // 读取结果
                canvas.setTextColor(TFT_GREEN, TFT_BLACK);
                sprintf(buf, "读取: %lu ms", read_time / 1000);
                canvas.drawString(buf, 233, y_pos);
                y_pos += line_h;
                sprintf(buf, "速度: %.2f KB/s (%.2f MB/s)", read_speed, read_speed / 1024.0f);
                canvas.drawString(buf, 233, y_pos);
                y_pos += line_h + 5;

                // 验证结果
                if (verify_ok) {
                    canvas.setTextColor(TFT_GREEN, TFT_BLACK);
                    canvas.drawString("数据验证: 通过", 233, y_pos);
                } else {
                    canvas.setTextColor(TFT_RED, TFT_BLACK);
                    canvas.drawString("数据验证: 失败", 233, y_pos);
                }
                y_pos += line_h + 10;

                canvas.setTextColor(TFT_WHITE, TFT_BLACK);
                canvas.drawString("按右键退出", 233, y_pos);

                free(write_buf);
                free(read_buf);
            } else {
                canvas.fillRect(0, 55, 466, 355, TFT_BLACK);
                canvas.setTextColor(TFT_RED, TFT_BLACK);
                canvas.drawString("内存不足！", 233, 200);
                if (write_buf) free(write_buf);
                if (read_buf) free(read_buf);
            }

            canvas.pushSprite(&gfx, 0, 0);
        }

        if (app._inputs.btnRightClicked) {
            return;
        }

        vTaskDelay(10);
    }
}

// 22. Full Load
void StopWatchApp::test_full_load()
{
    app.drawTitle("Full Load Mode");
    canvas.fillScreen(TFT_WHITE);
    canvas.pushSprite(&gfx, 0, 0);

    io_expander.setPwmDuty(PY32_MOTOR_PWM_CHANNEL, 100, false, true);

    wifi_manager_init();

    audio_init();
    audio_set_volume(100);
    audio_speaker_enable(true);
    audio_play_stop();

    unsigned long last_time = 0;
    while (1) {
        app.updateInputs(false);
        if (app._inputs.btnRightClicked) {
            io_expander.setPwmDuty(PY32_MOTOR_PWM_CHANNEL, 0, false, true);
            audio_play_stop();
            // audio_deinit();
            return;
        }
        // 循环播放音频
        if (!audio_is_playing()) {
            audio_play_demo(audio_type_t::AUDIO_DEMO_PIANO);
        }
        // 3秒扫描一次WIFI
        if (getMillis() - last_time > 3000) {
            canvas.fillScreen(TFT_WHITE);
            canvas.setTextColor(TFT_BLACK, TFT_WHITE);
            canvas.drawString("扫描WIFI中", 233, 233);
            canvas.pushSprite(&gfx, 0, 0);
            wifi_scan_config_t scan_config = {0};
            esp_wifi_scan_start(&scan_config, true);
            last_time = getMillis();
            canvas.drawString("WIFI扫描完成,三秒后重新扫描", 233, 233);
            canvas.pushSprite(&gfx, 0, 0);
        }
        vTaskDelay(10);
    }
}

// 23. Aging
void StopWatchApp::test_aging()
{
    wifi_manager_init();
    if (!wifi_manager_is_connected()) {
        wifi_manager_connect(WIFI_SSID, WIFI_PASS, 10000);
    }
    while (1) {
        app.updateInputs(false);
        if (app._inputs.btnRightClicked) {
            gfx.setRotation(0);
            return;
        }
        // 低电量关机
        static uint16_t vbat           = 0;
        static uint32_t vbat_last_read = 0;
        if (lgfx::v1::millis() - vbat_last_read > 1000) {
            pm1_vbat_read(&vbat);
            if (vbat < 3800) {
                canvas.drawString("电压低于3800mV,三秒后关机", 233, 233);
                canvas.pushSprite(&gfx, 0, 0);
                vTaskDelay(3000);
                pm1_sys_cmd(PM1_SYS_CMD_SHUTDOWN);
                return;
            }
            vbat_last_read = lgfx::v1::millis();
        }
        // 刷新显示
        display_gfx_loop();
    }
}

// 28. PMIC定时关机
void StopWatchApp::test_pmic_timer_shutdown()
{
    bool is_start       = false;
    uint32_t time_start = 0;
    canvas.setTextColor(TFT_WHITE, TFT_BLACK);
    canvas.setTextDatum(middle_center);
    while (1) {
        app.updateInputs(false);
        if (app._inputs.btnRightClicked) {
            pm1_tim_set(PM1_ADDR_TIM_DISABLE, PM1_TIM_ACTION_000, 0);
            is_start = false;
            return;
        }
        if (app._inputs.btnLeftClicked) {
            if (is_start) {
                pm1_tim_set(PM1_ADDR_TIM_ENABLE, PM1_TIM_ACTION_000, 0);
                is_start = false;
            } else {
                pm1_tim_set(PM1_ADDR_TIM_ENABLE, PM1_TIM_ACTION_100, 5);
                is_start   = true;
                time_start = lgfx::v1::millis();
            }
        }
        if (is_start) {
            char buf[100];
            sprintf(buf, "开始计时，%d秒后关机，右键退出", 5 - (uint8_t)((lgfx::v1::millis() - time_start) / 1000));
            canvas.fillScreen(TFT_BLACK);
            canvas.drawString(buf, 233, 233);
            canvas.pushSprite(&gfx, 0, 0);
        } else {
            canvas.fillScreen(TFT_BLACK);
            canvas.drawString("左键: 开始计时, 右键: 退出", 233, 233);
            canvas.pushSprite(&gfx, 0, 0);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// 29. PMIC定时开机
void StopWatchApp::test_pmic_timer_wake()
{
    bool is_start       = false;
    uint32_t time_start = 0;
    canvas.setTextColor(TFT_WHITE, TFT_BLACK);
    canvas.setTextDatum(middle_center);
    while (1) {
        app.updateInputs(false);
        if (app._inputs.btnRightClicked) {
            pm1_tim_set(PM1_ADDR_TIM_DISABLE, PM1_TIM_ACTION_000, 0);
            is_start = false;
            return;
        }
        if (app._inputs.btnLeftClicked) {
            if (is_start) {
                pm1_tim_set(PM1_ADDR_TIM_ENABLE, PM1_TIM_ACTION_000, 0);
                is_start = false;
            } else {
                is_start   = true;
                time_start = lgfx::v1::millis();
            }
        }
        if (is_start) {
            char buf[100];
            sprintf(buf, "开始计时，%d秒后关机，5秒后开机，右键退出",
                    5 - (uint8_t)((lgfx::v1::millis() - time_start) / 1000));
            canvas.fillScreen(TFT_BLACK);
            canvas.drawString(buf, 233, 233);
            canvas.pushSprite(&gfx, 0, 0);
            if (lgfx::v1::millis() - time_start > 5000) {
                pm1_tim_set(PM1_ADDR_TIM_ENABLE, PM1_TIM_ACTION_011, 5);
                vTaskDelay(50 / portTICK_PERIOD_MS);
                pm1_sys_cmd(PM1_SYS_CMD_SHUTDOWN);
            }
        } else {
            canvas.fillScreen(TFT_BLACK);
            canvas.drawString("左键: 准备关机, 右键: 退出", 233, 233);
            canvas.pushSprite(&gfx, 0, 0);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

// 30. PMIC定时重启
void StopWatchApp::test_pmic_timer_restart()
{
    bool is_start       = false;
    uint32_t time_start = 0;
    canvas.setTextColor(TFT_WHITE, TFT_BLACK);
    canvas.setTextDatum(middle_center);
    while (1) {
        app.updateInputs(false);
        if (app._inputs.btnRightClicked) {
            pm1_tim_set(PM1_ADDR_TIM_DISABLE, PM1_TIM_ACTION_000, 0);
            is_start = false;
            return;
        }
        if (app._inputs.btnLeftClicked) {
            if (is_start) {
                pm1_tim_set(PM1_ADDR_TIM_ENABLE, PM1_TIM_ACTION_000, 0);
                is_start = false;
            } else {
                pm1_tim_set(PM1_ADDR_TIM_ENABLE, PM1_TIM_ACTION_010, 5);
                is_start   = true;
                time_start = lgfx::v1::millis();
            }
        }
        if (is_start) {
            char buf[100];
            sprintf(buf, "开始计时，%d秒后重启，右键退出", 5 - (uint8_t)((lgfx::v1::millis() - time_start) / 1000));
            canvas.fillScreen(TFT_BLACK);
            canvas.drawString(buf, 233, 233);
            canvas.pushSprite(&gfx, 0, 0);
        } else {
            canvas.fillScreen(TFT_BLACK);
            canvas.drawString("左键: 准备重启, 右键: 退出", 233, 233);
            canvas.pushSprite(&gfx, 0, 0);
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}