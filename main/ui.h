#pragma once
#include "display_user.h"
#include "setting.h"
#include "power_management.h"
#include "touch_user.h"
#include "audio_user.h"
#include "imu_user.h"

// 定义测试项结构
struct TestItem {
    const char* name;
    void (*func)(void);
};

// UI主程序类
class StopWatchApp {
public:
    void init();
    void loop();

    enum AudioMode {
        MODE_PLAY_PIANO = 0,
        MODE_PLAY_TONE,
        MODE_PLAY_DEAR,
        MODE_RECORDER,
        MODE_COUNT
    };

    // 具体测试用例函数
    static void test_main_page();
    static void test_touch_draw();
    static void test_audio_record();
    static void test_audio_play_0();
    static void test_audio_play_1();
    static void test_audio_play_2();
    static void test_imu_data();
    static void test_vibration();
    static void test_audio_unified(int current_mode_idx);

private:
    // 基础UI功能
    void drawHeader();
    void drawFooter();
    void drawMenu();
    void updateInputs(bool updateTouch = false);

    // 输入快照状态
    struct InputState {
        bool btnLeftClicked  = false;
        bool btnRightClicked = false;
        bool touchClicked    = false;
        int touchX           = -1;
        int touchY           = -1;
    } _inputs;

    // 内部物理追踪变量
    struct PhysicalTracker {
        int lastState                 = 1;
        unsigned long lastTriggerTime = 0;
    } _trackL, _trackR;

    bool _isTouching = false;
    int _touchStartX = -1;
    float _touchStartY = 0;      // 触摸起始点 Y
    float _startScrollOffset = 0; // 触摸起始时的滚动偏移
    bool  _isDragging = false;    // 是否正在拖拽

    // 菜单状态
    int menu_index       = 0;
    int _last_menu_index = 0;
    float scroll_offset  = 0.0f;  // 当前滚动的渲染偏移（平滑跟随 menu_index）
    float target_offset  = 0.0f;  // 目标偏移
    const float spring_k = 0.3f;  // 弹性系数 (0.1 - 0.5)，值越小越丝滑

    // 工具函数
    bool checkTouchRegion(int x1, int y1, int x2, int y2);  // 屏幕点击
    void drawTitle(const char* title);
};

extern StopWatchApp app;