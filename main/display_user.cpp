#include "display_user.h"

M5StopWatch gfx;

void display_init()
{
    gfx.init();
    gfx.fillScreen(TFT_BLUE);
    gfx.setTextColor(TFT_WHITE);
    gfx.setTextSize(2);
}

void display_gfx_loop()
{
    static uint32_t prev_sec;
    static uint32_t count;
    count++;
    uint32_t t   = lgfx::v1::millis();
    uint32_t sec = t / 1000;
    if (prev_sec != sec) {
        prev_sec = sec;
        printf("count:%d\n", (int)count);
        fflush(stdout);
        lgfx::v1::delay(1);
        count = 0;
        gfx.fillScreen(rand());
        gfx.setRotation(gfx.getRotation() + 1);
        gfx.setCursor(112, 48);
        gfx.println("Hello! Stop Watch!");
    }

    // printf("M5StopWatch loop\n");fflush(stdout);
    gfx.fillCircle(105 + (rand() & 255), 105 + (rand() & 255), 16, rand());
}