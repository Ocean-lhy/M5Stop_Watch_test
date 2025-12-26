#include "display_user.h"

M5StopWatch gfx;
M5Canvas canvas(&gfx);

void display_init()
{
    gfx.init();
    gfx.fillScreen(TFT_BLACK);
    gfx.setTextColor(TFT_WHITE);
    gfx.setTextSize(2);
    gfx.setFont(&fonts::efontCN_16);
    canvas.createSprite(gfx.width(), gfx.height());
    canvas.fillScreen(TFT_BLACK);
    canvas.setTextColor(TFT_WHITE);
    canvas.setTextSize(2);
    canvas.setFont(&fonts::efontCN_16);
    canvas.pushSprite(&gfx, 0, 0);
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
        gfx.println("Stop Watch!");
    }

    // printf("M5StopWatch loop\n");fflush(stdout);
    gfx.fillCircle(105 + (rand() & 255), 105 + (rand() & 255), 16, rand());
}