/* Copyright 2024
 *
 * Licensed under the Apache License, Version 2.0.
 */

#include "app/spiritbox.h"
#include "app/action.h"
#include "app/fm.h"
#include "audio.h"
#include "driver/bk1080.h"
#include "driver/bk4819.h"   
#include "driver/st7565.h"
#include "driver/system.h"
#include "driver/keyboard.h"
#include "driver/backlight.h"
#include "ui/helper.h"
#include "ui/ui.h"
#include "ui/status.h"
#include "misc.h"
#include "settings.h"
#include "external/printf/printf.h"
#include <string.h>

#if defined(ENABLE_UART) || defined(ENABLE_USB)
#include "app/uart.h"
#endif

#ifdef ENABLE_FEAT_F4HWN_K5VIEWER
#include "k5viewer.h"
#endif

// アナログメーターの針の先端座標テーブル (オリジナルサイズ: 半径40, 中心(64,60))
static const uint8_t NeedleTable[101][2] = {
    {29, 40}, {29, 39}, {30, 39}, {30, 38}, {31, 38}, {31, 37}, {32, 36}, {32, 36}, {33, 35}, {33, 35},
    {34, 34}, {34, 34}, {35, 33}, {36, 32}, {36, 32}, {37, 31}, {37, 31}, {38, 30}, {39, 30}, {40, 29},
    {40, 28}, {41, 28}, {42, 27}, {42, 27}, {43, 26}, {44, 26}, {45, 25}, {45, 25}, {46, 24}, {47, 24},
    {48, 23}, {49, 23}, {49, 22}, {50, 22}, {51, 21}, {52, 21}, {53, 21}, {54, 20}, {55, 20}, {55, 20},
    {56, 20}, {57, 20}, {58, 20}, {59, 20}, {60, 20}, {61, 20}, {62, 20}, {63, 20}, {64, 20}, {65, 20},
    {66, 20}, {67, 20}, {68, 20}, {69, 20}, {69, 20}, {70, 20}, {71, 20}, {72, 21}, {73, 21}, {74, 21},
    {75, 22}, {75, 22}, {76, 23}, {77, 23}, {78, 24}, {79, 24}, {79, 25}, {80, 25}, {81, 26}, {82, 26},
    {82, 27}, {83, 27}, {84, 28}, {85, 28}, {85, 29}, {86, 30}, {87, 30}, {87, 31}, {88, 31}, {89, 32},
    {89, 32}, {90, 33}, {90, 34}, {91, 34}, {91, 35}, {92, 35}, {92, 36}, {93, 36}, {93, 37}, {94, 38},
    {94, 38}, {95, 39}, {95, 39}, {95, 40}, {96, 41}, {96, 42}, {97, 42}, {97, 43}, {97, 44}, {98, 45},
    {98, 46}
};

static uint16_t sweep_speed_ms = 200;

// スキャンモードの定義
enum {
    MODE_ASC = 0,
    MODE_DSC,
    MODE_RND
};
static const char *mode_names[] = {"ASC", "DSC", "RND"};

// Xorshiftアルゴリズムを用いた簡易で高速な乱数生成
static uint32_t xorshift32(uint32_t *state) {
    uint32_t x = *state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    *state = x;
    return x;
}

void ACTION_SpiritBox(void)
{
    APP_RunSpiritBox();
    GUI_SelectNextDisplay(DISPLAY_MAIN);
}

void APP_RunSpiritBox(void)
{
    bool isRunning = true;
    uint32_t freq = BK1080_GetFreqLoLimit(gEeprom.FM_Band);
    uint32_t freqHi = BK1080_GetFreqHiLimit(gEeprom.FM_Band);
    uint32_t freqLo = freq;
    
    uint8_t valid_channels[FM_CHANNELS_MAX];
    uint8_t valid_count = 0;
    for (uint8_t i = 0; i < FM_CHANNELS_MAX; i++) {
        if (FM_CheckValidChannel(i)) {
            valid_channels[valid_count++] = i;
        }
    }

    uint32_t rand_state = SysTick->VAL;
    if (rand_state == 0) rand_state = 12345;

    int32_t meter_val = 0;
    int32_t meter_vel = 0;   
    int32_t target_val = 0;

    uint16_t sweep_tick = 0;
    uint16_t meter_update_tick = 0;
    
    uint16_t current_rssi = 0; 
    int32_t  current_val = 0;

    uint8_t current_mode = MODE_ASC; // 初期モード: 昇順
    uint8_t current_mem_idx = 0;     // 現在のメモリインデックス

    BACKLIGHT_TurnOn();
    //UI_DisplayClear();
    UI_DisplayStatus();
    
    gKeyReading0 = KEY_INVALID;
    gKeyReading1 = KEY_INVALID;

    RADIO_SetupRegisters(true);

    BK4819_SetAF(BK4819_AF_MUTE);
    BK4819_PickRXFilterPathBasedOnFrequency(10320000); 
    
    if (valid_count > 0) {
        freq = gFM_Channels[valid_channels[current_mem_idx]];
    }
    BK1080_Init(freq, gEeprom.FM_Band);
    
    BK4819_SetFrequency(freq * 10000); 
    uint16_t reg30 = BK4819_ReadRegister(BK4819_REG_30);
    BK4819_WriteRegister(BK4819_REG_30, 0);
    BK4819_WriteRegister(BK4819_REG_30, reg30);

    BK1080_WriteRegister(BK1080_REG_02_POWER_CONFIGURATION, 0x0201);
    BK1080_WriteRegister(BK1080_REG_05_SYSTEM_CONFIGURATION2, 0x0A1F); 
    
    AUDIO_AudioPathOn();
    gEnableSpeaker = true;

    #ifdef ENABLE_FEAT_F4HWN_K5VIEWER
    K5VIEWER_Update(true);
    #endif

    KEY_Code_t prevKey = KEY_INVALID;

    while (isRunning) {

#if defined(ENABLE_UART) || defined(ENABLE_USB)
        UART_ServiceCommands();
#endif
#ifdef ENABLE_FEAT_F4HWN_K5VIEWER
        K5VIEWER_ParseInput(); // PCからの通信（Pingやリモートキー操作）を受け取る
#endif

        KEY_Code_t key = KEYBOARD_Poll();
        if (key != prevKey) {
            if (key == KEY_EXIT) {
                isRunning = false;
            } else if (key == KEY_UP) {
                if (sweep_speed_ms > 20) sweep_speed_ms -= 20;
            } else if (key == KEY_DOWN) {
                if (sweep_speed_ms < 1000) sweep_speed_ms += 20;
            } else if (key == KEY_MENU) {
                // MENUキーでスキャンモード切替 (ASC -> DSC -> RND -> ASC...)
                current_mode = (current_mode + 1) % 3;
            }
            prevKey = key;
        }
        
        sweep_tick += 20;
        if (sweep_tick >= sweep_speed_ms) {
            sweep_tick = 0;
            
            // --- 選択中のモードに応じて周波数を切り替える ---
            if (valid_count > 1) {
                if (current_mode == MODE_RND) {
                    current_mem_idx = xorshift32(&rand_state) % valid_count;
                } else if (current_mode == MODE_ASC) {
                    current_mem_idx++;
                    if (current_mem_idx >= valid_count) current_mem_idx = 0;
                } else if (current_mode == MODE_DSC) {
                    if (current_mem_idx == 0) current_mem_idx = valid_count - 1;
                    else current_mem_idx--;
                }
                freq = gFM_Channels[valid_channels[current_mem_idx]];
            } else {
                // メモリがない場合（VFO帯域全体をスイープ）
                if (current_mode == MODE_RND) {
                    uint32_t range = freqHi - freqLo + 1;
                    freq = freqLo + (xorshift32(&rand_state) % range);
                } else if (current_mode == MODE_DSC) {
                    if (freq <= freqLo) freq = freqHi;
                    else freq -= 1;
                } else {
                    freq += 1;
                    if (freq > freqHi) freq = freqLo;
                }
            }
            
            BK1080_SetFrequency(freq, gEeprom.FM_Band);
            
            BK4819_SetFrequency(freq * 10000); 
            reg30 = BK4819_ReadRegister(BK4819_REG_30);
            BK4819_WriteRegister(BK4819_REG_30, 0);
            BK4819_WriteRegister(BK4819_REG_30, reg30);
        }
        
        meter_update_tick += 20;
        if (meter_update_tick >= 500) {
            meter_update_tick = 0;
            
            uint16_t rssi = BK4819_GetRSSI();
            current_rssi = rssi; 
            
            int32_t base_val = (rssi - 80); 
            if (base_val < 0) base_val = 0;
            if (base_val > 100) base_val = 100;
            
            uint32_t r = xorshift32(&rand_state);
            uint32_t fluctuation = (r % 51) + 50; 
            
            int32_t val = (base_val * fluctuation) / 100;
            val += ((r >> 8) % 7) - 3; 

            if (val < 0) val = 0;
            if (val > 100) val = 100;

            current_val = val; 
            target_val = val * 1000;
        }

        int32_t diff = target_val - meter_val;
        meter_vel += (diff * 150) / 1000;      
        meter_vel = (meter_vel * 800) / 1000;  
        meter_val += meter_vel;

        if (meter_val < 0) meter_val = 0;
        if (meter_val > 100000) meter_val = 100000;

        UI_DisplayClear();
        //UI_StatusClear();
        UI_DisplayStatus();
        
        // --- テキスト描画 ---
        //UI_PrintStringSmallBold("UV SPIRIT BOX", 32, 0, 0);
        UI_PrintStringSmallBold("UV SPIRIT BOX", 0, 0, 0);

        char str[16];
        
        // R (左寄せ) / V (右寄せ) [Line 1]
        sprintf(str, "RSSI:%d", current_rssi);
        UI_PrintStringSmallNormal(str, 0, 0, 1);
        sprintf(str, "V:%3d%%", current_val);
        UI_PrintStringSmallNormal(str, 127 - (strlen(str) * 7), 0, 1);

        // ★スキャンモードの表示 [Line 2]
        sprintf(str, "MOD:%s", mode_names[current_mode]);
        UI_PrintStringSmallNormal(str, 0, 0, 2);

        // 周波数 & 速度表示 (最下段) [Line 6]
        if (valid_count > 0) {
            sprintf(str, "MEM: %3d.%d", freq / 10, freq % 10);
        } else {
            sprintf(str, "VFO: %3d.%d", freq / 10, freq % 10);
        }
        UI_PrintStringSmallNormal(str, 0, 0, 6);

        sprintf(str, "SPD:%3d", sweep_speed_ms);
        UI_PrintStringSmallNormal(str, 127 - (strlen(str) * 7), 0, 6);

        // --- メーター描画 (パラメーター化版) ---
        const uint8_t METER_CX = 64;       
        const uint8_t METER_CY = 40;       
        const uint8_t BASELINE_W = 30; 
        
        const int SCALE_X_TICK   = 85;     
        const int SCALE_Y_TICK   = 50;     
        const int SCALE_X_NEEDLE = 75;     
        const int SCALE_Y_NEEDLE = 40;     

        // ベースライン
        UI_DrawLineBuffer(gFrameBuffer, METER_CX - BASELINE_W, METER_CY, METER_CX + BASELINE_W, METER_CY, true);
        
        // 目盛り (10段階ごと)
        for (int i = 0; i <= 100; i += 10) {
            int dx = NeedleTable[i][0] - 64;
            int dy = NeedleTable[i][1] - 60;
            uint8_t sx = METER_CX + (dx * SCALE_X_TICK / 100);
            uint8_t sy = METER_CY + (dy * SCALE_Y_TICK / 100);
            UI_DrawPixelBuffer(gFrameBuffer, sx, sy, true);
        }

        // 針本体
        uint8_t needle_idx = meter_val / 1000;
        if (needle_idx > 100) needle_idx = 100;
        int dx = NeedleTable[needle_idx][0] - 64;
        int dy = NeedleTable[needle_idx][1] - 60;
        uint8_t nx = METER_CX + (dx * SCALE_X_NEEDLE / 100);
        uint8_t ny = METER_CY + (dy * SCALE_Y_NEEDLE / 100);
        UI_DrawLineBuffer(gFrameBuffer, METER_CX, METER_CY, nx, ny, true);
        
        // 針の支点の四角
        UI_DrawRectangleBuffer(gFrameBuffer, METER_CX - 1, METER_CY - 1, METER_CX + 1, METER_CY + 1, true);

        ST7565_BlitStatusLine();
        ST7565_BlitFullScreen();

#ifdef ENABLE_FEAT_F4HWN_K5VIEWER
        K5VIEWER_Update(false);
#endif

        SYSTEM_DelayMs(10);
    }

    AUDIO_AudioPathOff();
    gEnableSpeaker = false;
    BK1080_WriteRegister(BK1080_REG_02_POWER_CONFIGURATION, 0x0241);
    gFlagReconfigureVfos = true;
}