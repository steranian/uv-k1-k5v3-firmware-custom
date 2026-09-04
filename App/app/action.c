/* Copyright 2023 Dual Tachyon
 * https://github.com/DualTachyon
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *     Unless required by applicable law or agreed to in writing, software
 *     distributed under the License is distributed on an "AS IS" BASIS,
 *     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *     See the License for the specific language governing permissions and
 *     limitations under the License.
 */

#include <assert.h>
#include <string.h>
#include "external/printf/printf.h"

#include "app/action.h"
#include "app/app.h"
#include "app/chFrScanner.h"
#include "app/common.h"
#include "app/dtmf.h"
#ifdef ENABLE_FLASHLIGHT
    #include "app/flashlight.h"
#endif
#ifdef ENABLE_FMRADIO
    #include "app/fm.h"
#endif
#include "app/scanner.h"
#include "audio.h"
#ifdef ENABLE_FMRADIO
    #include "driver/bk1080.h"
#endif
#include "driver/bk4819.h"
#include "driver/gpio.h"
#include "driver/backlight.h"
#include "driver/st7565.h"
#include "driver/system.h"


#include "functions.h"
#include "misc.h"
#include "settings.h"
#include "ui/inputbox.h"
#include "ui/main.h"
#include "ui/ui.h"
#include "ui/status.h"
#include "ui/helper.h"


#ifdef ENABLE_FEAT_F4HWN_BEAM
    #include "app/beam.h"
#endif
#ifdef ENABLE_FEAT_F4HWN_RXTX_LOG
    #include "app/rxtx_log.h"
#endif
#ifdef ENABLE_FEAT_F4HWN_FOXHUNT
    #include "app/foxhunt.h"
#endif
#ifdef ENABLE_FEAT_F4HWN_ACTION_PICKER
    #include "ui/menu.h"
#endif

#ifdef ENABLE_FEAT_STERANIAN_SPIRITBOX
    #include "app/spiritbox.h"
#endif

#if defined(ENABLE_UART) || defined(ENABLE_USB)
#include "app/uart.h"
#endif
#ifdef ENABLE_FEAT_F4HWN_K5VIEWER
#include "k5viewer.h"
#endif

#if defined(ENABLE_FMRADIO)
static void ACTION_Scan_FM(bool bRestart);
#endif

#if defined(ENABLE_ALARM) || defined(ENABLE_TX1750)
static void ACTION_AlarmOr1750(bool b1750);
inline static void ACTION_Alarm() { ACTION_AlarmOr1750(false); }
inline static void ACTION_1750() { ACTION_AlarmOr1750(true); };
#endif

inline static void ACTION_ScanRestart() { ACTION_Scan(true); };

void (*const action_opt_table[])(void) = {
    [ACTION_OPT_NONE] = &FUNCTION_NOP,
    [ACTION_OPT_POWER] = &ACTION_Power,
    [ACTION_OPT_MONITOR] = &ACTION_Monitor,
    [ACTION_OPT_SCAN] = &ACTION_ScanRestart,
    [ACTION_OPT_KEYLOCK] = &COMMON_KeypadLockToggle,
    [ACTION_OPT_A_B] = &COMMON_SwitchVFOs,
    [ACTION_OPT_VFO_MR] = &COMMON_SwitchVFOMode,
    [ACTION_OPT_SWITCH_DEMODUL] = &ACTION_SwitchDemodul,

#ifdef ENABLE_FLASHLIGHT
    [ACTION_OPT_FLASHLIGHT] = &ACTION_FlashLight,
#else
    [ACTION_OPT_FLASHLIGHT] = &FUNCTION_NOP,
#endif

#ifdef ENABLE_VOX
    [ACTION_OPT_VOX] = &ACTION_Vox,
#else
    [ACTION_OPT_VOX] = &FUNCTION_NOP,
#endif

#ifdef ENABLE_FMRADIO
    [ACTION_OPT_FM] = &ACTION_FM,
#else
    [ACTION_OPT_FM] = &FUNCTION_NOP,
#endif

#ifdef ENABLE_ALARM
    [ACTION_OPT_ALARM] = &ACTION_Alarm,
#else
    [ACTION_OPT_ALARM] = &FUNCTION_NOP,
#endif

#ifdef ENABLE_TX1750
    [ACTION_OPT_1750] = &ACTION_1750,
#else
    [ACTION_OPT_1750] = &FUNCTION_NOP,
#endif

#ifdef ENABLE_BLMIN_TMP_OFF
    [ACTION_OPT_BLMIN_TMP_OFF] = &ACTION_BlminTmpOff,
#else
    [ACTION_OPT_BLMIN_TMP_OFF] = &FUNCTION_NOP,
#endif

#ifdef ENABLE_FEAT_F4HWN
    [ACTION_OPT_RXMODE] = &ACTION_RxMode,
    [ACTION_OPT_MAINONLY] = &ACTION_MainOnly,
    [ACTION_OPT_PTT] = &ACTION_Ptt,
    [ACTION_OPT_WN] = &ACTION_Wn,
    [ACTION_OPT_BACKLIGHT] = &ACTION_BackLight,
    //#if !defined(ENABLE_SPECTRUM) || !defined(ENABLE_FMRADIO)
        [ACTION_OPT_MUTE] = &ACTION_Mute,
    //#else
    //    [ACTION_OPT_MUTE] = &FUNCTION_NOP,
    //#endif
    #ifdef ENABLE_FEAT_F4HWN_AUDIO
        [ACTION_OPT_RXA] = &ACTION_RxA,
    #else
        [ACTION_OPT_RXA] = &FUNCTION_NOP,
    #endif

    #ifdef ENABLE_FEAT_F4HWN_RESCUE_OPS
        [ACTION_OPT_POWER_HIGH] = &ACTION_Power_High,
        [ACTION_OPT_REMOVE_OFFSET] = &ACTION_Remove_Offset,
    #endif
#else
    [ACTION_OPT_RXMODE] = &FUNCTION_NOP,
#endif
#ifdef ENABLE_FEAT_F4HWN_BEAM
    [ACTION_OPT_BEAM] = &ACTION_Beam,
#endif
#ifdef ENABLE_FEAT_F4HWN_RXTX_LOG
    [ACTION_OPT_RXTX_LOG] = &ACTION_RxTxLog,
#endif
#ifdef ENABLE_FEAT_F4HWN_FOXHUNT
    [ACTION_OPT_FOXHUNT] = &ACTION_FoxHunt,
#endif
#ifdef ENABLE_FEAT_STERANIAN_SPIRITBOX
    [ACTION_OPT_SPIRITBOX] = &ACTION_SpiritBox,
#endif
#ifdef ENABLE_FEAT_STERANIAN_MEM_RNG_SCAN
    [ACTION_OPT_MEM_RNG_SCN] = &ACTION_MemRangeScan,
#endif
};

static_assert(ARRAY_SIZE(action_opt_table) == ACTION_OPT_LEN);

void ACTION_Power(void)
{
    if (++gTxVfo->OUTPUT_POWER > OUTPUT_POWER_HIGH)
        gTxVfo->OUTPUT_POWER = OUTPUT_POWER_LOW1;

    gRequestSaveChannel = 1;

    gRequestDisplayScreen = gScreenToDisplay;

#ifdef ENABLE_VOICE
    gAnotherVoiceID   = VOICE_ID_POWER;
#endif

}

void ACTION_Monitor(void)
{
    if (gCurrentFunction != FUNCTION_MONITOR) { // enable the monitor
        RADIO_SelectVfos();
#ifdef ENABLE_NOAA
        if (IS_NOAA_CHANNEL(gRxVfo->CHANNEL_SAVE) && gIsNoaaMode)
            gNoaaChannel = gRxVfo->CHANNEL_SAVE - NOAA_CHANNEL_FIRST;
#endif
        RADIO_SetupRegisters(true);
        APP_StartListening(FUNCTION_MONITOR);
        return;
    }

    gMonitor = false;

    if (gScanStateDir != SCAN_OFF) {
        gScanPauseDelayIn_10ms = scan_pause_delay_in_1_10ms;
        gScheduleScanListen    = false;
        gScanPauseMode         = true;
    }

#ifdef ENABLE_NOAA
    if (gEeprom.DUAL_WATCH == DUAL_WATCH_OFF && gIsNoaaMode) {
        gNOAA_Countdown_10ms = NOAA_countdown_10ms;
        gScheduleNOAA        = false;
    }
#endif

    RADIO_SetupRegisters(true);

#ifdef ENABLE_FMRADIO
    if (gFmRadioMode) {
        FM_Start();
        gRequestDisplayScreen = DISPLAY_FM;
    }
    else
#endif
        gRequestDisplayScreen = gScreenToDisplay;
}

void ACTION_Scan(bool bRestart)
{
    (void)bRestart;

#ifdef ENABLE_FMRADIO
    if (gFmRadioMode) {
        ACTION_Scan_FM(bRestart);
        return;
    }
#endif

    if (SCANNER_IsScanning()) {
        return;
    }

    // not scanning
    gMonitor = false;

#ifdef ENABLE_DTMF_CALLING
    DTMF_clear_RX();
#endif
    gDTMF_RX_live_timeout = 0;
    DTMF_clear_input_box_memory();

    RADIO_SelectVfos();

#ifdef ENABLE_NOAA
    if (IS_NOAA_CHANNEL(gRxVfo->CHANNEL_SAVE)) {
        return;
    }
#endif

    GUI_SelectNextDisplay(DISPLAY_MAIN);

    if (gScanStateDir != SCAN_OFF) {
        // already scanning

        if (!IS_MR_CHANNEL(gNextMrChannel)) {
            CHFRSCANNER_Stop();
#ifdef ENABLE_VOICE
            gAnotherVoiceID = VOICE_ID_SCANNING_STOP;
#endif
            return;
        }

        // channel mode. Keep scanning but toggle between scan lists
        RADIO_NextValidList(1);
        UI_MAIN_NotifyScanListChanged();

        #ifdef ENABLE_FEAT_F4HWN_RESUME_STATE
            SETTINGS_WriteCurrentState();
        #endif

        // jump to the next channel
        CHFRSCANNER_ManualResume(gScanStateDir);
    } else {
        #ifdef ENABLE_FEAT_F4HWN_RESUME_STATE
        if(gScanRangeStart == 0) // No ScanRange
        {
            gEeprom.CURRENT_STATE = 1;
        }
        else // ScanRange
        {
            gEeprom.CURRENT_STATE = 2;
        }
        SETTINGS_WriteCurrentState();
        #endif
        // start scanning
        CHFRSCANNER_Start(true, SCAN_FWD);

#ifdef ENABLE_VOICE
        AUDIO_SetVoiceID(0, VOICE_ID_SCANNING_BEGIN);
        AUDIO_PlaySingleVoice(true);
#endif

        // clear the other vfo's rssi level (to hide the antenna symbol)
        gVFO_RSSI_bar_level[(gEeprom.RX_VFO + 1) & 1U] = 0;

        // let the user see DW is not active
        gDualWatchActive = false;
    }

    gUpdateStatus = true;
}


void ACTION_SwitchDemodul(void)
{
    gRequestSaveChannel = 1;

    gTxVfo->Modulation++;

    if(gTxVfo->Modulation == MODULATION_UKNOWN)
        gTxVfo->Modulation = MODULATION_FM;
}


#ifdef ENABLE_FMRADIO
inline static bool ACTION_IsBlockedInFM(uint8_t action)
{
    switch (action) {
        case ACTION_OPT_POWER:
        case ACTION_OPT_MONITOR:
        case ACTION_OPT_A_B:
        case ACTION_OPT_VFO_MR:
        case ACTION_OPT_SWITCH_DEMODUL:
#ifdef ENABLE_VOX
        case ACTION_OPT_VOX:
#endif
#ifdef ENABLE_FEAT_F4HWN
        case ACTION_OPT_RXMODE:
        case ACTION_OPT_MAINONLY:
        case ACTION_OPT_WN:
    #ifdef ENABLE_FEAT_F4HWN_AUDIO
        case ACTION_OPT_RXA:
    #endif
    #ifdef ENABLE_FEAT_F4HWN_RESCUE_OPS
        case ACTION_OPT_POWER_HIGH:
        case ACTION_OPT_REMOVE_OFFSET:
    #endif
#endif
#ifdef ENABLE_FEAT_F4HWN_BEAM
        case ACTION_OPT_BEAM:
#endif
#ifdef ENABLE_FEAT_F4HWN_FOXHUNT
        case ACTION_OPT_FOXHUNT:
#endif

#ifdef ENABLE_FEAT_STERANIAN_SPIRITBOX
        case ACTION_OPT_SPIRITBOX:
#endif
#ifdef ENABLE_FEAT_STERANIAN_MEM_RNG_SCAN
        case ACTION_OPT_MEM_RNG_SCN:
#endif
            return true;

        default:
            return false;
    }
}
#endif

#ifdef ENABLE_FEAT_F4HWN_ACTION_PICKER
static void ACTION_Execute(uint8_t action)
{
    if (action >= ACTION_OPT_LEN || action_opt_table[action] == NULL) {
        gBeepToPlay = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;
        return;
    }

#ifdef ENABLE_FMRADIO
    if (gFmRadioMode && ACTION_IsBlockedInFM(action)) {
        gBeepToPlay = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;
        return;
    }
#endif

    gBeepToPlay = BEEP_1KHZ_60MS_OPTIONAL;
    action_opt_table[action]();
}

uint8_t gActionPickerKey;
uint8_t gActionPickerSelection[2] = {1, 1};
uint8_t gActionPickerTimeout_500ms;

bool ACTION_PickerProcessKey(KEY_Code_t key, bool isPressed, bool isHeld)
{
    if (gActionPickerKey == 0)
        return false;
    if (isPressed)
        gActionPickerTimeout_500ms = ACTION_PICKER_TIMEOUT_500MS;
    uint8_t *selection = &gActionPickerSelection[gActionPickerKey - 1];

    switch (key) {
        case KEY_UP:
        case KEY_DOWN:
            if (isPressed && !isHeld) {
                if (key == KEY_UP) {
                    if (--*selection == 0)
                        *selection = gSubMenu_SIDEFUNCTIONS_size - 1;
                }
                else if (++*selection >= gSubMenu_SIDEFUNCTIONS_size) {
                    *selection = 1;
                }

                gBeepToPlay = BEEP_1KHZ_60MS_OPTIONAL;
                gUpdateDisplay = true;
            }
            return true;

        case KEY_MENU:
            if (!isPressed && !isHeld) {
                const uint8_t action = gSubMenu_SIDEFUNCTIONS[*selection].id;
                gActionPickerKey = 0;
                gUpdateDisplay = true;
                ACTION_Execute(action);
            }
            return true;

        case KEY_EXIT:
        case KEY_F:
            if (!isPressed) {
                gActionPickerKey = 0;
                gUpdateDisplay = true;
            }
            return true;

        case KEY_PTT:
            gActionPickerKey = 0;
            gUpdateDisplay = true;
            return false;

        default:
            return true;
    }
}
#endif

void ACTION_Handle(KEY_Code_t Key, bool bKeyPressed, bool bKeyHeld)
{
    HideFKeyIcon();

    if (gScreenToDisplay == DISPLAY_MAIN && gDTMF_InputMode){
         // entering DTMF code

        gPttWasReleased = true;

        if (Key != KEY_SIDE1 || bKeyHeld || !bKeyPressed){
            return;
        }

        // side1 btn pressed

        gBeepToPlay = BEEP_1KHZ_60MS_OPTIONAL;
        gRequestDisplayScreen = DISPLAY_MAIN;

        if (gDTMF_InputBox_Index <= 0) {
            // turn off DTMF input box if no codes left
            gDTMF_InputMode = false;
            return;
        }

        // DTMF codes are in the input box
        gDTMF_InputBox[--gDTMF_InputBox_Index] = '-'; // delete one code

#ifdef ENABLE_VOICE
        gAnotherVoiceID   = VOICE_ID_CANCEL;
#endif
        return;
    }

    enum ACTION_OPT_t func = ACTION_OPT_NONE;
    switch(Key) {
        case KEY_SIDE1:
            if (bKeyHeld)
                func = gEeprom.KEY_1_LONG_PRESS_ACTION;
            else
                func = gEeprom.KEY_1_SHORT_PRESS_ACTION;
            break;
        case KEY_SIDE2:
            if (bKeyHeld)
                func = gEeprom.KEY_2_LONG_PRESS_ACTION;
            else
                func = gEeprom.KEY_2_SHORT_PRESS_ACTION;
            break;
        case KEY_MENU:
            if (bKeyHeld)
                func = gEeprom.KEY_M_LONG_PRESS_ACTION;
            break;
#ifdef ENABLE_FEAT_STERANIAN_PTT_REMAP
        case KEY_PTT:
            func = gEeprom.KEY_PTT_SHORT_PRESS_ACTION;
            /*
            if (bKeyHeld)
                //func = gEeprom.KEY_PTT_LONG_PRESS_ACTION;
            else
                func = gEeprom.KEY_PTT_SHORT_PRESS_ACTION;
            */
            break;
#endif
        default:
            break;
    }

    if (bKeyHeld != bKeyPressed) { // button pushed or released after hold 
                                   // (!bKeyHeld && bKeyPressed) or (bKeyHeld && !bKeyPressed)
        return;
    }

    // held or released after short press
#ifdef ENABLE_FEAT_F4HWN_ACTION_PICKER
    ACTION_Execute(func);
#else
    gBeepToPlay = BEEP_1KHZ_60MS_OPTIONAL;
    
#ifdef ENABLE_FMRADIO
    if (gFmRadioMode && ACTION_IsBlockedInFM(func)) {
        gBeepToPlay = BEEP_500HZ_60MS_DOUBLE_BEEP_OPTIONAL;
        return;
    }
#endif

    action_opt_table[func]();
#endif
}


#ifdef ENABLE_FMRADIO
void ACTION_FM(void)
{
    if (gCurrentFunction != FUNCTION_TRANSMIT && gCurrentFunction != FUNCTION_MONITOR)
    {
        gInputBoxIndex = 0;

        if (gFmRadioMode) {
            FM_TurnOff();
            gFlagReconfigureVfos  = true;
            gRequestDisplayScreen = DISPLAY_MAIN;

#ifdef ENABLE_VOX
            gVoxResumeCountdown = 80;
#endif
            return;
        }

        gMonitor = false;

        if (gScanStateDir != SCAN_OFF) {
            // Stop the channel/frequency scan before switching to the FM radio.
            gScanKeepResult = false;
            CHFRSCANNER_Stop();
        }

        RADIO_SelectVfos();
        RADIO_SetupRegisters(true);

        FM_Start();

        gRequestDisplayScreen = DISPLAY_FM;
    }
}

static void ACTION_Scan_FM(bool bRestart)
{
    if (FUNCTION_IsRx())
        return;

    GUI_SelectNextDisplay(DISPLAY_FM);

    gMonitor = false;

    if (gFM_ScanState != FM_SCAN_OFF) {
        FM_PlayAndUpdate();

#ifdef ENABLE_VOICE
        gAnotherVoiceID = VOICE_ID_SCANNING_STOP;
#endif
        return;
    }

    uint16_t freq;

    if (bRestart) {
        gFM_AutoScan = true;
        gFM_ChannelPosition = 0;
        FM_EraseChannels();
        freq = BK1080_GetFreqLoLimit(gEeprom.FM_Band);
    } else {
        gFM_AutoScan = false;
        gFM_ChannelPosition = 0;
        freq = gEeprom.FM_FrequencyPlaying;
    }

    BK1080_GetFrequencyDeviation(freq);
    FM_Tune(freq, 1, bRestart);

#ifdef ENABLE_VOICE
    gAnotherVoiceID = VOICE_ID_SCANNING_BEGIN;
#endif

}

#endif


#if defined(ENABLE_ALARM) || defined(ENABLE_TX1750)
static void ACTION_AlarmOr1750(const bool b1750)
{

    if(gEeprom.KEY_LOCK && (gSetting_set_lck & SET_LCK_PTT))
        return;

    #if defined(ENABLE_ALARM)
        const AlarmState_t alarm_mode = (gEeprom.ALARM_MODE == ALARM_MODE_TONE) ? ALARM_STATE_TXALARM : ALARM_STATE_SITE_ALARM;
        gAlarmRunningCounter = 0;
    #endif

    #if defined(ENABLE_ALARM) && defined(ENABLE_TX1750)
        gAlarmState = b1750 ? ALARM_STATE_TX1750 : alarm_mode;
    #elif defined(ENABLE_ALARM)
        gAlarmState = alarm_mode;
    #else
        gAlarmState = ALARM_STATE_TX1750;
    #endif

    (void)b1750;
    gInputBoxIndex = 0;

    gFlagPrepareTX = gAlarmState != ALARM_STATE_OFF;

    if (gScreenToDisplay != DISPLAY_MENU)     // 1of11 .. don't close the menu
        gRequestDisplayScreen = DISPLAY_MAIN;
}


#endif

#ifdef ENABLE_VOX
void ACTION_Vox(void)
{
    gEeprom.VOX_SWITCH   = !gEeprom.VOX_SWITCH;
    gRequestSaveSettings = true;
    gFlagReconfigureVfos = true;
    gUpdateStatus        = true;

    #ifdef ENABLE_VOICE
        gAnotherVoiceID  = VOICE_ID_VOX;
    #endif
}
#endif

#ifdef ENABLE_BLMIN_TMP_OFF
void ACTION_BlminTmpOff(void)
{
    if(++gEeprom.BACKLIGHT_MIN_STAT == BLMIN_STAT_UNKNOWN) {
        gEeprom.BACKLIGHT_MIN_STAT = BLMIN_STAT_ON;
        BACKLIGHT_SetBrightness(gEeprom.BACKLIGHT_MIN);
    } else {
        BACKLIGHT_SetBrightness(0);
    }
}
#endif

#ifdef ENABLE_FEAT_F4HWN
void ACTION_Update(void)
{
    gSaveRxMode          = false;
    gFlagReconfigureVfos = true;
    gUpdateStatus        = true;
}

void ACTION_RxMode(void)
{
    static bool cycle = 0;

    if (cycle) {
        gEeprom.CROSS_BAND_RX_TX = !gEeprom.CROSS_BAND_RX_TX;
    } else {
        gEeprom.DUAL_WATCH = !gEeprom.DUAL_WATCH;
    }

    cycle = !cycle;
    ACTION_Update();
}

void ACTION_MainOnly(void)
{
    static bool cycle = 0;
    static uint8_t dw = 0;
    static uint8_t cb = 0;

    if (cycle) {
        gEeprom.DUAL_WATCH = dw;
        gEeprom.CROSS_BAND_RX_TX = cb;
    } else {
        dw = gEeprom.DUAL_WATCH;
        cb = gEeprom.CROSS_BAND_RX_TX;

        gEeprom.DUAL_WATCH = 0;
        gEeprom.CROSS_BAND_RX_TX = 0;
    }

    cycle = !cycle;
    ACTION_Update();
}

#ifdef ENABLE_FEAT_F4HWN_AUDIO
void ACTION_RxA(void)
{
    if(gRxVfo->Modulation == MODULATION_AM)
        gSetting_set_audio_am = (gSetting_set_audio_am + 1) % 3;
    else if (gRxVfo->Modulation == MODULATION_FM)
        gSetting_set_audio_fm = (gSetting_set_audio_fm + 1) % 5;

    RADIO_SetModulation(gRxVfo->Modulation);
}
#endif

void ACTION_Ptt(void)
{
    gSetting_set_ptt_session = !gSetting_set_ptt_session;

    ACTION_Update();
}

void ACTION_Wn(void)
{
    const bool isRx = FUNCTION_IsRx();
    VFO_Info_t *pVfo = isRx ? gRxVfo : gTxVfo;

    pVfo->CHANNEL_BANDWIDTH = !pVfo->CHANNEL_BANDWIDTH;

    if (pVfo->Modulation == MODULATION_AM)
    {
        BK4819_SetFilterBandwidth(RADIO_GetAMFilterBandwidth(pVfo), true);
        return;
    }

    uint8_t bw = pVfo->CHANNEL_BANDWIDTH;

    #ifdef ENABLE_FEAT_F4HWN_NARROWER
        if (isRx && bw == BANDWIDTH_NARROW && gSetting_set_nfm == 1)
        {
            bw++; 
        }
    #endif

    #ifdef ENABLE_AM_FIX
        BK4819_SetFilterBandwidth(bw, true);
    #else
        BK4819_SetFilterBandwidth(bw, false);
    #endif
}

void ACTION_BackLight(void)
{
    if(gBackLight)
    {
        gEeprom.BACKLIGHT_TIME = gBacklightTimeOriginal;
    }
    gBackLight = false;
    BACKLIGHT_TurnOn();
}

void ACTION_BackLightOnDemand(void)
{
    if(gBackLight == false)
    {
        gBacklightTimeOriginal = gEeprom.BACKLIGHT_TIME;
        gEeprom.BACKLIGHT_TIME = 61;
        gBackLight = true;
    }
    else
    {
        if(gBacklightBrightnessOld == gEeprom.BACKLIGHT_MAX)
        {
            gEeprom.BACKLIGHT_TIME = 0;
        }
        else
        {
            gEeprom.BACKLIGHT_TIME = 61;
        }
    }
    
    BACKLIGHT_TurnOn();
}

void ACTION_Mute(void)
{
    // Toggle mute state
    gMute = !gMute;

    // Update the registers
    #ifdef ENABLE_FMRADIO
        BK1080_WriteRegister(BK1080_REG_05_SYSTEM_CONFIGURATION2, gMute ? 0x0A10 : 0x0A1F);
    #endif
    gEeprom.VOLUME_GAIN = gMute ? 0 : gEeprom.VOLUME_GAIN_BACKUP;
    BK4819_SetRxAudioGain();

    gUpdateStatus = true;
}

#ifdef ENABLE_FEAT_F4HWN_RESCUE_OPS
void ACTION_ToggleVfoSetting(bool *setting) {
    *setting = !(*setting);
    gVfoConfigureMode = VFO_CONFIGURE_RELOAD;
}

void ACTION_Power_High(void)
{
    ACTION_ToggleVfoSetting(&gPowerHigh);
}

void ACTION_Remove_Offset(void)
{
    ACTION_ToggleVfoSetting(&gRemoveOffset);
}
#endif
#endif

#ifdef ENABLE_FEAT_STERANIAN_MEM_RNG_SCAN
void ACTION_MemRangeScan(void)
{
#ifdef ENABLE_FMRADIO
    if (gFmRadioMode) return;
#endif

    if (gEeprom.MEM_RNG_SCN_LIST == 0) {
        UI_DisplayPopup("SELECT LIST");
        return;
    }

    uint16_t pairs[16][2]; // [最大16ペアまで][0=開始CH, 1=終了CH] 
    uint8_t count = 0;
    uint8_t selected = 0;
    uint16_t first_ch = 0xFFFF;
    
    // スキャンリストの値をそのまま指定 (1〜24)
    uint8_t target_list = gEeprom.MEM_RNG_SCN_LIST;

    // 1. スキャンリストに属する有効なチャンネルを順に走査しペアを生成
    for (uint16_t i = 0; i < MR_CHANNELS_MAX && count < 16; i++) {
        if (RADIO_CheckValidChannel(i, true, target_list)) {
            if (first_ch == 0xFFFF) {
                first_ch = i;
            } else {
                pairs[count][0] = first_ch;
                pairs[count][1] = i;
                count++;
                first_ch = 0xFFFF; // リセット
            }
        }
    }

    if (count == 0) {
        UI_DisplayPopup("NO PAIRS");
        return;
    }

    gKeyReading0 = KEY_INVALID;
    gKeyReading1 = KEY_INVALID;

    KEY_Code_t prevKey = KEY_INVALID;
    bool isRunning = true;
    bool redraw = true;

#ifdef ENABLE_FEAT_F4HWN_K5VIEWER
    gKeyReading0 = KEY_INVALID;
    gKeyReading1 = KEY_INVALID;
    K5VIEWER_Update(true);
#endif

    // 開始時に一度だけステータス行を最新状態に強制上書き描画
    UI_DisplayStatus();

    // 2. インタラクティブUIループ
    while (isRunning) {
#if defined(ENABLE_UART) || defined(ENABLE_USB)
        UART_ServiceCommands();
#endif
#ifdef ENABLE_FEAT_F4HWN_K5VIEWER
        K5VIEWER_ParseInput();
#endif

        // 状態変化時のみ描画処理を実行
        if (redraw) {
            redraw = false;

            UI_DisplayClear();

            char name[13];
            char str[32];

            // 終了チャンネル（ペアの2番目）のチャンネル名を取得
            SETTINGS_FetchChannelName(name, pairs[selected][1]);
            if (name[0] == '\0' || (uint8_t)name[0] == 0xFF) {
                sprintf(name, "RANGE %02u", selected + 1);
            }

            // 画面描画の配置（gFrameBuffer[0..6] のみ。ステータス行は液晶に残したままにする）
            
            // Line 1 (gFrameBuffer[0]): タイトルを左側、インデックスを右側に配置
            UI_PrintStringSmallBold("SCAN RANGE", 2, 0, 0);
            sprintf(str, "%d/%d", selected + 1, count);
            UI_PrintStringSmallNormal(str, (uint8_t)(127 - (strlen(str) * 7)), 0, 0);

            // Line 3 & 4 (gFrameBuffer[2], [3]): レンジ名称を中央にデカ文字で表示
            UI_PrintString(name, 0, 127, 2, 8);

            // Line 6 (gFrameBuffer[5]): 周波数の範囲を表示
            uint32_t f1 = SETTINGS_FetchChannelFrequency(pairs[selected][0]);
            uint32_t f2 = SETTINGS_FetchChannelFrequency(pairs[selected][1]);

            /* 1行書きバージョン
            sprintf(str, "%u.%02u-%u.%02u MHz", 
                    f1 / 100000, (f1 % 100000) / 1000,
                    f2 / 100000, (f2 % 100000) / 1000);
            UI_PrintStringSmallNormal(str, 2, 0, 6);
            */

            // 2行書きバージョン
            sprintf(str, "from %u.%05u MHz", 
                    f1 / 100000, f1 % 100000);
            UI_PrintStringSmallNormal(str, 2, 0, 5);
            sprintf(str, "  to %u.%05u MHz", 
                    f2 / 100000, f2 % 100000);
            UI_PrintStringSmallNormal(str, 2, 0, 6);

            ST7565_BlitFullScreen();

#ifdef ENABLE_FEAT_F4HWN_K5VIEWER
            K5VIEWER_Update(false);
#endif
        }

        KEY_Code_t key = KEYBOARD_Poll();
        if (key != prevKey) {
            if (key == KEY_EXIT) {
                gKeyReading0     = KEY_INVALID;
                gKeyReading1     = KEY_INVALID;
                gDebounceCounter = 0;

                gRequestDisplayScreen = DISPLAY_MAIN;
                gUpdateDisplay = true;
                gUpdateStatus = true;
                isRunning = false;
            } else if (key == KEY_UP) {
                selected = (selected + count - 1) % count;
                redraw = true; // 描画フラグをセット
            } else if (key == KEY_DOWN) {
                selected = (selected + 1) % count;
                redraw = true; // 描画フラグをセット
            } else if (key == KEY_MENU) {
                uint32_t f1_start = SETTINGS_FetchChannelFrequency(pairs[selected][0]);
                uint32_t f2_stop  = SETTINGS_FetchChannelFrequency(pairs[selected][1]);

                uint8_t vfo_num = gEeprom.TX_VFO;
                
                // レンジスキャン終了時の復元用に、開始前のScreenChannel状態をバックアップ
#ifdef ENABLE_FEAT_STERANIAN_SCNRNG_VFO_COPY
                gSavedScreenChannel[0] = gEeprom.ScreenChannel[0];
                gSavedScreenChannel[1] = gEeprom.ScreenChannel[1];
                gWasScanRangeCopied = false; // 初期化
#endif

                // 開始メモリチャンネルの設定を一時的にセットし、
                // RADIO_ConfigureChannel を呼んでステップ（10kHz等）や変調、アトリビュートをRAM（gTxVfo）にロードする
                gEeprom.ScreenChannel[vfo_num] = pairs[selected][0];
                RADIO_SelectVfos(); // ポインタを一度開始チャンネルに合わせる
                RADIO_ConfigureChannel(vfo_num, VFO_CONFIGURE_RELOAD);

                // ロード完了後、メモリ上のステップ等を維持したまま、モードを開始周波数に対応するVFOモードに強制切り替え
                uint8_t band = FREQUENCY_GetBand(f1_start);
                gEeprom.ScreenChannel[vfo_num] = FREQ_CHANNEL_FIRST + band;
                gEeprom.FreqChannel[vfo_num]   = FREQ_CHANNEL_FIRST + band;
                gTxVfo->CHANNEL_SAVE           = gEeprom.ScreenChannel[vfo_num];
                gTxVfo->Band                   = band;

#ifdef ENABLE_FEAT_STERANIAN_SCNRNG_VFO_COPY
                // もし元のモードがメモリモード(MR)だった場合、VFOコピーが行われたことを示すフラグを立てる
                if (IS_MR_CHANNEL(gSavedScreenChannel[vfo_num])) {
                    gWasScanRangeCopied = true;
                }
#endif

                // グローバルポインタを新設したVFOチャンネルの実体にバインドし直す
                RADIO_SelectVfos();

                // スキャン範囲を設定
                gScanRangeStart = f1_start;
                gScanRangeStop  = f2_stop;
                if (gScanRangeStart > gScanRangeStop) {
                    SWAP(gScanRangeStart, gScanRangeStop);
                }

                // 構築されたVFOオブジェクトに対して周波数をセット
                gTxVfo->freq_config_RX.Frequency = gScanRangeStart;
                gTxVfo->freq_config_TX.Frequency = gScanRangeStart; // 送信側も初期同期
                RADIO_ApplyOffset(gTxVfo);
                RADIO_ConfigureSquelchAndOutputPower(gTxVfo);

                // 変更されたVFOデータ（下限値周波数と、引き継がれたステップ等のすべての設定）をEEPROMのVFO領域に保存する
                SETTINGS_SaveChannel(gEeprom.ScreenChannel[vfo_num], vfo_num, gTxVfo, 1);

                SETTINGS_SaveVfoIndices();
                RADIO_SetupRegisters(true);

                // レンジスキャンを開始
                CHFRSCANNER_Start(true, SCAN_FWD);

                gKeyReading0     = KEY_INVALID;
                gKeyReading1     = KEY_INVALID;
                gDebounceCounter = 0;

                gRequestDisplayScreen = DISPLAY_MAIN;
                gUpdateDisplay = true;
                gUpdateStatus = true;
                isRunning = false;
            }
            prevKey = key;
        }

        SYSTEM_DelayMs(10);
    }
}
#endif