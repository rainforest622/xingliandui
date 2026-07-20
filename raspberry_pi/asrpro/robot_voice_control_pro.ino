#include "asr.h"
extern "C" { void * __dso_handle = 0; }
#include "setup.h"
#include "HardwareSerial.h"
#include "myLib/asr_event.h"

// ASRPRO professional-mode firmware for the inspection robot.
// In Tianwen Block choose ASRPRO -> 编程模式 -> 字符编程, then generate the
// voice model before using 2M 编译下载. Baidu TTS avoids retired legacy voices.
uint32_t snid;

static const uint8_t EVENT_WAKE = 0xA0;
static const uint8_t EVENT_START = 0xA1;
static const uint8_t EVENT_PAUSE = 0xA2;
static const uint8_t EVENT_RESUME = 0xA3;
static const uint8_t EVENT_STOP = 0xA4;
static const uint8_t EVENT_STATUS = 0xA5;
static const uint8_t EVENT_CRITICAL = 0xA6;
static const uint8_t EVENT_CLEAR_ALARM = 0xA7;
static const uint8_t EVENT_ENVIRONMENT = 0xA8;
static const uint8_t EVENT_DISTANCE = 0xA9;
static const uint8_t EVENT_PATROL_REPORT = 0xAA;
static const uint8_t EVENT_BATTERY = 0xAB;

static const uint8_t REPLY_STARTED = 0xF1;
static const uint8_t REPLY_PAUSED = 0xF2;
static const uint8_t REPLY_STOPPED = 0xF3;
static const uint8_t REPLY_STATUS = 0xF4;
static const uint8_t REPLY_SLE_CONFIRM = 0xF5;
static const uint8_t REPLY_CRITICAL = 0xF6;
static const uint8_t REPLY_OBSTACLE_AVOID = 0xF7;
static const uint8_t REPLY_TEMP_ALARM = 0xF8;
static const uint8_t REPLY_HUMIDITY_ALARM = 0xF9;
static const uint8_t REPLY_PATROL_COMPLETE_NORMAL = 0xFA;
static const uint8_t REPLY_PATROL_COMPLETE_ALERT = 0xFB;
static const uint8_t REPLY_STATUS_DETAIL = 0xFC;
static const uint8_t REPLY_BATTERY_UNAVAILABLE = 0xFD;
static const uint8_t REPLY_SENSOR_UNAVAILABLE = 0xFE;
static const uint8_t DYNAMIC_FRAME = 0xE0;
static const uint8_t DYNAMIC_ENVIRONMENT = 0x01;
static const uint8_t DYNAMIC_DISTANCE = 0x02;

// Reusable offline speech pieces. The Pi sends real deci-unit values and the
// module combines the pieces locally; no cloud TTS is involved at runtime.
static const uint16_t PROMPT_SENSOR_UNAVAILABLE = 11026;
static const uint16_t PROMPT_TEMPERATURE = 11027;
static const uint16_t PROMPT_HUMIDITY = 11028;
static const uint16_t PROMPT_DISTANCE = 11029;
static const uint16_t PROMPT_DEGREE_C = 11030;
static const uint16_t PROMPT_PERCENT = 11031;
static const uint16_t PROMPT_MILLIMETRE = 11032;
static const uint16_t PROMPT_POINT = 11033;
static const uint16_t PROMPT_NEGATIVE = 11034;
static const uint16_t PROMPT_DIGIT_ZERO = 11035;
static const uint16_t PROMPT_TEN = 11045;
static const uint16_t PROMPT_HUNDRED = 11046;
static const uint16_t PROMPT_THOUSAND = 11047;

static prompt_play_info_t report_prompts[16];
static uint8_t report_prompt_count = 0;
static bool dynamic_frame_active = false;
static uint8_t dynamic_frame_body[6];
static uint8_t dynamic_frame_index = 0;

static void send_event(uint8_t event_code, uint16_t prompt_id)
{
    Serial.write(event_code);
    Serial.flush();
    play_audio(prompt_id);
}

void ASR_CODE()
{
    // Keep the command window open for 15 seconds after every valid result.
    set_state_enter_wakeup(15000);

    switch (snid) {
        case 1000:
        case 1001:
            Serial.write(EVENT_WAKE);
            Serial.flush();
            play_audio(11001);
            break;
        case 1002:
            send_event(EVENT_START, 11002);
            break;
        case 1003:
            send_event(EVENT_PAUSE, 11003);
            break;
        case 1004:
            send_event(EVENT_RESUME, 11004);
            break;
        case 1005:
            send_event(EVENT_STOP, 11005);
            break;
        case 1006:
            send_event(EVENT_STATUS, 11006);
            break;
        case 1007:
        case 1008:
            send_event(EVENT_CRITICAL, 11007);
            break;
        case 1009:
            send_event(EVENT_CLEAR_ALARM, 11008);
            break;
        case 1010:
            send_event(EVENT_ENVIRONMENT, 11009);
            break;
        case 1011:
            send_event(EVENT_DISTANCE, 11010);
            break;
        case 1012:
            send_event(EVENT_PATROL_REPORT, 11023);
            break;
        case 1013:
            send_event(EVENT_BATTERY, 11024);
            break;
    }
}

static void report_begin()
{
    report_prompt_count = 0;
}

static void report_append(uint16_t prompt_id)
{
    if (report_prompt_count >= 16) {
        return;
    }
    report_prompts[report_prompt_count].cmd_id = prompt_id;
    report_prompts[report_prompt_count].select_index = (uint16_t)-1;
    report_prompt_count++;
}

static void report_append_digit(uint8_t digit)
{
    report_append((uint16_t)(PROMPT_DIGIT_ZERO + (digit % 10)));
}

static void report_append_integer(uint16_t value)
{
    if (value == 0) {
        report_append_digit(0);
        return;
    }

    bool has_high_unit = false;
    if (value >= 1000) {
        report_append_digit((uint8_t)(value / 1000));
        report_append(PROMPT_THOUSAND);
        value %= 1000;
        has_high_unit = true;
        if (value > 0 && value < 100) {
            report_append_digit(0);
        }
    }
    if (value >= 100) {
        report_append_digit((uint8_t)(value / 100));
        report_append(PROMPT_HUNDRED);
        value %= 100;
        has_high_unit = true;
        if (value > 0 && value < 10) {
            report_append_digit(0);
        }
    }
    if (value >= 10) {
        uint8_t tens = (uint8_t)(value / 10);
        if (tens > 1 || has_high_unit) {
            report_append_digit(tens);
        }
        report_append(PROMPT_TEN);
        value %= 10;
    }
    if (value > 0) {
        report_append_digit((uint8_t)value);
    }
}

static void report_append_deci(int16_t deci_value)
{
    int32_t absolute_value = deci_value;
    if (absolute_value < 0) {
        report_append(PROMPT_NEGATIVE);
        absolute_value = -absolute_value;
    }
    report_append_integer((uint16_t)(absolute_value / 10));
    report_append(PROMPT_POINT);
    report_append_digit((uint8_t)(absolute_value % 10));
}

static void report_play()
{
    if (report_prompt_count == 0) {
        return;
    }
    while (prompt_play_by_multi_cmd_id(report_prompts, report_prompt_count, NULL) != 0) {
        delay(6);
    }
}

static void speak_environment_report(int16_t temperature_deci_c, int16_t humidity_deci_percent)
{
    report_begin();
    report_append(PROMPT_TEMPERATURE);
    report_append_deci(temperature_deci_c);
    report_append(PROMPT_DEGREE_C);
    report_append(PROMPT_HUMIDITY);
    report_append_deci(humidity_deci_percent);
    report_append(PROMPT_PERCENT);
    report_play();
}

static void speak_distance_report(uint16_t distance_mm)
{
    report_begin();
    report_append(PROMPT_DISTANCE);
    report_append_integer(distance_mm);
    report_append(PROMPT_MILLIMETRE);
    report_play();
}

static int16_t dynamic_signed_value(uint8_t high, uint8_t low)
{
    return (int16_t)(((uint16_t)high << 8) | low);
}

static uint16_t dynamic_unsigned_value(uint8_t high, uint8_t low)
{
    return (uint16_t)(((uint16_t)high << 8) | low);
}

static void finish_dynamic_frame()
{
    uint8_t checksum = DYNAMIC_FRAME;
    for (uint8_t index = 0; index < 5; index++) {
        checksum ^= dynamic_frame_body[index];
    }
    if (checksum != dynamic_frame_body[5]) {
        return;
    }
    if (dynamic_frame_body[0] == DYNAMIC_ENVIRONMENT) {
        speak_environment_report(
            dynamic_signed_value(dynamic_frame_body[1], dynamic_frame_body[2]),
            dynamic_signed_value(dynamic_frame_body[3], dynamic_frame_body[4])
        );
    } else if (dynamic_frame_body[0] == DYNAMIC_DISTANCE) {
        speak_distance_report(dynamic_unsigned_value(dynamic_frame_body[1], dynamic_frame_body[2]));
    }
}

static void handle_reply_byte(uint8_t value)
{
    if (dynamic_frame_active) {
        dynamic_frame_body[dynamic_frame_index++] = value;
        if (dynamic_frame_index >= sizeof(dynamic_frame_body)) {
            dynamic_frame_active = false;
            dynamic_frame_index = 0;
            finish_dynamic_frame();
        }
        return;
    }
    if (value == DYNAMIC_FRAME) {
        dynamic_frame_active = true;
        dynamic_frame_index = 0;
        return;
    }
    switch (value) {
        case REPLY_STARTED:     play_audio(11011); break;
        case REPLY_PAUSED:      play_audio(11012); break;
        case REPLY_STOPPED:     play_audio(11013); break;
        case REPLY_STATUS:      play_audio(11014); break;
        case REPLY_SLE_CONFIRM: play_audio(11015); break;
        case REPLY_CRITICAL:    play_audio(11016); break;
        case REPLY_OBSTACLE_AVOID: play_audio(11017); break;
        case REPLY_TEMP_ALARM: play_audio(11018); break;
        case REPLY_HUMIDITY_ALARM: play_audio(11019); break;
        case REPLY_PATROL_COMPLETE_NORMAL: play_audio(11020); break;
        case REPLY_PATROL_COMPLETE_ALERT: play_audio(11021); break;
        case REPLY_STATUS_DETAIL: play_audio(11022); break;
        case REPLY_BATTERY_UNAVAILABLE: play_audio(11025); break;
        case REPLY_SENSOR_UNAVAILABLE: play_audio(PROMPT_SENSOR_UNAVAILABLE); break;
        default: break;
    }
}

static void voice_reply_task(void *)
{
    while (true) {
        if (Serial.available() > 0) {
            handle_reply_byte((uint8_t)Serial.read());
        }
        delay(10);
    }
}

void hardware_init()
{
    // UART0 is PB5/PB6 and is also exposed by the CH340 Type-C interface.
    vol_set(7);
    setPinFun(13, SECOND_FUNCTION);
    setPinFun(14, SECOND_FUNCTION);
    Serial.begin(9600);
    xTaskCreate(voice_reply_task, "voice_reply", 256, NULL, 4, NULL);
    play_audio(11000);
    vTaskDelete(NULL);
}

void setup()
{
    // Dynamic telemetry prompt pieces. Tianwen Block generates local assets
    // from these directives during the ASRPRO 2M build.
    //{playid:11026,voice:传感器数据暂不可用。}
    //{playid:11027,voice:当前温度。}
    //{playid:11028,voice:当前湿度。}
    //{playid:11029,voice:当前前方距离。}
    //{playid:11030,voice:摄氏度。}
    //{playid:11031,voice:百分比。}
    //{playid:11032,voice:毫米。}
    //{playid:11033,voice:点。}
    //{playid:11034,voice:负。}
    //{playid:11035,voice:零。}
    //{playid:11036,voice:一。}
    //{playid:11037,voice:二。}
    //{playid:11038,voice:三。}
    //{playid:11039,voice:四。}
    //{playid:11040,voice:五。}
    //{playid:11041,voice:六。}
    //{playid:11042,voice:七。}
    //{playid:11043,voice:八。}
    //{playid:11044,voice:九。}
    //{playid:11045,voice:十。}
    //{playid:11046,voice:百。}
    //{playid:11047,voice:千。}

    // Tianwen Block's "播报音设置(百度TTS)" generator emits this exact form.
    //{speak:小鹿-甜美女声,vol:16,speed:8,platform:baidu}
    //{playid:11000,voice:智能巡检语音助手已启动，请说小星小星。}
    //{playid:11001,voice:我在，请说指令。}
    //{playid:11002,voice:已发送开始巡检请求。}
    //{playid:11003,voice:已发送暂停巡检请求。}
    //{playid:11004,voice:已发送继续巡检请求。}
    //{playid:11005,voice:已发送立即停车请求。}
    //{playid:11006,voice:正在请求系统状态。}
    //{playid:11007,voice:紧急事件已上报，正在停止巡检。}
    //{playid:11008,voice:解除报警需要星闪端确认。}
    //{playid:11011,voice:收到，开始执行巡检路线。}
    //{playid:11012,voice:巡检已暂停，等待新的指令。}
    //{playid:11013,voice:巡检已停止，已切换为手动待命。}
    //{playid:11014,voice:系统状态已同步到手机端。}
    //{playid:11015,voice:当前操作需要在星闪端确认。}
    //{playid:11016,voice:检测到紧急事件，巡检已停止，请人工接管。}
    //{playid:11017,voice:前方检测到障碍物，正在执行安全避障。}
    //{playid:11018,voice:温度异常，请注意设备环境。}
    //{playid:11019,voice:湿度异常，已上报移动端。}
    //{playid:11020,voice:本次巡检完成，未发现环境异常。}
    //{playid:11021,voice:本次巡检完成，发现环境异常，已记录并上报。}
    //{playid:11022,voice:当前环境、前方距离和巡检状态已同步到手机端。}
    //{playid:11023,voice:正在查询巡检进度。}
    //{playid:11024,voice:正在查询电量状态。}
    //{playid:11025,voice:当前未接入电量计，无法报告电量。}

    //{ID:1000,keyword:"唤醒词",ASR:"小星小星",ASRTO:""}
    //{ID:1001,keyword:"唤醒词",ASR:"巡检小车",ASRTO:""}
    //{ID:1002,keyword:"命令词",ASR:"开始巡检",ASRTO:""}
    //{ID:1003,keyword:"命令词",ASR:"暂停巡检",ASRTO:""}
    //{ID:1004,keyword:"命令词",ASR:"继续巡检",ASRTO:""}
    //{ID:1005,keyword:"命令词",ASR:"停止巡检",ASRTO:""}
    //{ID:1006,keyword:"命令词",ASR:"报告状态",ASRTO:""}
    //{ID:1007,keyword:"命令词",ASR:"救命",ASRTO:""}
    //{ID:1008,keyword:"命令词",ASR:"着火了",ASRTO:""}
    //{ID:1009,keyword:"命令词",ASR:"解除报警",ASRTO:""}
    //{ID:1010,keyword:"命令词",ASR:"报告温湿度",ASRTO:""}
    //{ID:1011,keyword:"命令词",ASR:"前方距离",ASRTO:""}
    //{ID:1012,keyword:"命令词",ASR:"巡检进度",ASRTO:""}
    //{ID:1013,keyword:"命令词",ASR:"报告电量",ASRTO:""}
}
