#include "app_init.h"
#include "at.h"
#include "common_def.h"
#include "robot_motor.h"
#include "robot_buzzer.h"
#include "robot_env.h"
#include "robot_mvp_control.h"
#include "robot_obstacle.h"
#include "robot_oled.h"
#include "robot_protocol.h"
#ifdef CONFIG_ROBOT_MVP_ENABLE_WAVE_ROVER_LINK
#include "robot_rover_link.h"
#endif
#ifdef CONFIG_ROBOT_MVP_ENABLE_SLE
#include "robot_sle_server.h"
#endif
#include "soc_osal.h"
#include "tcxo.h"
#include "uart.h"

#define ROBOT_TASK_STACK_SIZE 0x2000U
#define ROBOT_TASK_PRIORITY 23U
#define ROBOT_LOOP_DELAY_MS 2U
#define ROBOT_CONTROL_TIMEOUT_MS 500U
#define ROBOT_ROVER_KEEPALIVE_MS 100U
#define ROBOT_AT_SAFE_SPEED 100
#define ROBOT_AVOID_SPEED 100
#define ROBOT_AVOID_SAMPLE_MS 90U
#define ROBOT_AVOID_BACKWARD_MS 260U
#define ROBOT_AVOID_TURN_MS 420U
#define ROBOT_AVOID_TEST_DURATION_MS 1800U
#define ROBOT_AVOID_TEST_BLOCK_MS 120U
#define ROBOT_AVOID_TEST_BLOCK_DISTANCE_MM 120U
#define ROBOT_AVOID_TEST_CLEAR_DISTANCE_MM 450U
#define ROBOT_PATROL_SPEED 100
#define ROBOT_PATROL_ROUTE_LEGS_DEFAULT 4U
#define ROBOT_PATROL_SAMPLE_MS 90U
#define ROBOT_PATROL_FORWARD_MS 9000U
#define ROBOT_PATROL_TURN_MS 430U
#define ROBOT_PATROL_FORWARD_MIN_MS 500U
#define ROBOT_PATROL_FORWARD_MAX_MS 60000U
#define ROBOT_PATROL_TURN_MIN_MS 100U
#define ROBOT_PATROL_TURN_MAX_MS 8000U
#define ROBOT_PATROL_SPEED_MIN 30U
#define ROBOT_PATROL_SPEED_MAX 100U
#define ROBOT_PATROL_MAX_LOOPS_DEFAULT 1U
#define ROBOT_PATROL_MAX_LOOPS_LIMIT 20U
#define ROBOT_PATROL_BLOCK_CONFIRM_COUNT 2U
#define ROBOT_PATROL_START_CONFIRM_SAMPLES 3U
#define ROBOT_PATROL_START_BLOCK_CONFIRM_COUNT 2U
#define ROBOT_PATROL_START_SAMPLE_GAP_MS 80U
#define ROBOT_PATROL_OBSTACLE_BACKWARD_MS 300U
#define ROBOT_PATROL_OBSTACLE_TURN_MS 520U
#define ROBOT_PATROL_DETOUR_SIDE_MS 900U
#define ROBOT_PATROL_DETOUR_PASS_MS 2600U
#define ROBOT_PATROL_DETOUR_RETURN_MS 900U
#define ROBOT_PATROL_MIN_RESUME_FORWARD_MS 900U
#define ROBOT_PATROL_ENV_DWELL_MS 1200U
#define ROBOT_PATROL_ENV_REARM_MS 10000U
#define ROBOT_MONITOR_ENV_INTERVAL_MS 2000U
#define ROBOT_MONITOR_OBSTACLE_INTERVAL_MS 100U
#define ROBOT_MONITOR_REPORT_INTERVAL_MS 100U
#define ROBOT_MONITOR_TEMP_HIGH_DECI_C 350
#define ROBOT_MONITOR_HUMIDITY_HIGH_DECI_PERCENT 850U
#define ROBOT_MONITOR_AGE_UNKNOWN 0xFFFFFFFFU
#define ROBOT_ENABLE_RAW_UART 0

typedef struct {
    uint8_t frame[ROBOT_COMMAND_SIZE];
    uint8_t index;
} frame_reader_t;

typedef enum {
    ROBOT_AVOID_IDLE = 0,
    ROBOT_AVOID_FORWARD = 1,
    ROBOT_AVOID_BACKWARD = 2,
    ROBOT_AVOID_TURN_RIGHT = 3,
    ROBOT_AVOID_SENSOR_ERROR = 4
} robot_avoid_phase_t;

typedef enum {
    ROBOT_PATROL_IDLE = 0,
    ROBOT_PATROL_FORWARD = 1,
    ROBOT_PATROL_TURN_RIGHT = 2,
    ROBOT_PATROL_OBSTACLE_BACKWARD = 3,
    ROBOT_PATROL_OBSTACLE_TURN_RIGHT = 4,
    ROBOT_PATROL_DETOUR_SIDE_FORWARD = 5,
    ROBOT_PATROL_DETOUR_TURN_LEFT_TO_PARALLEL = 6,
    ROBOT_PATROL_DETOUR_PASS_FORWARD = 7,
    ROBOT_PATROL_DETOUR_TURN_LEFT_TO_RETURN = 8,
    ROBOT_PATROL_DETOUR_RETURN_FORWARD = 9,
    ROBOT_PATROL_DETOUR_TURN_RIGHT_TO_TRACK = 10,
    ROBOT_PATROL_ENV_ALERT = 11,
    ROBOT_PATROL_SENSOR_ERROR = 12,
    ROBOT_PATROL_TURN_LEFT = 13
} robot_patrol_phase_t;

static bool g_motor_ready;
static bool g_moving;
static bool g_received_command;
static bool g_avoid_active;
static bool g_avoid_test_active;
static bool g_patrol_active;
static bool g_robot_task_started;
static int8_t g_motion_left;
static int8_t g_motion_right;
static uint64_t g_last_command_ms;
static uint64_t g_next_motion_keepalive_ms;
static uint64_t g_avoid_phase_until_ms;
static uint64_t g_next_obstacle_sample_ms;
static uint64_t g_avoid_test_until_ms;
static uint64_t g_avoid_test_block_until_ms;
static uint64_t g_patrol_phase_until_ms;
static uint64_t g_patrol_forward_until_ms;
static uint64_t g_patrol_next_obstacle_sample_ms;
static uint64_t g_patrol_next_env_alert_ms;
static uint64_t g_next_env_monitor_ms;
static uint64_t g_next_obstacle_monitor_ms;
static uint64_t g_env_sample_ms;
static uint64_t g_obstacle_sample_ms;
static uint64_t g_last_monitor_report_ms;
static uint32_t g_monitor_sample_count;
static uint32_t g_last_monitor_report_flags;
static bool g_monitor_reported_once;
static uint32_t g_patrol_loop_count;
static uint32_t g_patrol_resume_forward_ms;
static uint32_t g_patrol_forward_ms = ROBOT_PATROL_FORWARD_MS;
static uint32_t g_patrol_turn_ms = ROBOT_PATROL_TURN_MS;
static uint32_t g_patrol_max_loops = ROBOT_PATROL_MAX_LOOPS_DEFAULT;
static robot_mvp_route_segment_t g_patrol_route[ROBOT_MVP_ROUTE_MAX_SEGMENTS] = {
    {ROBOT_PATROL_FORWARD_MS, ROBOT_PATROL_TURN_MS, ROBOT_MVP_ROUTE_TURN_RIGHT},
    {ROBOT_PATROL_FORWARD_MS, ROBOT_PATROL_TURN_MS, ROBOT_MVP_ROUTE_TURN_RIGHT},
    {ROBOT_PATROL_FORWARD_MS, ROBOT_PATROL_TURN_MS, ROBOT_MVP_ROUTE_TURN_RIGHT},
    {ROBOT_PATROL_FORWARD_MS, ROBOT_PATROL_TURN_MS, ROBOT_MVP_ROUTE_TURN_RIGHT}
};
static uint8_t g_patrol_route_count = ROBOT_PATROL_ROUTE_LEGS_DEFAULT;
static uint8_t g_at_sequence;
static uint8_t g_last_command;
static uint8_t g_last_sequence;
static uint8_t g_patrol_speed = ROBOT_PATROL_SPEED;
static uint8_t g_patrol_leg_index;
static uint8_t g_patrol_block_count;
static uint8_t g_patrol_invalid_count;
static robot_avoid_phase_t g_avoid_phase;
static robot_patrol_phase_t g_patrol_phase;
static robot_status_t g_patrol_status;
static robot_env_data_t g_env;
static robot_obstacle_data_t g_obstacle;

static void refresh_oled(uint64_t now_ms)
{
    robot_oled_render(
        g_motor_ready,
        g_moving,
        g_last_command,
        g_last_sequence,
        (uint32_t)now_ms,
        g_env.valid,
        g_env.temperature_deci_c,
        g_env.humidity_deci_percent
    );
}

static uint32_t monitor_age_ms(uint64_t now_ms, uint64_t sample_ms)
{
    if (sample_ms == 0U) {
        return ROBOT_MONITOR_AGE_UNKNOWN;
    }
    return (uint32_t)(now_ms - sample_ms);
}

static uint32_t monitor_alarm_flags(void)
{
    uint32_t flags = ROBOT_MONITOR_ALARM_NONE;

    if (!g_env.valid) {
        flags |= ROBOT_MONITOR_ALARM_ENV_INVALID;
    } else {
        if (g_env.temperature_deci_c >= ROBOT_MONITOR_TEMP_HIGH_DECI_C) {
            flags |= ROBOT_MONITOR_ALARM_TEMP_HIGH;
        }
        if (g_env.humidity_deci_percent >= ROBOT_MONITOR_HUMIDITY_HIGH_DECI_PERCENT) {
            flags |= ROBOT_MONITOR_ALARM_HUMIDITY_HIGH;
        }
    }

    if (!g_obstacle.enabled || !g_obstacle.valid) {
        flags |= ROBOT_MONITOR_ALARM_OBSTACLE_INVALID;
    } else if (g_obstacle.blocked) {
        flags |= ROBOT_MONITOR_ALARM_OBSTACLE_BLOCKED;
    }
    return flags;
}

static bool sample_env(uint64_t now_ms)
{
    bool ok = robot_env_read(&g_env);
    g_env_sample_ms = now_ms;
    ++g_monitor_sample_count;
    return ok;
}

static bool sample_obstacle_into(robot_obstacle_data_t *output, uint64_t now_ms)
{
    bool ok = robot_obstacle_read(&g_obstacle);
    g_obstacle_sample_ms = now_ms;
    ++g_monitor_sample_count;
    if (output != NULL) {
        *output = g_obstacle;
    }
    return ok;
}

static void report_monitor_telemetry(uint64_t now_ms)
{
    uint32_t alarm_flags = monitor_alarm_flags();

    if (g_monitor_reported_once && (alarm_flags == g_last_monitor_report_flags) &&
        ((now_ms - g_last_monitor_report_ms) < ROBOT_MONITOR_REPORT_INTERVAL_MS)) {
        return;
    }

    /* A compact, documented monitor frame lets the Pi voice gateway announce
     * changes even when the HarmonyOS page is not polling the SLE service. */
    osal_printk(
        "\r\n+ROBOT:MON,%u,%u,%u,%u,%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
        (uint32_t)now_ms,
        g_motor_ready ? 1U : 0U,
        g_moving ? 1U : 0U,
        g_env.valid ? 1U : 0U,
        g_env.temperature_deci_c,
        g_env.humidity_deci_percent,
        g_obstacle.enabled ? 1U : 0U,
        g_obstacle.valid ? 1U : 0U,
        g_obstacle.blocked ? 1U : 0U,
        g_obstacle.distance_mm,
        g_obstacle.threshold_mm,
        g_obstacle.reason,
        alarm_flags,
        g_monitor_sample_count,
        monitor_age_ms(now_ms, g_env_sample_ms),
        monitor_age_ms(now_ms, g_obstacle_sample_ms)
    );
    g_last_monitor_report_ms = now_ms;
    g_last_monitor_report_flags = alarm_flags;
    g_monitor_reported_once = true;
}

static void update_monitoring(uint64_t now_ms)
{
    bool refresh_needed = false;

    if ((g_next_env_monitor_ms == 0U) || (now_ms >= g_next_env_monitor_ms)) {
        (void)sample_env(now_ms);
        g_next_env_monitor_ms = now_ms + ROBOT_MONITOR_ENV_INTERVAL_MS;
        refresh_needed = true;
    }

    if (!g_avoid_active && !g_patrol_active &&
        ((g_next_obstacle_monitor_ms == 0U) || (now_ms >= g_next_obstacle_monitor_ms))) {
        (void)sample_obstacle_into(NULL, now_ms);
        g_next_obstacle_monitor_ms = now_ms + ROBOT_MONITOR_OBSTACLE_INTERVAL_MS;
    }

    if (refresh_needed) {
        refresh_oled(now_ms);
    }
    report_monitor_telemetry(now_ms);
}

#if ROBOT_ENABLE_RAW_UART
static void send_ack(uint8_t sequence, robot_status_t status)
{
    uint8_t ack[ROBOT_ACK_SIZE];
    robot_ack_encode(sequence, status, ack);
    (void)uapi_uart_write(UART_BUS_0, ack, sizeof(ack), 0U);
}
#endif

static bool motor_backend_init(void)
{
#ifdef CONFIG_ROBOT_MVP_ENABLE_WAVE_ROVER_LINK
    return robot_rover_link_init();
#else
    return robot_motor_init();
#endif
}

static bool motor_backend_stop(void)
{
#ifdef CONFIG_ROBOT_MVP_ENABLE_WAVE_ROVER_LINK
    return robot_rover_link_stop();
#else
    return robot_motor_stop();
#endif
}

static bool motor_backend_apply(int8_t left, int8_t right)
{
#ifdef CONFIG_ROBOT_MVP_ENABLE_WAVE_ROVER_LINK
    return robot_rover_link_apply(left, right);
#else
    return robot_motor_apply(left, right);
#endif
}

static void stop_motion(void)
{
    if (g_motor_ready) {
        (void)motor_backend_stop();
    }
    g_moving = false;
    g_motion_left = 0;
    g_motion_right = 0;
    g_next_motion_keepalive_ms = 0U;
}

static bool drive_motion(int8_t left, int8_t right)
{
    if (!g_motor_ready) {
        g_moving = false;
        return false;
    }

    /* Do not interrupt a continuous leg by re-sending an identical speed command. */
    if (g_moving && (g_motion_left == left) && (g_motion_right == right)) {
        return true;
    }

    if (!motor_backend_apply(left, right)) {
        g_moving = false;
        return false;
    }
    g_moving = (left != 0) || (right != 0);
    g_motion_left = left;
    g_motion_right = right;
    g_next_motion_keepalive_ms = uapi_tcxo_get_ms() + ROBOT_ROVER_KEEPALIVE_MS;
    return true;
}

static void update_motion_keepalive(uint64_t now_ms)
{
#ifdef CONFIG_ROBOT_MVP_ENABLE_WAVE_ROVER_LINK
    if (!g_motor_ready || !g_moving || (now_ms < g_next_motion_keepalive_ms)) {
        return;
    }

    /* The Pi bridge stops a moving rover when it has no fresh JSON for 0.3s. */
    if (!motor_backend_apply(g_motion_left, g_motion_right)) {
        g_moving = false;
        g_motion_left = 0;
        g_motion_right = 0;
        g_next_motion_keepalive_ms = 0U;
        if (g_patrol_active) {
            g_patrol_active = false;
            g_patrol_phase = ROBOT_PATROL_IDLE;
            g_patrol_status = ROBOT_STATUS_MOTOR_ERROR;
        }
        if (g_avoid_active) {
            g_avoid_active = false;
            g_avoid_phase = ROBOT_AVOID_IDLE;
        }
        (void)motor_backend_stop();
        return;
    }
    g_next_motion_keepalive_ms = now_ms + ROBOT_ROVER_KEEPALIVE_MS;
#else
    unused(now_ms);
#endif
}

static void reset_patrol_state(robot_patrol_phase_t phase)
{
    g_patrol_active = false;
    g_patrol_phase = phase;
    g_patrol_phase_until_ms = 0U;
    g_patrol_forward_until_ms = 0U;
    g_patrol_next_obstacle_sample_ms = 0U;
    g_patrol_next_env_alert_ms = 0U;
    g_patrol_leg_index = 0U;
    g_patrol_loop_count = 0U;
    g_patrol_resume_forward_ms = 0U;
    g_patrol_block_count = 0U;
    g_patrol_invalid_count = 0U;
}

static void clear_patrol_timers(void)
{
    g_patrol_phase_until_ms = 0U;
    g_patrol_forward_until_ms = 0U;
    g_patrol_next_obstacle_sample_ms = 0U;
    g_patrol_next_env_alert_ms = 0U;
    g_patrol_resume_forward_ms = 0U;
}

static void disable_patrol(robot_patrol_phase_t phase)
{
    stop_motion();
    g_patrol_active = false;
    g_patrol_phase = phase;
    clear_patrol_timers();
}

static void disable_avoidance(robot_avoid_phase_t phase)
{
    stop_motion();
    g_avoid_active = false;
    g_avoid_test_active = false;
    g_avoid_phase = phase;
    g_avoid_phase_until_ms = 0U;
    g_next_obstacle_sample_ms = 0U;
    g_avoid_test_until_ms = 0U;
    g_avoid_test_block_until_ms = 0U;
}

static void disable_autonomy(void)
{
    g_avoid_active = false;
    g_avoid_test_active = false;
    g_avoid_phase = ROBOT_AVOID_IDLE;
    g_avoid_phase_until_ms = 0U;
    g_next_obstacle_sample_ms = 0U;
    g_avoid_test_until_ms = 0U;
    g_avoid_test_block_until_ms = 0U;
    reset_patrol_state(ROBOT_PATROL_IDLE);
    g_patrol_status = ROBOT_STATUS_OK;
}

static bool read_avoidance_obstacle(robot_obstacle_data_t *output, uint64_t now_ms)
{
    if (g_avoid_test_active) {
        output->enabled = true;
        output->valid = true;
        output->threshold_mm = robot_obstacle_threshold_mm();
        output->distance_mm = (now_ms < g_avoid_test_block_until_ms) ?
            ROBOT_AVOID_TEST_BLOCK_DISTANCE_MM : ROBOT_AVOID_TEST_CLEAR_DISTANCE_MM;
        output->blocked = output->distance_mm < output->threshold_mm;
        g_obstacle = *output;
        g_obstacle_sample_ms = now_ms;
        ++g_monitor_sample_count;
        return true;
    }
    return sample_obstacle_into(output, now_ms);
}

static bool avoid_forward(uint64_t now_ms)
{
    if (!drive_motion(ROBOT_AVOID_SPEED, ROBOT_AVOID_SPEED)) {
        disable_avoidance(ROBOT_AVOID_IDLE);
        return false;
    }
    g_avoid_phase = ROBOT_AVOID_FORWARD;
    g_avoid_phase_until_ms = 0U;
    g_next_obstacle_sample_ms = now_ms + ROBOT_AVOID_SAMPLE_MS;
    return true;
}

static bool avoid_backward(uint64_t now_ms)
{
    if (!drive_motion(-ROBOT_AVOID_SPEED, -ROBOT_AVOID_SPEED)) {
        disable_avoidance(ROBOT_AVOID_IDLE);
        return false;
    }
    g_avoid_phase = ROBOT_AVOID_BACKWARD;
    g_avoid_phase_until_ms = now_ms + ROBOT_AVOID_BACKWARD_MS;
    return true;
}

static bool avoid_turn_right(uint64_t now_ms)
{
    if (!drive_motion(ROBOT_AVOID_SPEED, -ROBOT_AVOID_SPEED)) {
        disable_avoidance(ROBOT_AVOID_IDLE);
        return false;
    }
    g_avoid_phase = ROBOT_AVOID_TURN_RIGHT;
    g_avoid_phase_until_ms = now_ms + ROBOT_AVOID_TURN_MS;
    return true;
}

static void update_avoidance(uint64_t now_ms)
{
    bool sampled;

    if (!g_avoid_active) {
        return;
    }

    if (!g_motor_ready) {
        disable_avoidance(ROBOT_AVOID_IDLE);
        osal_printk("\r\nROBOT AVOID STOP motor not ready\r\n");
        return;
    }

    if (g_avoid_test_active && (now_ms >= g_avoid_test_until_ms)) {
        disable_avoidance(ROBOT_AVOID_IDLE);
        osal_printk("\r\nROBOT AVOID TEST DONE\r\n");
        return;
    }

    if (g_avoid_phase == ROBOT_AVOID_BACKWARD) {
        if (now_ms >= g_avoid_phase_until_ms) {
            (void)avoid_turn_right(now_ms);
        }
        return;
    }

    if (g_avoid_phase == ROBOT_AVOID_TURN_RIGHT) {
        if (now_ms >= g_avoid_phase_until_ms) {
            (void)avoid_forward(now_ms);
        }
        return;
    }

    if (now_ms < g_next_obstacle_sample_ms) {
        return;
    }
    g_next_obstacle_sample_ms = now_ms + ROBOT_AVOID_SAMPLE_MS;

    sampled = read_avoidance_obstacle(&g_obstacle, now_ms);
    if (!sampled || !g_obstacle.enabled || !g_obstacle.valid) {
        disable_avoidance(ROBOT_AVOID_SENSOR_ERROR);
        osal_printk("\r\nROBOT AVOID STOP sensor invalid\r\n");
        return;
    }

    if (g_obstacle.blocked) {
        stop_motion();
        osal_printk("\r\nROBOT AVOID blocked distance=%u threshold=%u\r\n",
            g_obstacle.distance_mm, g_obstacle.threshold_mm);
        (void)avoid_backward(now_ms);
        return;
    }

    if ((g_avoid_phase != ROBOT_AVOID_FORWARD) || !g_moving) {
        (void)avoid_forward(now_ms);
    }
}

static uint32_t patrol_current_forward_ms(void)
{
    if ((g_patrol_route_count == 0U) || (g_patrol_leg_index >= g_patrol_route_count)) {
        return g_patrol_forward_ms;
    }
    return g_patrol_route[g_patrol_leg_index].forward_ms;
}

static void patrol_finish_route_turn(uint64_t now_ms);

static bool patrol_forward(uint64_t now_ms)
{
    if (!drive_motion((int8_t)g_patrol_speed, (int8_t)g_patrol_speed)) {
        g_patrol_status = ROBOT_STATUS_MOTOR_ERROR;
        disable_patrol(ROBOT_PATROL_IDLE);
        return false;
    }
    if ((g_patrol_forward_until_ms == 0U) || (now_ms >= g_patrol_forward_until_ms)) {
        g_patrol_forward_until_ms = now_ms + patrol_current_forward_ms();
    }
    g_patrol_phase = ROBOT_PATROL_FORWARD;
    g_patrol_next_obstacle_sample_ms = now_ms + ROBOT_PATROL_SAMPLE_MS;
    return true;
}

static bool patrol_begin_route_turn(uint64_t now_ms)
{
    robot_mvp_route_segment_t *segment = NULL;
    int8_t left;
    int8_t right;

    if ((g_patrol_route_count == 0U) || (g_patrol_leg_index >= g_patrol_route_count)) {
        g_patrol_status = ROBOT_STATUS_INVALID_COMMAND;
        disable_patrol(ROBOT_PATROL_IDLE);
        return false;
    }

    segment = &g_patrol_route[g_patrol_leg_index];
    if ((segment->turn_direction == ROBOT_MVP_ROUTE_TURN_NONE) || (segment->turn_ms == 0U)) {
        patrol_finish_route_turn(now_ms);
        return true;
    }

    if (segment->turn_direction == ROBOT_MVP_ROUTE_TURN_LEFT) {
        left = -(int8_t)g_patrol_speed;
        right = (int8_t)g_patrol_speed;
        g_patrol_phase = ROBOT_PATROL_TURN_LEFT;
    } else {
        left = (int8_t)g_patrol_speed;
        right = -(int8_t)g_patrol_speed;
        g_patrol_phase = ROBOT_PATROL_TURN_RIGHT;
    }

    if (!drive_motion(left, right)) {
        g_patrol_status = ROBOT_STATUS_MOTOR_ERROR;
        disable_patrol(ROBOT_PATROL_IDLE);
        return false;
    }
    g_patrol_phase_until_ms = now_ms + segment->turn_ms;
    return true;
}

static bool patrol_obstacle_backward(uint64_t now_ms)
{
    if (!drive_motion(-(int8_t)g_patrol_speed, -(int8_t)g_patrol_speed)) {
        g_patrol_status = ROBOT_STATUS_MOTOR_ERROR;
        disable_patrol(ROBOT_PATROL_IDLE);
        return false;
    }
    g_patrol_phase = ROBOT_PATROL_OBSTACLE_BACKWARD;
    g_patrol_phase_until_ms = now_ms + ROBOT_PATROL_OBSTACLE_BACKWARD_MS;
    return true;
}

static bool patrol_obstacle_turn_right(uint64_t now_ms)
{
    if (!drive_motion((int8_t)g_patrol_speed, -(int8_t)g_patrol_speed)) {
        g_patrol_status = ROBOT_STATUS_MOTOR_ERROR;
        disable_patrol(ROBOT_PATROL_IDLE);
        return false;
    }
    g_patrol_phase = ROBOT_PATROL_OBSTACLE_TURN_RIGHT;
    g_patrol_phase_until_ms = now_ms + ROBOT_PATROL_OBSTACLE_TURN_MS;
    return true;
}

static bool patrol_detour_forward(uint64_t now_ms, robot_patrol_phase_t phase, uint32_t duration_ms)
{
    if (!drive_motion((int8_t)g_patrol_speed, (int8_t)g_patrol_speed)) {
        g_patrol_status = ROBOT_STATUS_MOTOR_ERROR;
        disable_patrol(ROBOT_PATROL_IDLE);
        return false;
    }
    g_patrol_phase = phase;
    g_patrol_phase_until_ms = now_ms + duration_ms;
    return true;
}

static bool patrol_detour_turn_left(uint64_t now_ms, robot_patrol_phase_t phase)
{
    if (!drive_motion(-(int8_t)g_patrol_speed, (int8_t)g_patrol_speed)) {
        g_patrol_status = ROBOT_STATUS_MOTOR_ERROR;
        disable_patrol(ROBOT_PATROL_IDLE);
        return false;
    }
    g_patrol_phase = phase;
    g_patrol_phase_until_ms = now_ms + ROBOT_PATROL_OBSTACLE_TURN_MS;
    return true;
}

static bool patrol_detour_turn_right_to_track(uint64_t now_ms)
{
    if (!drive_motion((int8_t)g_patrol_speed, -(int8_t)g_patrol_speed)) {
        g_patrol_status = ROBOT_STATUS_MOTOR_ERROR;
        disable_patrol(ROBOT_PATROL_IDLE);
        return false;
    }
    g_patrol_phase = ROBOT_PATROL_DETOUR_TURN_RIGHT_TO_TRACK;
    g_patrol_phase_until_ms = now_ms + ROBOT_PATROL_OBSTACLE_TURN_MS;
    return true;
}

static void patrol_resume_track(uint64_t now_ms)
{
    uint32_t resume_ms = g_patrol_resume_forward_ms;
    if (resume_ms < ROBOT_PATROL_MIN_RESUME_FORWARD_MS) {
        resume_ms = ROBOT_PATROL_MIN_RESUME_FORWARD_MS;
    }
    g_patrol_resume_forward_ms = 0U;
    g_patrol_forward_until_ms = now_ms + resume_ms;
    (void)patrol_forward(now_ms);
}

static void patrol_begin_env_alert(uint64_t now_ms, uint32_t alarm_flags)
{
    stop_motion();
    g_patrol_phase = ROBOT_PATROL_ENV_ALERT;
    g_patrol_phase_until_ms = now_ms + ROBOT_PATROL_ENV_DWELL_MS;
    g_patrol_next_env_alert_ms = now_ms + ROBOT_PATROL_ENV_REARM_MS;
    if (g_patrol_forward_until_ms > now_ms) {
        g_patrol_forward_until_ms += ROBOT_PATROL_ENV_DWELL_MS;
    }
    osal_printk("\r\nROBOT PATROL ENV alarm=0x%x dwell=%u\r\n", alarm_flags, ROBOT_PATROL_ENV_DWELL_MS);
}

static void patrol_finish_route_turn(uint64_t now_ms)
{
    g_patrol_block_count = 0U;
    g_patrol_invalid_count = 0U;
    if (g_patrol_route_count == 0U) {
        g_patrol_status = ROBOT_STATUS_INVALID_COMMAND;
        disable_patrol(ROBOT_PATROL_IDLE);
        return;
    }
    g_patrol_leg_index = (uint8_t)((g_patrol_leg_index + 1U) % g_patrol_route_count);
    if (g_patrol_leg_index == 0U) {
        ++g_patrol_loop_count;
        if ((g_patrol_max_loops != 0U) && (g_patrol_loop_count >= g_patrol_max_loops)) {
            g_patrol_status = ROBOT_STATUS_OK;
            disable_patrol(ROBOT_PATROL_IDLE);
            osal_printk("\r\nROBOT PATROL complete loops=%u\r\n", g_patrol_loop_count);
            return;
        }
    }
    g_patrol_forward_until_ms = now_ms + patrol_current_forward_ms();
    (void)patrol_forward(now_ms);
}

static bool patrol_start_obstacle_confirm(uint64_t now_ms, bool *blocked)
{
    uint8_t sample_index;
    uint8_t blocked_count = 0U;
    bool sampled_any = false;

    if (blocked == NULL) {
        return false;
    }
    *blocked = false;

    for (sample_index = 0U; sample_index < ROBOT_PATROL_START_CONFIRM_SAMPLES; ++sample_index) {
        if (sample_index != 0U) {
            osal_msleep(ROBOT_PATROL_START_SAMPLE_GAP_MS);
            now_ms = uapi_tcxo_get_ms();
        }
        (void)sample_obstacle_into(NULL, now_ms);
        if (!g_obstacle.enabled) {
            return false;
        }
        if (!g_obstacle.valid) {
            continue;
        }
        sampled_any = true;
        if (g_obstacle.blocked) {
            ++blocked_count;
        }
    }

    *blocked = blocked_count >= ROBOT_PATROL_START_BLOCK_CONFIRM_COUNT;
    return sampled_any;
}

static void update_patrol(uint64_t now_ms)
{
    robot_obstacle_data_t obstacle = {0};
    uint32_t alarm_flags;
    bool env_alarm;
    bool sampled;

    if (!g_patrol_active) {
        return;
    }

    if (!g_motor_ready) {
        g_patrol_status = ROBOT_STATUS_MOTOR_ERROR;
        disable_patrol(ROBOT_PATROL_IDLE);
        osal_printk("\r\nROBOT PATROL STOP motor not ready\r\n");
        return;
    }

    alarm_flags = monitor_alarm_flags();
    env_alarm = (alarm_flags & (ROBOT_MONITOR_ALARM_TEMP_HIGH | ROBOT_MONITOR_ALARM_HUMIDITY_HIGH)) != 0U;

    if (((g_patrol_phase == ROBOT_PATROL_FORWARD) || (g_patrol_phase == ROBOT_PATROL_TURN_RIGHT) ||
        (g_patrol_phase == ROBOT_PATROL_TURN_LEFT)) &&
        env_alarm && (now_ms >= g_patrol_next_env_alert_ms)) {
        patrol_begin_env_alert(now_ms, alarm_flags);
        return;
    }

    if (g_patrol_phase == ROBOT_PATROL_ENV_ALERT) {
        if (now_ms >= g_patrol_phase_until_ms) {
            (void)patrol_forward(now_ms);
        }
        return;
    }

    if ((g_patrol_phase == ROBOT_PATROL_TURN_RIGHT) || (g_patrol_phase == ROBOT_PATROL_TURN_LEFT)) {
        if (now_ms >= g_patrol_phase_until_ms) {
            patrol_finish_route_turn(now_ms);
        }
        return;
    }

    if (g_patrol_phase == ROBOT_PATROL_OBSTACLE_BACKWARD) {
        if (now_ms >= g_patrol_phase_until_ms) {
            (void)patrol_obstacle_turn_right(now_ms);
        }
        return;
    }

    if (g_patrol_phase == ROBOT_PATROL_OBSTACLE_TURN_RIGHT) {
        if (now_ms >= g_patrol_phase_until_ms) {
            (void)patrol_detour_forward(now_ms, ROBOT_PATROL_DETOUR_SIDE_FORWARD, ROBOT_PATROL_DETOUR_SIDE_MS);
        }
        return;
    }

    if (g_patrol_phase == ROBOT_PATROL_DETOUR_SIDE_FORWARD) {
        if (now_ms >= g_patrol_phase_until_ms) {
            (void)patrol_detour_turn_left(now_ms, ROBOT_PATROL_DETOUR_TURN_LEFT_TO_PARALLEL);
        }
        return;
    }

    if (g_patrol_phase == ROBOT_PATROL_DETOUR_TURN_LEFT_TO_PARALLEL) {
        if (now_ms >= g_patrol_phase_until_ms) {
            (void)patrol_detour_forward(now_ms, ROBOT_PATROL_DETOUR_PASS_FORWARD, ROBOT_PATROL_DETOUR_PASS_MS);
        }
        return;
    }

    if (g_patrol_phase == ROBOT_PATROL_DETOUR_PASS_FORWARD) {
        if (now_ms >= g_patrol_phase_until_ms) {
            (void)patrol_detour_turn_left(now_ms, ROBOT_PATROL_DETOUR_TURN_LEFT_TO_RETURN);
        }
        return;
    }

    if (g_patrol_phase == ROBOT_PATROL_DETOUR_TURN_LEFT_TO_RETURN) {
        if (now_ms >= g_patrol_phase_until_ms) {
            (void)patrol_detour_forward(now_ms, ROBOT_PATROL_DETOUR_RETURN_FORWARD, ROBOT_PATROL_DETOUR_RETURN_MS);
        }
        return;
    }

    if (g_patrol_phase == ROBOT_PATROL_DETOUR_RETURN_FORWARD) {
        if (now_ms >= g_patrol_phase_until_ms) {
            (void)patrol_detour_turn_right_to_track(now_ms);
        }
        return;
    }

    if (g_patrol_phase == ROBOT_PATROL_DETOUR_TURN_RIGHT_TO_TRACK) {
        if (now_ms >= g_patrol_phase_until_ms) {
            patrol_resume_track(now_ms);
        }
        return;
    }

    if (g_patrol_phase != ROBOT_PATROL_FORWARD) {
        (void)patrol_forward(now_ms);
        return;
    }

    if (now_ms >= g_patrol_forward_until_ms) {
        (void)patrol_begin_route_turn(now_ms);
        return;
    }

    if (now_ms < g_patrol_next_obstacle_sample_ms) {
        return;
    }
    g_patrol_next_obstacle_sample_ms = now_ms + ROBOT_PATROL_SAMPLE_MS;

    sampled = sample_obstacle_into(&obstacle, now_ms);
    if (!obstacle.enabled) {
        g_patrol_status = ROBOT_STATUS_OBSTACLE_STOP;
        disable_patrol(ROBOT_PATROL_SENSOR_ERROR);
        osal_printk("\r\nROBOT PATROL STOP sensor invalid reason=%u\r\n", obstacle.reason);
        return;
    }
    if (!sampled || !obstacle.valid) {
        ++g_patrol_invalid_count;
        g_patrol_block_count = 0U;
        osal_printk("\r\nROBOT PATROL ignore invalid distance reason=%u count=%u\r\n",
            obstacle.reason, g_patrol_invalid_count);
        if (!g_moving) {
            (void)patrol_forward(now_ms);
        }
        return;
    }
    g_patrol_invalid_count = 0U;

    if (obstacle.blocked) {
        ++g_patrol_block_count;
        if (g_patrol_block_count < ROBOT_PATROL_BLOCK_CONFIRM_COUNT) {
            osal_printk("\r\nROBOT PATROL confirm obstacle distance=%u threshold=%u count=%u\r\n",
                obstacle.distance_mm, obstacle.threshold_mm, g_patrol_block_count);
            return;
        }
        g_patrol_block_count = 0U;
        stop_motion();
        if (g_patrol_forward_until_ms > now_ms) {
            g_patrol_resume_forward_ms = (uint32_t)(g_patrol_forward_until_ms - now_ms);
        } else {
            g_patrol_resume_forward_ms = ROBOT_PATROL_MIN_RESUME_FORWARD_MS;
        }
        osal_printk("\r\nROBOT PATROL blocked distance=%u threshold=%u leg=%u loop=%u\r\n",
            obstacle.distance_mm, obstacle.threshold_mm, g_patrol_leg_index, g_patrol_loop_count);
        (void)patrol_obstacle_backward(now_ms);
        return;
    }

    g_patrol_block_count = 0U;
    if (!g_moving) {
        (void)patrol_forward(now_ms);
    }
}

static robot_status_t apply_command(const robot_command_t *command)
{
    int8_t left = (int8_t)command->speed_left;
    int8_t right = (int8_t)command->speed_right;

    disable_autonomy();

    if (command->command == ROBOT_CMD_STOP) {
        stop_motion();
        return ROBOT_STATUS_OK;
    }

    if (!g_motor_ready) {
        g_moving = false;
        return ROBOT_STATUS_MOTOR_ERROR;
    }

    if (command->command == ROBOT_CMD_FORWARD) {
        (void)sample_obstacle_into(NULL, uapi_tcxo_get_ms());
        if (g_obstacle.enabled && g_obstacle.valid && g_obstacle.blocked) {
            stop_motion();
            return ROBOT_STATUS_OBSTACLE_STOP;
        }
    }

    switch (command->command) {
        case ROBOT_CMD_FORWARD:
            break;
        case ROBOT_CMD_BACKWARD:
            left = -left;
            right = -right;
            break;
        case ROBOT_CMD_LEFT:
            left = -left;
            break;
        case ROBOT_CMD_RIGHT:
            right = -right;
            break;
        default:
            stop_motion();
            return ROBOT_STATUS_INVALID_COMMAND;
    }

    if (!drive_motion(left, right)) {
        return ROBOT_STATUS_MOTOR_ERROR;
    }
    return ROBOT_STATUS_OK;
}

#if ROBOT_ENABLE_RAW_UART
static void process_frame(const uint8_t frame[ROBOT_COMMAND_SIZE])
{
    robot_command_t command;
    robot_status_t status = robot_command_decode(frame, &command);
    uint8_t sequence = frame[4];

    if (status == ROBOT_STATUS_OK) {
        status = apply_command(&command);
        g_last_command_ms = uapi_tcxo_get_ms();
        g_received_command = true;
    } else {
        (void)motor_backend_stop();
        g_moving = false;
    }

    send_ack(sequence, status);
    osal_printk(
        "\r\nROBOT ACK seq=%u cmd=%u status=%u moving=%u\r\n",
        sequence,
        frame[1],
        status,
        g_moving ? 1U : 0U
    );
}
#endif

static robot_status_t handle_command(uint8_t command, int8_t left, int8_t right, uint8_t sequence)
{
    robot_command_t robot_command;
    robot_status_t status;

    robot_command.command = command;
    robot_command.speed_left = (uint8_t)left;
    robot_command.speed_right = (uint8_t)right;
    robot_command.sequence = sequence;

    status = apply_command(&robot_command);
    g_last_command_ms = uapi_tcxo_get_ms();
    g_received_command = true;
    g_last_command = command;
    g_last_sequence = sequence;
    refresh_oled(g_last_command_ms);
    return status;
}

static void fill_avoid_result(
    robot_mvp_avoid_result_t *result,
    robot_status_t status,
    uint32_t test_duration_ms)
{
    if (result == NULL) {
        return;
    }
    result->active = g_avoid_active;
    result->phase = (uint8_t)g_avoid_phase;
    result->status = status;
    result->obstacle = g_obstacle;
    result->test_duration_ms = test_duration_ms;
}

static void fill_patrol_result(robot_mvp_patrol_result_t *result, robot_status_t status)
{
    if (result == NULL) {
        return;
    }
    result->active = g_patrol_active;
    result->phase = (uint8_t)g_patrol_phase;
    result->status = status;
    result->leg_index = g_patrol_leg_index;
    result->loop_count = g_patrol_loop_count;
    result->alarm_flags = monitor_alarm_flags();
    result->obstacle = g_obstacle;
}

robot_status_t robot_mvp_control_motion(
    robot_command_code_t command,
    uint8_t speed,
    uint8_t *sequence,
    bool *moving)
{
    uint8_t next_sequence = ++g_at_sequence;
    robot_status_t status;

    if (speed > 100U) {
        speed = 100U;
    }

    status = handle_command(command, (int8_t)speed, (int8_t)speed, next_sequence);
    if (sequence != NULL) {
        *sequence = next_sequence;
    }
    if (moving != NULL) {
        *moving = g_moving;
    }
    return status;
}

bool robot_mvp_control_motor_init(bool *ready)
{
    bool motor_ready = true;

    disable_autonomy();

    if (!g_motor_ready) {
        motor_ready = motor_backend_init();
    } else {
        motor_ready = motor_backend_stop();
    }
    g_motor_ready = motor_ready;
    g_moving = false;
    refresh_oled(uapi_tcxo_get_ms());

    if (ready != NULL) {
        *ready = motor_ready;
    }
    return motor_ready;
}

bool robot_mvp_control_oled_init(bool *ready)
{
    bool oled_ready = robot_oled_init();
    refresh_oled(uapi_tcxo_get_ms());

    if (ready != NULL) {
        *ready = oled_ready;
    }
    return oled_ready;
}

bool robot_mvp_control_read_env(robot_env_data_t *env)
{
    uint64_t now_ms = uapi_tcxo_get_ms();
    bool ok = sample_env(now_ms);

    refresh_oled(now_ms);
    if (env != NULL) {
        *env = g_env;
    }
    return ok;
}

bool robot_mvp_control_read_obstacle(robot_obstacle_data_t *obstacle)
{
    uint64_t now_ms = uapi_tcxo_get_ms();
    bool ok = sample_obstacle_into(obstacle, now_ms);

    refresh_oled(now_ms);
    return ok;
}

robot_status_t robot_mvp_control_start_avoid(robot_mvp_avoid_result_t *result)
{
    uint64_t now_ms = uapi_tcxo_get_ms();
    robot_status_t status = ROBOT_STATUS_OK;
    bool sampled = false;

    disable_patrol(ROBOT_PATROL_IDLE);
    g_avoid_test_active = false;
    g_avoid_test_until_ms = 0U;
    g_avoid_test_block_until_ms = 0U;

    if (!g_motor_ready) {
        disable_avoidance(ROBOT_AVOID_IDLE);
        status = ROBOT_STATUS_MOTOR_ERROR;
    } else {
        sampled = sample_obstacle_into(NULL, now_ms);
        if (!sampled || !g_obstacle.enabled || !g_obstacle.valid) {
            disable_avoidance(ROBOT_AVOID_SENSOR_ERROR);
            status = ROBOT_STATUS_OBSTACLE_STOP;
        } else {
            g_avoid_active = true;
            g_received_command = true;
            g_last_command_ms = now_ms;
            g_last_command = ROBOT_CMD_FORWARD;
            g_last_sequence = ++g_at_sequence;
            if (g_obstacle.blocked) {
                stop_motion();
                if (!avoid_backward(now_ms)) {
                    status = ROBOT_STATUS_MOTOR_ERROR;
                }
            } else if (!avoid_forward(now_ms)) {
                status = ROBOT_STATUS_MOTOR_ERROR;
            }
        }
    }

    refresh_oled(now_ms);
    fill_avoid_result(result, status, 0U);
    return status;
}

robot_status_t robot_mvp_control_start_avoid_test(robot_mvp_avoid_result_t *result)
{
    uint64_t now_ms = uapi_tcxo_get_ms();
    robot_status_t status = ROBOT_STATUS_OK;
    bool sampled = false;

    disable_patrol(ROBOT_PATROL_IDLE);
    if (!g_motor_ready) {
        disable_avoidance(ROBOT_AVOID_IDLE);
        status = ROBOT_STATUS_MOTOR_ERROR;
    } else {
        g_avoid_test_active = true;
        g_avoid_test_until_ms = now_ms + ROBOT_AVOID_TEST_DURATION_MS;
        g_avoid_test_block_until_ms = now_ms + ROBOT_AVOID_TEST_BLOCK_MS;
        sampled = read_avoidance_obstacle(&g_obstacle, now_ms);
        if (!sampled || !g_obstacle.enabled || !g_obstacle.valid) {
            disable_avoidance(ROBOT_AVOID_SENSOR_ERROR);
            status = ROBOT_STATUS_OBSTACLE_STOP;
        } else {
            g_avoid_active = true;
            g_received_command = true;
            g_last_command_ms = now_ms;
            g_last_command = ROBOT_CMD_FORWARD;
            g_last_sequence = ++g_at_sequence;
            if (g_obstacle.blocked) {
                stop_motion();
                if (!avoid_backward(now_ms)) {
                    status = ROBOT_STATUS_MOTOR_ERROR;
                }
            } else if (!avoid_forward(now_ms)) {
                status = ROBOT_STATUS_MOTOR_ERROR;
            }
        }
    }

    refresh_oled(now_ms);
    fill_avoid_result(result, status, ROBOT_AVOID_TEST_DURATION_MS);
    return status;
}

bool robot_mvp_control_config_patrol(const robot_mvp_patrol_config_t *config)
{
    if (config == NULL) {
        return false;
    }
    if ((config->forward_ms < ROBOT_PATROL_FORWARD_MIN_MS) ||
        (config->forward_ms > ROBOT_PATROL_FORWARD_MAX_MS) ||
        (config->turn_ms < ROBOT_PATROL_TURN_MIN_MS) ||
        (config->turn_ms > ROBOT_PATROL_TURN_MAX_MS) ||
        (config->speed < ROBOT_PATROL_SPEED_MIN) ||
        (config->speed > ROBOT_PATROL_SPEED_MAX) ||
        (config->max_loops > ROBOT_PATROL_MAX_LOOPS_LIMIT)) {
        return false;
    }

    disable_patrol(ROBOT_PATROL_IDLE);
    g_patrol_forward_ms = config->forward_ms;
    g_patrol_turn_ms = config->turn_ms;
    g_patrol_speed = config->speed;
    g_patrol_max_loops = config->max_loops;
    g_patrol_route_count = ROBOT_PATROL_ROUTE_LEGS_DEFAULT;
    for (uint8_t index = 0U; index < g_patrol_route_count; ++index) {
        g_patrol_route[index].forward_ms = config->forward_ms;
        g_patrol_route[index].turn_ms = config->turn_ms;
        g_patrol_route[index].turn_direction = ROBOT_MVP_ROUTE_TURN_RIGHT;
    }
    g_patrol_status = ROBOT_STATUS_OK;
    osal_printk("\r\nROBOT PATROL config forward=%u turn=%u speed=%u loops=%u\r\n",
        g_patrol_forward_ms, g_patrol_turn_ms, g_patrol_speed, g_patrol_max_loops);
    return true;
}

void robot_mvp_control_get_patrol_config(robot_mvp_patrol_config_t *config)
{
    if (config == NULL) {
        return;
    }
    config->forward_ms = g_patrol_forward_ms;
    config->turn_ms = g_patrol_turn_ms;
    config->speed = g_patrol_speed;
    config->max_loops = g_patrol_max_loops;
}

bool robot_mvp_control_config_route(const robot_mvp_route_config_t *config)
{
    uint8_t index;

    if ((config == NULL) || (config->segment_count == 0U) ||
        (config->segment_count > ROBOT_MVP_ROUTE_MAX_SEGMENTS) ||
        (config->speed < ROBOT_PATROL_SPEED_MIN) || (config->speed > ROBOT_PATROL_SPEED_MAX) ||
        (config->max_loops > ROBOT_PATROL_MAX_LOOPS_LIMIT)) {
        return false;
    }

    for (index = 0U; index < config->segment_count; ++index) {
        const robot_mvp_route_segment_t *segment = &config->segments[index];
        if ((segment->forward_ms < ROBOT_PATROL_FORWARD_MIN_MS) ||
            (segment->forward_ms > ROBOT_PATROL_FORWARD_MAX_MS) ||
            (segment->turn_direction > ROBOT_MVP_ROUTE_TURN_RIGHT)) {
            return false;
        }
        if (segment->turn_direction == ROBOT_MVP_ROUTE_TURN_NONE) {
            if (segment->turn_ms != 0U) {
                return false;
            }
        } else if ((segment->turn_ms < ROBOT_PATROL_TURN_MIN_MS) ||
            (segment->turn_ms > ROBOT_PATROL_TURN_MAX_MS)) {
            return false;
        }
    }

    disable_patrol(ROBOT_PATROL_IDLE);
    g_patrol_route_count = config->segment_count;
    g_patrol_speed = config->speed;
    g_patrol_max_loops = config->max_loops;
    for (index = 0U; index < g_patrol_route_count; ++index) {
        g_patrol_route[index] = config->segments[index];
    }
    g_patrol_forward_ms = g_patrol_route[0].forward_ms;
    g_patrol_turn_ms = g_patrol_route[0].turn_ms;
    g_patrol_status = ROBOT_STATUS_OK;
    osal_printk("\r\nROBOT ROUTE config segments=%u speed=%u loops=%u\r\n",
        g_patrol_route_count, g_patrol_speed, g_patrol_max_loops);
    return true;
}

void robot_mvp_control_get_route_config(robot_mvp_route_config_t *config)
{
    uint8_t index;

    if (config == NULL) {
        return;
    }
    (void)memset_s(config, sizeof(*config), 0, sizeof(*config));
    config->segment_count = g_patrol_route_count;
    config->speed = g_patrol_speed;
    config->max_loops = g_patrol_max_loops;
    for (index = 0U; index < g_patrol_route_count; ++index) {
        config->segments[index] = g_patrol_route[index];
    }
}

bool robot_mvp_control_set_wheel_calibration(const robot_mvp_wheel_calibration_t *calibration)
{
    if (calibration == NULL) {
        return false;
    }

    disable_autonomy();
    stop_motion();
#ifdef CONFIG_ROBOT_MVP_ENABLE_WAVE_ROVER_LINK
    return robot_rover_link_set_wheel_scale(calibration->left_percent, calibration->right_percent);
#else
    return false;
#endif
}

void robot_mvp_control_get_wheel_calibration(robot_mvp_wheel_calibration_t *calibration)
{
    if (calibration == NULL) {
        return;
    }

    calibration->left_percent = 100U;
    calibration->right_percent = 100U;
#ifdef CONFIG_ROBOT_MVP_ENABLE_WAVE_ROVER_LINK
    robot_rover_link_get_wheel_scale(&calibration->left_percent, &calibration->right_percent);
#endif
}

robot_status_t robot_mvp_control_start_patrol(robot_mvp_patrol_result_t *result)
{
    uint64_t now_ms = uapi_tcxo_get_ms();
    robot_status_t status = ROBOT_STATUS_OK;
    bool motor_ready = true;
    bool start_blocked = false;

    disable_avoidance(ROBOT_AVOID_IDLE);
    reset_patrol_state(ROBOT_PATROL_IDLE);
    g_patrol_status = ROBOT_STATUS_OK;

    if (!g_motor_ready) {
        motor_ready = motor_backend_init();
        g_motor_ready = motor_ready;
        g_moving = false;
    }

    if (!motor_ready) {
        status = ROBOT_STATUS_MOTOR_ERROR;
        g_patrol_status = status;
    } else {
        if (!patrol_start_obstacle_confirm(now_ms, &start_blocked)) {
            status = ROBOT_STATUS_OBSTACLE_STOP;
            g_patrol_status = status;
            disable_patrol(ROBOT_PATROL_SENSOR_ERROR);
        } else {
            now_ms = uapi_tcxo_get_ms();
            g_patrol_active = true;
            g_patrol_phase = ROBOT_PATROL_FORWARD;
            g_patrol_status = ROBOT_STATUS_OK;
            g_patrol_leg_index = 0U;
            g_patrol_loop_count = 0U;
            g_patrol_block_count = 0U;
            g_patrol_invalid_count = 0U;
            g_patrol_resume_forward_ms = 0U;
            g_patrol_forward_until_ms = now_ms + patrol_current_forward_ms();
            g_patrol_next_env_alert_ms = now_ms;
            g_received_command = true;
            g_last_command_ms = now_ms;
            g_last_command = ROBOT_CMD_FORWARD;
            g_last_sequence = ++g_at_sequence;
            if (start_blocked) {
                g_patrol_resume_forward_ms = patrol_current_forward_ms();
                stop_motion();
                if (!patrol_obstacle_backward(now_ms)) {
                    status = ROBOT_STATUS_MOTOR_ERROR;
                    g_patrol_status = status;
                }
            } else if (!patrol_forward(now_ms)) {
                status = ROBOT_STATUS_MOTOR_ERROR;
                g_patrol_status = status;
            }
        }
    }

    refresh_oled(now_ms);
    fill_patrol_result(result, status);
    return status;
}

void robot_mvp_control_get_patrol(robot_mvp_patrol_result_t *result)
{
    fill_patrol_result(result, g_patrol_status);
}

void robot_mvp_control_get_state(robot_mvp_state_t *state)
{
    uint64_t now_ms;

    if (state == NULL) {
        return;
    }

    now_ms = uapi_tcxo_get_ms();
    state->uptime_ms = (uint32_t)now_ms;
    state->motor_ready = g_motor_ready;
    state->moving = g_moving;
    state->last_command = g_last_command;
    state->last_sequence = g_last_sequence;
    state->command_age_ms = g_received_command ? (uint32_t)(now_ms - g_last_command_ms) : 0xFFFFFFFFU;
}

bool robot_mvp_control_get_monitor(robot_mvp_monitor_t *monitor)
{
    uint64_t now_ms;

    if (monitor == NULL) {
        return false;
    }

    now_ms = uapi_tcxo_get_ms();
    if (g_env_sample_ms == 0U) {
        (void)sample_env(now_ms);
    }
    if (g_obstacle_sample_ms == 0U) {
        (void)sample_obstacle_into(NULL, now_ms);
    }

    robot_mvp_control_get_state(&monitor->state);
    monitor->env = g_env;
    monitor->obstacle = g_obstacle;
    monitor->alarm_flags = monitor_alarm_flags();
    monitor->sample_count = g_monitor_sample_count;
    monitor->env_age_ms = monitor_age_ms(now_ms, g_env_sample_ms);
    monitor->obstacle_age_ms = monitor_age_ms(now_ms, g_obstacle_sample_ms);
    return (monitor->env.valid && monitor->obstacle.enabled && monitor->obstacle.valid);
}

static at_ret_t robot_at_command(uint8_t command)
{
    uint8_t sequence = 0U;
    bool moving = false;
    robot_status_t status = robot_mvp_control_motion(
        (robot_command_code_t)command,
        ROBOT_AT_SAFE_SPEED,
        &sequence,
        &moving
    );

    uapi_at_printf(
        "+ROBOT:ACK,%u,%u,%u,%u\r\n",
        sequence,
        command,
        status,
        moving ? 1U : 0U
    );
    osal_printk(
        "\r\nROBOT AT ACK seq=%u cmd=%u status=%u moving=%u\r\n",
        sequence,
        command,
        status,
        moving ? 1U : 0U
    );
    return AT_RET_OK;
}

static at_ret_t robot_at_forward(void)
{
    return robot_at_command(ROBOT_CMD_FORWARD);
}

static at_ret_t robot_at_backward(void)
{
    return robot_at_command(ROBOT_CMD_BACKWARD);
}

static at_ret_t robot_at_left(void)
{
    return robot_at_command(ROBOT_CMD_LEFT);
}

static at_ret_t robot_at_right(void)
{
    return robot_at_command(ROBOT_CMD_RIGHT);
}

static at_ret_t robot_at_stop(void)
{
    return robot_at_command(ROBOT_CMD_STOP);
}

static at_ret_t robot_at_motor_init(void)
{
    bool ready = false;

    (void)robot_mvp_control_motor_init(&ready);

    uapi_at_printf("+ROBOT:MOTOR,%u\r\n", ready ? 1U : 0U);
    osal_printk("\r\nROBOT MOTOR init ready=%u\r\n", ready ? 1U : 0U);
    return AT_RET_OK;
}

static at_ret_t robot_at_oled_init(void)
{
    bool ready = false;

    (void)robot_mvp_control_oled_init(&ready);

    uapi_at_printf("+ROBOT:OLED,%u\r\n", ready ? 1U : 0U);
    osal_printk("\r\nROBOT OLED init ready=%u\r\n", ready ? 1U : 0U);
    return AT_RET_OK;
}

static at_ret_t robot_at_env(void)
{
    robot_env_data_t env = {0};
    bool ok = robot_mvp_control_read_env(&env);

    uapi_at_printf(
        "+ROBOT:ENV,%u,%d,%u\r\n",
        ok ? 1U : 0U,
        env.temperature_deci_c,
        env.humidity_deci_percent
    );
    osal_printk(
        "\r\nROBOT ENV ok=%u temp_deci=%d hum_deci=%u\r\n",
        ok ? 1U : 0U,
        env.temperature_deci_c,
        env.humidity_deci_percent
    );
    return AT_RET_OK;
}

static at_ret_t robot_at_obstacle(void)
{
    robot_obstacle_data_t obstacle = {0};
    bool ok = robot_mvp_control_read_obstacle(&obstacle);

    uapi_at_printf(
        "+ROBOT:OBS,%u,%u,%u,%u,%u,%u\r\n",
        obstacle.enabled ? 1U : 0U,
        ok ? 1U : 0U,
        obstacle.blocked ? 1U : 0U,
        obstacle.distance_mm,
        obstacle.threshold_mm,
        obstacle.reason
    );
    osal_printk(
        "\r\nROBOT OBS enabled=%u valid=%u blocked=%u distance=%u threshold=%u reason=%u\r\n",
        obstacle.enabled ? 1U : 0U,
        ok ? 1U : 0U,
        obstacle.blocked ? 1U : 0U,
        obstacle.distance_mm,
        obstacle.threshold_mm,
        obstacle.reason
    );
    return AT_RET_OK;
}

static at_ret_t robot_at_avoid(void)
{
    robot_mvp_avoid_result_t result = {0};

    (void)robot_mvp_control_start_avoid(&result);

    uapi_at_printf(
        "+ROBOT:AVOID,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
        result.active ? 1U : 0U,
        result.phase,
        result.status,
        result.obstacle.enabled ? 1U : 0U,
        result.obstacle.valid ? 1U : 0U,
        result.obstacle.blocked ? 1U : 0U,
        result.obstacle.distance_mm,
        result.obstacle.threshold_mm,
        result.obstacle.reason
    );
    osal_printk(
        "\r\nROBOT AVOID active=%u phase=%u status=%u distance=%u threshold=%u reason=%u\r\n",
        result.active ? 1U : 0U,
        result.phase,
        result.status,
        result.obstacle.distance_mm,
        result.obstacle.threshold_mm,
        result.obstacle.reason
    );
    return AT_RET_OK;
}

static at_ret_t robot_at_avoid_test(void)
{
    robot_mvp_avoid_result_t result = {0};

    (void)robot_mvp_control_start_avoid_test(&result);

    uapi_at_printf(
        "+ROBOT:AVOIDTEST,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
        result.active ? 1U : 0U,
        result.phase,
        result.status,
        result.obstacle.enabled ? 1U : 0U,
        result.obstacle.valid ? 1U : 0U,
        result.obstacle.blocked ? 1U : 0U,
        result.obstacle.distance_mm,
        result.obstacle.threshold_mm,
        result.test_duration_ms
    );
    osal_printk(
        "\r\nROBOT AVOID TEST active=%u phase=%u status=%u distance=%u threshold=%u duration=%u\r\n",
        result.active ? 1U : 0U,
        result.phase,
        result.status,
        result.obstacle.distance_mm,
        result.obstacle.threshold_mm,
        result.test_duration_ms
    );
    return AT_RET_OK;
}

static void robot_at_report_patrol(const robot_mvp_patrol_result_t *result)
{
    if (result == NULL) {
        return;
    }

    uapi_at_printf(
        "+ROBOT:PATROL,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
        result->active ? 1U : 0U,
        result->phase,
        result->status,
        result->leg_index,
        result->loop_count,
        result->alarm_flags,
        result->obstacle.enabled ? 1U : 0U,
        result->obstacle.valid ? 1U : 0U,
        result->obstacle.blocked ? 1U : 0U,
        result->obstacle.distance_mm,
        result->obstacle.threshold_mm,
        result->obstacle.reason
    );
    osal_printk(
        "\r\nROBOT PATROL active=%u phase=%u status=%u leg=%u loop=%u alarm=0x%x distance=%u threshold=%u reason=%u\r\n",
        result->active ? 1U : 0U,
        result->phase,
        result->status,
        result->leg_index,
        result->loop_count,
        result->alarm_flags,
        result->obstacle.distance_mm,
        result->obstacle.threshold_mm,
        result->obstacle.reason
    );
}

static at_ret_t robot_at_patrol(void)
{
    robot_mvp_patrol_result_t result = {0};

    (void)robot_mvp_control_start_patrol(&result);
    robot_at_report_patrol(&result);
    return AT_RET_OK;
}

static at_ret_t robot_at_patrol_status(void)
{
    robot_mvp_patrol_result_t result = {0};

    robot_mvp_control_get_patrol(&result);
    robot_at_report_patrol(&result);
    return AT_RET_OK;
}

static at_ret_t robot_at_status(void)
{
    robot_mvp_state_t state = {0};

    robot_mvp_control_get_state(&state);

    uapi_at_printf(
        "+ROBOT:STATE,%u,%u,%u,%u,%u,%u\r\n",
        state.uptime_ms,
        state.motor_ready ? 1U : 0U,
        state.moving ? 1U : 0U,
        state.last_command,
        state.last_sequence,
        state.command_age_ms
    );
    osal_printk(
        "\r\nROBOT STATE uptime=%u ready=%u moving=%u last_cmd=%u seq=%u age=%u\r\n",
        state.uptime_ms,
        state.motor_ready ? 1U : 0U,
        state.moving ? 1U : 0U,
        state.last_command,
        state.last_sequence,
        state.command_age_ms
    );
    refresh_oled(state.uptime_ms);
    return AT_RET_OK;
}

static at_ret_t robot_at_monitor(void)
{
    robot_mvp_monitor_t monitor = {0};

    (void)robot_mvp_control_get_monitor(&monitor);

    uapi_at_printf(
        "+ROBOT:MON,%u,%u,%u,%u,%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
        monitor.state.uptime_ms,
        monitor.state.motor_ready ? 1U : 0U,
        monitor.state.moving ? 1U : 0U,
        monitor.env.valid ? 1U : 0U,
        monitor.env.temperature_deci_c,
        monitor.env.humidity_deci_percent,
        monitor.obstacle.enabled ? 1U : 0U,
        monitor.obstacle.valid ? 1U : 0U,
        monitor.obstacle.blocked ? 1U : 0U,
        monitor.obstacle.distance_mm,
        monitor.obstacle.threshold_mm,
        monitor.obstacle.reason,
        monitor.alarm_flags,
        monitor.sample_count,
        monitor.env_age_ms,
        monitor.obstacle_age_ms
    );
    osal_printk(
        "\r\nROBOT MON uptime=%u ready=%u moving=%u env=%u temp=%d hum=%u obs=%u valid=%u blocked=%u distance=%u threshold=%u reason=%u alarm=0x%x samples=%u env_age=%u obs_age=%u\r\n",
        monitor.state.uptime_ms,
        monitor.state.motor_ready ? 1U : 0U,
        monitor.state.moving ? 1U : 0U,
        monitor.env.valid ? 1U : 0U,
        monitor.env.temperature_deci_c,
        monitor.env.humidity_deci_percent,
        monitor.obstacle.enabled ? 1U : 0U,
        monitor.obstacle.valid ? 1U : 0U,
        monitor.obstacle.blocked ? 1U : 0U,
        monitor.obstacle.distance_mm,
        monitor.obstacle.threshold_mm,
        monitor.obstacle.reason,
        monitor.alarm_flags,
        monitor.sample_count,
        monitor.env_age_ms,
        monitor.obstacle_age_ms
    );
    return AT_RET_OK;
}

static at_ret_t robot_at_buzzer_test(void)
{
    bool ok = robot_buzzer_test();
    bool led_ok = robot_buzzer_alarm_led_ready();

    uapi_at_printf("+ROBOT:BEEP,%u,%u\r\n", ok ? 1U : 0U, led_ok ? 1U : 0U);
    osal_printk("\r\nROBOT BEEP buzzer=%u led=%u\r\n", ok ? 1U : 0U, led_ok ? 1U : 0U);
    return AT_RET_OK;
}

void robot_at_cmd_register(void)
{
    static const at_cmd_entry_t robot_at_table[] = {
        {"ROBOTF", 0x7101, 0, NULL, robot_at_forward, NULL, NULL, NULL},
        {"ROBOTB", 0x7102, 0, NULL, robot_at_backward, NULL, NULL, NULL},
        {"ROBOTL", 0x7103, 0, NULL, robot_at_left, NULL, NULL, NULL},
        {"ROBOTR", 0x7104, 0, NULL, robot_at_right, NULL, NULL, NULL},
        {"ROBOTS", 0x7105, 0, NULL, robot_at_stop, NULL, NULL, NULL},
        {"ROBOTMI", 0x7106, 0, NULL, robot_at_motor_init, NULL, NULL, NULL},
        {"ROBOTST", 0x7107, 0, NULL, robot_at_status, NULL, NULL, NULL},
        {"ROBOTOLED", 0x7108, 0, NULL, robot_at_oled_init, NULL, NULL, NULL},
        {"ROBOTENV", 0x7109, 0, NULL, robot_at_env, NULL, NULL, NULL},
        {"ROBOTOBS", 0x710A, 0, NULL, robot_at_obstacle, NULL, NULL, NULL},
        {"ROBOTAVOID", 0x710B, 0, NULL, robot_at_avoid, NULL, NULL, NULL},
        {"ROBOTAVOIDTEST", 0x710C, 0, NULL, robot_at_avoid_test, NULL, NULL, NULL},
        {"ROBOTMON", 0x710D, 0, NULL, robot_at_monitor, NULL, NULL, NULL},
        {"ROBOTBEEP", 0x710E, 0, NULL, robot_at_buzzer_test, NULL, NULL, NULL},
        {"ROBOTPATROL", 0x710F, 0, NULL, robot_at_patrol, NULL, NULL, NULL},
        {"ROBOTPATROLST", 0x7110, 0, NULL, robot_at_patrol_status, NULL, NULL, NULL},
    };

    uapi_at_report("ROBOT AT register enter\r\n");
    errcode_t ret = uapi_at_cmd_table_register(
        robot_at_table,
        sizeof(robot_at_table) / sizeof(robot_at_table[0]),
        0
    );
    if (ret == ERRCODE_SUCC) {
        uapi_at_report("ROBOT AT register ok\r\n");
    } else {
        uapi_at_report("ROBOT AT register fail\r\n");
    }
    osal_printk("\r\nROBOT AT register ret=0x%x\r\n", ret);
}

#if ROBOT_ENABLE_RAW_UART
static void read_serial(frame_reader_t *reader)
{
    uint8_t byte = 0U;
    if (uapi_uart_read(UART_BUS_0, &byte, 1U, 0U) != 1) {
        return;
    }

    if (reader->index == 0U) {
        if (byte == ROBOT_COMMAND_HEADER) {
            reader->frame[reader->index++] = byte;
        }
        return;
    }

    reader->frame[reader->index++] = byte;
    if (reader->index == ROBOT_COMMAND_SIZE) {
        process_frame(reader->frame);
        reader->index = 0U;
    }
}
#endif

static void check_watchdog(void)
{
    uint64_t now_ms;
    if (!g_received_command || !g_moving) {
        return;
    }
    if (g_avoid_active || g_patrol_active) {
        return;
    }
    if (g_last_command == ROBOT_CMD_FORWARD) {
        (void)sample_obstacle_into(NULL, uapi_tcxo_get_ms());
        if (g_obstacle.enabled && g_obstacle.valid && g_obstacle.blocked) {
            stop_motion();
            osal_printk("\r\nROBOT OBSTACLE STOP distance=%u threshold=%u\r\n",
                g_obstacle.distance_mm, g_obstacle.threshold_mm);
            return;
        }
    }
    now_ms = uapi_tcxo_get_ms();
    if ((now_ms - g_last_command_ms) >= ROBOT_CONTROL_TIMEOUT_MS) {
        stop_motion();
        osal_printk("\r\nROBOT WATCHDOG STOP\r\n");
    }
}

static void *robot_task(const char *argument)
{
#if ROBOT_ENABLE_RAW_UART
    frame_reader_t reader = {0};
#endif
    unused(argument);
    osal_printk("\r\nROBOT task enter\r\n");

    /*
     * Bring up the MVP protocol before touching the external PWM/I2C motor board.
     * Some robot car boards hold or reset the PWM STM8 during early boot; blocking
     * here would make the serial control MVP look dead. Motor enable is reintroduced
     * after the COM5 ACK loop is proven on the real board.
     */
#ifdef CONFIG_ROBOT_MVP_ENABLE_WAVE_ROVER_LINK
    g_motor_ready = motor_backend_init();
#else
    g_motor_ready = false;
#endif
    g_moving = false;
    g_received_command = false;
    g_avoid_active = false;
    g_avoid_test_active = false;
    g_patrol_active = false;
    g_avoid_phase = ROBOT_AVOID_IDLE;
    reset_patrol_state(ROBOT_PATROL_IDLE);
    g_patrol_status = ROBOT_STATUS_OK;
    g_next_env_monitor_ms = 0U;
    g_next_obstacle_monitor_ms = 0U;
    g_env_sample_ms = 0U;
    g_obstacle_sample_ms = 0U;
    g_last_monitor_report_ms = 0U;
    g_monitor_sample_count = 0U;
    g_last_monitor_report_flags = ROBOT_MONITOR_ALARM_NONE;
    g_monitor_reported_once = false;
    g_env.valid = false;
    g_env.temperature_deci_c = 0;
    g_env.humidity_deci_percent = 0U;
    g_obstacle.enabled = false;
    g_obstacle.valid = false;
    g_obstacle.blocked = false;
    g_obstacle.distance_mm = 0U;
    g_obstacle.threshold_mm = robot_obstacle_threshold_mm();
    g_obstacle.reason = ROBOT_OBSTACLE_REASON_NOT_READY;
    (void)robot_buzzer_init();
#ifdef CONFIG_ROBOT_MVP_ENABLE_SLE
    (void)robot_sle_server_start();
#endif
    osal_printk(
        "\r\nROBOT_MVP READY protocol=at/sle uart=0 baud=115200 motor=SKIPPED obstacle=%u\r\n",
        g_obstacle.enabled ? 1U : 0U
    );
    (void)robot_obstacle_init();

    while (1) {
#if ROBOT_ENABLE_RAW_UART
        read_serial(&reader);
#endif
        update_avoidance(uapi_tcxo_get_ms());
        update_patrol(uapi_tcxo_get_ms());
        update_motion_keepalive(uapi_tcxo_get_ms());
        update_monitoring(uapi_tcxo_get_ms());
        check_watchdog();
        robot_buzzer_update(monitor_alarm_flags(), uapi_tcxo_get_ms());
        osal_msleep(ROBOT_LOOP_DELAY_MS);
    }
    return NULL;
}

void robot_mvp_entry(void)
{
    osal_task *task = NULL;
    osal_printk("\r\nROBOT entry enter\r\n");
    uapi_at_report("ROBOT entry enter\r\n");
    osal_kthread_lock();
    if (g_robot_task_started) {
        osal_kthread_unlock();
        osal_printk("\r\nROBOT entry already started\r\n");
        uapi_at_report("ROBOT entry already started\r\n");
        return;
    }
    task = osal_kthread_create(
        (osal_kthread_handler)robot_task,
        NULL,
        "RobotMvp",
        ROBOT_TASK_STACK_SIZE
    );
    if (task != NULL) {
        (void)osal_kthread_set_priority(task, ROBOT_TASK_PRIORITY);
        g_robot_task_started = true;
        osal_printk("\r\nROBOT entry create ok\r\n");
        uapi_at_report("ROBOT entry create ok\r\n");
    } else {
        osal_printk("\r\nROBOT entry create fail\r\n");
        uapi_at_report("ROBOT entry create fail\r\n");
    }
    osal_kthread_unlock();
}

app_run(robot_mvp_entry);
