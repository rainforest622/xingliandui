#include "app_init.h"
#include "at.h"
#include "common_def.h"
#include "dut_motor.h"
#include "robot_env.h"
#include "robot_mvp_control.h"
#include "robot_obstacle.h"
#include "robot_oled.h"
#include "robot_protocol.h"
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
#define ROBOT_AT_SAFE_SPEED 20
#define ROBOT_AVOID_SPEED 18
#define ROBOT_AVOID_SAMPLE_MS 90U
#define ROBOT_AVOID_BACKWARD_MS 260U
#define ROBOT_AVOID_TURN_MS 420U
#define ROBOT_AVOID_TEST_DURATION_MS 1800U
#define ROBOT_AVOID_TEST_BLOCK_MS 120U
#define ROBOT_AVOID_TEST_BLOCK_DISTANCE_MM 120U
#define ROBOT_AVOID_TEST_CLEAR_DISTANCE_MM 450U
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

static bool g_motor_ready;
static bool g_moving;
static bool g_received_command;
static bool g_avoid_active;
static bool g_avoid_test_active;
static uint64_t g_last_command_ms;
static uint64_t g_avoid_phase_until_ms;
static uint64_t g_next_obstacle_sample_ms;
static uint64_t g_avoid_test_until_ms;
static uint64_t g_avoid_test_block_until_ms;
static uint8_t g_at_sequence;
static uint8_t g_last_command;
static uint8_t g_last_sequence;
static robot_avoid_phase_t g_avoid_phase;
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

#if ROBOT_ENABLE_RAW_UART
static void send_ack(uint8_t sequence, robot_status_t status)
{
    uint8_t ack[ROBOT_ACK_SIZE];
    robot_ack_encode(sequence, status, ack);
    (void)uapi_uart_write(UART_BUS_0, ack, sizeof(ack), 0U);
}
#endif

static void stop_motion(void)
{
    if (g_motor_ready) {
        (void)dut_motor_stop();
    }
    g_moving = false;
}

static bool drive_motion(int8_t left, int8_t right)
{
    if (!g_motor_ready) {
        g_moving = false;
        return false;
    }

    if (!dut_motor_apply(left, right)) {
        g_moving = false;
        return false;
    }
    g_moving = (left != 0) || (right != 0);
    return true;
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

static bool read_avoidance_obstacle(robot_obstacle_data_t *output, uint64_t now_ms)
{
    if (g_avoid_test_active) {
        output->enabled = true;
        output->valid = true;
        output->threshold_mm = robot_obstacle_threshold_mm();
        output->distance_mm = (now_ms < g_avoid_test_block_until_ms) ?
            ROBOT_AVOID_TEST_BLOCK_DISTANCE_MM : ROBOT_AVOID_TEST_CLEAR_DISTANCE_MM;
        output->blocked = output->distance_mm < output->threshold_mm;
        return true;
    }
    return robot_obstacle_read(output);
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

static robot_status_t apply_command(const robot_command_t *command)
{
    int8_t left = (int8_t)command->speed_left;
    int8_t right = (int8_t)command->speed_right;

    g_avoid_active = false;
    g_avoid_test_active = false;
    g_avoid_phase = ROBOT_AVOID_IDLE;
    g_avoid_phase_until_ms = 0U;
    g_next_obstacle_sample_ms = 0U;
    g_avoid_test_until_ms = 0U;
    g_avoid_test_block_until_ms = 0U;

    if (command->command == ROBOT_CMD_STOP) {
        stop_motion();
        return ROBOT_STATUS_OK;
    }

    if (!g_motor_ready) {
        g_moving = false;
        return ROBOT_STATUS_MOTOR_ERROR;
    }

    if (command->command == ROBOT_CMD_FORWARD) {
        (void)robot_obstacle_read(&g_obstacle);
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
        (void)dut_motor_stop();
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

    g_avoid_active = false;
    g_avoid_test_active = false;
    g_avoid_phase = ROBOT_AVOID_IDLE;
    g_avoid_phase_until_ms = 0U;
    g_next_obstacle_sample_ms = 0U;
    g_avoid_test_until_ms = 0U;
    g_avoid_test_block_until_ms = 0U;

    if (!g_motor_ready) {
        motor_ready = dut_motor_init();
    } else {
        motor_ready = dut_motor_stop();
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
    bool ok = robot_env_read(&g_env);

    refresh_oled(uapi_tcxo_get_ms());
    if (env != NULL) {
        *env = g_env;
    }
    return ok;
}

bool robot_mvp_control_read_obstacle(robot_obstacle_data_t *obstacle)
{
    bool ok = robot_obstacle_read(&g_obstacle);

    refresh_oled(uapi_tcxo_get_ms());
    if (obstacle != NULL) {
        *obstacle = g_obstacle;
    }
    return ok;
}

robot_status_t robot_mvp_control_start_avoid(robot_mvp_avoid_result_t *result)
{
    uint64_t now_ms = uapi_tcxo_get_ms();
    robot_status_t status = ROBOT_STATUS_OK;
    bool sampled = false;

    g_avoid_test_active = false;
    g_avoid_test_until_ms = 0U;
    g_avoid_test_block_until_ms = 0U;

    if (!g_motor_ready) {
        disable_avoidance(ROBOT_AVOID_IDLE);
        status = ROBOT_STATUS_MOTOR_ERROR;
    } else {
        sampled = robot_obstacle_read(&g_obstacle);
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

    uapi_at_print(
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

    uapi_at_print("+ROBOT:MOTOR,%u\r\n", ready ? 1U : 0U);
    osal_printk("\r\nROBOT MOTOR init ready=%u\r\n", ready ? 1U : 0U);
    return AT_RET_OK;
}

static at_ret_t robot_at_oled_init(void)
{
    bool ready = false;

    (void)robot_mvp_control_oled_init(&ready);

    uapi_at_print("+ROBOT:OLED,%u\r\n", ready ? 1U : 0U);
    osal_printk("\r\nROBOT OLED init ready=%u\r\n", ready ? 1U : 0U);
    return AT_RET_OK;
}

static at_ret_t robot_at_env(void)
{
    robot_env_data_t env = {0};
    bool ok = robot_mvp_control_read_env(&env);

    uapi_at_print(
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

    uapi_at_print(
        "+ROBOT:OBS,%u,%u,%u,%u,%u\r\n",
        obstacle.enabled ? 1U : 0U,
        ok ? 1U : 0U,
        obstacle.blocked ? 1U : 0U,
        obstacle.distance_mm,
        obstacle.threshold_mm
    );
    osal_printk(
        "\r\nROBOT OBS enabled=%u valid=%u blocked=%u distance=%u threshold=%u\r\n",
        obstacle.enabled ? 1U : 0U,
        ok ? 1U : 0U,
        obstacle.blocked ? 1U : 0U,
        obstacle.distance_mm,
        obstacle.threshold_mm
    );
    return AT_RET_OK;
}

static at_ret_t robot_at_avoid(void)
{
    robot_mvp_avoid_result_t result = {0};

    (void)robot_mvp_control_start_avoid(&result);

    uapi_at_print(
        "+ROBOT:AVOID,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
        result.active ? 1U : 0U,
        result.phase,
        result.status,
        result.obstacle.enabled ? 1U : 0U,
        result.obstacle.valid ? 1U : 0U,
        result.obstacle.blocked ? 1U : 0U,
        result.obstacle.distance_mm,
        result.obstacle.threshold_mm
    );
    osal_printk(
        "\r\nROBOT AVOID active=%u phase=%u status=%u distance=%u threshold=%u\r\n",
        result.active ? 1U : 0U,
        result.phase,
        result.status,
        result.obstacle.distance_mm,
        result.obstacle.threshold_mm
    );
    return AT_RET_OK;
}

static at_ret_t robot_at_avoid_test(void)
{
    robot_mvp_avoid_result_t result = {0};

    (void)robot_mvp_control_start_avoid_test(&result);

    uapi_at_print(
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

static at_ret_t robot_at_status(void)
{
    robot_mvp_state_t state = {0};

    robot_mvp_control_get_state(&state);

    uapi_at_print(
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

void at_custom_cmd_register(void)
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
    };

    errcode_t ret = uapi_at_cmd_table_register(
        robot_at_table,
        sizeof(robot_at_table) / sizeof(robot_at_table[0]),
        0
    );
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
    if (g_avoid_active) {
        return;
    }
    if (g_last_command == ROBOT_CMD_FORWARD) {
        (void)robot_obstacle_read(&g_obstacle);
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

    /*
     * Bring up the MVP protocol before touching the external PWM/I2C motor board.
     * Some DUT car boards hold or reset the PWM STM8 during early boot; blocking
     * here would make the serial control MVP look dead. Motor enable is reintroduced
     * after the COM5 ACK loop is proven on the real board.
     */
    g_motor_ready = false;
    g_moving = false;
    g_received_command = false;
    g_avoid_active = false;
    g_avoid_test_active = false;
    g_avoid_phase = ROBOT_AVOID_IDLE;
    (void)robot_obstacle_init();
    (void)robot_obstacle_read(&g_obstacle);
#ifdef CONFIG_ROBOT_MVP_ENABLE_SLE
    (void)robot_sle_server_start();
#endif
    osal_printk(
        "\r\nROBOT_MVP READY protocol=at/sle uart=0 baud=115200 motor=SKIPPED obstacle=%u\r\n",
        g_obstacle.enabled ? 1U : 0U
    );

    while (1) {
#if ROBOT_ENABLE_RAW_UART
        read_serial(&reader);
#endif
        update_avoidance(uapi_tcxo_get_ms());
        check_watchdog();
        osal_msleep(ROBOT_LOOP_DELAY_MS);
    }
    return NULL;
}

void robot_mvp_entry(void)
{
    osal_task *task = NULL;
    osal_kthread_lock();
    task = osal_kthread_create(
        (osal_kthread_handler)robot_task,
        NULL,
        "RobotMvp",
        ROBOT_TASK_STACK_SIZE
    );
    if (task != NULL) {
        (void)osal_kthread_set_priority(task, ROBOT_TASK_PRIORITY);
    }
    osal_kthread_unlock();
}
