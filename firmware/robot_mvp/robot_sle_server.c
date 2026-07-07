#include "robot_sle_server.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "common_def.h"
#include "errcode.h"
#include "robot_mvp_control.h"
#include "securec.h"
#include "sle_common.h"
#include "sle_connection_manager.h"
#include "sle_device_discovery.h"
#include "sle_errcode.h"
#include "sle_ssap_server.h"
#include "soc_osal.h"

#define ROBOT_SLE_TASK_STACK_SIZE 0x2000U
#define ROBOT_SLE_TASK_PRIORITY 24U
#define ROBOT_SLE_ADV_HANDLE 1U
#define ROBOT_SLE_SERVICE_UUID 0x7100U
#define ROBOT_SLE_COMMAND_UUID 0x7101U
#define ROBOT_SLE_RESPONSE_UUID 0x7102U
#define ROBOT_SLE_SAFE_SPEED 20U
#define ROBOT_SLE_RESPONSE_MAX_LEN 192U
#define ROBOT_SLE_MTU_SIZE 256U
#define ROBOT_SLE_CONN_INTERVAL 0x14U
#define ROBOT_SLE_CONN_TIMEOUT 0x1F4U
#define ROBOT_SLE_EVENT_COMMAND 0x01U

#define ROBOT_SLE_ADV_INTERVAL 0xC8U
#define ROBOT_SLE_ADV_TX_POWER 20
#define ROBOT_SLE_ADV_TYPE_DISCOVERY_LEVEL 0x01U
#define ROBOT_SLE_ADV_TYPE_COMPLETE_LIST_OF_16BIT_SERVICE_UUIDS 0x05U
#define ROBOT_SLE_ADV_TYPE_COMPLETE_LOCAL_NAME 0x0BU
#define ROBOT_SLE_ADV_TYPE_TX_POWER_LEVEL 0x0CU

static const uint8_t g_robot_sle_name[] = "ws63_robot_mvp";
static const uint8_t g_robot_sle_uuid_base[SLE_UUID_LEN] = {
    0x37, 0xBE, 0xA8, 0x80, 0xFC, 0x70, 0x11, 0xEA,
    0xB7, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

static osal_event g_robot_sle_event;
static volatile bool g_robot_sle_event_ready;
static volatile bool g_robot_sle_pending;
static uint8_t g_robot_sle_pending_key;
static uint16_t g_robot_sle_pending_conn_id;
static uint16_t g_robot_sle_conn_id;
static uint8_t g_robot_sle_server_id;
static uint16_t g_robot_sle_service_handle;
static uint16_t g_robot_sle_command_handle;
static uint16_t g_robot_sle_response_handle;
static char g_robot_sle_last_response[ROBOT_SLE_RESPONSE_MAX_LEN] = "+ROBOT:SLE,BOOT\r\n";

static void robot_sle_uuid_set(sle_uuid_t *out, uint16_t short_uuid)
{
    if (out == NULL) {
        return;
    }
    (void)memcpy_s(out->uuid, SLE_UUID_LEN, g_robot_sle_uuid_base, SLE_UUID_LEN);
    out->len = 2U;
    out->uuid[14] = (uint8_t)(short_uuid & 0xFFU);
    out->uuid[15] = (uint8_t)((short_uuid >> 8) & 0xFFU);
}

static void robot_sle_save_response(const char *response)
{
    if (response == NULL) {
        return;
    }
    (void)strncpy_s(g_robot_sle_last_response, sizeof(g_robot_sle_last_response),
        response, sizeof(g_robot_sle_last_response) - 1U);
}

static errcode_t robot_sle_notify_text(uint16_t conn_id, const char *response)
{
    ssaps_ntf_ind_t param = {0};
    uint16_t len;

    if ((response == NULL) || (g_robot_sle_response_handle == 0U)) {
        return ERRCODE_SLE_FAIL;
    }

    robot_sle_save_response(response);
    len = (uint16_t)strnlen(response, ROBOT_SLE_RESPONSE_MAX_LEN);
    param.handle = g_robot_sle_response_handle;
    param.type = SSAP_PROPERTY_TYPE_VALUE;
    param.value = (uint8_t *)response;
    param.value_len = len;
    return ssaps_notify_indicate(g_robot_sle_server_id, conn_id, &param);
}

static void robot_sle_send_read_response(uint16_t conn_id, ssaps_req_read_cb_t *read_cb)
{
    ssaps_send_rsp_t rsp = {0};

    if ((read_cb == NULL) || !read_cb->need_rsp) {
        return;
    }

    rsp.request_id = read_cb->request_id;
    rsp.status = ERRCODE_SLE_SUCCESS;
    rsp.value = (uint8_t *)g_robot_sle_last_response;
    rsp.value_len = (uint16_t)strnlen(g_robot_sle_last_response, ROBOT_SLE_RESPONSE_MAX_LEN);
    (void)ssaps_send_response(g_robot_sle_server_id, conn_id, &rsp);
}

static void robot_sle_send_write_response(uint16_t conn_id, ssaps_req_write_cb_t *write_cb, errcode_t status)
{
    ssaps_send_rsp_t rsp = {0};

    if ((write_cb == NULL) || !write_cb->need_rsp) {
        return;
    }

    rsp.request_id = write_cb->request_id;
    rsp.status = (uint8_t)status;
    rsp.value = NULL;
    rsp.value_len = 0U;
    (void)ssaps_send_response(g_robot_sle_server_id, conn_id, &rsp);
}

static bool robot_sle_command_from_key(uint8_t key, robot_command_code_t *command)
{
    if (command == NULL) {
        return false;
    }

    switch (key) {
        case 'F':
            *command = ROBOT_CMD_FORWARD;
            return true;
        case 'B':
            *command = ROBOT_CMD_BACKWARD;
            return true;
        case 'L':
            *command = ROBOT_CMD_LEFT;
            return true;
        case 'R':
            *command = ROBOT_CMD_RIGHT;
            return true;
        case 'S':
            *command = ROBOT_CMD_STOP;
            return true;
        default:
            return false;
    }
}

static bool robot_sle_is_supported_key(uint8_t key)
{
    robot_command_code_t command;

    if ((key >= 'a') && (key <= 'z')) {
        key = (uint8_t)(key - 'a' + 'A');
    }
    if (robot_sle_command_from_key(key, &command)) {
        return true;
    }

    switch (key) {
        case 'A':
        case 'D':
        case 'E':
        case 'I':
        case 'O':
        case 'T':
        case 'X':
            return true;
        default:
            return false;
    }
}

static int robot_sle_format_avoid_response(char *buffer, size_t size, const char *name,
    const robot_mvp_avoid_result_t *result)
{
    if ((buffer == NULL) || (name == NULL) || (result == NULL)) {
        return -1;
    }

    if (result->test_duration_ms > 0U) {
        return snprintf(buffer, size,
            "+ROBOT:%s,%u,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
            name,
            result->active ? 1U : 0U,
            result->phase,
            (uint8_t)result->status,
            result->obstacle.enabled ? 1U : 0U,
            result->obstacle.valid ? 1U : 0U,
            result->obstacle.blocked ? 1U : 0U,
            result->obstacle.distance_mm,
            result->obstacle.threshold_mm,
            result->test_duration_ms);
    }

    return snprintf(buffer, size,
        "+ROBOT:%s,%u,%u,%u,%u,%u,%u,%u,%u\r\n",
        name,
        result->active ? 1U : 0U,
        result->phase,
        (uint8_t)result->status,
        result->obstacle.enabled ? 1U : 0U,
        result->obstacle.valid ? 1U : 0U,
        result->obstacle.blocked ? 1U : 0U,
        result->obstacle.distance_mm,
        result->obstacle.threshold_mm);
}

static void robot_sle_execute_key(uint8_t key, char *response, size_t response_size)
{
    robot_command_code_t motion_command;
    uint8_t sequence = 0U;
    bool moving = false;

    if ((response == NULL) || (response_size == 0U)) {
        return;
    }

    if ((key >= 'a') && (key <= 'z')) {
        key = (uint8_t)(key - 'a' + 'A');
    }

    if (robot_sle_command_from_key(key, &motion_command)) {
        robot_status_t status = robot_mvp_control_motion(
            motion_command,
            ROBOT_SLE_SAFE_SPEED,
            &sequence,
            &moving
        );
        (void)snprintf(response, response_size,
            "+ROBOT:ACK,%u,%u,%u,%u\r\n",
            sequence, (uint8_t)motion_command, (uint8_t)status, moving ? 1U : 0U);
        return;
    }

    switch (key) {
        case 'I': {
            bool ready = false;
            (void)robot_mvp_control_motor_init(&ready);
            (void)snprintf(response, response_size, "+ROBOT:MOTOR,%u\r\n", ready ? 1U : 0U);
            return;
        }
        case 'O': {
            bool ready = false;
            (void)robot_mvp_control_oled_init(&ready);
            (void)snprintf(response, response_size, "+ROBOT:OLED,%u\r\n", ready ? 1U : 0U);
            return;
        }
        case 'E': {
            robot_env_data_t env = {0};
            bool ok = robot_mvp_control_read_env(&env);
            (void)snprintf(response, response_size, "+ROBOT:ENV,%u,%d,%u\r\n",
                ok ? 1U : 0U, env.temperature_deci_c, env.humidity_deci_percent);
            return;
        }
        case 'D': {
            robot_obstacle_data_t obstacle = {0};
            bool ok = robot_mvp_control_read_obstacle(&obstacle);
            (void)snprintf(response, response_size, "+ROBOT:OBS,%u,%u,%u,%u,%u\r\n",
                obstacle.enabled ? 1U : 0U,
                ok ? 1U : 0U,
                obstacle.blocked ? 1U : 0U,
                obstacle.distance_mm,
                obstacle.threshold_mm);
            return;
        }
        case 'A': {
            robot_mvp_avoid_result_t result = {0};
            (void)robot_mvp_control_start_avoid(&result);
            (void)robot_sle_format_avoid_response(response, response_size, "AVOID", &result);
            return;
        }
        case 'X': {
            robot_mvp_avoid_result_t result = {0};
            (void)robot_mvp_control_start_avoid_test(&result);
            (void)robot_sle_format_avoid_response(response, response_size, "AVOIDTEST", &result);
            return;
        }
        case 'T': {
            robot_mvp_state_t state = {0};
            robot_mvp_control_get_state(&state);
            (void)snprintf(response, response_size, "+ROBOT:STATE,%u,%u,%u,%u,%u,%u\r\n",
                state.uptime_ms,
                state.motor_ready ? 1U : 0U,
                state.moving ? 1U : 0U,
                state.last_command,
                state.last_sequence,
                state.command_age_ms);
            return;
        }
        default:
            (void)snprintf(response, response_size, "+ROBOT:ERR,INVALID,%u\r\n", key);
            return;
    }
}

static void *robot_sle_worker(const char *argument)
{
    char response[ROBOT_SLE_RESPONSE_MAX_LEN];
    uint8_t key;
    uint16_t conn_id;
    unused(argument);

    while (1) {
        if (osal_event_read(&g_robot_sle_event, ROBOT_SLE_EVENT_COMMAND, OSAL_EVENT_FOREVER,
            OSAL_WAITMODE_OR | OSAL_WAITMODE_CLR) != OSAL_SUCCESS) {
            continue;
        }

        key = g_robot_sle_pending_key;
        conn_id = g_robot_sle_pending_conn_id;
        if (!g_robot_sle_pending) {
            continue;
        }
        g_robot_sle_pending = false;

        (void)memset_s(response, sizeof(response), 0, sizeof(response));
        robot_sle_execute_key(key, response, sizeof(response));
        (void)robot_sle_notify_text(conn_id, response);
        osal_printk("\r\nROBOT SLE key=%c response=%s", key, response);
    }
    return NULL;
}

static void robot_sle_read_request_cb(uint8_t server_id, uint16_t conn_id, ssaps_req_read_cb_t *read_cb,
    errcode_t status)
{
    unused(server_id);
    unused(status);
    robot_sle_send_read_response(conn_id, read_cb);
}

static void robot_sle_write_request_cb(uint8_t server_id, uint16_t conn_id, ssaps_req_write_cb_t *write_cb,
    errcode_t status)
{
    char response[ROBOT_SLE_RESPONSE_MAX_LEN];
    uint8_t key;
    unused(server_id);

    if ((status != ERRCODE_SLE_SUCCESS) || (write_cb == NULL)) {
        return;
    }

    osal_printk("\r\nROBOT SLE write handle=%u expect=%u len=%u need_rsp=%u type=%u status=0x%x\r\n",
        write_cb->handle, g_robot_sle_command_handle, write_cb->length,
        write_cb->need_rsp ? 1U : 0U, write_cb->type, status);

    if ((write_cb->length == 0U) || (write_cb->value == NULL)) {
        robot_sle_send_write_response(conn_id, write_cb, ERRCODE_SLE_SUCCESS);
        return;
    }

    key = write_cb->value[0];
    if ((key >= 'a') && (key <= 'z')) {
        key = (uint8_t)(key - 'a' + 'A');
    }

    if ((write_cb->handle != g_robot_sle_command_handle) && !robot_sle_is_supported_key(key)) {
        robot_sle_send_write_response(conn_id, write_cb, ERRCODE_SLE_SUCCESS);
        return;
    }

    if (g_robot_sle_pending) {
        osal_printk("\r\nROBOT SLE recover stale pending key=%c\r\n", g_robot_sle_pending_key);
        g_robot_sle_pending = false;
    }

    robot_sle_send_write_response(conn_id, write_cb, ERRCODE_SLE_SUCCESS);
    (void)memset_s(response, sizeof(response), 0, sizeof(response));
    robot_sle_execute_key(key, response, sizeof(response));
    (void)robot_sle_notify_text(conn_id, response);
    osal_printk("\r\nROBOT SLE key=%c response=%s", key, response);
}

static void robot_sle_mtu_changed_cb(uint8_t server_id, uint16_t conn_id, ssap_exchange_info_t *mtu_size,
    errcode_t status)
{
    unused(server_id);
    osal_printk("\r\nROBOT SLE mtu conn=%u mtu=%u status=0x%x\r\n",
        conn_id, mtu_size == NULL ? 0U : mtu_size->mtu_size, status);
}

static void robot_sle_start_service_cb(uint8_t server_id, uint16_t handle, errcode_t status)
{
    osal_printk("\r\nROBOT SLE service start server=%u handle=%u status=0x%x\r\n", server_id, handle, status);
}

static void robot_sle_register_ssaps_callbacks(void)
{
    ssaps_callbacks_t callbacks = {0};
    callbacks.start_service_cb = robot_sle_start_service_cb;
    callbacks.read_request_cb = robot_sle_read_request_cb;
    callbacks.write_request_cb = robot_sle_write_request_cb;
    callbacks.mtu_changed_cb = robot_sle_mtu_changed_cb;
    (void)ssaps_register_callbacks(&callbacks);
}

static errcode_t robot_sle_add_service(void)
{
    sle_uuid_t app_uuid = {0};
    sle_uuid_t service_uuid = {0};
    ssaps_property_info_t command = {0};
    ssaps_property_info_t response = {0};
    ssaps_desc_info_t response_desc = {0};
    ssap_exchange_info_t info = {0};
    uint8_t command_value = 0U;
    uint8_t response_value[] = "+ROBOT:SLE,READY\r\n";
    uint8_t ntf_value[] = {0x01, 0x00};
    errcode_t ret;

    robot_sle_uuid_set(&app_uuid, ROBOT_SLE_SERVICE_UUID);
    ret = ssaps_register_server(&app_uuid, &g_robot_sle_server_id);
    if (ret != ERRCODE_SLE_SUCCESS) {
        osal_printk("\r\nROBOT SLE register server fail ret=0x%x\r\n", ret);
        return ret;
    }

    robot_sle_uuid_set(&service_uuid, ROBOT_SLE_SERVICE_UUID);
    ret = ssaps_add_service_sync(g_robot_sle_server_id, &service_uuid, true, &g_robot_sle_service_handle);
    if (ret != ERRCODE_SLE_SUCCESS) {
        osal_printk("\r\nROBOT SLE add service fail ret=0x%x\r\n", ret);
        return ret;
    }

    robot_sle_uuid_set(&command.uuid, ROBOT_SLE_COMMAND_UUID);
    command.permissions = SSAP_PERMISSION_WRITE;
    command.operate_indication = SSAP_OPERATE_INDICATION_BIT_WRITE | SSAP_OPERATE_INDICATION_BIT_WRITE_NO_RSP;
    command.value = &command_value;
    command.value_len = sizeof(command_value);
    ret = ssaps_add_property_sync(g_robot_sle_server_id, g_robot_sle_service_handle,
        &command, &g_robot_sle_command_handle);
    if (ret != ERRCODE_SLE_SUCCESS) {
        osal_printk("\r\nROBOT SLE add command fail ret=0x%x\r\n", ret);
        return ret;
    }

    robot_sle_uuid_set(&response.uuid, ROBOT_SLE_RESPONSE_UUID);
    response.permissions = SSAP_PERMISSION_READ;
    response.operate_indication = SSAP_OPERATE_INDICATION_BIT_READ | SSAP_OPERATE_INDICATION_BIT_NOTIFY;
    response.value = response_value;
    response.value_len = (uint16_t)strnlen((const char *)response_value, sizeof(response_value));
    ret = ssaps_add_property_sync(g_robot_sle_server_id, g_robot_sle_service_handle,
        &response, &g_robot_sle_response_handle);
    if (ret != ERRCODE_SLE_SUCCESS) {
        osal_printk("\r\nROBOT SLE add response fail ret=0x%x\r\n", ret);
        return ret;
    }

    response_desc.permissions = SSAP_PERMISSION_READ | SSAP_PERMISSION_WRITE;
    response_desc.operate_indication = SSAP_OPERATE_INDICATION_BIT_READ | SSAP_OPERATE_INDICATION_BIT_WRITE;
    response_desc.type = SSAP_DESCRIPTOR_CLIENT_CONFIGURATION;
    response_desc.value = ntf_value;
    response_desc.value_len = sizeof(ntf_value);
    ret = ssaps_add_descriptor_sync(g_robot_sle_server_id, g_robot_sle_service_handle,
        g_robot_sle_response_handle, &response_desc);
    if (ret != ERRCODE_SLE_SUCCESS) {
        osal_printk("\r\nROBOT SLE add response descriptor fail ret=0x%x\r\n", ret);
        return ret;
    }

    info.mtu_size = ROBOT_SLE_MTU_SIZE;
    info.version = 1U;
    (void)ssaps_set_info(g_robot_sle_server_id, &info);
    return ssaps_start_service(g_robot_sle_server_id, g_robot_sle_service_handle);
}

static errcode_t robot_sle_set_adv(void)
{
    uint8_t announce_data[] = {
        ROBOT_SLE_ADV_TYPE_DISCOVERY_LEVEL, 0x01, SLE_ANNOUNCE_LEVEL_NORMAL,
        ROBOT_SLE_ADV_TYPE_COMPLETE_LIST_OF_16BIT_SERVICE_UUIDS, 0x02,
        (uint8_t)(ROBOT_SLE_SERVICE_UUID & 0xFFU), (uint8_t)((ROBOT_SLE_SERVICE_UUID >> 8) & 0xFFU),
    };
    uint8_t scan_rsp_data[5U + sizeof(g_robot_sle_name) - 1U] = {
        ROBOT_SLE_ADV_TYPE_TX_POWER_LEVEL, 0x01, ROBOT_SLE_ADV_TX_POWER,
    };
    sle_announce_param_t param = {0};
    sle_announce_data_t data = {0};
    sle_addr_t addr = {0};
    uint8_t mac[SLE_ADDR_LEN] = {0xC0, 0xD3, 0x63, 0x71, 0x00, 0x01};

    addr.type = SLE_ADDRESS_TYPE_PUBLIC;
    (void)memcpy_s(addr.addr, SLE_ADDR_LEN, mac, SLE_ADDR_LEN);
    (void)sle_set_local_addr(&addr);
    (void)sle_set_local_name(g_robot_sle_name, (uint8_t)(sizeof(g_robot_sle_name) - 1U));

    param.announce_mode = SLE_ANNOUNCE_MODE_CONNECTABLE_SCANABLE;
    param.announce_handle = ROBOT_SLE_ADV_HANDLE;
    param.announce_gt_role = SLE_ANNOUNCE_ROLE_T_CAN_NEGO;
    param.announce_level = SLE_ANNOUNCE_LEVEL_NORMAL;
    param.announce_channel_map = 0x07U;
    param.announce_interval_min = ROBOT_SLE_ADV_INTERVAL;
    param.announce_interval_max = ROBOT_SLE_ADV_INTERVAL;
    param.conn_interval_min = ROBOT_SLE_CONN_INTERVAL;
    param.conn_interval_max = ROBOT_SLE_CONN_INTERVAL;
    param.conn_max_latency = 0U;
    param.conn_supervision_timeout = ROBOT_SLE_CONN_TIMEOUT;
    param.announce_tx_power = ROBOT_SLE_ADV_TX_POWER;
    param.own_addr = addr;
    (void)sle_set_announce_param(ROBOT_SLE_ADV_HANDLE, &param);

    scan_rsp_data[3] = ROBOT_SLE_ADV_TYPE_COMPLETE_LOCAL_NAME;
    scan_rsp_data[4] = (uint8_t)(sizeof(g_robot_sle_name) - 1U);
    (void)memcpy_s(&scan_rsp_data[5], sizeof(scan_rsp_data) - 5U,
        g_robot_sle_name, sizeof(g_robot_sle_name) - 1U);

    data.announce_data = announce_data;
    data.announce_data_len = sizeof(announce_data);
    data.seek_rsp_data = scan_rsp_data;
    data.seek_rsp_data_len = sizeof(scan_rsp_data);
    (void)sle_set_announce_data(ROBOT_SLE_ADV_HANDLE, &data);
    return sle_start_announce(ROBOT_SLE_ADV_HANDLE);
}

static void robot_sle_connect_state_cb(uint16_t conn_id, const sle_addr_t *addr,
    sle_acb_state_t conn_state, sle_pair_state_t pair_state, sle_disc_reason_t disc_reason)
{
    sle_connection_param_update_t param = {0};
    unused(addr);
    unused(pair_state);
    unused(disc_reason);

    osal_printk("\r\nROBOT SLE conn state conn=%u state=%u\r\n", conn_id, conn_state);
    if (conn_state == SLE_ACB_STATE_CONNECTED) {
        g_robot_sle_conn_id = conn_id;
        param.conn_id = conn_id;
        param.interval_min = ROBOT_SLE_CONN_INTERVAL;
        param.interval_max = ROBOT_SLE_CONN_INTERVAL;
        param.max_latency = 0U;
        param.supervision_timeout = ROBOT_SLE_CONN_TIMEOUT;
        (void)sle_update_connect_param(&param);
        (void)robot_sle_notify_text(conn_id, "+ROBOT:SLE,CONNECTED\r\n");
    } else if (conn_state == SLE_ACB_STATE_DISCONNECTED) {
        g_robot_sle_conn_id = 0U;
        (void)sle_start_announce(ROBOT_SLE_ADV_HANDLE);
    }
}

static void robot_sle_auth_complete_cb(uint16_t conn_id, const sle_addr_t *addr, errcode_t status,
    const sle_auth_info_evt_t *evt)
{
    unused(conn_id);
    unused(evt);
    if ((status != ERRCODE_SLE_SUCCESS) && (addr != NULL)) {
        (void)sle_remove_paired_remote_device(addr);
        (void)sle_start_announce(ROBOT_SLE_ADV_HANDLE);
    }
}

static void robot_sle_pair_complete_cb(uint16_t conn_id, const sle_addr_t *addr, errcode_t status)
{
    unused(conn_id);
    if ((status != ERRCODE_SLE_SUCCESS) && (addr != NULL)) {
        (void)sle_remove_paired_remote_device(addr);
        (void)sle_start_announce(ROBOT_SLE_ADV_HANDLE);
    }
}

static void robot_sle_register_conn_callbacks(void)
{
    sle_connection_callbacks_t callbacks = {0};
    callbacks.connect_state_changed_cb = robot_sle_connect_state_cb;
    callbacks.auth_complete_cb = robot_sle_auth_complete_cb;
    callbacks.pair_complete_cb = robot_sle_pair_complete_cb;
    (void)sle_connection_register_callbacks(&callbacks);
}

static void robot_sle_announce_enable_cb(uint32_t announce_id, errcode_t status)
{
    osal_printk("\r\nROBOT SLE announce enable id=%u status=0x%x name=%s\r\n",
        announce_id, status, g_robot_sle_name);
}

static void robot_sle_announce_disable_cb(uint32_t announce_id, errcode_t status)
{
    osal_printk("\r\nROBOT SLE announce disable id=%u status=0x%x\r\n", announce_id, status);
}

static void robot_sle_announce_terminal_cb(uint32_t announce_id)
{
    osal_printk("\r\nROBOT SLE announce terminal id=%u\r\n", announce_id);
}

static void robot_sle_enable_cb(errcode_t status)
{
    errcode_t ret;

    osal_printk("\r\nROBOT SLE enable status=0x%x\r\n", status);
    if (status != ERRCODE_SLE_SUCCESS) {
        return;
    }

    ret = robot_sle_add_service();
    osal_printk("\r\nROBOT SLE add service ret=0x%x server=%u service=%u cmd=%u rsp=%u\r\n",
        ret, g_robot_sle_server_id, g_robot_sle_service_handle,
        g_robot_sle_command_handle, g_robot_sle_response_handle);
    if (ret != ERRCODE_SLE_SUCCESS) {
        return;
    }

    ret = robot_sle_set_adv();
    osal_printk("\r\nROBOT SLE adv ret=0x%x name=%s\r\n", ret, g_robot_sle_name);
}

static void robot_sle_register_announce_callbacks(void)
{
    sle_announce_seek_callbacks_t callbacks = {0};
    callbacks.announce_enable_cb = robot_sle_announce_enable_cb;
    callbacks.announce_disable_cb = robot_sle_announce_disable_cb;
    callbacks.announce_terminal_cb = robot_sle_announce_terminal_cb;
    callbacks.sle_enable_cb = robot_sle_enable_cb;
    (void)sle_announce_seek_register_callbacks(&callbacks);
}

bool robot_sle_server_start(void)
{
    osal_task *worker = NULL;

    if (!g_robot_sle_event_ready) {
        if (osal_event_init(&g_robot_sle_event) != OSAL_SUCCESS) {
            osal_printk("\r\nROBOT SLE event init failed\r\n");
            return false;
        }
        g_robot_sle_event_ready = true;
    }

    osal_kthread_lock();
    worker = osal_kthread_create(
        (osal_kthread_handler)robot_sle_worker,
        NULL,
        "RobotSle",
        ROBOT_SLE_TASK_STACK_SIZE
    );
    if (worker != NULL) {
        (void)osal_kthread_set_priority(worker, ROBOT_SLE_TASK_PRIORITY);
    }
    osal_kthread_unlock();

    if (worker == NULL) {
        osal_printk("\r\nROBOT SLE worker create failed\r\n");
        return false;
    }

    robot_sle_register_ssaps_callbacks();
    robot_sle_register_conn_callbacks();
    robot_sle_register_announce_callbacks();
    (void)enable_sle();
    osal_printk("\r\nROBOT SLE start requested name=%s service=0x%x\r\n",
        g_robot_sle_name, ROBOT_SLE_SERVICE_UUID);
    return true;
}
