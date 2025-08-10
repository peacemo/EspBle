#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_mac.h"
#include "sdkconfig.h"
#include "esp_timer.h"

static const char* TAG = "BLE_CONFIGURABLE";

/* ------------------- NVS 定义 ------------------- */
#define NVS_NAMESPACE "ble_config"
#define NVS_KEY_MAC "mac_addr"
#define NVS_KEY_ADV "adv_data"
#define NVS_KEY_SCAN "scan_data"

/* ------------------- GATT 定义 ------------------- */
#define GATTS_SERVICE_UUID   0x00FF
#define GATTS_CHAR_UUID_MAC  0xFF01
#define GATTS_CHAR_UUID_ADV  0xFF02
#define GATTS_CHAR_UUID_CTL  0xFF03
#define GATTS_NUM_HANDLE     8

#define PROFILE_NUM 1
#define PROFILE_APP_ID 0

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle_mac;
    uint16_t char_handle_adv;
    uint16_t char_handle_ctl;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_ID] = { .gatts_cb = gatts_profile_event_handler, .gatts_if = ESP_GATT_IF_NONE, },
};

/* ------------------- 全局变量 ------------------- */
static uint8_t default_mac[6] = {0x1A, 0x2B, 0x3C, 0x4D, 0x5E, 0x6F};
static uint8_t default_adv_data[] = {0x02, 0x01, 0x06, 0x03, 0x03, 0xAA, 0xFE, 0x0C, 0x16, 0xAA, 0xFE, 0x00, 0x00, 0x45, 0x53, 0x50, 0x33, 0x32};
static uint8_t default_scan_rsp_data[] = {0x0F, 0x09, 'E', 'S', 'P', '_', 'C', 'O', 'N', 'F', 'I', 'G', 'U', 'R', 'E'};

static uint8_t current_mac[6];
static uint8_t current_adv_data[31];
static uint8_t current_scan_rsp_data[31];
static uint8_t current_adv_data_len = 0;
static uint8_t current_scan_rsp_data_len = 0;

static uint8_t new_mac[6];
static uint8_t new_adv_data[62];
static bool new_mac_received = false;
static bool new_adv_data_received = false;
static uint16_t new_adv_data_len = 0;

static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 352, .adv_int_max        = 352, .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC, .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

/* ------------------- NVS 函数 ------------------- */
void read_config_from_nvs() {
    nvs_handle_t nvs_handle;
    esp_err_t err;

    err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS: 打开失败, 为所有配置使用默认值");
        memcpy(current_mac, default_mac, sizeof(default_mac));
        memcpy(current_adv_data, default_adv_data, sizeof(default_adv_data));
        current_adv_data_len = sizeof(default_adv_data);
        memcpy(current_scan_rsp_data, default_scan_rsp_data, sizeof(default_scan_rsp_data));
        current_scan_rsp_data_len = sizeof(default_scan_rsp_data);
        return;
    }

    size_t mac_size = sizeof(current_mac);
    err = nvs_get_blob(nvs_handle, NVS_KEY_MAC, current_mac, &mac_size);
    if (err != ESP_OK || mac_size != sizeof(current_mac)) {
        ESP_LOGW(TAG, "NVS: 未找到MAC地址, 使用默认值");
        memcpy(current_mac, default_mac, sizeof(default_mac));
    }

    size_t adv_size = sizeof(current_adv_data);
    err = nvs_get_blob(nvs_handle, NVS_KEY_ADV, current_adv_data, &adv_size);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS: 未找到广播数据, 使用默认值");
        memcpy(current_adv_data, default_adv_data, sizeof(default_adv_data));
        current_adv_data_len = sizeof(default_adv_data);
    } else {
        current_adv_data_len = adv_size;
    }

    size_t scan_size = sizeof(current_scan_rsp_data);
    err = nvs_get_blob(nvs_handle, NVS_KEY_SCAN, current_scan_rsp_data, &scan_size);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS: 未找到扫描响应, 使用默认值");
        memcpy(current_scan_rsp_data, default_scan_rsp_data, sizeof(default_scan_rsp_data));
        current_scan_rsp_data_len = sizeof(default_scan_rsp_data);
    } else {
        current_scan_rsp_data_len = scan_size;
    }
    nvs_close(nvs_handle);
}

esp_err_t write_config_to_nvs() {
    nvs_handle_t nvs_handle;
    esp_err_t err;
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) { ESP_LOGE(TAG, "打开NVS失败!"); return err; }

    if (new_mac_received) {
        err = nvs_set_blob(nvs_handle, NVS_KEY_MAC, new_mac, sizeof(new_mac));
        if (err == ESP_OK) ESP_LOGI(TAG, "NVS: 新MAC地址已写入");
        new_mac_received = false;
    }

    if (new_adv_data_received) {
        uint8_t adv_len = new_adv_data[0];
        if (adv_len > 0 && new_adv_data_len > adv_len) {
            err = nvs_set_blob(nvs_handle, NVS_KEY_ADV, &new_adv_data[1], adv_len);
            if (err == ESP_OK) ESP_LOGI(TAG, "NVS: 新广播数据已写入");
            uint8_t scan_len_index = 1 + adv_len;
            if (new_adv_data_len > scan_len_index) {
                uint8_t scan_len = new_adv_data[scan_len_index];
                if (scan_len > 0 && new_adv_data_len >= scan_len_index + 1 + scan_len) {
                     err = nvs_set_blob(nvs_handle, NVS_KEY_SCAN, &new_adv_data[scan_len_index + 1], scan_len);
                     if (err == ESP_OK) ESP_LOGI(TAG, "NVS: 新扫描响应已写入");
                } else { nvs_erase_key(nvs_handle, NVS_KEY_SCAN); }
            } else { nvs_erase_key(nvs_handle, NVS_KEY_SCAN); }
        }
        new_adv_data_received = false;
    }

    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) ESP_LOGE(TAG, "NVS提交失败!");
    nvs_close(nvs_handle);
    return err;
}

/* ------------------- GAP 回调 ------------------- */
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            esp_ble_gap_config_scan_rsp_data_raw(current_scan_rsp_data, current_scan_rsp_data_len);
            break;
        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) { ESP_LOGE(TAG, "广播启动失败"); } 
            else { ESP_LOGI(TAG, "广播启动成功"); }
            break;
        default: break;
    }
}

/* ------------------- GATT 回调 ------------------- */
static const esp_bt_uuid_t mac_char_uuid = {.len = ESP_UUID_LEN_16, .uuid.uuid16 = GATTS_CHAR_UUID_MAC};
static const esp_bt_uuid_t adv_char_uuid = {.len = ESP_UUID_LEN_16, .uuid.uuid16 = GATTS_CHAR_UUID_ADV};
static const esp_bt_uuid_t ctl_char_uuid = {.len = ESP_UUID_LEN_16, .uuid.uuid16 = GATTS_CHAR_UUID_CTL};
static esp_gatt_srvc_id_t service_id;

static void restart_timer_callback(void* arg) {
    ESP_LOGI(TAG, "定时器触发，正在重启...");
    esp_restart();
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
        case ESP_GATTS_REG_EVT: {
            ESP_LOGI(TAG, "GATT应用注册成功, app_id %04x, gatts_if %d", param->reg.app_id, gatts_if);
            gl_profile_tab[PROFILE_APP_ID].gatts_if = gatts_if;
            service_id.is_primary = true;
            service_id.id.inst_id = 0x00;
            service_id.id.uuid.len = ESP_UUID_LEN_16;
            service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID;
            esp_ble_gatts_create_service(gatts_if, &service_id, GATTS_NUM_HANDLE);
            break;
        }
        case ESP_GATTS_CREATE_EVT: {
            ESP_LOGI(TAG, "服务创建成功, service_handle %d", param->create.service_handle);
            gl_profile_tab[PROFILE_APP_ID].service_handle = param->create.service_handle;
            esp_ble_gatts_start_service(param->create.service_handle);
            esp_ble_gatts_add_char(param->create.service_handle, &mac_char_uuid, ESP_GATT_PERM_WRITE, ESP_GATT_CHAR_PROP_BIT_WRITE, NULL, NULL);
            esp_ble_gatts_add_char(param->create.service_handle, &adv_char_uuid, ESP_GATT_PERM_WRITE, ESP_GATT_CHAR_PROP_BIT_WRITE, NULL, NULL);
            esp_ble_gatts_add_char(param->create.service_handle, &ctl_char_uuid, ESP_GATT_PERM_WRITE, ESP_GATT_CHAR_PROP_BIT_WRITE, NULL, NULL);
            break;
        }
        case ESP_GATTS_ADD_CHAR_EVT:
            ESP_LOGI(TAG, "特征添加成功, attr_handle %d, uuid %04x", param->add_char.attr_handle, param->add_char.char_uuid.uuid.uuid16);
            if (param->add_char.char_uuid.uuid.uuid16 == GATTS_CHAR_UUID_MAC) { gl_profile_tab[PROFILE_APP_ID].char_handle_mac = param->add_char.attr_handle; } 
            else if (param->add_char.char_uuid.uuid.uuid16 == GATTS_CHAR_UUID_ADV) { gl_profile_tab[PROFILE_APP_ID].char_handle_adv = param->add_char.attr_handle; } 
            else if (param->add_char.char_uuid.uuid.uuid16 == GATTS_CHAR_UUID_CTL) { gl_profile_tab[PROFILE_APP_ID].char_handle_ctl = param->add_char.attr_handle; }
            break;
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(TAG, "设备连接: conn_id %d", param->connect.conn_id);
            gl_profile_tab[PROFILE_APP_ID].conn_id = param->connect.conn_id;
            esp_ble_gap_stop_advertising();
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(TAG, "设备断开连接, reason 0x%x", param->disconnect.reason);
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GATTS_WRITE_EVT: {
            ESP_LOGI(TAG, "GATT写事件: handle %d, len %d", param->write.handle, param->write.len);
            
            // **FIX**: Respond to the write request to prevent disconnection.
            if (param->write.need_rsp){
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
            }
            
            if (param->write.handle == gl_profile_tab[PROFILE_APP_ID].char_handle_mac) {
                if (param->write.len == sizeof(new_mac)) {
                    memcpy(new_mac, param->write.value, sizeof(new_mac));
                    new_mac_received = true;
                    ESP_LOGI(TAG, "收到新的MAC地址");
                }
            } else if (param->write.handle == gl_profile_tab[PROFILE_APP_ID].char_handle_adv) {
                if (param->write.len <= sizeof(new_adv_data)) {
                    memcpy(new_adv_data, param->write.value, param->write.len);
                    new_adv_data_len = param->write.len;
                    new_adv_data_received = true;
                    ESP_LOGI(TAG, "收到新的广播数据");
                }
            } else if (param->write.handle == gl_profile_tab[PROFILE_APP_ID].char_handle_ctl) {
                if (param->write.len == 1 && param->write.value[0] == 0x01) {
                    ESP_LOGI(TAG, "收到控制命令: 保存并重启");
                    write_config_to_nvs();
                    
                    const esp_timer_create_args_t restart_timer_args = {
                        .callback = &restart_timer_callback,
                        .name = "restart-timer"
                    };
                    esp_timer_handle_t restart_timer;
                    esp_timer_create(&restart_timer_args, &restart_timer);
                    esp_timer_start_once(restart_timer, 2000 * 1000); // 2 seconds
                }
            }
            break;
        }
        default: break;
    }
}

void app_main(void) {
    esp_err_t ret;
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    read_config_from_nvs();
    
    ESP_LOGI(TAG, "设置基础MAC地址: %02x:%02x:%02x:%02x:%02x:%02x",
             current_mac[0], current_mac[1], current_mac[2], current_mac[3], current_mac[4], current_mac[5]);
    ret = esp_base_mac_addr_set(current_mac);
    if (ret != ESP_OK) { ESP_LOGE(TAG, "设置基础MAC地址失败: %s", esp_err_to_name(ret)); }

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) { ESP_LOGE(TAG, "初始化蓝牙控制器失败: %s", esp_err_to_name(ret)); return; }
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) { ESP_LOGE(TAG, "使能蓝牙控制器失败: %s", esp_err_to_name(ret)); return; }

    ret = esp_bluedroid_init();
    if (ret) { ESP_LOGE(TAG, "初始化Bluedroid失败: %s", esp_err_to_name(ret)); return; }
    ret = esp_bluedroid_enable();
    if (ret) { ESP_LOGE(TAG, "使能Bluedroid失败: %s", esp_err_to_name(ret)); return; }

    ret = esp_ble_gatts_register_callback(gatts_profile_event_handler);
    if (ret) { ESP_LOGE(TAG, "注册GATT回调失败, 错误代码 = %x", ret); return; }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) { ESP_LOGE(TAG, "注册GAP回调失败, 错误代码 = %x", ret); return; }
    ret = esp_ble_gatts_app_register(PROFILE_APP_ID);
    if (ret) { ESP_LOGE(TAG, "注册GATT应用失败, 错误代码 = %x", ret); return; }

    esp_ble_gap_config_adv_data_raw(current_adv_data, current_adv_data_len);
}
