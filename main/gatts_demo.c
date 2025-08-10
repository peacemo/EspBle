#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"

// --- BLE和MAC地址相关头文件 ---
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_mac.h"

// 日志打印时使用的标签
static const char* TAG = "BLE_ADV_FIXED";

// 广播包数据 (前19字节)
static uint8_t adv_raw_data[] = {
    // AD 1: Flags (3 bytes)
    0x02, 0x01, 0x06,
    // AD 2: Manufacturer Specific Data (24 bytes)
    0x17, 0xFF, 0x00, 0x01, 0xB5, 0x00, 0x02, 0x08, 0x21, 0x26, 0x37, 0x00,
    0x00, 0x04, 0x99, 0x31, 0x39, 0x59, 0x06, 0x01, 0x10, 0x00, 0x00, 0x00,
    // AD 3: Complete List of 16-bit Service Class UUIDs (4 bytes)
    0x03, 0x03, 0x3C, 0xFE
};

// 新的扫描响应包数据 (后13字节)
static uint8_t scan_rsp_raw_data[] = {
    // AD 1: Complete Local Name "RTK_BT_4.1" (13 bytes)
    0x0C, 0x09, 0x52, 0x54, 0x4B, 0x5F, 0x42, 0x54, 0x5F, 0x34, 0x2E, 0x31
};

// 广播参数配置
// 希望每秒广播一次 (1000ms). 计算公式: 1000ms / 0.625ms = 1600.
static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 1600,
    .adv_int_max        = 1600,
    .adv_type           = ADV_TYPE_IND, // 可被扫描和连接的广播类型
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy  = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// GAP事件回调函数
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
        // 事件：原始广播数据设置完成
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "原始广播数据设置成功.");
            ESP_LOGI(TAG, "开始配置扫描响应数据...");
            // 在广播数据设置完成后，再开始设置扫描响应数据
            esp_err_t scan_rsp_ret = esp_ble_gap_config_scan_rsp_data_raw(scan_rsp_raw_data, sizeof(scan_rsp_raw_data));
            if (scan_rsp_ret){
                ESP_LOGE(TAG, "配置扫描响应数据失败, 错误代码: %x", scan_rsp_ret);
            }
            break;

        // 事件：原始扫描响应数据设置完成
        case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
            ESP_LOGI(TAG, "扫描响应数据设置成功.");
            ESP_LOGI(TAG, "数据全部配置完成，开始广播...");
            // 在扫描响应数据也设置完成后，才启动广播
            esp_err_t adv_start_ret = esp_ble_gap_start_advertising(&adv_params);
            if (adv_start_ret) {
                ESP_LOGE(TAG, "启动广播失败, 错误代码: %x", adv_start_ret);
            }
            break;

        // 事件：广播启动完成
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "广播启动失败, 错误代码: %d", param->adv_start_cmpl.status);
            } else {
                ESP_LOGI(TAG, "广播启动成功! 设备现在可以被扫描到。");
            }
            break;
        
        // 事件：广播停止完成 (可选，用于完整性)
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
            if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "停止广播失败, 错误代码: %d", param->adv_stop_cmpl.status);
            } else {
                ESP_LOGI(TAG, "停止广播成功.");
            }
            break;

        default:
            break;
    }
}


// --- 主函数 app_main ---
void app_main(void)
{
    esp_err_t ret;
    ESP_LOGI(TAG, "程序启动...");

    // 1. 初始化NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. 设置自定义MAC地址 (应在蓝牙控制器初始化之前)
    ESP_LOGI(TAG, "设置自定义基础MAC地址...");
    // 注意: 蓝牙最终的地址会是 base_mac + 2 (for public) or + 1 (for random)
    uint8_t new_mac[6] = {0x10, 0xBB, 0xF3, 0xCC, 0xF8, 0x9F};
    ret = esp_base_mac_addr_set(new_mac);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "设置MAC地址失败: %s", esp_err_to_name(ret));
    }

    // 释放传统蓝牙内存，仅使用BLE
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "初始化蓝牙控制器失败: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "使能蓝牙控制器失败: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "初始化Bluedroid失败: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "使能Bluedroid失败: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gap_register_callback(esp_gap_cb);
    if (ret){
        ESP_LOGE(TAG, "注册GAP回调失败, 错误代码: %d", ret);
        return;
    }

    ESP_LOGI(TAG, "开始配置广播数据...");
    ret = esp_ble_gap_config_adv_data_raw(adv_raw_data, sizeof(adv_raw_data));
    if (ret) {
        ESP_LOGE(TAG, "配置广播数据失败, 错误代码: %x", ret);
    }

    ESP_LOGI(TAG, "初始化完成，等待蓝牙事件链完成...");
}
