/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "string.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_rgb.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "lvgl.h"
#include <nvs_flash.h>
//#include "demos/lv_demos.h"
#include "ui_files/ui.h"

#include "driver/i2c.h" 
#include "esp_lcd_touch_gt911.h"

#include "modbus_params.h"
#include "mbcontroller.h"
#include "hexnet_bluetooth.h"
#include "hexnet_canbus.h"
#include "hexnet_wifi.h"
#include "hexnet_nvs.h"
#include "display_manager.h"
#include "data_manager.h"

#define NVS_NAMESPACE "bootdemo"
#define KEY_BOOT_FLAG "booted_once"

void app_main(void)
{
    ESP_LOGI("VANGO_MAIN----------", "Booting up...");
    //data_manager_init();
    twai_ini();

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    
    nvs_handle_t nvs;
    bool booted_once = false;  

    // Open NVS handle
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs) == ESP_OK) {
        size_t required_size = sizeof(booted_once);
        if (nvs_get_blob(nvs, KEY_BOOT_FLAG, &booted_once, &required_size) != ESP_OK) {
            booted_once = false;  // Default if not set
        }

        if (!booted_once) {
            // First boot
            ESP_LOGI("BOOT-----------", "First boot detected. Waiting 2s before reboot...");
            booted_once = true;
            nvs_set_blob(nvs, KEY_BOOT_FLAG, &booted_once, sizeof(booted_once));
            nvs_commit(nvs);
            nvs_close(nvs);

            vTaskDelay(pdMS_TO_TICKS(2000));
            esp_restart();  // Software reboot
        } else {
            // Second boot
            ESP_LOGI("BOOT------", "Initialized.");
            nvs_erase_key(nvs, KEY_BOOT_FLAG);  // Clear for next full test
            nvs_commit(nvs);
            nvs_close(nvs);

            ESP_LOGI("BOOT--------------", "Running.");
        }
    } else {
        ESP_LOGE("BOOT-------------", "Failed to open NVS.");
    }

    ESP_ERROR_CHECK(ret);
    vTaskDelay(pdMS_TO_TICKS(200));
    display_manager_init(); // Display manager'ı başlat
    ble_init();
    //wifi_ini();
    
}

