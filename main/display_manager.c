// display_manager.c

#include "display_manager.h"
#include "data_manager.h"
#include "definitions.h"
#include "ui_files/ui.h"
#include "hexnet_bluetooth.h"
#include "hexnet_nvs.h"
#include "hexnet_canbus.h"

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
#include "cJson.h"

#include "driver/i2c.h" 
#include "esp_lcd_touch_gt911.h"



static const char *TAG = "DISPLAY_MANAGER";



extern lv_obj_t * ui_scrTheme;
extern lv_obj_t * ui_scrWallpaper;

extern lv_obj_t * ui_lblVangoText;
extern lv_obj_t *ui_imgDevice;
extern lv_obj_t *ui_imgsconnected;
extern lv_obj_t *ui_imgsnotconnected;
extern lv_obj_t *ui_imgBluetoothNotConnected;
extern lv_obj_t *ui_imgBluetoothConnected;
extern lv_obj_t *ui_arcGrup1;
extern lv_obj_t *ui_arcGrup2;
extern lv_obj_t *ui_swO1; 
extern lv_obj_t *ui_swO2; 
extern lv_obj_t *ui_swO3; 
extern lv_obj_t *ui_swO4; 
extern lv_obj_t *ui_swO5; 
extern lv_obj_t *ui_swO6; 
extern lv_obj_t *ui_swO7; 
extern lv_obj_t *ui_swO8; 
extern lv_obj_t *ui_swO9; 
extern lv_obj_t *ui_swO10;
extern lv_obj_t *ui_swO11;
extern lv_obj_t *ui_swO12;
extern lv_obj_t *ui_swO13;
extern lv_obj_t *ui_swO14;
extern lv_obj_t *ui_swO15;
extern lv_obj_t *ui_swO16;

extern lv_obj_t* ui_cbxO1;
extern lv_obj_t* ui_cbxO2;
extern lv_obj_t* ui_cbxO3;
extern lv_obj_t* ui_cbxO4;
extern lv_obj_t* ui_cbxO5;
extern lv_obj_t* ui_cbxO6;
extern lv_obj_t* ui_cbxO7;
extern lv_obj_t* ui_cbxO8;
extern lv_obj_t* ui_cbxO9;
extern lv_obj_t* ui_cbxO10;
extern lv_obj_t* ui_cbxO11;
extern lv_obj_t* ui_cbxO12;
extern lv_obj_t* ui_cbxO13;
extern lv_obj_t* ui_cbxO14;
extern lv_obj_t* ui_cbxO15;
extern lv_obj_t* ui_cbxO16;



extern lv_obj_t* ui_swDim1;
extern lv_obj_t* ui_swDim2;
extern lv_obj_t* ui_swDim3;
extern lv_obj_t* ui_swDim4;

extern lv_obj_t* ui_cbxDim1;
extern lv_obj_t* ui_cbxDim2;
extern lv_obj_t* ui_cbxDim3;
extern lv_obj_t* ui_cbxDim4;

extern lv_obj_t* ui_Checkbox1;
extern lv_obj_t* ui_Checkbox2;
extern lv_obj_t* ui_Checkbox3;
extern lv_obj_t* ui_Checkbox4;
extern lv_obj_t* ui_Checkbox5;

extern lv_obj_t* ui_Checkbox6;


// Declare the panels
extern lv_obj_t* ui_pnlGrup1;
extern lv_obj_t* ui_pnlGrup2;
extern lv_obj_t* ui_pnlGrup3;
extern lv_obj_t* ui_pnlOutputs;
extern lv_obj_t* ui_pnlConnectionLost;

extern lv_obj_t *ui_imgWForecast;
extern lv_obj_t *ui_lblDateAndTime;
lv_obj_t * ui_btnIOGot;


extern lv_obj_t *ui_lblSelectTheme;
extern lv_obj_t *ui_lblWallpaper; 
extern lv_obj_t *ui_lblRolllerTime;
extern lv_obj_t *ui_swEnableWallpaper;
extern lv_obj_t *ui_rlrTime;

extern lv_obj_t * ui_scrInit;
extern lv_obj_t *ui_scrRules;
extern lv_obj_t *ui_scrPanelSettings;
extern lv_obj_t *ui_lblPanelSettings;
extern lv_obj_t *ui_lblSensors;
extern lv_obj_t *ui_lblDimmableOutputs;
extern lv_obj_t *ui_pnlSensors;
extern lv_obj_t *ui_lblWeather;
extern lv_obj_t *ui_lblSettingsB;

extern lv_obj_t *ui_Colorwheel1;
extern lv_obj_t *ui_btnRGBColor;
extern lv_obj_t *ui_brInit;
extern lv_obj_t *ui_Label1;

extern lv_obj_t* ui_img_lamp_png;
extern lv_obj_t* ui_img_water_png;
extern lv_obj_t* ui_img_outlet_png;
extern lv_obj_t* ui_img_oven_png;
extern lv_obj_t* ui_img_tv_png;
extern lv_obj_t* ui_img_refrigerator_png;
extern lv_obj_t* ui_img_toilet_png;
extern lv_obj_t* ui_img_usb_png;
extern lv_obj_t* ui_img_ac_png;
extern lv_obj_t* ui_img_readinglamp_png;
extern lv_obj_t* ui_img_heater_png;


// Slave ID 
#define SLAVE_ID 50

// General CID
#define CID_MIDITHREE 0

// Voltage and Current
#define VOLTAGE_INDIS 1
#define CURRENT_INDIS 2

// Control Registers
#define RESET_INDIS 3
#define BLUETOOTH_INDIS 4



// Inputs
#define INPUT_1_INDIS 21
#define INPUT_2_INDIS 22
#define INPUT_3_INDIS 23
#define INPUT_4_INDIS 24
#define INPUT_5_INDIS 25
#define INPUT_6_INDIS 26
#define INPUT_7_INDIS 27
#define INPUT_8_INDIS 28
#define INPUT_9_INDIS 29
#define INPUT_10_INDIS 30
#define INPUT_11_INDIS 31
#define INPUT_12_INDIS 32
#define INPUT_13_INDIS 33
#define INPUT_14_INDIS 34
#define INPUT_15_INDIS 35
#define INPUT_16_INDIS 36



// Define weather conditions
#define WEATHER_SUNNY         1
#define WEATHER_PARTLY_SUNNY  2
#define WEATHER_THUNDER       3
#define WEATHER_RAINY         4
#define WEATHER_SNOWY         5
#define WEATHER_CLOUDY        6

// Define weather icon positions
#define ICON_SUNNY            0
#define ICON_PARTLY_SUNNY     34
#define ICON_THUNDER          75
#define ICON_RAINY            115
#define ICON_SNOWY            150
#define ICON_CLOUDY           190

// Analog Inputs
#define ANALOG_INPUT_1_INDIS 37
#define ANALOG_INPUT_2_INDIS 38
#define ANALOG_INPUT_3_INDIS 39
#define ANALOG_INPUT_4_INDIS 40
#define ANALOG_INPUT_5_INDIS 41

// Dimmable Outputs
#define DIMMABLE_OUTPUT_1_INDIS 42
#define DIMMABLE_OUTPUT_2_INDIS 43
#define DIMMABLE_OUTPUT_3_INDIS 44
#define DIMMABLE_OUTPUT_4_INDIS 45

// RGB Outputs
#define RGB_R_INDIS 46
#define RGB_G_INDIS 47
#define RGB_B_INDIS 48



#define I2C_MASTER_SCL_IO           9       /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           8       /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0       /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define GPIO_INPUT_IO_4    4
#define GPIO_INPUT_PIN_SEL  1ULL<<GPIO_INPUT_IO_4


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////// Please update the following configuration according to your LCD spec //////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ     (18 * 1000 * 1000)
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL  1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_BK_LIGHT       -1
#define EXAMPLE_PIN_NUM_HSYNC          46
#define EXAMPLE_PIN_NUM_VSYNC          3
#define EXAMPLE_PIN_NUM_DE             5
#define EXAMPLE_PIN_NUM_PCLK           7
#define EXAMPLE_PIN_NUM_DATA0          14 // B3
#define EXAMPLE_PIN_NUM_DATA1          38 // B4
#define EXAMPLE_PIN_NUM_DATA2          18 // B5
#define EXAMPLE_PIN_NUM_DATA3          17 // B6
#define EXAMPLE_PIN_NUM_DATA4          10 // B7
#define EXAMPLE_PIN_NUM_DATA5          39 // G2
#define EXAMPLE_PIN_NUM_DATA6          0 // G3
#define EXAMPLE_PIN_NUM_DATA7          45 // G4
#define EXAMPLE_PIN_NUM_DATA8          48 // G5
#define EXAMPLE_PIN_NUM_DATA9          47 // G6
#define EXAMPLE_PIN_NUM_DATA10         21 // G7
#define EXAMPLE_PIN_NUM_DATA11         1  // R3
#define EXAMPLE_PIN_NUM_DATA12         2  // R4
#define EXAMPLE_PIN_NUM_DATA13         42 // R5
#define EXAMPLE_PIN_NUM_DATA14         41 // R6
#define EXAMPLE_PIN_NUM_DATA15         40 // R7
#define EXAMPLE_PIN_NUM_DISP_EN        -1

// The pixel number in horizontal and vertical
#define EXAMPLE_LCD_H_RES              800
#define EXAMPLE_LCD_V_RES              480

#if CONFIG_EXAMPLE_DOUBLE_FB
#define EXAMPLE_LCD_NUM_FB             2
#else
#define EXAMPLE_LCD_NUM_FB             1
#endif // CONFIG_EXAMPLE_DOUBLE_FB

#define EXAMPLE_LVGL_TICK_PERIOD_MS    2
#define EXAMPLE_LVGL_TASK_MAX_DELAY_MS 500
#define EXAMPLE_LVGL_TASK_MIN_DELAY_MS 1
#define EXAMPLE_LVGL_TASK_STACK_SIZE   (8 * 1024)
#define EXAMPLE_LVGL_TASK_PRIORITY     2

static SemaphoreHandle_t lvgl_mux = NULL;

// we use two semaphores to sync the VSYNC event and the LVGL task, to avoid potential tearing effect
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
SemaphoreHandle_t sem_vsync_end;
SemaphoreHandle_t sem_gui_ready;
#endif

// Define the maximum number of outputs
#define MAX_OUTPUTS 16


// Declare button, label, and image objects
lv_obj_t* btnIO[MAX_OUTPUTS] = {NULL};
lv_obj_t* lblIO[MAX_OUTPUTS] = {NULL};
lv_obj_t* imgIO[MAX_OUTPUTS] = {NULL};
lv_obj_t* sldDims[4] = {NULL};
lv_obj_t* lblDims[4] = {NULL};
// Define the button names and icons
const char* lblBtnNames[18] = {
    "LAMP", "TOILET", "KITCHEN", "BEDROOM", "CORRIDOR", "STEP", "AC", "USB", "REGRIGE.", "WATER P.", "OUTLET", "OVEN", "TV", "EX.LIGHT", "EX.OUTLET", "HEATER", "SPOT", "READING L."
};



// Example data to save
int numOfOutputs = 16;
int numOfDims = 4;
int numOfSensors = 5;
float batarya_volt = 0;
bool slaveConnectionStatus = true;
int panelThemeType = 0;
int panelWallpaperEnable = false;
int panelWallpaperTime = 0;
int numberOfNotifications = 0;
cJSON* notifications = NULL;
// Declare the global outputsBuffer
int outputsBuffer[16] = {0};
int sensorsBuffer[5] = {0};
int dimsBuffer[4] = {0};
int rgbBuffer[3] = {0};
int btn_index = 0;

int panelWallpaperEnableCounter = 1;


extern void example_lvgl_demo_ui(lv_disp_t *disp);
void parse_read_data(cJSON* json);
void parse_write_data(cJSON* json);
void parse_configuration_data(cJSON* json);
void parse_rules_data(cJSON* json);
char* create_json_data_packet(const uint16_t* regs_data, int numOfOutputs, int numOfDims, int numOfSensors, bool slaveConnectionStatus, int themeType, int numberOfNotifications, cJSON* notifications);
void parse_ble_data(const char* json_data);


//###############################   LVGL FUNCTIONS   ##########################################
static bool example_on_vsync_event(esp_lcd_panel_handle_t panel, const esp_lcd_rgb_panel_event_data_t *event_data, void *user_data)
{
    BaseType_t high_task_awoken = pdFALSE;
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
    if (xSemaphoreTakeFromISR(sem_gui_ready, &high_task_awoken) == pdTRUE) {
        xSemaphoreGiveFromISR(sem_vsync_end, &high_task_awoken);
    }
#endif
    return high_task_awoken == pdTRUE;
}

static void example_lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t) drv->user_data;
    int offsetx1 = area->x1;
    int offsetx2 = area->x2;
    int offsety1 = area->y1;
    int offsety2 = area->y2;
#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
    xSemaphoreGive(sem_gui_ready);
    xSemaphoreTake(sem_vsync_end, portMAX_DELAY);
#endif
    // pass the draw buffer to the driver
    esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, color_map);
    lv_disp_flush_ready(drv);
}

static void example_increase_lvgl_tick(void *arg)
{
    /* Tell LVGL how many milliseconds has elapsed */
    lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

bool example_lvgl_lock(int timeout_ms)
{
    // Convert timeout in milliseconds to FreeRTOS ticks
    // If `timeout_ms` is set to -1, the program will block until the condition is met
    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(lvgl_mux, timeout_ticks) == pdTRUE;
}

void example_lvgl_unlock(void)
{
    xSemaphoreGiveRecursive(lvgl_mux);
}

static void example_lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");
    uint32_t task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
    while (1) {
        // Lock the mutex due to the LVGL APIs are not thread-safe
        if (example_lvgl_lock(-1)) {
            task_delay_ms = lv_timer_handler();
            // Release the mutex
            example_lvgl_unlock();
        }
        if (task_delay_ms > EXAMPLE_LVGL_TASK_MAX_DELAY_MS) {
            task_delay_ms = EXAMPLE_LVGL_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < EXAMPLE_LVGL_TASK_MIN_DELAY_MS) {
            task_delay_ms = EXAMPLE_LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}


/**
 * @brief i2c master initialization
 */
static esp_err_t i2c_master_init(void)
{
    int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &conf);

    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}

void gpio_init(void)
{
    //zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //bit mask of the pins, use GPIO6 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //enable pull-up mode
    // io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
}

// extern lv_obj_t *scr;
static void example_lvgl_touch_cb(lv_indev_drv_t * drv, lv_indev_data_t * data)
{
    uint16_t touchpad_x[1] = {0};
    uint16_t touchpad_y[1] = {0};
    uint8_t touchpad_cnt = 0;

    /* Read touch controller data */
    esp_lcd_touch_read_data(drv->user_data);

    /* Get coordinates */
    bool touchpad_pressed = esp_lcd_touch_get_coordinates(drv->user_data, touchpad_x, touchpad_y, NULL, &touchpad_cnt, 1);

    if (touchpad_pressed && touchpad_cnt > 0) {
        data->point.x = touchpad_x[0];
        data->point.y = touchpad_y[0];
        data->state = LV_INDEV_STATE_PR;
        panelWallpaperEnableCounter = 0;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}

//########################################################################################################

//###############################   DISPLAY MANAGER FUNCTIONS   ##########################################
void my_btnThemeWhiteFunc(void)
{
    panelThemeType = 0;
    lv_obj_set_style_bg_color(ui_scrMain, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_scrTheme, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_scrSettings, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_scrRules, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_scrPanelSettings, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);

    
    lv_obj_set_style_text_color(ui_lblPanelSettings, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lblSensors, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lblDimmableOutputs, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_Checkbox1, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_Checkbox2, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_Checkbox3, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_Checkbox4, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_Checkbox5, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);


    lv_obj_set_style_text_color(ui_lblPnlGrup1Sicaklik1, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lblPnlGrup1Sicaklik2, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lblPnlGrup1SicaklikDeger1, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lblPnlGrup1SicaklikDeger2, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_pnlGrupSicaklik1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_pnlGrupSicaklik2, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);


    lv_obj_set_style_bg_color(ui_pnlGrup1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lblGrup1Oran1, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lblGrup1, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_pnlGrup2, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lblGrup1Oran2, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lblGrup2, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_pnlGrup3, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lblGrup1Oran3, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lblGrup3, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);


    lv_obj_set_style_bg_color(ui_pnlSensors, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_pnlOutputs, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_text_color(ui_lblVangoText, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);


    lv_obj_set_style_text_color(ui_lblSelectTheme, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lblWallpaper, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lblRolllerTime, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_text_color(ui_lblRolllerTime, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lblRolllerTime, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lblRolllerTime, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_text_color(ui_lblWeather, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lblDateAndTime, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lblSettingsB, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    

    for (int i = 0; i < numOfDims; i++) {
        lv_obj_set_style_text_color(lblDims[i], lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    }

}




void my_btnBlackThemeFunc(void)
{
    panelThemeType = 1;
    lv_obj_set_style_bg_color(ui_scrMain, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_scrTheme, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_scrSettings, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_scrRules, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_scrPanelSettings, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);

    
    lv_obj_set_style_text_color(ui_lblPanelSettings, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lblSensors, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lblDimmableOutputs, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_Checkbox1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_Checkbox2, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_Checkbox3, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_Checkbox4, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_Checkbox5, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_Checkbox6, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);


    lv_obj_set_style_text_color(ui_lblPnlGrup1Sicaklik1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lblPnlGrup1Sicaklik2, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lblPnlGrup1SicaklikDeger1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lblPnlGrup1SicaklikDeger2, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_pnlGrupSicaklik1, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_pnlGrupSicaklik2, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);


    lv_obj_set_style_bg_color(ui_pnlGrup1, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lblGrup1Oran1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lblGrup1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_pnlGrup2, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lblGrup1Oran2, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lblGrup2, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_bg_color(ui_pnlGrup3, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lblGrup1Oran3, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lblGrup3, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);


    lv_obj_set_style_bg_color(ui_pnlSensors, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_pnlOutputs, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_text_color(ui_lblVangoText, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_text_color(ui_lblSelectTheme, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lblWallpaper, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lblRolllerTime, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_text_color(ui_lblWeather, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lblDateAndTime, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_lblSettingsB, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);

    for (int i = 0; i < numOfDims; i++) {
        lv_obj_set_style_text_color(lblDims[i], lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    }
}



// Function to get the image name based on the button name
const void* get_image_for_button(int outputBufferIndex) {
    if (outputBufferIndex < 0 || outputBufferIndex >= 19) {
        return NULL; // Invalid index
    }

    const char* btnName = lblBtnNames[outputBufferIndex];

    if (strcmp(btnName, "KITCHEN") == 0 || strcmp(btnName, "LAMP") == 0 || strcmp(btnName, "EX.LIGHT") == 0 || strcmp(btnName, "SPOT") == 0 || strcmp(btnName, "BEDROOM") == 0 || strcmp(btnName, "CORRIDOR") == 0 || strcmp(btnName, "STEP") == 0) {
        return &ui_img_lamp_png;
    } else if (strcmp(btnName, "TOILET") == 0) {
        return &ui_img_toilet_png;
    } else if (strcmp(btnName, "OUTLET") == 0 || strcmp(btnName, "EX.OUTLET") == 0 || strcmp(btnName, "HEATER") == 0) {
        return &ui_img_outlet_png;
    } else if (strcmp(btnName, "USB") == 0) {
        return &ui_img_usb_png;
    } else if (strcmp(btnName, "REGRIGE.") == 0) {
        return &ui_img_refrigerator_png;
    } else if (strcmp(btnName, "WATER P.") == 0) {
        return &ui_img_water_png;
    } else if (strcmp(btnName, "OVEN") == 0) {
        return &ui_img_oven_png;
    } else if (strcmp(btnName, "TV") == 0) {
        return &ui_img_tv_png;
    } else if (strcmp(btnName, "AC") == 0) {
        return &ui_img_ac_png;
    } else if (strcmp(btnName, "HEATER") == 0) {
        return &ui_img_heater_png;
    } else if (strcmp(btnName, "READING L.") == 0) {
        return &ui_img_readinglamp_png;
    } else {
        return NULL; // No matching image found
    }
}


// Function to set panel coordinates dynamically
void set_sensor_panels_coordinates(int numOfSensors, int sensorsBuffer[5]) {
    lv_obj_t* panels[3] = {ui_pnlGrup1, ui_pnlGrup2, ui_pnlGrup3};
    lv_obj_t* panels2[2] = {ui_pnlGrupSicaklik1, ui_pnlGrupSicaklik2};

    int x_coords[3] = {-320, -210, -101};
    int y_coords[3] = {60, 61, 59};
    int x2_coords[2] = {-320, -208};
    int y2_coords[2] = {147, 151};


    lv_obj_clear_flag(ui_pnlGrup1, LV_OBJ_FLAG_HIDDEN);     /// Flags
    lv_obj_clear_flag(ui_pnlGrup2, LV_OBJ_FLAG_HIDDEN);     /// Flags
    lv_obj_clear_flag(ui_pnlGrup3, LV_OBJ_FLAG_HIDDEN);     /// Flags
    lv_obj_clear_flag(ui_pnlGrupSicaklik1, LV_OBJ_FLAG_HIDDEN);     /// Flags
    lv_obj_clear_flag(ui_pnlGrupSicaklik2, LV_OBJ_FLAG_HIDDEN);     /// Flags

    int current_index = 0;
    for (int i = 0; i < 3; i++) {
        if (sensorsBuffer[i + 2] == 1) {
            lv_obj_set_x(panels[i], x_coords[current_index]);
            lv_obj_set_y(panels[i], y_coords[current_index]);
            current_index++;
        }
        else {
            lv_obj_set_x(panels[i], LV_COORD_MAX);
            lv_obj_set_y(panels[i], LV_COORD_MAX);
        }
    }
    current_index = 0;
    for (int i = 0; i < 2; i++) {
        if (sensorsBuffer[i] == 1) {
            lv_obj_set_x(panels2[i], x2_coords[current_index]);
            lv_obj_set_y(panels2[i], y2_coords[current_index]);
            current_index++;
        }
        else {
            lv_obj_set_x(panels2[i], LV_COORD_MAX);
            lv_obj_set_y(panels2[i], LV_COORD_MAX);
        }
    }
}

// Function to toggle button color based on regs_data
void button_events(lv_event_t* e) {
    lv_obj_t* btn = lv_event_get_target(e);
    btn_index = (int)lv_event_get_user_data(e);
    toggle_regs_data(btn_index + OUTPUT_1_INDIS);
    ESP_LOGI(TAG, "Button index: %d", btn_index);
}

// Function to create the UI dynamically based on numOfOutputs
void create_dynamic_ui(lv_obj_t* parent) {

    int btn_width = 105;
    int btn_height = 190;
    int btn_x_offset = 106; // btn_width + 1 for spacing
    int btn_y_offset = 191; // btn_height + 1 for spacing
    int x_start = 14;
    int y_start = -102;

    // Apply numOfOutputs and outputsBuffer to swO1-swO16 and cbxO1-cbxO16
    lv_obj_t* switches[16] = {ui_swO1, ui_swO2, ui_swO3, ui_swO4, ui_swO5, ui_swO6, ui_swO7, ui_swO8, ui_swO9, ui_swO10, ui_swO11, ui_swO12, ui_swO13, ui_swO14, ui_swO15, ui_swO16};
    lv_obj_t* dropdowns[16] = {ui_cbxO1, ui_cbxO2, ui_cbxO3, ui_cbxO4, ui_cbxO5, ui_cbxO6, ui_cbxO7, ui_cbxO8, ui_cbxO9, ui_cbxO10, ui_cbxO11, ui_cbxO12, ui_cbxO13, ui_cbxO14, ui_cbxO15, ui_cbxO16};
    lv_obj_t* checkboxes[5] = {ui_Checkbox1, ui_Checkbox2, ui_Checkbox3, ui_Checkbox4, ui_Checkbox5};

    lv_obj_t* dimcheckboxes[4] = {ui_swDim1, ui_swDim2, ui_swDim3, ui_swDim4};
    lv_obj_t* dimdropdowns[4] = {ui_cbxDim1, ui_cbxDim2, ui_cbxDim3, ui_cbxDim4};

    // Adjust button size and spacing if numOfOutputs is greater than 8
    if (numOfOutputs > 8) {
        btn_width = 105;
        btn_height = 95;
        btn_x_offset = 106; // btn_width + 1 for spacing
        btn_y_offset = 96; // btn_height + 1 for spacing
        y_start = -150;
    }

    for (int i = 0; i < numOfOutputs; i++) {
        int row = i / 4;
        int col = i % 4;


        btnIO[i] = lv_btn_create(parent);
        lv_obj_set_width(btnIO[i], btn_width);
        lv_obj_set_height(btnIO[i], btn_height);
        lv_obj_set_x(btnIO[i], x_start + col * btn_x_offset);
        lv_obj_set_y(btnIO[i], y_start + row * btn_y_offset);
        lv_obj_set_align(btnIO[i], LV_ALIGN_CENTER);
        lv_obj_add_flag(btnIO[i], LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
        lv_obj_clear_flag(btnIO[i], LV_OBJ_FLAG_SCROLLABLE);      /// Flags
        lv_obj_set_style_radius(btnIO[i], 0, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_color(btnIO[i], lv_color_hex(0x5A5A5A), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_bg_opa(btnIO[i], 255, LV_PART_MAIN | LV_STATE_DEFAULT);

        lblIO[i] = lv_label_create(btnIO[i]);
        lv_obj_set_width(lblIO[i], LV_SIZE_CONTENT);   /// 1
        lv_obj_set_height(lblIO[i], LV_SIZE_CONTENT);    /// 1
        lv_obj_set_x(lblIO[i], 0);

        if (numOfOutputs > 8) {
            lv_obj_set_align(lblIO[i], LV_ALIGN_BOTTOM_MID);
            lv_obj_set_y(lblIO[i], btn_height / 2 - 35); // Adjust y position to align at the bottom mid
        }
        else {

            lv_obj_set_y(lblIO[i], 0); // Adjust y position to align at the bottom mid
            lv_obj_set_align(lblIO[i], LV_ALIGN_CENTER);
        }
        lv_obj_set_align(lblIO[i], LV_ALIGN_BOTTOM_MID);
        lv_label_set_text(lblIO[i], lblBtnNames[outputsBuffer[i] - 1]);
        lv_obj_set_style_text_color(lblIO[i], lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_opa(lblIO[i], 255, LV_PART_MAIN | LV_STATE_DEFAULT);
        lv_obj_set_style_text_font(lblIO[i], &lv_font_montserrat_18, LV_PART_MAIN | LV_STATE_DEFAULT);

        imgIO[i] = lv_img_create(btnIO[i]);
        lv_img_set_src(imgIO[i], get_image_for_button(outputsBuffer[i] - 1));
        lv_obj_set_width(imgIO[i], LV_SIZE_CONTENT);   /// 1
        lv_obj_set_height(imgIO[i], LV_SIZE_CONTENT);    /// 1
        lv_obj_set_x(imgIO[i], 0);
        lv_obj_set_y(imgIO[i], 5);
        if(numOfOutputs > 8) {
            lv_obj_set_align(imgIO[i], LV_ALIGN_TOP_MID);
        }
        else {
            lv_obj_set_align(imgIO[i], LV_ALIGN_CENTER);
        }
        
        lv_obj_add_flag(imgIO[i], LV_OBJ_FLAG_ADV_HITTEST);     /// Flags
        lv_obj_clear_flag(imgIO[i], LV_OBJ_FLAG_SCROLLABLE);      /// Flags
        // Add event callback for button click
        lv_obj_add_event_cb(btnIO[i], button_events, LV_EVENT_CLICKED, (void*)i);
    }


        for (int i = 0; i < numOfDims; i++) {
        // Create the slider
        sldDims[i] = lv_slider_create(ui_pnlSensors);
        //lv_slider_set_value(sldDims[i], dimsBuffer[i], LV_ANIM_OFF);
        if (lv_slider_get_mode(sldDims[i]) == LV_SLIDER_MODE_RANGE) {
            lv_slider_set_left_value(sldDims[i], 0, LV_ANIM_OFF);
        }
        lv_obj_set_width(sldDims[i], 305);
        lv_obj_set_height(sldDims[i], 14);
        lv_obj_set_x(sldDims[i], -6);
        lv_obj_set_y(sldDims[i], -56 + i * 40); // Adjust y position dynamically
        lv_obj_set_align(sldDims[i], LV_ALIGN_CENTER);

        lv_obj_set_style_pad_left(sldDims[i], 15, LV_PART_KNOB | LV_STATE_PRESSED);
        lv_obj_set_style_pad_right(sldDims[i], 15, LV_PART_KNOB | LV_STATE_PRESSED);
        lv_obj_set_style_pad_top(sldDims[i], 15, LV_PART_KNOB | LV_STATE_PRESSED);
        lv_obj_set_style_pad_bottom(sldDims[i], 15, LV_PART_KNOB | LV_STATE_PRESSED);

        // Create the label
        lblDims[i] = lv_label_create(ui_pnlSensors);
        lv_obj_set_width(lblDims[i], LV_SIZE_CONTENT);
        lv_obj_set_height(lblDims[i], LV_SIZE_CONTENT);
        lv_obj_set_x(lblDims[i], -135);
        lv_obj_set_y(lblDims[i], -77 + i * 40); // Adjust y position dynamically
        lv_obj_set_align(lblDims[i], LV_ALIGN_RIGHT_MID);
        lv_label_set_text_fmt(lblDims[i], "%s:", lblBtnNames[dimsBuffer[i] - 1]);
    }





    for (int i = 0; i < 16; i++) {
        if (i < numOfOutputs) {
            lv_obj_add_state(switches[i], LV_STATE_CHECKED); // Check the switch
            lv_dropdown_set_selected(dropdowns[i], outputsBuffer[i] - 1); // Set the dropdown value
            lv_obj_add_flag(dropdowns[i], LV_OBJ_FLAG_CLICKABLE); // Make the dropdown clickable
        } else{
            lv_obj_clear_state(switches[i], LV_STATE_CHECKED); // Uncheck the switch
            lv_obj_clear_flag(switches[i], LV_OBJ_FLAG_CLICKABLE); // Make the switch non-clickable
            lv_dropdown_set_selected(dropdowns[i], 0); // Reset the dropdown value
            lv_obj_clear_flag(dropdowns[i], LV_OBJ_FLAG_CLICKABLE); // Make the dropdown non-clickable
        }
    }
    
    // Ensure only the last checkbox is clickable
    for (int i = 0; i < 16; i++) {
        if (i == numOfOutputs - 1) {
            lv_obj_add_flag(switches[i], LV_OBJ_FLAG_CLICKABLE); // Make the last switch clickable
            lv_obj_add_flag(switches[i + 1], LV_OBJ_FLAG_CLICKABLE);
            break;
        } else {
            lv_obj_clear_flag(switches[i], LV_OBJ_FLAG_CLICKABLE);
        }
    }


    //Function to apply sensorsBuffer to checkboxes
    for (int i = 0; i < 5; i++) {
        if (sensorsBuffer[i] == 1) {
            lv_obj_add_state(checkboxes[i], LV_STATE_CHECKED); // Check the checkbox
        } else {
            lv_obj_clear_state(checkboxes[i], LV_STATE_CHECKED); // Uncheck the checkbox
        }
    }



    //Dims buffer apply to panel settings
    for (int i = 0; i < 4; i++) {
        if (i < numOfDims) {
            lv_obj_add_state(dimcheckboxes[i], LV_STATE_CHECKED); // Check the switch
            lv_dropdown_set_selected(dimdropdowns[i], dimsBuffer[i] - 1); // Set the dropdown value
            lv_obj_add_flag(dimdropdowns[i], LV_OBJ_FLAG_CLICKABLE); // Make the dropdown clickable
        } else {
            lv_obj_clear_state(dimcheckboxes[i], LV_STATE_CHECKED); // Uncheck the switch
            lv_obj_clear_flag(dimcheckboxes[i], LV_OBJ_FLAG_CLICKABLE); // Make the switch non-clickable
            lv_dropdown_set_selected(dimdropdowns[i], 0); // Reset the dropdown value
            lv_obj_clear_flag(dimdropdowns[i], LV_OBJ_FLAG_CLICKABLE); // Make the dropdown non-clickable
        }
    }
    
    // Ensure only the last checkbox is clickable
    for (int i = 0; i < 4; i++) {
        if (i == numOfDims - 1) {
            lv_obj_add_flag(dimcheckboxes[i], LV_OBJ_FLAG_CLICKABLE); // Make the last switch clickable
            lv_obj_add_flag(dimcheckboxes[i + 1], LV_OBJ_FLAG_CLICKABLE);
            break;
        } else {
            lv_obj_clear_flag(dimcheckboxes[i], LV_OBJ_FLAG_CLICKABLE);
        }
    }

    if (numOfDims == 0) {
        lv_obj_add_flag(dimcheckboxes[0], LV_OBJ_FLAG_CLICKABLE);
    }
    if (numOfOutputs == 0) {
        lv_obj_add_flag(switches[0], LV_OBJ_FLAG_CLICKABLE);
    }

    set_sensor_panels_coordinates(3, sensorsBuffer);
    if (panelThemeType){
        my_btnBlackThemeFunc();
    }
    else{
        my_btnThemeWhiteFunc();
    }
    apply_theme_settings();

}


// Function to set the image source based on connection status
void set_device_image(bool connected) {
    static bool condition = false;
    if (connected) {
        lv_obj_clear_flag(ui_imgsconnected, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_imgsnotconnected, LV_OBJ_FLAG_HIDDEN);
        condition = true;
    } else {
        lv_obj_clear_flag(ui_imgsnotconnected, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_imgsconnected, LV_OBJ_FLAG_HIDDEN);
        if (condition && !connected)
        {
            condition = false;
            lv_obj_clear_flag(ui_pnlConnectionLost, LV_OBJ_FLAG_HIDDEN);     /// Flags
            lv_obj_move_foreground(ui_pnlConnectionLost);
            lv_label_set_text(ui_Label1, "IO Module Connection Lost!");
        }
        
    }  
}


// Function to set the image source based on connection status
void set_bluetooth_icon(bool connected) {
    static bool bcondition = false;
    if (connected) {
        lv_obj_add_flag(ui_imgBluetoothNotConnected, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(ui_imgBluetoothConnected, LV_OBJ_FLAG_HIDDEN);  
        bcondition = true;
    } else {
        lv_obj_clear_flag(ui_imgBluetoothNotConnected, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(ui_imgBluetoothConnected, LV_OBJ_FLAG_HIDDEN);
        if (bcondition && !connected)
        {
            bcondition = false;
            lv_obj_clear_flag(ui_pnlConnectionLost, LV_OBJ_FLAG_HIDDEN);     /// Flags
            lv_obj_move_foreground(ui_pnlConnectionLost);
            lv_label_set_text(ui_Label1, "Bluetooth Connection Lost!");
        }
    }
}


int initBarCounter = 0;
int initCounter = 0;
int scrMode = 0;
static void init_timer(lv_timer_t * timer) {
    if (initBarCounter < 21) {
        lv_bar_set_value(ui_brInit, initBarCounter * 5, LV_ANIM_OFF);   
    }
    initBarCounter++;
}

// Timer callback function
static void timer_updateTimer_callback(lv_timer_t * timer) {
    if (initCounter < 3) {
        if (scrMode == 0) {
            scrMode = 1;
        }
    }
    else {
        if (scrMode == 1) {
            lv_scr_load(ui_scrMain);
            scrMode = 0;
        }
        initCounter = 11;
        // Your code here, e.g., update display with new data
        const uint16_t* regs_data = getSlavesRegsData();
        update_display_with_data((const uint8_t*)regs_data, 70);
    }
    initCounter++;

}


static void wallpaper_update_timer_callback(lv_timer_t * timer) {
    if (panelWallpaperEnable) {
        if (panelWallpaperEnableCounter == panelWallpaperTime) {
            lv_scr_load(ui_scrWallpaper);

        }
        panelWallpaperEnableCounter++;
    }
    else {
        panelWallpaperEnableCounter = 0;
    }
    
}

// Function to set the button color based on the value
void set_button_color(lv_obj_t *btn, uint16_t value, int connected) {
    if (connected == 0) {
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x5F5F5F), LV_PART_MAIN | LV_STATE_DEFAULT); // Gray
        return;
    }
    if (value == 1) {
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x37C600), LV_PART_MAIN | LV_STATE_DEFAULT); // Green
    } else if (value == 0) {
        lv_obj_set_style_bg_color(btn, lv_color_hex(0x5A5A5A), LV_PART_MAIN | LV_STATE_DEFAULT); // Gray
    } else if (value == 2) {
        lv_obj_set_style_bg_color(btn, lv_color_hex(0xC60000), LV_PART_MAIN | LV_STATE_DEFAULT); // Red
    }
}

void get_data_json_format(const uint16_t* regs_data, int txPacketType, char** json_str)  {

    // Call create_json_data_packet function
    *json_str = create_json_data_packet(regs_data, numOfOutputs, numOfDims, numOfSensors, slaveConnectionStatus, panelThemeType, numberOfNotifications, notifications);
    // Check if notifications is not NULL before deleting
    if (notifications != NULL) {
        cJSON_Delete(notifications);
        notifications = NULL; // Set to NULL after deletion to avoid dangling pointer
    }
} 





// Function to update display with new data
void update_display_with_data(const uint8_t *data, int length) {
    const uint16_t* regs_data = (const uint16_t*)data;

    // Fetch data from registers
    uint16_t analog_input_1 = get_analog_input(0);
    uint16_t analog_input_2 = get_analog_input(1);
    uint16_t analog_input_3 = get_analog_input(2);
    uint16_t analog_input_4 = get_analog_input(3);
    uint16_t analog_input_5 = get_analog_input(4);
    batarya_volt = get_voltage() / 100.0;
    float amper = regs_data[CURRENT_INDIS] / 100.0;

    // Convert voltage to string with comma
    char batarya_volt_str[20];
    if (batarya_volt < 7.0 || batarya_volt > 16.0) {
        //ESP_LOGW(TAG, "Voltage out of range: %.2fV", batarya_volt);
        snprintf(batarya_volt_str, sizeof(batarya_volt_str), "Fail");
    } else {
        int before_comma_volt = (int)batarya_volt;
        int after_comma_volt = (int)((batarya_volt - before_comma_volt) * 100);
        snprintf(batarya_volt_str, sizeof(batarya_volt_str), "Battery: %d,%02dV", before_comma_volt, after_comma_volt);
    }

    // Convert current to string with comma
    char amper_str[10];
    if (amper < 0 || amper > 100.0) {
        //ESP_LOGW(TAG, "Current out of range: %.2fA", amper);
        snprintf(amper_str, sizeof(amper_str), "Fail");
    } else {
        int before_comma_amper = (int)amper;
        int after_comma_amper = (int)((amper - before_comma_amper) * 100);
        snprintf(amper_str, sizeof(amper_str), "%d,%02dA", before_comma_amper, after_comma_amper);
    }


    // Update the display labels with the fetched data
    lv_label_set_text_fmt(ui_lblPnlGrup1SicaklikDeger1, "%d°C", analog_input_4);
    lv_label_set_text_fmt(ui_lblPnlGrup1SicaklikDeger2, "%d°C", analog_input_5);
    lv_label_set_text_fmt(ui_lblGrup1Oran1, "%d%%", analog_input_1);
    lv_label_set_text_fmt(ui_lblGrup1Oran2, "%d%%", analog_input_2);
    lv_label_set_text_fmt(ui_lblGrup1Oran3, "%d%%", analog_input_3);


    // Update the arcs with the fetched data
    lv_arc_set_value(ui_arcGrup1, analog_input_1);
    lv_arc_set_value(ui_arcGrup2, analog_input_2);
    lv_arc_set_value(ui_arcGrup3, analog_input_3);


    // Check Modbus connection status and update device image
    //bool Deviceconnected = get_modbus_connection_status();
    bool Deviceconnected = get_canbus_connection_status();
    set_device_image(Deviceconnected);

    bool btConnected = get_connection_status();
    set_bluetooth_icon(btConnected);

    if(!btConnected){
        //If there is no connection, the time and date will not be updated. Only Battery voltage will be updated.
        lv_label_set_text(ui_lblDateAndTime, batarya_volt_str);
        lv_obj_add_flag(ui_imgWForecast, LV_OBJ_FLAG_HIDDEN);     /// Flags
    }
    else{
        char* converted_json_data;
        get_data_json_format(regs_data, 0, &converted_json_data);
        set_converted_json_data(converted_json_data);
    }



    parse_ble_data((const char*)get_spp_cmd_buff());
    reset_spp_cmd_buff();


    // Ensure create_dynamic_ui is called only once
    static int ui_initialized = 0;
    if (ui_initialized==2) {
        create_dynamic_ui(ui_scrMain);
        ui_initialized = 3;
    }
    if (ui_initialized > 3 && Deviceconnected) {
        for (int i = 0; i < numOfOutputs; i++) {
            set_button_color(btnIO[i], (get_outputs() >> i)&0x01, Deviceconnected);
        }
        for (int i = 0; i < numOfDims; i++) {
            lv_slider_set_value(sldDims[i], get_dimmable_output(i), LV_ANIM_OFF);
        }
    }
    ui_initialized++;
}


//######################################################################################################################




//###################################### JSON DATA PACKET FUNCTIONS ####################################################

// Example function to create JSON data packet as a C string
char* create_json_data_packet(const uint16_t* regs_data, int numOfOutputs, int numOfDims, int numOfSensors, bool slaveConnectionStatus, int themeType, int numberOfNotifications, cJSON* notifications) {
    // Create a JSON object
    cJSON *json = cJSON_CreateObject();

    // Add number of outputs, dims, sensors, slave connection status, and theme type to the JSON object
    cJSON_AddStringToObject(json, "slvConn", slaveConnectionStatus ? "Yes" : "No");
    cJSON_AddNumberToObject(json, "numOfOutputs", numOfOutputs);
    cJSON_AddNumberToObject(json, "numOfDims", numOfDims);
    cJSON_AddNumberToObject(json, "numOfSensors", numOfSensors);
    cJSON_AddStringToObject(json, "RGBEnabled", "yes");
    cJSON_AddNumberToObject(json, "Theme", themeType);
    cJSON_AddNumberToObject(json, "volt", batarya_volt);


    // Add outputsBuffer to the JSON object
    cJSON *outputnames = cJSON_CreateIntArray(outputsBuffer, numOfOutputs);
    cJSON_AddItemToObject(json, "outputsNameBuffer", outputnames);

    // Add dimsBuffer to the JSON object
    cJSON *dimnames = cJSON_CreateIntArray(dimsBuffer, numOfDims);
    cJSON_AddItemToObject(json, "DimsNameBuffer", dimnames);

    // Add sensorsBuffer to the JSON object
    cJSON *sensornames = cJSON_CreateIntArray(sensorsBuffer, 5);
    cJSON_AddItemToObject(json, "SensorsEnabledBuffer", sensornames);


    // Fetch outputsBuffer from regs_data
    int buf[16];
    for (int i = 0; i < numOfOutputs; i++) {
        buf[i] = regs_data[OUTPUT_1_INDIS + i];
    }
    cJSON *outputs = cJSON_CreateIntArray(buf, numOfOutputs);
    cJSON_AddItemToObject(json, "outputsDataBuffer", outputs);

    // Fetch dimsBuffer from regs_data
    for (int i = 0; i < numOfDims; i++) {
        buf[i] = regs_data[DIMMABLE_OUTPUT_1_INDIS + i];
    }
    cJSON *dims = cJSON_CreateIntArray(buf, numOfDims);
    cJSON_AddItemToObject(json, "DimsDataBuffer", dims);


    // Fetch sensorsBuffer from regs_data
    for (int i = 0; i < numOfSensors; i++) {
        buf[i] = regs_data[ANALOG_INPUT_1_INDIS + i];
    }
    cJSON *sensors = cJSON_CreateIntArray(buf, numOfSensors);
    cJSON_AddItemToObject(json, "SensorsDataBuffer", sensors);


    // Add rgbBuffer to the JSON object
    cJSON *rgb = cJSON_CreateIntArray(rgbBuffer, 3);
    cJSON_AddItemToObject(json, "RGBDataBuffer", rgb);


    // Convert JSON object to string
    char *json_str = cJSON_PrintUnformatted(json);
    ESP_LOGI("JSON_DATA_PACKET", "%s", json_str);

    // Free the JSON object
    cJSON_Delete(json);

    return json_str; // Caller is responsible for freeing the returned string
}



void show_weather_icon(int index) {
    lv_obj_clear_flag(ui_imgWForecast, LV_OBJ_FLAG_HIDDEN);     /// Flags
    lv_img_set_offset_x(ui_imgWForecast, index );
}


void set_weather_icon(int weather) {
    switch (weather) {
        case WEATHER_SUNNY:
            show_weather_icon(ICON_SUNNY);
            break;
        case WEATHER_PARTLY_SUNNY:
            show_weather_icon(ICON_PARTLY_SUNNY);
            break;
        case WEATHER_THUNDER:
            show_weather_icon(ICON_THUNDER);
            break;
        case WEATHER_RAINY:
            show_weather_icon(ICON_RAINY);
            break;
        case WEATHER_SNOWY:
            show_weather_icon(ICON_SNOWY);
            break;
        case WEATHER_CLOUDY:
            show_weather_icon(ICON_CLOUDY);
            break;
        default:
            ESP_LOGI(TAG, "Unknown weather condition: %d", weather);
            break;
    }
}


// Function to parse BLE data and call the appropriate parsing function
void parse_ble_data(const char* json_data) {
    cJSON* json = cJSON_Parse(json_data);
    if (json == NULL) {
        //ESP_LOGE("PARSE_BLE_DATA", "Invalid JSON data");
        return;
    }

    cJSON* messageType = cJSON_GetObjectItem(json, "MessageType");
    if (messageType == NULL || !cJSON_IsString(messageType)) {
        ESP_LOGE("PARSE_BLE_DATA", "MessageType not found or invalid");
        cJSON_Delete(json);
        return;
    }

    if (strcmp(messageType->valuestring, "Read") == 0) {
        parse_read_data(json);
    } else if (strcmp(messageType->valuestring, "Write") == 0) {
        parse_write_data(json);
    } else if (strcmp(messageType->valuestring, "Configuration") == 0) {
        parse_configuration_data(json);
    } else if (strcmp(messageType->valuestring, "Rules") == 0) {
        parse_rules_data(json);
    } else {
        ESP_LOGE("PARSE_BLE_DATA", "Unknown MessageType: %s", messageType->valuestring);
    }

    cJSON_Delete(json);
}


// Function to parse Read data
void parse_read_data(cJSON* json) {
    cJSON* time = cJSON_GetObjectItem(json, "Time");
    cJSON* date = cJSON_GetObjectItem(json, "Date");
    cJSON* weather = cJSON_GetObjectItem(json, "Weather");
    cJSON* location = cJSON_GetObjectItem(json, "Location");
    cJSON* temper = cJSON_GetObjectItem(json, "Temperature");

    if (time && cJSON_IsString(time)) {
        ESP_LOGI("PARSE_READ_DATA", "Time: %s", time->valuestring);
    }
    if (date && cJSON_IsString(date)) {
        ESP_LOGI("PARSE_READ_DATA", "Date: %s", date->valuestring);
    }
    if (weather && cJSON_IsString(weather)) {
        ESP_LOGI("PARSE_READ_DATA", "Weather: %s", weather->valuestring);
    }
    if (location && cJSON_IsString(location)) {
        ESP_LOGI("PARSE_READ_DATA", "Location: %s", location->valuestring);
    }

    if (temper && cJSON_IsString(temper)) {
        ESP_LOGI("PARSE_READ_DATA", "Temperature: %s", temper->valuestring);
    }

    // Merge time, date, and batarya_volt
    char merged_str[64];
    snprintf(merged_str, sizeof(merged_str), "Date: %s  Time: %s  Battery: %.2fV", date->valuestring, time->valuestring, batarya_volt);

    // Set the combined value to ui_lblDateAndTime
    lv_label_set_text(ui_lblDateAndTime, merged_str);

    snprintf(merged_str, sizeof(merged_str), "%s %s°C", location->valuestring, temper->valuestring);
    lv_label_set_text(ui_lblWeather, merged_str);

    // Set the weather icon based on the weather condition
    if (weather && cJSON_IsString(weather)) {
        if (strcmp(weather->valuestring, "Sunny") == 0) {
            set_weather_icon(WEATHER_SUNNY);
        } else if (strcmp(weather->valuestring, "P. Cloudy") == 0) {
            set_weather_icon(WEATHER_PARTLY_SUNNY);
        } else if (strcmp(weather->valuestring, "Thunder") == 0) {
            set_weather_icon(WEATHER_THUNDER);
        } else if (strcmp(weather->valuestring, "Rainy") == 0) {
            set_weather_icon(WEATHER_RAINY);
        } else if (strcmp(weather->valuestring, "Snowy") == 0) {
            set_weather_icon(WEATHER_SNOWY);
        } else if (strcmp(weather->valuestring, "Cloudy") == 0) {
            set_weather_icon(WEATHER_CLOUDY);
        } else {
            ESP_LOGI("PARSE_READ_DATA", "Unknown weather condition: %s", weather->valuestring);
        }
    }
}



// Function to parse Write data
void parse_write_data(cJSON* json) {
    cJSON* writeDataType = cJSON_GetObjectItem(json, "writeDataType");
    cJSON* writeNo = cJSON_GetObjectItem(json, "writeNo");
    cJSON* writeData = cJSON_GetObjectItem(json, "writeData");
    uint8_t can_data[8] = {0}; // CAN verisi için buffer

    if (writeDataType && cJSON_IsString(writeDataType)) {
        ESP_LOGI("PARSE_WRITE_DATA", "writeDataType: %s", writeDataType->valuestring);
    }
    if (writeNo && cJSON_IsNumber(writeNo)) {
        ESP_LOGI("PARSE_WRITE_DATA", "writeNo: %d", writeNo->valueint);
    }

    if (writeDataType && cJSON_IsString(writeDataType) && writeNo && cJSON_IsNumber(writeNo)) {
        if (strcmp(writeDataType->valuestring, "Output") == 0) {
            if (writeData && cJSON_IsNumber(writeData)) {
                ESP_LOGI("PARSE_WRITE_DATA", "Output Value: %d", writeData->valueint);
                can_data[0] = (uint8_t)writeData->valueint;  // İlk byte veri
                send_can_frame(0x720, can_data);  // Output için CAN ID: 0x720
            }
        } else if (strcmp(writeDataType->valuestring, "Dim") == 0) {
            if (writeData && cJSON_IsNumber(writeData)) {
                ESP_LOGI("PARSE_WRITE_DATA", "Dim Value: %d", writeData->valueint);
                can_data[0] = (uint8_t)writeData->valueint;  // İlk byte veri
                send_can_frame(0x730, can_data);  // Dim için CAN ID: 0x730
            }
        } else if (strcmp(writeDataType->valuestring, "RGB") == 0) {
            cJSON* rgbArray = cJSON_GetObjectItem(json, "writeData");
            if (rgbArray && cJSON_IsArray(rgbArray) && cJSON_GetArraySize(rgbArray) == 3) {
                can_data[0] = (uint8_t)cJSON_GetArrayItem(rgbArray, 0)->valueint; // Red
                can_data[1] = (uint8_t)cJSON_GetArrayItem(rgbArray, 1)->valueint; // Green
                can_data[2] = (uint8_t)cJSON_GetArrayItem(rgbArray, 2)->valueint; // Blue
                ESP_LOGI("PARSE_WRITE_DATA", "RGB Values: R=%d, G=%d, B=%d", can_data[0], can_data[1], can_data[2]);
                send_can_frame(0x740, can_data);  // RGB için CAN ID: 0x740
            } else {
                ESP_LOGE("PARSE_WRITE_DATA", "RGB writeData must be an array of 3 values.");
            }
        } else {
            ESP_LOGI("PARSE_WRITE_DATA", "Unknown writeDataType: %s", writeDataType->valuestring);
        }
    }
}


// Function to parse Configuration data
void parse_configuration_data(cJSON* json) {
    cJSON* numOfOutputs = cJSON_GetObjectItem(json, "numOfOutputs");
    cJSON* outputsNameBuffer = cJSON_GetObjectItem(json, "OutputsNameBuffer");
    cJSON* numOfDims = cJSON_GetObjectItem(json, "numOfDims");
    cJSON* dimsNameBuffer = cJSON_GetObjectItem(json, "DimsNameBuffer");
    cJSON* numOfSensors = cJSON_GetObjectItem(json, "numOfSensors");
    cJSON* sensorsNameBuffer = cJSON_GetObjectItem(json, "SensorsNameBuffer");
    cJSON* rgbEnabled = cJSON_GetObjectItem(json, "RGBEnables");
    cJSON* theme = cJSON_GetObjectItem(json, "Theme");

    int outputsBuf[16] = {0};
    int dimsBuf[4] = {0};
    int sensorsBuf[5] = {0};

    if (numOfOutputs && cJSON_IsNumber(numOfOutputs)) {
        ESP_LOGI("PARSE_CONFIGURATION_DATA", "numOfOutputs: %d", numOfOutputs->valueint);
    }
    if (outputsNameBuffer && cJSON_IsArray(outputsNameBuffer)) {
        int size = cJSON_GetArraySize(outputsNameBuffer);
        ESP_LOGI("PARSE_CONFIGURATION_DATA", "OutputsNameBuffer:");
        for (int i = 0; i < size && i < 16; i++) {
            cJSON* item = cJSON_GetArrayItem(outputsNameBuffer, i);
            if (cJSON_IsNumber(item)) {
                outputsBuf[i] = item->valueint;
                ESP_LOGI("PARSE_CONFIGURATION_DATA", "  %d", item->valueint);
            }
        }
    }
    if (numOfDims && cJSON_IsNumber(numOfDims)) {
        ESP_LOGI("PARSE_CONFIGURATION_DATA", "numOfDims: %d", numOfDims->valueint);
    }
    if (dimsNameBuffer && cJSON_IsArray(dimsNameBuffer)) {
        int size = cJSON_GetArraySize(dimsNameBuffer);
        ESP_LOGI("PARSE_CONFIGURATION_DATA", "DimsNameBuffer:");
        for (int i = 0; i < size && i < 4; i++) {
            cJSON* item = cJSON_GetArrayItem(dimsNameBuffer, i);
            if (cJSON_IsNumber(item)) {
                dimsBuf[i] = item->valueint;
                ESP_LOGI("PARSE_CONFIGURATION_DATA", "  %d", item->valueint);
            }
        }
    }
    if (numOfSensors && cJSON_IsNumber(numOfSensors)) {
        ESP_LOGI("PARSE_CONFIGURATION_DATA", "numOfSensors: %d", numOfSensors->valueint);
    }
    if (sensorsNameBuffer && cJSON_IsArray(sensorsNameBuffer)) {
        int size = cJSON_GetArraySize(sensorsNameBuffer);
        ESP_LOGI("PARSE_CONFIGURATION_DATA", "SensorsNameBuffer:");
        for (int i = 0; i < size && i < 5; i++) {
            cJSON* item = cJSON_GetArrayItem(sensorsNameBuffer, i);
            if (cJSON_IsNumber(item)) {
                sensorsBuf[i] = item->valueint;
                ESP_LOGI("PARSE_CONFIGURATION_DATA", "  %d", item->valueint);
            }
        }
    }
    if (rgbEnabled && cJSON_IsString(rgbEnabled)) {
        ESP_LOGI("PARSE_CONFIGURATION_DATA", "RGBEnables: %s", rgbEnabled->valuestring);
    }
    if (theme && cJSON_IsString(theme)) {
        ESP_LOGI("PARSE_CONFIGURATION_DATA", "Theme: %s", theme->valuestring);
    }

    save_panel_configuration_to_nvs(numOfOutputs->valueint, outputsBuf, numOfSensors->valueint, sensorsBuf, numOfDims->valueint, dimsBuf);
}
// Function to parse Rules data
void parse_rules_data(cJSON* json) {
    cJSON* numOfRules = cJSON_GetObjectItem(json, "NumOfRules");
    cJSON* rules = cJSON_GetObjectItem(json, "Rules");
    cJSON* numOfNotifications = cJSON_GetObjectItem(json, "NumOfNotifications");
    cJSON* notifications = cJSON_GetObjectItem(json, "Notifications");

    if (numOfRules && cJSON_IsNumber(numOfRules)) {
        ESP_LOGI("PARSE_RULES_DATA", "NumOfRules: %d", numOfRules->valueint);
    }
    if (rules && cJSON_IsObject(rules)) {
        ESP_LOGI("PARSE_RULES_DATA", "Rules:");
        cJSON* rule;
        cJSON_ArrayForEach(rule, rules) {
            if (cJSON_IsString(rule)) {
                ESP_LOGI("PARSE_RULES_DATA", "  %s: %s", rule->string, rule->valuestring);
            }
        }
    }
    if (numOfNotifications && cJSON_IsNumber(numOfNotifications)) {
        ESP_LOGI("PARSE_RULES_DATA", "NumOfNotifications: %d", numOfNotifications->valueint);
    }
    if (notifications && cJSON_IsObject(notifications)) {
        ESP_LOGI("PARSE_RULES_DATA", "Notifications:");
        cJSON* notification;
        cJSON_ArrayForEach(notification, notifications) {
            if (cJSON_IsString(notification)) {
                ESP_LOGI("PARSE_RULES_DATA", "  %s: %s", notification->string, notification->valuestring);
            }
        }
    }
}

//######################################################################################################################


  
//############################ NVS FUNCTIONS #########################################################################
// Function to save configuration data to NVS
void save_panel_configuration_to_nvs(int totalOutps, int buffer1[16], int totalSensors, int buffer2[5], int totalDims, int buffer3[4]) {
    // Ensure the values do not exceed the maximum allowed sizes
    if (totalOutps > 16) {
        totalOutps = 16;
    }
    if (totalSensors > 5) {
        totalSensors = 5;
    }
    if (totalDims > 4) {
        totalDims = 4;
    }

    // Save totalOutps to NVS
    nvs_write_int("numOfOutputs", totalOutps);

    // Save buffer1 to NVS
    for (int i = 0; i < 16; i++) {
        if (buffer1[i] < 1 || buffer1[i] > 18) {
            buffer1[i] = 1; // Set to default value if out of range
        }
        char key[16];
        snprintf(key, sizeof(key), "outBuf%d", i);
        nvs_write_int(key, buffer1[i]);
    }

    // Save totalSensors to NVS
    nvs_write_int("numSens", totalSensors);

    // Save buffer2 to NVS
    for (int i = 0; i < 5; i++) {
        if (buffer2[i] < 0 || buffer2[i] > 1) {
            buffer2[i] = 0; // Set to default value if out of range
        }
        char key[16];
        snprintf(key, sizeof(key), "sensBuf%d", i);
        nvs_write_int(key, buffer2[i]);
    }

    // Save totalDims to NVS
    nvs_write_int("numDims", totalDims);

    // Save buffer3 to NVS
    for (int i = 0; i < 4; i++) {
        if (buffer3[i] < 0 || buffer3[i] > 8) {
            buffer3[i] = 0; // Set to default value if out of range
        }
        char key[16];
        snprintf(key, sizeof(key), "dimsBuf%d", i);
        nvs_write_int(key, buffer3[i]);
    }
}

void save_theme_configuration_to_nvs(int16_t* themeType, uint16_t* wallpaperEnabled, uint16_t* wallpaperTimeIndex){


    //Save themeType to NVS
    nvs_write_int("thmTyp", panelThemeType);

    //Save wallpaperEnabled to NVS
    nvs_write_int("wallpEn", panelWallpaperEnable);

    //Save wallpaperTimeIndex to NVS
    nvs_write_int("wllTimI", panelWallpaperTime);


}


// Debug function to print parameters to the screen
void debug_print_configuration(int totalOutpts, int buffer1[16], int totalSensors, int buffer2[5], int totalDims, int buffer3[4]) {
    ESP_LOGI("DEBUG", "totalOutpts: %d", totalOutpts);
    for (int i = 0; i < 16; i++) {
        ESP_LOGI("DEBUG", "buffer1[%d]: %d", i, buffer1[i]);
    }
    ESP_LOGI("DEBUG", "totalSensors: %d", totalSensors);
    for (int i = 0; i < 5; i++) {
        ESP_LOGI("DEBUG", "buffer2[%d]: %d", i, buffer2[i]);
    }
    ESP_LOGI("DEBUG", "totalDims: %d", totalSensors);
    for (int i = 0; i < 4; i++) {
        ESP_LOGI("DEBUG", "buffer3[%d]: %d", i, buffer3[i]);
    }
}

// Function to read configuration data from NVS
void load_panel_configuration_from_nvs(int *totalOutpts, int buffer1[16], int *totalSensors, int buffer2[5], int *totalDims, int buffer3[4]) {
    // Read totalOutpts from NVS
    if (nvs_read_int("numOfOutputs", totalOutpts) != ESP_OK || *totalOutpts < 0 || *totalOutpts > 16) {
        *totalOutpts = 4; // Set to default value if out of range
    }

    // Read buffer1 from NVS
    for (int i = 0; i < 16; i++) {
        char key[16];
        snprintf(key, sizeof(key), "outBuf%d", i);
        if (nvs_read_int(key, &buffer1[i]) != ESP_OK || buffer1[i] < 1 || buffer1[i] > 18) {
            buffer1[i] = 1; // Set to default value if out of range
        }
    }

    // Read totalSensors from NVS
    if (nvs_read_int("numSens", totalSensors) != ESP_OK || *totalSensors < 0 || *totalSensors > 5) {
        *totalSensors = 1; // Set to default value if out of range
    }

    // Read buffer2 from NVS
    for (int i = 0; i < 5; i++) {
        char key[16];
        snprintf(key, sizeof(key), "sensBuf%d", i);
        if (nvs_read_int(key, &buffer2[i]) != ESP_OK || buffer2[i] < 0 || buffer2[i] > 1) {
            buffer2[i] = 1; // Set to default value if out of range
        }
    }

    // Read totalSensors from NVS
    if (nvs_read_int("numDims", totalDims) != ESP_OK || *totalDims < 0 || *totalDims > 4) {
        *totalDims = 1; // Set to default value if out of range
    }

    // Read buffer3 from NVS
    for (int i = 0; i < 4; i++) {
        char key[16];
        snprintf(key, sizeof(key), "dimsBuf%d", i);
        if (nvs_read_int(key, &buffer3[i]) != ESP_OK || buffer3[i] < 1 || buffer3[i] > 8) {
            buffer3[i] = 1; // Set to default value if out of range
        }
    }

}



// Load the theme settings from NVS
void load_theme_configuration_from_nvs(int* themeType, int* wallpaperEnabled, int* wallpaperTimeIndex) {

    // Read themeType from NVS
    if (nvs_read_int("thmTyp", themeType) != ESP_OK || *themeType < 0 || *themeType > 1) {
        *themeType = 1; // Set to default value if out of range
    }


    // Read wallpaperEnabled from NVS
    if (nvs_read_int("wallpEn", wallpaperEnabled) != ESP_OK || *wallpaperEnabled < 0 || *wallpaperEnabled > 1) {
        *wallpaperEnabled = 1; // Set to default value if out of range
    }

    // Read wallpaperEnabled from NVS
    if (nvs_read_int("wllTimI", wallpaperTimeIndex) != ESP_OK || *wallpaperTimeIndex < 0 || *wallpaperTimeIndex > 600) {
        *wallpaperTimeIndex = 30; // Set to default value if out of range
    }


    // Log the loaded configuration
    ESP_LOGI(TAG, "Loaded Theme Configuration: Theme=%d, WallpaperEnabled=%d, WallpaperTimeIndex=%d",
             *themeType, *wallpaperEnabled, *wallpaperTimeIndex);

}



// Function to check switches and get corresponding dropdown values
void check_switches_and_get_dropdown_values() {
    // Reset the outputsBuffer
    memset(outputsBuffer, 0, sizeof(outputsBuffer));

    numOfOutputs = 0; // Initialize numOfOutputs
    // Apply numOfOutputs and outputsBuffer to swO1-swO16 and cbxO1-cbxO16
    lv_obj_t* switches[16] = {ui_swO1, ui_swO2, ui_swO3, ui_swO4, ui_swO5, ui_swO6, ui_swO7, ui_swO8, ui_swO9, ui_swO10, ui_swO11, ui_swO12, ui_swO13, ui_swO14, ui_swO15, ui_swO16};
    lv_obj_t* dropdowns[16] = {ui_cbxO1, ui_cbxO2, ui_cbxO3, ui_cbxO4, ui_cbxO5, ui_cbxO6, ui_cbxO7, ui_cbxO8, ui_cbxO9, ui_cbxO10, ui_cbxO11, ui_cbxO12, ui_cbxO13, ui_cbxO14, ui_cbxO15, ui_cbxO16};


    for (int i = 0; i < 16; i++) {
        if (lv_obj_has_state(switches[i], LV_STATE_CHECKED)) {
            outputsBuffer[i] = 1 + lv_dropdown_get_selected(dropdowns[i]);
            numOfOutputs++; // Increment numOfOutputs for each checked switch
            ESP_LOGI("SWITCH_CHECK", "Switch %d is checked. Dropdown value index: %d", i + 1, outputsBuffer[i]);
        } else {
            outputsBuffer[i] = 0; // Indicate that the switch is not checked
        }
    }

}


// Function to check switches and get corresponding dropdown values
void check_switches_and_get_dropdown_values_for_dims() {
    // Reset the outputsBuffer
    memset(dimsBuffer, 0, sizeof(dimsBuffer));

    numOfDims = 0; // Initialize numOfOutputs
    // Apply numOfOutputs and outputsBuffer to swO1-swO16 and cbxO1-cbxO16
    lv_obj_t* dimcheckboxes[4] = {ui_swDim1, ui_swDim2, ui_swDim3, ui_swDim4};
    lv_obj_t* dimdropdowns[4] = {ui_cbxDim1, ui_cbxDim2, ui_cbxDim3, ui_cbxDim4};

    for (int i = 0; i < 4; i++) {
        if (lv_obj_has_state(dimcheckboxes[i], LV_STATE_CHECKED)) {
            dimsBuffer[i] = 1 + lv_dropdown_get_selected(dimdropdowns[i]);
            numOfDims++; // Increment numOfOutputs for each checked switch
            ESP_LOGI("SWITCH_CHECK", "Switch %d is checked. Dropdown value index: %d", i + 1, dimsBuffer[i]);
        } else {
            dimsBuffer[i] = 0; // Indicate that the switch is not checked
        }
    }

}


// Function to check the state of the first 5 switches and update sensorsBuffer
void check_sensors_and_update_buffer() {
    // Reset the sensorsBuffer
    memset(sensorsBuffer, 0, sizeof(sensorsBuffer));

    numOfSensors = 0; // Initialize numOfSensors

    lv_obj_t* switches[5] = {ui_Checkbox1, ui_Checkbox2, ui_Checkbox3, ui_Checkbox4, ui_Checkbox5};

    for (int i = 0; i < 5; i++) {
        if (lv_obj_has_state(switches[i], LV_STATE_CHECKED)) {
            sensorsBuffer[i] = 1; // Indicate that the switch is checked
            numOfSensors++; // Increment numOfSensors for each checked switch
        } else {
            sensorsBuffer[i] = 0; // Indicate that the switch is not checked
        }
        ESP_LOGI("SWITCH_CHECK", "Switch %d is checked. Value: %d", i + 1, sensorsBuffer[i]);
    }

    ESP_LOGI("SWITCH_CHECK", "Total number of checked switches: %d", numOfSensors);
}

void save_panel_settings()
{
    check_switches_and_get_dropdown_values();
    check_sensors_and_update_buffer();
    check_switches_and_get_dropdown_values_for_dims();
    save_panel_configuration_to_nvs(numOfOutputs, outputsBuffer, numOfSensors, sensorsBuffer, numOfDims, dimsBuffer);
    // Save the panel settings to NVS
    ESP_LOGI(TAG, "##### Panel Settings Saved Successfully! #####");
}

void save_theme_settings()
{
    uint16_t selected_index = 0;
    static char selected_text[32];  // Buffer to store text

    selected_index = lv_roller_get_selected(ui_rlrTime);
    // Get the selected item text
    lv_roller_get_selected_str(ui_rlrTime, selected_text, sizeof(selected_text));
    // Check if switch is enabled (ON)
    if (lv_obj_has_state(ui_swEnableWallpaper, LV_STATE_CHECKED)) {
        // Get the selected roller item index
        panelWallpaperEnable = 1;  // Enable wallpaper
        // Print the selected roller item
        ESP_LOGI(TAG, "Wallpaper Enabled, Selected Time: %s-----index = %d", selected_text, selected_index);
    } else {
        panelWallpaperEnable = 0;  // Disable wallpaper
        ESP_LOGI(TAG, "Wallpaper Disabled");
    }

    // Set panelWallpaperTime based on the selected index
    switch (selected_index) {
        case 0: { panelWallpaperTime = 30;  break;}
        case 1: { panelWallpaperTime = 60;  break;}
        case 2: { panelWallpaperTime = 120; break;}
        case 3: { panelWallpaperTime = 300; break;}
        case 4: { panelWallpaperTime = 600; break;}
        default:
            {
                ESP_LOGW(TAG, "Invalid roller index: %d, setting default to 30", selected_index);
                panelWallpaperTime = 30;  // Default if index is out of range
                break;
            }
    }
    ESP_LOGI(TAG, "Wallpaper Enabled, Selected Time: %s-----index = %d panelThemeType =%d panelWallpaperEnable =%d panelWallpaperTime =%d ", selected_text, selected_index,
             panelThemeType, panelWallpaperEnable, panelWallpaperTime);
    save_theme_configuration_to_nvs(panelThemeType, panelWallpaperEnable, panelWallpaperTime);
}


void apply_theme_settings()
{
    // Apply theme enabled status to the switch
    if (panelWallpaperEnable) {
        lv_obj_add_state(ui_swEnableWallpaper, LV_STATE_CHECKED);
        lv_obj_clear_flag(ui_rlrTime, LV_OBJ_FLAG_HIDDEN);     /// Flags
        lv_obj_clear_flag(ui_lblRolllerTime, LV_OBJ_FLAG_HIDDEN);     /// Flags
    } else {
        lv_obj_clear_state(ui_swEnableWallpaper, LV_STATE_CHECKED);
        lv_obj_add_flag(ui_rlrTime, LV_OBJ_FLAG_HIDDEN);     /// Flags
        lv_obj_add_flag(ui_lblRolllerTime, LV_OBJ_FLAG_HIDDEN);     /// Flags
    }

    // Map panelWallpaperTime to the roller index
    uint16_t roller_index = 0;

    switch (panelWallpaperTime) {
        case 30:  roller_index = 0; break;
        case 60:  roller_index = 1; break;
        case 120: roller_index = 2; break;
        case 300: roller_index = 3; break;
        case 600: roller_index = 4; break;
        default:
            ESP_LOGW(TAG, "Invalid panelWallpaperTime: %d, setting default to 30s", panelWallpaperTime);
            roller_index = 0;  // Default to first option
            break;
    }

    // Set roller selection
    lv_roller_set_selected(ui_rlrTime, roller_index, LV_ANIM_OFF);

    // Log applied settings
    ESP_LOGI(TAG, " ############################Applied Theme Settings:panelThemeType = %d panelWallpaperEnable=%d, WallpaperTimeIndex=%d",panelThemeType, panelWallpaperEnable, roller_index);
}



// Callback function for color changes
 void color_wheel_event_cb() {
    lv_color_t selected_color = lv_colorwheel_get_rgb(ui_Colorwheel1); // Get selected color
    // Apply selected color to the panel background
    lv_obj_set_style_bg_color(ui_btnRGBColor, selected_color, LV_PART_MAIN | LV_STATE_DEFAULT);
}
//######################################################################################################################



//###################################### Display Manager ################################################################
void display_manager_init() {
  
    static lv_disp_draw_buf_t disp_buf; // contains internal graphic buffer(s) called draw buffer(s)
    static lv_disp_drv_t disp_drv;      // contains callback functions



#if CONFIG_EXAMPLE_AVOID_TEAR_EFFECT_WITH_SEM
    ESP_LOGI(TAG, "Create semaphores");
    sem_vsync_end = xSemaphoreCreateBinary();
    assert(sem_vsync_end);
    sem_gui_ready = xSemaphoreCreateBinary();
    assert(sem_gui_ready);
#endif

#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
#endif

    ESP_LOGI(TAG, "Install RGB LCD panel driver");
    esp_lcd_panel_handle_t panel_handle = NULL;
    esp_lcd_rgb_panel_config_t panel_config = {
        .data_width = 16, // RGB565 in parallel mode, thus 16bit in width
        .psram_trans_align = 64,
        .num_fbs = EXAMPLE_LCD_NUM_FB,
#if CONFIG_EXAMPLE_USE_BOUNCE_BUFFER
        .bounce_buffer_size_px = 10 * EXAMPLE_LCD_H_RES,
#endif
        .clk_src = LCD_CLK_SRC_DEFAULT,
        .disp_gpio_num = EXAMPLE_PIN_NUM_DISP_EN,
        .pclk_gpio_num = EXAMPLE_PIN_NUM_PCLK,
        .vsync_gpio_num = EXAMPLE_PIN_NUM_VSYNC,
        .hsync_gpio_num = EXAMPLE_PIN_NUM_HSYNC,
        .de_gpio_num = EXAMPLE_PIN_NUM_DE,
        .data_gpio_nums = {
            EXAMPLE_PIN_NUM_DATA0,
            EXAMPLE_PIN_NUM_DATA1,
            EXAMPLE_PIN_NUM_DATA2,
            EXAMPLE_PIN_NUM_DATA3,
            EXAMPLE_PIN_NUM_DATA4,
            EXAMPLE_PIN_NUM_DATA5,
            EXAMPLE_PIN_NUM_DATA6,
            EXAMPLE_PIN_NUM_DATA7,
            EXAMPLE_PIN_NUM_DATA8,
            EXAMPLE_PIN_NUM_DATA9,
            EXAMPLE_PIN_NUM_DATA10,
            EXAMPLE_PIN_NUM_DATA11,
            EXAMPLE_PIN_NUM_DATA12,
            EXAMPLE_PIN_NUM_DATA13,
            EXAMPLE_PIN_NUM_DATA14,
            EXAMPLE_PIN_NUM_DATA15,
        },
        .timings = {
            .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
            .h_res = EXAMPLE_LCD_H_RES,
            .v_res = EXAMPLE_LCD_V_RES,
            // The following parameters should refer to LCD spec
            .hsync_back_porch = 8,
            .hsync_front_porch = 8,
            .hsync_pulse_width = 4,
            .vsync_back_porch = 16,
            .vsync_front_porch = 16,
            .vsync_pulse_width = 4,
            .flags.pclk_active_neg = true,
        },
        .flags.fb_in_psram = true, // allocate frame buffer in PSRAM
    };
    ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));

    ESP_LOGI(TAG, "Register event callbacks");
    esp_lcd_rgb_panel_event_callbacks_t cbs = {
        .on_vsync = example_on_vsync_event,
    };
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(panel_handle, &cbs, &disp_drv));

    ESP_LOGI(TAG, "Initialize RGB LCD panel");
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));

#if EXAMPLE_PIN_NUM_BK_LIGHT >= 0
    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);
#endif

    ESP_ERROR_CHECK(i2c_master_init());
    ESP_LOGI(TAG, "I2C initialized successfully");
    gpio_init();

    uint8_t write_buf = 0x01;
    i2c_master_write_to_device(I2C_MASTER_NUM, 0x24, &write_buf, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    //Reset the touch screen. It is recommended that you reset the touch screen before using it.
    write_buf = 0x2C;
    i2c_master_write_to_device(I2C_MASTER_NUM, 0x38, &write_buf, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    esp_rom_delay_us(100 * 1000);

    gpio_set_level(GPIO_INPUT_IO_4,0);
    esp_rom_delay_us(100 * 1000);

    write_buf = 0x2E;
    i2c_master_write_to_device(I2C_MASTER_NUM, 0x38, &write_buf, 1, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    esp_rom_delay_us(200 * 1000);
    
    esp_lcd_touch_handle_t tp = NULL;
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;

    ESP_LOGI(TAG, "Initialize I2C");

    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();

    ESP_LOGI(TAG, "Initialize touch IO (I2C)");
    /* Touch IO handle */
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)I2C_MASTER_NUM, &tp_io_config, &tp_io_handle));
    esp_lcd_touch_config_t tp_cfg = {
        .x_max = EXAMPLE_LCD_V_RES,
        .y_max = EXAMPLE_LCD_H_RES,
        .rst_gpio_num = -1,
        .int_gpio_num = -1,
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
    };
    /* Initialize touch */
    ESP_LOGI(TAG, "Initialize touch controller GT911");
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, &tp));

    ESP_LOGI(TAG, "Initialize LVGL library");
    lv_init();
    void *buf1 = NULL;
    void *buf2 = NULL;
#if CONFIG_EXAMPLE_DOUBLE_FB
    ESP_LOGI(TAG, "Use frame buffers as LVGL draw buffers");
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_get_frame_buffer(panel_handle, 2, &buf1, &buf2));
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * EXAMPLE_LCD_V_RES);
#else
    ESP_LOGI(TAG, "Allocate separate LVGL draw buffers from PSRAM");
    buf1 = heap_caps_malloc(EXAMPLE_LCD_H_RES * 100 * sizeof(lv_color_t), MALLOC_CAP_SPIRAM);
    assert(buf1);
    // initialize LVGL draw buffers
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, EXAMPLE_LCD_H_RES * 100);
#endif // CONFIG_EXAMPLE_DOUBLE_FB

    ESP_LOGI(TAG, "Register display driver to LVGL");
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = EXAMPLE_LCD_H_RES;
    disp_drv.ver_res = EXAMPLE_LCD_V_RES;
    disp_drv.flush_cb = example_lvgl_flush_cb;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = panel_handle;
#if CONFIG_EXAMPLE_DOUBLE_FB
    disp_drv.full_refresh = true; // the full_refresh mode can maintain the synchronization between the two frame buffers
#endif
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);

    ESP_LOGI(TAG, "Install LVGL tick timer");
    // Tick interface for LVGL (using esp_timer to generate 2ms periodic event)
    const esp_timer_create_args_t lvgl_tick_timer_args = {
        .callback = &example_increase_lvgl_tick,
        .name = "lvgl_tick"
    };

    static lv_indev_drv_t indev_drv;    // Input device driver (Touch)
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.disp = disp;
    indev_drv.read_cb = example_lvgl_touch_cb;
    indev_drv.user_data = tp;

    lv_indev_drv_register(&indev_drv);

    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000));

    lv_timer_t * updateScreentimer = lv_timer_create(timer_updateTimer_callback, 500, NULL);
    lv_timer_t * wallpaperTimer = lv_timer_create(wallpaper_update_timer_callback, 1000, NULL);
    lv_timer_t * initTim = lv_timer_create(init_timer, 100, NULL);
    


    load_panel_configuration_from_nvs(&numOfOutputs, outputsBuffer, &numOfSensors, sensorsBuffer, &numOfDims, dimsBuffer);
    load_theme_configuration_from_nvs(&panelThemeType, &panelWallpaperEnable, &panelWallpaperTime);




    lvgl_mux = xSemaphoreCreateRecursiveMutex();
    assert(lvgl_mux);
    ESP_LOGI(TAG, "Create LVGL task");
    xTaskCreate(example_lvgl_port_task, "LVGL", EXAMPLE_LVGL_TASK_STACK_SIZE, NULL, EXAMPLE_LVGL_TASK_PRIORITY, NULL);



    ESP_LOGI(TAG, "Display LVGL Scatter Chart");
    // Lock the mutex due to the LVGL APIs are not thread-safe
    if (example_lvgl_lock(-1)) {
         //example_lvgl_demo_ui(disp);
        //lv_demo_widgets();
        // lv_demo_benchmark();
        // lv_demo_music();
        // lv_demo_stress();
        // Release the mutex
        ui_init();
        lv_scr_load(ui_scrInit);
        example_lvgl_unlock();
    }
}