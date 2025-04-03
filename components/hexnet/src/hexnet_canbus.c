#include "hexnet_canbus.h"
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/twai.h"



#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<20) | (1ULL<<19))

/* --------------------- Definitions and static variables ------------------ */
//Example Configuration
#define TX_GPIO_NUM             20
#define RX_GPIO_NUM             19
#define EXAMPLE_TAG             "TWAI Master"

static bool driver_installed = false;
#define POLLING_RATE_MS 1000

static const twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
static const twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_LISTEN_ONLY);




#define FRAME_1_ID  0x100
#define FRAME_2_ID  0x200
#define FRAME_3_ID  0x300

static const char *TAG = "CAN_HANDLER";

// Global variables to store received data
uint16_t voltage = 0;
uint16_t outputs = 0;
uint16_t inputs = 0;
uint8_t analog_inputs[5] = {0};
uint8_t dimmable_outputs[4] = {0};
uint8_t rgb_values[3] = {0};
uint8_t canbusConnection = 0;


// Getter Functions
uint16_t get_voltage() {
    return voltage;
}

uint16_t get_outputs() {
    return outputs;
}

uint16_t get_inputs() {
    return inputs;
}

uint8_t get_analog_input(uint8_t index) {
    if (index < 5) {
        return analog_inputs[index];
    }
    return 0;  // Hatalı index
}

uint8_t get_dimmable_output(uint8_t index) {
    if (index < 4) {
        return dimmable_outputs[index];
    }
    return 0;  // Hatalı index
}

uint8_t get_rgb_value(uint8_t index) {
    if (index < 3) {
        return rgb_values[index];
    }
    return 0;  // Hatalı index
}

uint8_t get_canbus_connection_status() {
    return canbusConnection;
}

// Function to send a CAN frame
void send_can_frame(uint32_t id, uint8_t *data) {
    twai_message_t message;
    message.identifier = id;
    message.rtr = 0;  // Data frame
    message.data_length_code = 8;  // Max CAN data length is 8 bytes
    memcpy(message.data, data, 8);

    // Send the message over the CAN bus
    esp_err_t res = twai_transmit(&message, pdMS_TO_TICKS(100));  // 100 ms timeout
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "Failed to send frame ID: 0x%03X", (unsigned int)id);
    }
}

void handle_rx_message(twai_message_t message) {
    
    switch (message.identifier) {
        case FRAME_1_ID:
            // Assign to global variables
            voltage = (message.data[0] << 8) | message.data[1];
            outputs = (message.data[2] << 8) | message.data[3];
            inputs = (message.data[4] << 8) | message.data[5];
            break;
        case FRAME_2_ID:
            // Assign to global arrays
            for (int i = 0; i < 5; i++) {
                analog_inputs[i] = message.data[i];
                
            }

            break;
        case FRAME_3_ID:
            // Assign to global RGB array
            dimmable_outputs[0] = message.data[0];
            dimmable_outputs[1] = message.data[1];
            dimmable_outputs[2] = message.data[2];
            dimmable_outputs[3] = message.data[3];
            rgb_values[0] = message.data[4];
            rgb_values[1] = message.data[5];
            rgb_values[2] = message.data[6];
            break;
        default:
            break;
    }

}

int twaiCounter = 0;
void twai_task(void *pvParameter)
{
    twai_message_t message;
    
    while(1)
    {
        if (!driver_installed) {
            // Driver not installed
            vTaskDelay(pdMS_TO_TICKS(1000));
            return;
        }
        // Check if alert happened
        uint32_t alerts_triggered;
        twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));
        twai_status_info_t twaistatus;
        twai_get_status_info(&twaistatus);

        // Handle alerts
        if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
            //ESP_LOGI(EXAMPLE_TAG,"Alert: TWAI controller has become error passive.");
        }
        if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
            //ESP_LOGI(EXAMPLE_TAG,"Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
            //ESP_LOGI(EXAMPLE_TAG,"Bus error count: %"PRIu32, twaistatus.bus_error_count);
        }

        if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
            //ESP_LOGI(EXAMPLE_TAG,"Alert: The RX queue is full causing a received frame to be lost.");
            //ESP_LOGI(EXAMPLE_TAG,"RX buffered: %"PRIu32, twaistatus.msgs_to_rx);
            //ESP_LOGI(EXAMPLE_TAG,"RX missed: %"PRIu32, twaistatus.rx_missed_count);
            //ESP_LOGI(EXAMPLE_TAG,"RX overrun %"PRIu32, twaistatus.rx_overrun_count);
        }

        // Check if message is received
        if (alerts_triggered & TWAI_ALERT_RX_DATA) {
            // One or more messages received. Handle all.
            
            if (twai_receive(&message, 0) == ESP_OK) {
                handle_rx_message(message);
                canbusConnection = 1;
            }
        }
        else {
            canbusConnection = 0;
        }  
        vTaskDelay(pdMS_TO_TICKS(100));

        if(twaiCounter == 100) {    
            twaiCounter = 0;
    }
    twaiCounter++;
    }
}

void twai_ini(void)
{
    // Install TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        ESP_LOGI(EXAMPLE_TAG,"Driver installed");
    } else {
        ESP_LOGE(EXAMPLE_TAG,"Failed to install driver");
        return;
    }
    
    // Start TWAI driver
    if (twai_start() == ESP_OK) {
        ESP_LOGI(EXAMPLE_TAG,"Driver started");
    } else {
        ESP_LOGE(EXAMPLE_TAG,"Failed to start driver");
        return;
    }

    // Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states
    uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
    if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
        ESP_LOGI(EXAMPLE_TAG,"CAN Alerts reconfigured");
    } else {
        ESP_LOGE(EXAMPLE_TAG,"Failed to reconfigure alerts");
        return;
    }

    // TWAI driver is now successfully installed and started
    driver_installed = true;
 
    //define twai task
    xTaskCreate(twai_task, "twai_task", 2048, NULL, 5, NULL);
}
