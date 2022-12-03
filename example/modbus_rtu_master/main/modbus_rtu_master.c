#include "stdio.h"
#include "stdint.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_vfs.h"
#include "esp_vfs_dev.h"
#include "driver/uart.h"
#include "esp_log.h"

#include "modbus.h"

static const char *TAG = "modbus_rtu_master";


#define TEST_MBM_SLAVE_ID (1)
#define TEST_MBM_START_REGISTER (0x0001)
#define TEST_MBM_REGISTER_COUNT (0x0002)
#define TEST_MBM_BAUDRATE (9600)
#define TEST_MBM_UART UART_NUM_2
#define TEST_MBM_UART_TXD (17)
#define TEST_MBM_UART_RXD (16)

void app_main(void)
{
    int ret;
    modbus_t *ctx = NULL;

    uint16_t holdingRegisters[TEST_MBM_REGISTER_COUNT * 2] = {};

    const uart_config_t uart_config = {
        .baud_rate = TEST_MBM_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };

    if (uart_driver_install(TEST_MBM_UART, 256, 256, 0, NULL, 0) != ESP_OK) {
        ESP_LOGE(TAG, "uart_driver_install  failed");
        goto APP_END;
    }
    if (uart_param_config(TEST_MBM_UART, &uart_config) != ESP_OK) {
        ESP_LOGE(TAG, "uart_param_config failed");
        goto APP_END;
    }
    if (uart_set_pin(TEST_MBM_UART, TEST_MBM_UART_TXD, TEST_MBM_UART_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
        ESP_LOGE(TAG, "uart_set_pin failed");
        goto APP_END;
    }

    esp_vfs_dev_uart_use_driver(TEST_MBM_UART);
    // esp_vfs_dev_uart_use_nonblocking(UART_NUM_2);

    ctx = modbus_new_rtu("/dev/uart/2", TEST_MBM_BAUDRATE, 'N', 8, 1);
    if (ctx == NULL)
    {
        ESP_LOGE(TAG, "Unable to create the libmodbus context");
        goto APP_END;
    }
    modbus_set_response_timeout(ctx, 1, 0);

    ret = modbus_set_slave(ctx, TEST_MBM_SLAVE_ID);
    if (ret < 0)
    {
        ESP_LOGE(TAG, "modbus_set_slave error");
        goto APP_END;
    }

    /*	ret = modbus_rtu_set_serial_mode(ctx, MODBUS_RTU_RS232);
        if(ret < 0){
            perror("modbus_rtu_set_serial_mode error\n");
            return;
        }
    */
    ret = modbus_connect(ctx);
    if (ret < 0)
    {
        ESP_LOGE(TAG, "modbus_connect error");
        goto APP_END;
    }

    for (;;)
    {
        ret = modbus_read_input_registers(ctx, TEST_MBM_START_REGISTER, TEST_MBM_REGISTER_COUNT, holdingRegisters);
        if (ret < 0)
            ESP_LOGE(TAG, "modbus_read_regs error %d", ret);
        else
        {
            ESP_LOGI(TAG, "modbus_read_regs success");
            for (uint i = 0; i < TEST_MBM_REGISTER_COUNT; i++)
                ESP_LOGI(TAG, "holdingRegisters[%u] = %u", i, holdingRegisters[i]);
        }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

APP_END:
    modbus_close(ctx);
    modbus_free(ctx);

    ESP_LOGW(TAG, "scheduler crashed, restarting esp");
    esp_restart();
}