/* Example to show use of custom ili9341 / xpt2046 shared SPI bus interface.
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "lvgl/lvgl.h"
#include "lv_examples/lv_apps/demo/demo.h"
#include "eli_ili9341_xpt2046.h"

#define TAG "MAIN"


void app_main()
{
    assert(CONFIG_LVGL_TOUCH_SPI_MOSI == CONFIG_LVGL_DISP_SPI_MOSI);
    assert(CONFIG_LVGL_TOUCH_SPI_CLK == CONFIG_LVGL_DISP_SPI_CLK);

    eli_ili9341_xpt2046_config_t new_config = {
        .mosi = CONFIG_LVGL_TOUCH_SPI_MOSI,
        .clk = CONFIG_LVGL_TOUCH_SPI_CLK,
        .ili9341_cs = CONFIG_LVGL_DISP_SPI_CS,
        .xpt2046_cs = CONFIG_LVGL_TOUCH_SPI_CS,
        .dc = CONFIG_LVGL_DISP_PIN_DC,
        .rst = CONFIG_LVGL_DISP_PIN_RST,
        .bckl = CONFIG_LVGL_DISP_PIN_BCKL,
        .miso = CONFIG_LVGL_TOUCH_SPI_MISO,
        .irq = CONFIG_LVGL_TOUCH_PIN_IRQ,
        .x_min = CONFIG_LVGL_TOUCH_X_MIN,
        .y_min = CONFIG_LVGL_TOUCH_Y_MIN,
        .x_max = CONFIG_LVGL_TOUCH_X_MAX,
        .y_max = CONFIG_LVGL_TOUCH_Y_MAX,
        .spi_host = HSPI_HOST,
    };

    eli_ili9341_xpt2046_initialize(&new_config);

    demo_create();

    while (1) {
        vTaskDelay(1);
        lv_task_handler();
    }
}

