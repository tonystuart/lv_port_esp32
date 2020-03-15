/* ESP-IDF LittlevGL Interface (ELI) for ILI9341 and XPT2046 with a shared SPI bus
 *
 * This code is derived from littlevgl/lv_port_esp32 which
 * references the following license and copyright statement:

 * MIT License
 *
 * Copyright (c) 2019 Littlev Graphics Library
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "lvgl/lvgl.h"
#include "esp_freertos_hooks.h"
#include "eli_ili9341_xpt2046.h"

#define TAG "ELI"

#define $ ESP_ERROR_CHECK

#define DISP_BUF_SIZE (LV_HOR_RES_MAX * 40)

#define CMD_X_READ  0b10010000
#define CMD_Y_READ  0b11010000

#define XPT2046_AVG 4

#define DATA_MODE     0x01u
#define COMMAND_MODE  0x02u
#define FLUSH_READY   0x04u

#define LOW 0
#define CONCURRENT_SPI_TRANSACTIONS 5

typedef struct {
    uint8_t command;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

static spi_device_handle_t ili9341_spi;
static spi_device_handle_t xpt2046_spi;

static int16_t avg_buf_x[XPT2046_AVG];
static int16_t avg_buf_y[XPT2046_AVG];
static uint8_t avg_last;

static eli_ili9341_xpt2046_config_t config;

static void configure_gpio(uint8_t pin, gpio_mode_t mode)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = 1UL << pin,
        .mode = mode,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_PIN_INTR_DISABLE,
    };
    $(gpio_config(&io_conf));
}

static void IRAM_ATTR pre_cb(spi_transaction_t *transaction)
{
    if ((uint32_t)transaction->user & DATA_MODE) {
        $(gpio_set_level(config.dc, 1));	 // data mode
    } else {
        $(gpio_set_level(config.dc, 0));	 // command mode
    }
}

static void IRAM_ATTR post_cb(spi_transaction_t *transaction)
{
    if((uint32_t)transaction->user & FLUSH_READY) {
        lv_disp_t *disp = lv_refr_get_disp_refreshing();
        lv_disp_flush_ready(&disp->driver);
    }
}

static void disp_spi_send(uint8_t *data, uint16_t length, uint32_t flags)
{
    if (!length) {
        return;
    }

    static uint8_t next_transaction = 0;
    static spi_transaction_t transactions[CONCURRENT_SPI_TRANSACTIONS];

    spi_transaction_t *t = &transactions[next_transaction];
    memset(t, 0, sizeof(spi_transaction_t));
    t->length = length * 8;
    t->tx_buffer = data;
    t->user = (void*)flags;
    $(spi_device_queue_trans(ili9341_spi, t, portMAX_DELAY));
    next_transaction = (next_transaction + 1) % CONCURRENT_SPI_TRANSACTIONS;
}

static void ili9341_send_command(uint8_t command)
{
    disp_spi_send(&command, 1, COMMAND_MODE);
}

static void ili9341_send_data(void *data, uint16_t length)
{
    disp_spi_send(data, length, DATA_MODE);
}

static void ili9341_send_color(void *data, uint16_t length)
{
    disp_spi_send(data, length, DATA_MODE | FLUSH_READY);
}

static void ili9341_init(void)
{
    lcd_init_cmd_t commands[]={
        {0xCF, {0x00, 0x83, 0X30}, 3},
        {0xED, {0x64, 0x03, 0X12, 0X81}, 4},
        {0xE8, {0x85, 0x01, 0x79}, 3},
        {0xCB, {0x39, 0x2C, 0x00, 0x34, 0x02}, 5},
        {0xF7, {0x20}, 1},
        {0xEA, {0x00, 0x00}, 2},
        {0xC0, {0x26}, 1},			/*Power control*/
        {0xC1, {0x11}, 1},			/*Power control */
        {0xC5, {0x35, 0x3E}, 2},	/*VCOM control*/
        {0xC7, {0xBE}, 1},			/*VCOM control*/
        {0x36, {0x28}, 1},			/*Memory Access Control*/
        {0x3A, {0x55}, 1},			/*Pixel Format Set*/
        {0xB1, {0x00, 0x1B}, 2},
        {0xF2, {0x08}, 1},
        {0x26, {0x01}, 1},
        {0xE0, {0x1F, 0x1A, 0x18, 0x0A, 0x0F, 0x06, 0x45, 0X87, 0x32, 0x0A, 0x07, 0x02, 0x07, 0x05, 0x00}, 15},
        {0XE1, {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3A, 0x78, 0x4D, 0x05, 0x18, 0x0D, 0x38, 0x3A, 0x1F}, 15},
        {0x2A, {0x00, 0x00, 0x00, 0xEF}, 4},
        {0x2B, {0x00, 0x00, 0x01, 0x3f}, 4},
        {0x2C, {0}, 0},
        {0xB7, {0x07}, 1},
        {0xB6, {0x0A, 0x82, 0x27, 0x00}, 4},
        {0x11, {0}, 0x80},
        {0x29, {0}, 0x80},
        {0, {0}, 0xff},
    };

    //Initialize non-SPI GPIOs
    configure_gpio(config.dc, GPIO_MODE_OUTPUT);

    //Reset the display
    if (config.rst != -1) {
        configure_gpio(config.rst, GPIO_MODE_OUTPUT);
        $(gpio_set_level(config.rst, 0));
        vTaskDelay(100 / portTICK_RATE_MS);
        $(gpio_set_level(config.rst, 1));
        vTaskDelay(100 / portTICK_RATE_MS);
    }

    ESP_LOGI(TAG, "Initializing ili9341");

    //Send all the commands
    uint16_t index = 0;
    while (commands[index].databytes != 0xff) {
        ili9341_send_command(commands[index].command);
        ili9341_send_data(commands[index].data, commands[index].databytes & 0x1F);
        if (commands[index].databytes & 0x80) {
            vTaskDelay(100 / portTICK_RATE_MS);
        }
        index++;
    }

    //Turn on the display
    if (config.bckl != -1) {
        configure_gpio(config.bckl, GPIO_MODE_OUTPUT);
        $(gpio_set_level(config.bckl, 1));
    }
}

static void ili9341_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    uint8_t data[4];

    /*Column addresses*/
    ili9341_send_command(0x2A);
    data[0] = (area->x1 >> 8) & 0xFF;
    data[1] = area->x1 & 0xFF;
    data[2] = (area->x2 >> 8) & 0xFF;
    data[3] = area->x2 & 0xFF;
    ili9341_send_data(data, 4);

    /*Page addresses*/
    ili9341_send_command(0x2B);
    data[0] = (area->y1 >> 8) & 0xFF;
    data[1] = area->y1 & 0xFF;
    data[2] = (area->y2 >> 8) & 0xFF;
    data[3] = area->y2 & 0xFF;
    ili9341_send_data(data, 4);

    /*Memory write*/
    ili9341_send_command(0x2C);

    uint32_t size = lv_area_get_width(area) *lv_area_get_height(area);

    ili9341_send_color((void*)color_map, size * 2);
}

static void IRAM_ATTR lv_tick_task(void) {
    lv_tick_inc(portTICK_RATE_MS);
}

static void tp_spi_xchg(uint8_t data_send[], uint8_t data_recv[], uint8_t byte_count)
{
    spi_transaction_t t = {
        .length = byte_count * 8, // SPI transaction length is in bits
        .tx_buffer = data_send,
        .rx_buffer = data_recv};

    $(spi_device_transmit(xpt2046_spi, &t));
}

static void xpt2046_init(void)
{
    configure_gpio(config.irq, GPIO_MODE_INPUT);
}

static void xpt2046_corr(int16_t *x, int16_t *y)
{
    if (*x > config.x_min) {
        *x -= config.x_min;
    }
    else {
        *x = 0;
    }

    if (*y > config.y_min) {
        *y -= config.y_min;
    }
    else {
        *y = 0;
    }

    *x = (uint32_t)((uint32_t)*x * LV_HOR_RES) / (config.x_max - config.x_min);
    *y = (uint32_t)((uint32_t)*y * LV_VER_RES) / (config.y_max - config.y_min);

    *x =  LV_HOR_RES - *x;
    *y =  LV_VER_RES - *y;
}

static void xpt2046_avg(int16_t *x, int16_t *y)
{
    /*Shift out the oldest data*/
    uint8_t i;
    for(i = XPT2046_AVG - 1; i > 0 ; i--) {
        avg_buf_x[i] = avg_buf_x[i - 1];
        avg_buf_y[i] = avg_buf_y[i - 1];
    }

    /*Insert the new point*/
    avg_buf_x[0] = *x;
    avg_buf_y[0] = *y;
    if(avg_last < XPT2046_AVG) avg_last++;

    /*Sum the x and y coordinates*/
    int32_t x_sum = 0;
    int32_t y_sum = 0;
    for(i = 0; i < avg_last ; i++) {
        x_sum += avg_buf_x[i];
        y_sum += avg_buf_y[i];
    }

    /*Normalize the sums*/
    *x = (int32_t)x_sum / avg_last;
    *y = (int32_t)y_sum / avg_last;
}

static bool xpt2046_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    static int16_t last_x = 0;
    static int16_t last_y = 0;
    bool valid = true;

    int16_t x = 0;
    int16_t y = 0;

    uint8_t data_ready = gpio_get_level(config.irq) == LOW;

    if (data_ready) {
        uint8_t data_send[] = {
            CMD_X_READ,
            0,
            CMD_Y_READ,
            0,
            0
        };
        uint8_t data_recv[sizeof(data_send)] = {};
        tp_spi_xchg(data_send, data_recv, sizeof(data_send));
        x = data_recv[1] << 8 | data_recv[2];
        y = data_recv[3] << 8 | data_recv[4];

        /*Normalize Data*/
        x = x >> 3;
        y = y >> 3;
        xpt2046_corr(&x, &y);
        xpt2046_avg(&x, &y);
        last_x = x;
        last_y = y;

    } else {
        x = last_x;
        y = last_y;
        avg_last = 0;
        valid = false;
    }

    data->point.x = x;
    data->point.y = y;
    data->state = valid == false ? LV_INDEV_STATE_REL : LV_INDEV_STATE_PR;

    return false;
}

static void configure_shared_spi_bus()
{
    spi_bus_config_t buscfg = {
        .miso_io_num = config.miso,
        .mosi_io_num = config.mosi,
        .sclk_io_num = config.clk,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = DISP_BUF_SIZE * 2,
    };

    $(spi_bus_initialize(config.spi_host, &buscfg, 1));

    spi_device_interface_config_t ili9341_config = {
        .clock_speed_hz = 20 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = config.ili9341_cs,
        .queue_size = 1,
        .pre_cb = pre_cb,
        .post_cb = post_cb
    };

    $(spi_bus_add_device(config.spi_host, &ili9341_config, &ili9341_spi));
    ili9341_init();

    spi_device_interface_config_t xpt2046_config = {
        .clock_speed_hz = 2 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = config.xpt2046_cs,
        .queue_size = 1,
        .pre_cb = NULL,
        .post_cb = NULL,
    };

    $(spi_bus_add_device(config.spi_host, &xpt2046_config, &xpt2046_spi));
    xpt2046_init();
}

void eli_ili9341_xpt2046_initialize(eli_ili9341_xpt2046_config_t *new_config)
{
    config = *new_config;

    lv_init();

    configure_shared_spi_bus();
    static lv_color_t buf1[DISP_BUF_SIZE];
    static lv_color_t buf2[DISP_BUF_SIZE];
    static lv_disp_buf_t disp_buf;
    lv_disp_buf_init(&disp_buf, buf1, buf2, DISP_BUF_SIZE);

    lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.flush_cb = ili9341_flush;
    disp_drv.buffer = &disp_buf;
    lv_disp_drv_register(&disp_drv);

    lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.read_cb = xpt2046_read;
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    lv_indev_drv_register(&indev_drv);

    esp_register_freertos_tick_hook(lv_tick_task);
}

