
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "lvgl/lvgl.h"
#include "lv_examples/lv_apps/demo/demo.h"
#include "esp_freertos_hooks.h"

#include "tp_spi.h"
#include "xpt2046.h"
#include "tp_i2c.h"
#include "ft6x36.h"

#include "driver/spi_master.h"
#include <freertos/semphr.h>

#define TAG "MAIN"

#define DISP_SPI_MOSI CONFIG_LVGL_DISP_SPI_MOSI
#define DISP_SPI_CLK CONFIG_LVGL_DISP_SPI_CLK
#define DISP_SPI_CS CONFIG_LVGL_DISP_SPI_CS

#define DISP_BUF_SIZE (LV_HOR_RES_MAX * 40)
#define ILI9341_DC   CONFIG_LVGL_DISP_PIN_DC
#define ILI9341_RST  CONFIG_LVGL_DISP_PIN_RST
#define ILI9341_BCKL CONFIG_LVGL_DISP_PIN_BCKL

// if text/images are backwards, try setting this to 1
#define ILI9341_INVERT_DISPLAY CONFIG_LVGL_INVERT_DISPLAY

#define DATA_MODE     0x01u
#define COMMAND_MODE  0x02u
#define FLUSH_READY   0x04u

typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

static spi_device_handle_t ili9341_spi;

static void configure_gpio_output(uint8_t pin)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 1 << pin;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
}

static void ili9341_enable_backlight(bool backlight)
{
    gpio_set_level(ILI9341_BCKL, backlight);
}

static void IRAM_ATTR pre_cb(spi_transaction_t *trans)
{
    if ((uint32_t)trans->user & DATA_MODE) {
        gpio_set_level(ILI9341_DC, 1);	 // data mode
    } else {
        gpio_set_level(ILI9341_DC, 0);	 // command mode
    }
}

static void IRAM_ATTR post_cb(spi_transaction_t *trans)
{
    if((uint32_t)trans->user & FLUSH_READY) {
        lv_disp_t * disp = lv_refr_get_disp_refreshing();
        lv_disp_flush_ready(&disp->driver);
    }
}

#define MAX_CONCURRENT_TRANSACTIONS 5

static uint8_t next_transaction = 0;
static spi_transaction_t transactions[MAX_CONCURRENT_TRANSACTIONS];

static void disp_spi_send(uint8_t *data, uint16_t length, uint32_t flags)
{
    if (!length) {
        return;
    }

    spi_transaction_t *t = &transactions[next_transaction];
    memset(t, 0, sizeof(spi_transaction_t));
    t->length = length * 8;
    t->tx_buffer = data;
    t->user = (void*)flags;
    spi_device_queue_trans(ili9341_spi, t, portMAX_DELAY);
    next_transaction = (next_transaction + 1) % MAX_CONCURRENT_TRANSACTIONS;
}

static void ili9341_send_cmd(uint8_t cmd)
{
    disp_spi_send(&cmd, 1, COMMAND_MODE);
}

static void ili9341_send_data(void * data, uint16_t length)
{
    disp_spi_send(data, length, DATA_MODE);
}

static void ili9341_send_color(void * data, uint16_t length)
{
    disp_spi_send(data, length, DATA_MODE | FLUSH_READY);
}

static void ili9341_init(void)
{
    lcd_init_cmd_t ili_init_cmds[]={
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
    configure_gpio_output(ILI9341_DC);
    configure_gpio_output(ILI9341_RST);
    configure_gpio_output(ILI9341_BCKL);

    //Reset the display
    gpio_set_level(ILI9341_RST, 0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(ILI9341_RST, 1);
    vTaskDelay(100 / portTICK_RATE_MS);

    ESP_LOGI(TAG, "sending ili9341 initialization commands");

    //Send all the commands
    uint16_t cmd = 0;
    while (ili_init_cmds[cmd].databytes!=0xff) {
        ili9341_send_cmd(ili_init_cmds[cmd].cmd);
        ili9341_send_data(ili_init_cmds[cmd].data, ili_init_cmds[cmd].databytes&0x1F);
        if (ili_init_cmds[cmd].databytes & 0x80) {
            vTaskDelay(100 / portTICK_RATE_MS);
        }
        cmd++;
    }

    ESP_LOGI(TAG, "ili9341 initialization commands sent");

    ili9341_enable_backlight(true);
}

static void ili9341_flush(lv_disp_drv_t * drv, const lv_area_t * area, lv_color_t * color_map)
{
    uint8_t data[4];

    /*Column addresses*/
    ili9341_send_cmd(0x2A);
    data[0] = (area->x1 >> 8) & 0xFF;
    data[1] = area->x1 & 0xFF;
    data[2] = (area->x2 >> 8) & 0xFF;
    data[3] = area->x2 & 0xFF;
    ili9341_send_data(data, 4);

    /*Page addresses*/
    ili9341_send_cmd(0x2B);
    data[0] = (area->y1 >> 8) & 0xFF;
    data[1] = area->y1 & 0xFF;
    data[2] = (area->y2 >> 8) & 0xFF;
    data[3] = area->y2 & 0xFF;
    ili9341_send_data(data, 4);

    /*Memory write*/
    ili9341_send_cmd(0x2C);

    uint32_t size = lv_area_get_width(area) * lv_area_get_height(area);

    ili9341_send_color((void*)color_map, size * 2);
}

static void IRAM_ATTR lv_tick_task(void) {
    lv_tick_inc(portTICK_RATE_MS);
}

static void configure_shared_spi_bus()
{
    assert(TP_SPI_MOSI == DISP_SPI_MOSI);
    assert(TP_SPI_CLK == DISP_SPI_CLK);

    spi_bus_config_t buscfg = {
        .miso_io_num = TP_SPI_MISO,
        .mosi_io_num = TP_SPI_MOSI,
        .sclk_io_num = TP_SPI_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = DISP_BUF_SIZE * 2,
    };

    esp_err_t ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    assert(ret == ESP_OK);

    spi_device_interface_config_t ili9341_config = {
        .clock_speed_hz = 20 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = DISP_SPI_CS,
        .queue_size = 1,
        .pre_cb = pre_cb,
        .post_cb = post_cb
    };

    ret = spi_bus_add_device(HSPI_HOST, &ili9341_config, &ili9341_spi);
    assert(ret == ESP_OK);

    ili9341_init();

    spi_device_interface_config_t xpt2046_config = {
        .clock_speed_hz = 2 * 1000 * 1000,
        .mode = 0,
        .spics_io_num = TP_SPI_CS,
        .queue_size = 1,
        .pre_cb = NULL,
        .post_cb = NULL,
    };

    tp_spi_add_device_config(HSPI_HOST, &xpt2046_config);
    xpt2046_init();
}

void app_main() {
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

    demo_create();

    while (1) {
        vTaskDelay(1);
        lv_task_handler();
    }
}

