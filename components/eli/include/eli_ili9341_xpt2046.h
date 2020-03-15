
#pragma once

typedef struct {
    uint8_t mosi;
    uint8_t miso;
    uint8_t clk;
    uint8_t ili9341_cs;
    uint8_t xpt2046_cs;
    uint8_t dc;
    int8_t rst;
    int8_t bckl;
    uint8_t irq;
    uint16_t x_min;
    uint16_t y_min;
    uint16_t x_max;
    uint16_t y_max;
    spi_host_device_t spi_host;
} eli_ili9341_xpt2046_config_t;

void eli_ili9341_xpt2046_initialize(eli_ili9341_xpt2046_config_t *new_config);
