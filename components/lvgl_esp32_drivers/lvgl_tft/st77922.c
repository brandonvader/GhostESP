/**
 * @file st77922.c
 */

#include "st77922.h"

#include <string.h>

#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_st77922.h"
#include "esp_log.h"
#include "sdkconfig.h"

#define TAG "st77922"

#define ST77922_HOST SPI2_HOST

/* The panel is natively 320x480 portrait. It honours MADCTL.MY but ignores the
 * axis swap (MV), so the landscape UI is rotated in software while flushing.
 * Its dual-gate drive also needs draw windows aligned to 4 pixels on both axes.
 * Partial LVGL areas rarely are, so they are merged into a native-orientation
 * shadow frame in PSRAM and an aligned superset is sent from there. */
#define ST77922_NATIVE_W CONFIG_TFT_HEIGHT
#define ST77922_NATIVE_H CONFIG_TFT_WIDTH

#define ST77922_ALIGN 4

/* DMA staging buffer (internal RAM), in pixels. */
#define ST77922_SCRATCH_PX 8192

static const st77922_lcd_init_cmd_t s_init_cmds[] = {
#include "st77922_init.inc"
};

static esp_lcd_panel_handle_t s_panel;
static uint16_t *s_scratch;
static uint16_t *s_shadow; /* ST77922_NATIVE_W x ST77922_NATIVE_H */
static SemaphoreHandle_t s_tx_done;

static bool st77922_color_done(esp_lcd_panel_io_handle_t io, esp_lcd_panel_io_event_data_t *edata, void *ctx)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_tx_done, &woken);
    return woken == pdTRUE;
}

void st77922_init(void)
{
    s_tx_done = xSemaphoreCreateBinary();
    s_scratch = heap_caps_malloc(ST77922_SCRATCH_PX * sizeof(uint16_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    s_shadow = heap_caps_calloc(ST77922_NATIVE_W * ST77922_NATIVE_H, sizeof(uint16_t), MALLOC_CAP_SPIRAM);
    ESP_ERROR_CHECK(s_tx_done && s_scratch && s_shadow ? ESP_OK : ESP_ERR_NO_MEM);

    const int max_transfer = ST77922_SCRATCH_PX * sizeof(uint16_t);
    const spi_bus_config_t bus_cfg = ST77922_PANEL_BUS_QSPI_CONFIG(
        CONFIG_LV_ST77922_PIN_CLK, CONFIG_LV_ST77922_PIN_D0, CONFIG_LV_ST77922_PIN_D1,
        CONFIG_LV_ST77922_PIN_D2, CONFIG_LV_ST77922_PIN_D3, max_transfer);
    ESP_ERROR_CHECK(spi_bus_initialize(ST77922_HOST, &bus_cfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_spi_config_t io_cfg =
        ST77922_PANEL_IO_QSPI_CONFIG(CONFIG_LV_ST77922_PIN_CS, st77922_color_done, NULL);
    io_cfg.pclk_hz = CONFIG_LV_ST77922_CLOCK_HZ;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)ST77922_HOST, &io_cfg, &io));

    st77922_vendor_config_t vendor_cfg = {
        .init_cmds = s_init_cmds,
        .init_cmds_size = sizeof(s_init_cmds) / sizeof(s_init_cmds[0]),
        .flags = {
            .use_qspi_interface = 1,
        },
    };
    const esp_lcd_panel_dev_config_t panel_cfg = {
        /* The panel's reset line is tied to CHIP_PU, so there is no GPIO to pulse. */
        .reset_gpio_num = -1,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .vendor_config = &vendor_cfg,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st77922(io, &panel_cfg, &s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(s_panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));
    ESP_LOGI(TAG, "ST77922 QSPI panel up (%dx%d landscape, %d Hz)", CONFIG_TFT_WIDTH, CONFIG_TFT_HEIGHT,
             CONFIG_LV_ST77922_CLOCK_HZ);
}

void st77922_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    const uint16_t *src = (const uint16_t *)color_map;
    const int w = area->x2 - area->x1 + 1;
    const int h = area->y2 - area->y1 + 1;

    /* Rotate into the shadow frame and work out the touched native rectangle. */
    int nx0, nx1, ny0, ny1; /* inclusive */
#if defined(CONFIG_LV_ST77922_ROTATE_270)
    /* native X = y, native Y = last column - x */
    nx0 = area->y1;
    nx1 = area->y2;
    ny0 = ST77922_NATIVE_H - 1 - area->x2;
    ny1 = ST77922_NATIVE_H - 1 - area->x1;
    for (int ny = ny0; ny <= ny1; ny++) {
        const uint16_t *col = src + (ST77922_NATIVE_H - 1 - ny - area->x1);
        uint16_t *dst = s_shadow + ny * ST77922_NATIVE_W + nx0;
        for (int i = 0; i < h; i++) {
            dst[i] = col[i * w];
        }
    }
#else
    /* native X = last row - y, native Y = x */
    nx0 = ST77922_NATIVE_W - 1 - area->y2;
    nx1 = ST77922_NATIVE_W - 1 - area->y1;
    ny0 = area->x1;
    ny1 = area->x2;
    for (int ny = ny0; ny <= ny1; ny++) {
        const uint16_t *col = src + (ny - area->x1);
        uint16_t *dst = s_shadow + ny * ST77922_NATIVE_W + nx0;
        for (int i = 0; i < h; i++) {
            dst[i] = col[(h - 1 - i) * w];
        }
    }
#endif

    /* Grow to the 4-pixel grid; the panel is 320x480 so the grid fits exactly. */
    const int xa = nx0 & ~(ST77922_ALIGN - 1);
    const int xb = (nx1 + ST77922_ALIGN) & ~(ST77922_ALIGN - 1);
    const int ya = ny0 & ~(ST77922_ALIGN - 1);
    const int yb = (ny1 + ST77922_ALIGN) & ~(ST77922_ALIGN - 1);
    const int rect_w = xb - xa;
    int rows_per = (ST77922_SCRATCH_PX / rect_w) & ~(ST77922_ALIGN - 1);

    for (int y = ya; y < yb; y += rows_per) {
        const int n = (y + rows_per < yb) ? rows_per : yb - y;
        for (int r = 0; r < n; r++) {
            memcpy(s_scratch + r * rect_w, s_shadow + (y + r) * ST77922_NATIVE_W + xa, rect_w * sizeof(uint16_t));
        }
        esp_err_t err = esp_lcd_panel_draw_bitmap(s_panel, xa, y, xb, y + n, s_scratch);
        if (err != ESP_OK || xSemaphoreTake(s_tx_done, pdMS_TO_TICKS(100)) != pdTRUE) {
            ESP_LOGE(TAG, "flush failed: %s", esp_err_to_name(err));
            break;
        }
    }
    lv_disp_flush_ready(drv);
}
