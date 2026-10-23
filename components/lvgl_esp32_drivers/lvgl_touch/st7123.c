/**
 * @file st7123.c
 *
 * Register map as used by Espressif's esp_lcd_touch_st7123 and ESPHome's
 * st7123 platform; only the first touch point is reported to LVGL.
 */

#include "st7123.h"

#include <stdlib.h>

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl_i2c/i2c_manager.h"
#include "sdkconfig.h"

#define TAG "ST7123"

#define ST7123_REG_MAX_X_H 0x0005
#define ST7123_REG_ADV_INFO 0x0010
#define ST7123_REG_REPORT0 0x0014

#define ST7123_ADV_WITH_COORD (1u << 3)
#define ST7123_REPORT_VALID 0x80
#define ST7123_REPORT_X_H_MASK 0x3F
#define ST7123_REPORT_LEN 7
#define ST7123_MAX_TOUCHES 5

/* A report that lands this far (Manhattan, in pixels) from the held position
 * during a debounce hold is a new touch, not the same finger. */
#define ST7123_NEW_TOUCH_DISTANCE 24
/* Once a touch has travelled this far from where it started it is a drag; a
 * fast scroll can move further than that between reports and must not be split. */
#define ST7123_DRAG_DISTANCE 12

static bool s_inited;
static uint8_t s_max_touches = ST7123_MAX_TOUCHES;
static int16_t s_last_x;
static int16_t s_last_y;
static int64_t s_last_valid_us;
static bool s_pressed;
static bool s_holding; /* last poll reported "pressed" only because of the debounce hold */
static int16_t s_start_x; /* where the current touch began */
static int16_t s_start_y;
static bool s_dragging;

static esp_err_t st7123_read_reg(uint16_t reg, uint8_t *buf, uint16_t len)
{
    return lvgl_i2c_read(CONFIG_LV_I2C_TOUCH_PORT, CONFIG_LV_ST7123_I2C_ADDRESS, reg | I2C_REG_16, buf, len);
}

static void st7123_hardware_reset(void)
{
#if CONFIG_LV_ST7123_RESET_PIN >= 0
    const gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << CONFIG_LV_ST7123_RESET_PIN,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&cfg));
    gpio_set_level(CONFIG_LV_ST7123_RESET_PIN, 0);
    vTaskDelay(pdMS_TO_TICKS(20));
    gpio_set_level(CONFIG_LV_ST7123_RESET_PIN, 1);
    /* The controller does not answer for a few hundred ms after reset. */
    vTaskDelay(pdMS_TO_TICKS(400));
#endif
}

void st7123_init(void)
{
    s_inited = false;
    esp_err_t ret = lvgl_i2c_init(CONFIG_LV_I2C_TOUCH_PORT);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(ret));
        return;
    }
    st7123_hardware_reset();

    uint8_t info[5];
    ret = st7123_read_reg(ST7123_REG_MAX_X_H, info, sizeof(info));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "probe at 0x%02x failed: %s", CONFIG_LV_ST7123_I2C_ADDRESS, esp_err_to_name(ret));
        return;
    }
    if (info[4] >= 1 && info[4] <= ST7123_MAX_TOUCHES) {
        s_max_touches = info[4];
    }
    s_inited = true;
    ESP_LOGI(TAG, "touch controller found: max %ux%u, %u touches", ((unsigned)info[0] << 8) | info[1],
             ((unsigned)info[2] << 8) | info[3], info[4]);
}

/* No valid report this poll. The controller drops reports for ~30-60 ms in the
 * middle of a steady touch, so stay "pressed" for the debounce window. */
static bool st7123_no_touch(lv_indev_data_t *data, int64_t now_us)
{
    if (s_pressed && (now_us - s_last_valid_us) < (int64_t)CONFIG_LV_ST7123_RELEASE_HOLD_MS * 1000) {
        data->state = LV_INDEV_STATE_PR;
        s_holding = true;
    } else {
        s_pressed = false;
        s_holding = false;
    }
    return false;
}

bool st7123_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    data->point.x = s_last_x;
    data->point.y = s_last_y;
    data->state = LV_INDEV_STATE_REL;
    if (!s_inited) {
        return false;
    }

    const int64_t now_us = esp_timer_get_time();
    uint8_t adv = 0;
    if (st7123_read_reg(ST7123_REG_ADV_INFO, &adv, 1) != ESP_OK || !(adv & ST7123_ADV_WITH_COORD)) {
        return st7123_no_touch(data, now_us);
    }

    /* The controller only refreshes its report when the whole buffer (every
     * touch slot) is read in one burst, as esp_lcd_touch_st7123 does. Reading a
     * single slot returns a stale first report. */
    uint8_t buf[ST7123_REPORT_LEN * ST7123_MAX_TOUCHES];
    if (st7123_read_reg(ST7123_REG_REPORT0, buf, ST7123_REPORT_LEN * s_max_touches) != ESP_OK) {
        return st7123_no_touch(data, now_us);
    }
    const uint8_t *rep = NULL;
    for (int i = 0; i < s_max_touches; i++) {
        if (buf[i * ST7123_REPORT_LEN] & ST7123_REPORT_VALID) {
            rep = &buf[i * ST7123_REPORT_LEN];
            break;
        }
    }
    if (!rep) {
        return st7123_no_touch(data, now_us);
    }

    int x = ((rep[0] & ST7123_REPORT_X_H_MASK) << 8) | rep[1];
    int y = (rep[2] << 8) | rep[3];
#if CONFIG_LV_ST7123_SWAPXY
    int t = x;
    x = y;
    y = t;
#endif
#if CONFIG_LV_ST7123_INVERT_X
    x = (LV_HOR_RES - 1) - x;
#endif
#if CONFIG_LV_ST7123_INVERT_Y
    y = (LV_VER_RES - 1) - y;
#endif

    /* Finger lifted and landed elsewhere inside the hold window (fast typing on
     * neighbouring keys): report the release at the old spot so LVGL sees two taps. */
    if (s_holding && !s_dragging && (abs(x - s_last_x) + abs(y - s_last_y)) > ST7123_NEW_TOUCH_DISTANCE) {
        s_holding = false;
        s_pressed = false;
        return false; /* state stays RELEASED at the previous coordinates */
    }

    if (!s_pressed) {
        s_start_x = x;
        s_start_y = y;
        s_dragging = false;
    } else if (abs(x - s_start_x) + abs(y - s_start_y) > ST7123_DRAG_DISTANCE) {
        s_dragging = true;
    }
    s_holding = false;
    s_pressed = true;
    s_last_valid_us = now_us;
    s_last_x = x;
    s_last_y = y;
    data->point.x = x;
    data->point.y = y;
    data->state = LV_INDEV_STATE_PR;
    return false;
}
