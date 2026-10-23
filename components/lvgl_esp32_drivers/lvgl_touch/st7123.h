/**
 * @file st7123.h
 *
 * Sitronix ST7123-family touch controller integrated in the ST77922 panel
 * module (ES3C35P). 16-bit big-endian register addresses over I2C.
 */

#ifndef ST7123_H
#define ST7123_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif

void st7123_init(void);
bool st7123_read(lv_indev_drv_t *drv, lv_indev_data_t *data);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* ST7123_H */
