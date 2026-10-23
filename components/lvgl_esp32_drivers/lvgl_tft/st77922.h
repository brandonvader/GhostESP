/**
 * @file st77922.h
 *
 * Sitronix ST77922 QSPI panel, driven through Espressif's esp_lcd_st77922.
 * The display owns its SPI bus, so it does not use disp_spi.
 */

#ifndef ST77922_H
#define ST77922_H

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

void st77922_init(void);
void st77922_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* ST77922_H */
