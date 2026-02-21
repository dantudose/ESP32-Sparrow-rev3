#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

extern const struct device *const bme680;
extern bool ltr303_configured;
extern bool ltr303_enabled;

void sensors_init(void);
void bme_log_get_path(char *buf, size_t len);

int ltr303_check_bus(void);
int ltr303_init_device(void);
int ltr303_wait_data_ready(uint32_t timeout_ms);
int ltr303_read_channels(uint16_t *ch0, uint16_t *ch1);
int ltr303_calc_lux(uint16_t ch0, uint16_t ch1, struct sensor_value *val);
