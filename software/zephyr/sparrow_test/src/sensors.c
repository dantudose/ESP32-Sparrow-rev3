#include "sensors.h"

#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/fuel_gauge.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/fs/fs.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "app_paths.h"
#include "boot_log.h"
#include "fs_utils.h"
#include "net_time.h"

#define BME680_NODE DT_NODELABEL(bme688)
#define LSM6DSL_NODE DT_NODELABEL(lsm6dsl)
#define LSM6DSL_DEFAULT_ACCEL_ODR_HZ 104
#define LSM6DSL_DEFAULT_GYRO_ODR_HZ 104
#define LSM6DSL_DEFAULT_ACCEL_RANGE_G 2
#define LSM6DSL_DEFAULT_GYRO_RANGE_DPS 250
#if DT_HAS_COMPAT_STATUS_OKAY(maxim_max17048)
#define MAX17048_NODE DT_INST(0, maxim_max17048)
#if DT_NODE_EXISTS(DT_NODELABEL(max17048_alert))
#define MAX17048_ALERT_NODE DT_NODELABEL(max17048_alert)
#define MAX17048_ALERT_AVAILABLE 1
#else
#define MAX17048_ALERT_AVAILABLE 0
#endif
#else
#define MAX17048_ALERT_AVAILABLE 0
#endif

#define BME_LOG_THREAD_STACK_SIZE 3072
#define BME_LOG_THREAD_PRIORITY 5
#define BME_LOG_DEFAULT_INTERVAL_S 5
#define LTR_LOG_THREAD_STACK_SIZE 3072
#define LTR_LOG_THREAD_PRIORITY 5
#define LTR_LOG_DEFAULT_INTERVAL_S 5
#define LSM6DSL_LOG_THREAD_STACK_SIZE 3072
#define LSM6DSL_LOG_THREAD_PRIORITY 5
#define LSM6DSL_LOG_DEFAULT_INTERVAL_S 5
#define MAX17048_LOG_THREAD_STACK_SIZE 3072
#define MAX17048_LOG_THREAD_PRIORITY 5
#define MAX17048_LOG_DEFAULT_INTERVAL_S 5

#define BME_LOG_EVENT_START BIT(0)
#define BME_LOG_EVENT_STOP BIT(1)
#define BME_LOG_EVENT_INTERVAL BIT(2)
#define LTR_LOG_EVENT_START BIT(0)
#define LTR_LOG_EVENT_STOP BIT(1)
#define LTR_LOG_EVENT_INTERVAL BIT(2)
#define LSM6DSL_LOG_EVENT_START BIT(0)
#define LSM6DSL_LOG_EVENT_STOP BIT(1)
#define LSM6DSL_LOG_EVENT_INTERVAL BIT(2)
#define MAX17048_LOG_EVENT_START BIT(0)
#define MAX17048_LOG_EVENT_STOP BIT(1)
#define MAX17048_LOG_EVENT_INTERVAL BIT(2)

const struct device *const bme680 = DEVICE_DT_GET(BME680_NODE);
static const struct i2c_dt_spec bme680_i2c = I2C_DT_SPEC_GET(BME680_NODE);
static const struct device *const lsm6dsl_dev = DEVICE_DT_GET(LSM6DSL_NODE);
static const struct gpio_dt_spec lsm6dsl_int_gpio =
	GPIO_DT_SPEC_GET_OR(LSM6DSL_NODE, irq_gpios, {0});
static struct gpio_callback lsm6dsl_int_cb;
static atomic_t lsm6dsl_int_count = ATOMIC_INIT(0);
static bool lsm6dsl_int_enabled;
static bool lsm6dsl_int_callback_added;
static int lsm6dsl_accel_odr_hz = LSM6DSL_DEFAULT_ACCEL_ODR_HZ;
static int lsm6dsl_gyro_odr_hz = LSM6DSL_DEFAULT_GYRO_ODR_HZ;
static int lsm6dsl_accel_range_g = LSM6DSL_DEFAULT_ACCEL_RANGE_G;
static int lsm6dsl_gyro_range_dps = LSM6DSL_DEFAULT_GYRO_RANGE_DPS;
static bool lsm6dsl_powered = true;
static atomic_t bme_log_running = ATOMIC_INIT(0);
static atomic_t bme_log_interval_s = ATOMIC_INIT(BME_LOG_DEFAULT_INTERVAL_S);
static struct k_thread bme_log_thread;
static K_THREAD_STACK_DEFINE(bme_log_stack, BME_LOG_THREAD_STACK_SIZE);
static K_EVENT_DEFINE(bme_log_event);
static char bme_log_path[BME_LOG_PATH_MAX] = BME_LOG_PATH_DEFAULT;
static struct k_mutex bme_log_path_lock;
static atomic_t ltr_log_running = ATOMIC_INIT(0);
static atomic_t ltr_log_interval_s = ATOMIC_INIT(LTR_LOG_DEFAULT_INTERVAL_S);
static struct k_thread ltr_log_thread;
static K_THREAD_STACK_DEFINE(ltr_log_stack, LTR_LOG_THREAD_STACK_SIZE);
static K_EVENT_DEFINE(ltr_log_event);
static char ltr_log_path[LTR_LOG_PATH_MAX] = LTR_LOG_PATH_DEFAULT;
static struct k_mutex ltr_log_path_lock;
static atomic_t lsm6dsl_log_running = ATOMIC_INIT(0);
static atomic_t lsm6dsl_log_interval_s = ATOMIC_INIT(LSM6DSL_LOG_DEFAULT_INTERVAL_S);
static struct k_thread lsm6dsl_log_thread;
static K_THREAD_STACK_DEFINE(lsm6dsl_log_stack, LSM6DSL_LOG_THREAD_STACK_SIZE);
static K_EVENT_DEFINE(lsm6dsl_log_event);
static char lsm6dsl_log_path[LSM6DSL_LOG_PATH_MAX] = LSM6DSL_LOG_PATH_DEFAULT;
static struct k_mutex lsm6dsl_log_path_lock;
#if DT_HAS_COMPAT_STATUS_OKAY(maxim_max17048)
static atomic_t max17048_log_running = ATOMIC_INIT(0);
static atomic_t max17048_log_interval_s = ATOMIC_INIT(MAX17048_LOG_DEFAULT_INTERVAL_S);
static struct k_thread max17048_log_thread;
static K_THREAD_STACK_DEFINE(max17048_log_stack, MAX17048_LOG_THREAD_STACK_SIZE);
static K_EVENT_DEFINE(max17048_log_event);
static char max17048_log_path[MAX17048_LOG_PATH_MAX] = MAX17048_LOG_PATH_DEFAULT;
static struct k_mutex max17048_log_path_lock;
static const struct device *const max17048_dev = DEVICE_DT_GET(MAX17048_NODE);
#if MAX17048_ALERT_AVAILABLE
static const struct gpio_dt_spec max17048_alert = GPIO_DT_SPEC_GET(MAX17048_ALERT_NODE, gpios);
static struct gpio_callback max17048_alert_cb;
static atomic_t max17048_alert_count = ATOMIC_INIT(0);
#endif
#endif

void bme_log_get_path(char *buf, size_t len)
{
	if (!buf || len == 0U) {
		return;
	}

	k_mutex_lock(&bme_log_path_lock, K_FOREVER);
	snprintk(buf, len, "%s", bme_log_path);
	k_mutex_unlock(&bme_log_path_lock);
}

static int bme_log_set_path(const char *path)
{
	char tmp[BME_LOG_PATH_MAX];
	const char *use_path = path;
	size_t len;

	if (!path || path[0] == '\0') {
		return -EINVAL;
	}

	if (path[0] != '/') {
		int rc = snprintk(tmp, sizeof(tmp), "%s/%s", BME_LOG_DIR, path);
		if (rc < 0 || rc >= (int)sizeof(tmp)) {
			return -ENAMETOOLONG;
		}
		use_path = tmp;
	}

	if (strncmp(use_path, LITTLEFS_MOUNT_POINT "/", strlen(LITTLEFS_MOUNT_POINT) + 1) != 0) {
		return -EINVAL;
	}

	len = strlen(use_path);
	if (len >= sizeof(bme_log_path)) {
		return -ENAMETOOLONG;
	}

	k_mutex_lock(&bme_log_path_lock, K_FOREVER);
	memcpy(bme_log_path, use_path, len + 1);
	k_mutex_unlock(&bme_log_path_lock);

	return 0;
}

static int ensure_bme_log_path(void)
{
	char path[BME_LOG_PATH_MAX];

	bme_log_get_path(path, sizeof(path));
	return ensure_dir_tree_for_file(path);
}

static void ltr_log_get_path(char *buf, size_t len)
{
	if (!buf || len == 0U) {
		return;
	}

	k_mutex_lock(&ltr_log_path_lock, K_FOREVER);
	snprintk(buf, len, "%s", ltr_log_path);
	k_mutex_unlock(&ltr_log_path_lock);
}

static int ltr_log_set_path(const char *path)
{
	char tmp[LTR_LOG_PATH_MAX];
	const char *use_path = path;
	size_t len;

	if (!path || path[0] == '\0') {
		return -EINVAL;
	}

	if (path[0] != '/') {
		int rc = snprintk(tmp, sizeof(tmp), "%s/%s", LTR_LOG_DIR, path);
		if (rc < 0 || rc >= (int)sizeof(tmp)) {
			return -ENAMETOOLONG;
		}
		use_path = tmp;
	}

	if (strncmp(use_path, LITTLEFS_MOUNT_POINT "/", strlen(LITTLEFS_MOUNT_POINT) + 1) != 0) {
		return -EINVAL;
	}

	len = strlen(use_path);
	if (len >= sizeof(ltr_log_path)) {
		return -ENAMETOOLONG;
	}

	k_mutex_lock(&ltr_log_path_lock, K_FOREVER);
	memcpy(ltr_log_path, use_path, len + 1);
	k_mutex_unlock(&ltr_log_path_lock);

	return 0;
}

static int ensure_ltr_log_path(void)
{
	char path[LTR_LOG_PATH_MAX];

	ltr_log_get_path(path, sizeof(path));
	return ensure_dir_tree_for_file(path);
}

static void lsm6dsl_log_get_path(char *buf, size_t len)
{
	if (!buf || len == 0U) {
		return;
	}

	k_mutex_lock(&lsm6dsl_log_path_lock, K_FOREVER);
	snprintk(buf, len, "%s", lsm6dsl_log_path);
	k_mutex_unlock(&lsm6dsl_log_path_lock);
}

static int lsm6dsl_log_set_path(const char *path)
{
	char tmp[LSM6DSL_LOG_PATH_MAX];
	const char *use_path = path;
	size_t len;

	if (!path || path[0] == '\0') {
		return -EINVAL;
	}

	if (path[0] != '/') {
		int rc = snprintk(tmp, sizeof(tmp), "%s/%s", LSM6DSL_LOG_DIR, path);
		if (rc < 0 || rc >= (int)sizeof(tmp)) {
			return -ENAMETOOLONG;
		}
		use_path = tmp;
	}

	if (strncmp(use_path, LITTLEFS_MOUNT_POINT "/", strlen(LITTLEFS_MOUNT_POINT) + 1) != 0) {
		return -EINVAL;
	}

	len = strlen(use_path);
	if (len >= sizeof(lsm6dsl_log_path)) {
		return -ENAMETOOLONG;
	}

	k_mutex_lock(&lsm6dsl_log_path_lock, K_FOREVER);
	memcpy(lsm6dsl_log_path, use_path, len + 1);
	k_mutex_unlock(&lsm6dsl_log_path_lock);

	return 0;
}

static int ensure_lsm6dsl_log_path(void)
{
	char path[LSM6DSL_LOG_PATH_MAX];

	lsm6dsl_log_get_path(path, sizeof(path));
	return ensure_dir_tree_for_file(path);
}

#if DT_HAS_COMPAT_STATUS_OKAY(maxim_max17048)
static void max17048_log_get_path(char *buf, size_t len)
{
	if (!buf || len == 0U) {
		return;
	}

	k_mutex_lock(&max17048_log_path_lock, K_FOREVER);
	snprintk(buf, len, "%s", max17048_log_path);
	k_mutex_unlock(&max17048_log_path_lock);
}

static int max17048_log_set_path(const char *path)
{
	char tmp[MAX17048_LOG_PATH_MAX];
	const char *use_path = path;
	size_t len;

	if (!path || path[0] == '\0') {
		return -EINVAL;
	}

	if (path[0] != '/') {
		int rc = snprintk(tmp, sizeof(tmp), "%s/%s", MAX17048_LOG_DIR, path);
		if (rc < 0 || rc >= (int)sizeof(tmp)) {
			return -ENAMETOOLONG;
		}
		use_path = tmp;
	}

	if (strncmp(use_path, LITTLEFS_MOUNT_POINT "/", strlen(LITTLEFS_MOUNT_POINT) + 1) != 0) {
		return -EINVAL;
	}

	len = strlen(use_path);
	if (len >= sizeof(max17048_log_path)) {
		return -ENAMETOOLONG;
	}

	k_mutex_lock(&max17048_log_path_lock, K_FOREVER);
	memcpy(max17048_log_path, use_path, len + 1);
	k_mutex_unlock(&max17048_log_path_lock);

	return 0;
}

static int ensure_max17048_log_path(void)
{
	char path[MAX17048_LOG_PATH_MAX];

	max17048_log_get_path(path, sizeof(path));
	return ensure_dir_tree_for_file(path);
}
#endif

static void format_micro_value(char *buf, size_t len, int64_t micro)
{
	int64_t abs_val = llabs(micro);
	int64_t whole = abs_val / 1000000;
	int64_t frac = abs_val % 1000000;

	if (micro < 0) {
		snprintk(buf, len, "-%" PRId64 ".%06" PRId64, whole, frac);
	} else {
		snprintk(buf, len, "%" PRId64 ".%06" PRId64, whole, frac);
	}
}

int ltr303_check_bus(void);
int ltr303_init_device(void);
int ltr303_wait_data_ready(uint32_t timeout_ms);
int ltr303_read_channels(uint16_t *ch0, uint16_t *ch1);
int ltr303_calc_lux(uint16_t ch0, uint16_t ch1, struct sensor_value *val);

#define BME680_REG_COEFF1 0x8A
#define BME680_REG_COEFF2 0xE1
#define BME680_REG_COEFF3 0x00
#define BME680_REG_RES_HEAT0 0x5A
#define BME680_REG_GAS_WAIT0 0x64
#define BME680_REG_CTRL_GAS_1 0x71
#define BME680_REG_CTRL_HUM 0x72
#define BME680_REG_CTRL_MEAS 0x74
#define BME680_REG_CONFIG 0x75
#define BME680_LEN_COEFF1 23
#define BME680_LEN_COEFF2 14
#define BME680_LEN_COEFF3 5
#define BME680_LEN_COEFF_ALL (BME680_LEN_COEFF1 + BME680_LEN_COEFF2 + BME680_LEN_COEFF3)
#define BME680_MSK_RH_RANGE 0x30
#define BME680_CTRL_GAS_RUN BIT(4)
#define BME680_CONCAT_BYTES(msb, lsb) (((uint16_t)(msb) << 8) | (uint16_t)(lsb))

struct bme680_heater_calib {
	bool valid;
	int8_t par_gh1;
	int16_t par_gh2;
	int8_t par_gh3;
	int8_t res_heat_val;
	uint8_t res_heat_range;
};

static struct bme680_heater_calib bme680_heater_calib;
static bool bme_heater_configured;
static uint16_t bme_heater_temp_c;
static uint16_t bme_heater_dur_ms;

static int bme680_i2c_ready(void)
{
	return i2c_is_ready_dt(&bme680_i2c) ? 0 : -ENODEV;
}

static int bme680_reg_read(uint8_t reg, uint8_t *val)
{
	return i2c_reg_read_byte_dt(&bme680_i2c, reg, val);
}

static int bme680_reg_write(uint8_t reg, uint8_t val)
{
	return i2c_reg_write_byte_dt(&bme680_i2c, reg, val);
}

static int bme680_reg_read_buf(uint8_t reg, uint8_t *buf, size_t len)
{
	return i2c_burst_read_dt(&bme680_i2c, reg, buf, len);
}

static int bme680_read_heater_calib(struct bme680_heater_calib *calib)
{
	uint8_t buf[BME680_LEN_COEFF_ALL];
	int rc;

	if (calib->valid) {
		return 0;
	}

	rc = bme680_reg_read_buf(BME680_REG_COEFF1, buf, BME680_LEN_COEFF1);
	if (rc < 0) {
		return rc;
	}

	rc = bme680_reg_read_buf(BME680_REG_COEFF2,
				 &buf[BME680_LEN_COEFF1], BME680_LEN_COEFF2);
	if (rc < 0) {
		return rc;
	}

	rc = bme680_reg_read_buf(BME680_REG_COEFF3,
				 &buf[BME680_LEN_COEFF1 + BME680_LEN_COEFF2],
				 BME680_LEN_COEFF3);
	if (rc < 0) {
		return rc;
	}

	calib->par_gh1 = (int8_t)buf[35];
	calib->par_gh2 = (int16_t)BME680_CONCAT_BYTES(buf[34], buf[33]);
	calib->par_gh3 = (int8_t)buf[36];
	calib->res_heat_val = (int8_t)buf[37];
	calib->res_heat_range = (buf[39] & BME680_MSK_RH_RANGE) >> 4;
	calib->valid = true;
	return 0;
}

static uint8_t bme680_calc_res_heat(const struct bme680_heater_calib *calib,
				    uint16_t heatr_temp)
{
	uint8_t heatr_res;
	int32_t var1, var2, var3, var4, var5;
	int32_t heatr_res_x100;
	int32_t amb_temp = 25;

	if (heatr_temp > 400) {
		heatr_temp = 400;
	}

	var1 = ((amb_temp * calib->par_gh3) / 1000) * 256;
	var2 = (calib->par_gh1 + 784) * (((((calib->par_gh2 + 154009)
				    * heatr_temp * 5) / 100)
				  + 3276800) / 10);
	var3 = var1 + (var2 / 2);
	var4 = (var3 / (calib->res_heat_range + 4));
	var5 = (131 * calib->res_heat_val) + 65536;
	heatr_res_x100 = ((var4 / var5) - 250) * 34;
	heatr_res = (heatr_res_x100 + 50) / 100;

	return heatr_res;
}

static uint8_t bme680_calc_gas_wait(uint16_t dur)
{
	uint8_t factor = 0;
	uint8_t durval;

	if (dur >= 0xFC0) {
		durval = 0xFF;
	} else {
		while (dur > 0x3F) {
			dur = dur / 4;
			factor += 1;
		}
		const uint16_t max_duration = dur + (factor * 64);

		durval = CLAMP(max_duration, 0, 0xFF);
	}

	return durval;
}

static int bme680_osr_from_val(int val, uint8_t *reg)
{
	switch (val) {
	case 0:
		*reg = 0;
		return 0;
	case 1:
		*reg = 1;
		return 0;
	case 2:
		*reg = 2;
		return 0;
	case 4:
		*reg = 3;
		return 0;
	case 8:
		*reg = 4;
		return 0;
	case 16:
		*reg = 5;
		return 0;
	default:
		return -EINVAL;
	}
}

static int bme680_osr_to_val(uint8_t reg)
{
	switch (reg) {
	case 0:
		return 0;
	case 1:
		return 1;
	case 2:
		return 2;
	case 3:
		return 4;
	case 4:
		return 8;
	case 5:
		return 16;
	default:
		return -EINVAL;
	}
}

static int bme680_filter_from_val(int val, uint8_t *reg)
{
	switch (val) {
	case 0:
		*reg = 0;
		return 0;
	case 2:
		*reg = 1;
		return 0;
	case 4:
		*reg = 2;
		return 0;
	case 8:
		*reg = 3;
		return 0;
	case 16:
		*reg = 4;
		return 0;
	case 32:
		*reg = 5;
		return 0;
	case 64:
		*reg = 6;
		return 0;
	case 128:
		*reg = 7;
		return 0;
	default:
		return -EINVAL;
	}
}

static int bme680_filter_to_val(uint8_t reg)
{
	switch (reg) {
	case 0:
		return 0;
	case 1:
		return 2;
	case 2:
		return 4;
	case 3:
		return 8;
	case 4:
		return 16;
	case 5:
		return 32;
	case 6:
		return 64;
	case 7:
		return 128;
	default:
		return -EINVAL;
	}
}

static int bme_log_write_entry(void)
{
	static const char header[] = "timestamp_ms,temp_c,press_kpa,humidity_pct,gas_ohm\n";
	struct sensor_value temp;
	struct sensor_value press;
	struct sensor_value humidity;
	struct sensor_value gas;
	struct fs_file_t file;
	struct fs_dirent entry;
	char temp_buf[20];
	char press_buf[20];
	char hum_buf[20];
	char gas_buf[20];
	char line[128];
	char log_path[BME_LOG_PATH_MAX];
	bool write_header = false;
	int rc;

	if (!device_is_ready(bme680)) {
		return -ENODEV;
	}

	rc = ensure_littlefs_ready();
	if (rc < 0) {
		return rc;
	}

	rc = ensure_bme_log_path();
	if (rc < 0) {
		return rc;
	}

	rc = sensor_sample_fetch(bme680);
	if (rc < 0) {
		return rc;
	}

	rc = sensor_channel_get(bme680, SENSOR_CHAN_AMBIENT_TEMP, &temp);
	if (rc < 0) {
		return rc;
	}

	rc = sensor_channel_get(bme680, SENSOR_CHAN_PRESS, &press);
	if (rc < 0) {
		return rc;
	}

	rc = sensor_channel_get(bme680, SENSOR_CHAN_HUMIDITY, &humidity);
	if (rc < 0) {
		return rc;
	}

	rc = sensor_channel_get(bme680, SENSOR_CHAN_GAS_RES, &gas);
	if (rc < 0) {
		return rc;
	}

	format_micro_value(temp_buf, sizeof(temp_buf), sensor_value_to_micro(&temp));
	format_micro_value(press_buf, sizeof(press_buf), sensor_value_to_micro(&press));
	format_micro_value(hum_buf, sizeof(hum_buf), sensor_value_to_micro(&humidity));
	format_micro_value(gas_buf, sizeof(gas_buf), sensor_value_to_micro(&gas));

	bme_log_get_path(log_path, sizeof(log_path));
	rc = fs_stat(log_path, &entry);
	if (rc == 0) {
		if (entry.type != FS_DIR_ENTRY_FILE) {
			return -EISDIR;
		}
		write_header = (entry.size == 0U);
	} else if (rc == -ENOENT) {
		write_header = true;
	} else {
		return rc;
	}

	fs_file_t_init(&file);
	rc = fs_open(&file, log_path, FS_O_CREATE | FS_O_WRITE);
	if (rc < 0) {
		return rc;
	}

	rc = fs_seek(&file, 0, FS_SEEK_END);
	if (rc < 0) {
		fs_close(&file);
		return rc;
	}

	if (write_header) {
		rc = fs_write(&file, header, sizeof(header) - 1);
		if (rc < 0) {
			fs_close(&file);
			return rc;
		}
	}

	rc = snprintk(line, sizeof(line), "%" PRId64 ",%s,%s,%s,%s\n",
		      time_now_ms(), temp_buf, press_buf, hum_buf, gas_buf);
	if (rc < 0) {
		fs_close(&file);
		return rc;
	}

	rc = fs_write(&file, line, rc);
	if (rc < 0) {
		fs_close(&file);
		return rc;
	}

	fs_close(&file);
	return 0;
}

static int ltr_log_write_entry(void)
{
	static const char header[] = "timestamp_ms,ch0,ch1,lux\n";
	static bool ltr_log_sensor_ready;
	struct fs_file_t file;
	struct fs_dirent entry;
	struct sensor_value lux;
	char lux_buf[20];
	char line[128];
	char log_path[LTR_LOG_PATH_MAX];
	uint16_t ch0;
	uint16_t ch1;
	bool write_header = false;
	int rc;

	rc = ltr303_check_bus();
	if (rc < 0) {
		return rc;
	}

	if (!ltr_log_sensor_ready) {
		rc = ltr303_init_device();
		if (rc < 0) {
			return rc;
		}
		ltr_log_sensor_ready = true;
	}

	rc = ensure_littlefs_ready();
	if (rc < 0) {
		return rc;
	}

	rc = ensure_ltr_log_path();
	if (rc < 0) {
		return rc;
	}

	rc = ltr303_wait_data_ready(500);
	if (rc < 0 && rc != -ETIMEDOUT) {
		return rc;
	}

	rc = ltr303_read_channels(&ch0, &ch1);
	if (rc < 0) {
		return rc;
	}

	rc = ltr303_calc_lux(ch0, ch1, &lux);
	if (rc == 0) {
		format_micro_value(lux_buf, sizeof(lux_buf),
				   sensor_value_to_micro(&lux));
	} else {
		snprintk(lux_buf, sizeof(lux_buf), "nan");
	}

	ltr_log_get_path(log_path, sizeof(log_path));
	rc = fs_stat(log_path, &entry);
	if (rc == 0) {
		if (entry.type != FS_DIR_ENTRY_FILE) {
			return -EISDIR;
		}
		write_header = (entry.size == 0U);
	} else if (rc == -ENOENT) {
		write_header = true;
	} else {
		return rc;
	}

	fs_file_t_init(&file);
	rc = fs_open(&file, log_path, FS_O_CREATE | FS_O_WRITE);
	if (rc < 0) {
		return rc;
	}

	rc = fs_seek(&file, 0, FS_SEEK_END);
	if (rc < 0) {
		fs_close(&file);
		return rc;
	}

	if (write_header) {
		rc = fs_write(&file, header, sizeof(header) - 1);
		if (rc < 0) {
			fs_close(&file);
			return rc;
		}
	}

	rc = snprintk(line, sizeof(line), "%" PRId64 ",%u,%u,%s\n",
		      time_now_ms(), ch0, ch1, lux_buf);
	if (rc < 0) {
		fs_close(&file);
		return rc;
	}

	rc = fs_write(&file, line, rc);
	if (rc < 0) {
		fs_close(&file);
		return rc;
	}

	fs_close(&file);
	return 0;
}

static int lsm6dsl_log_write_entry(void)
{
	static const char header[] =
		"timestamp_ms,accel_x_ms2,accel_y_ms2,accel_z_ms2,gyro_x_rads,gyro_y_rads,gyro_z_rads,temp_c\n";
	struct sensor_value accel[3];
	struct sensor_value gyro[3];
	struct sensor_value temp;
	struct fs_file_t file;
	struct fs_dirent entry;
	char accel_buf[3][20];
	char gyro_buf[3][20];
	char temp_buf[20];
	char line[196];
	char log_path[LSM6DSL_LOG_PATH_MAX];
	bool write_header = false;
	int rc;

	if (!device_is_ready(lsm6dsl_dev)) {
		return -ENODEV;
	}

	rc = ensure_littlefs_ready();
	if (rc < 0) {
		return rc;
	}

	rc = ensure_lsm6dsl_log_path();
	if (rc < 0) {
		return rc;
	}

	rc = sensor_sample_fetch(lsm6dsl_dev);
	if (rc < 0) {
		return rc;
	}

	rc = sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
	if (rc < 0) {
		return rc;
	}

	rc = sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ, gyro);
	if (rc < 0) {
		return rc;
	}

	rc = sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_DIE_TEMP, &temp);
	if (rc == 0) {
		format_micro_value(temp_buf, sizeof(temp_buf), sensor_value_to_micro(&temp));
	} else if (rc == -ENOTSUP) {
		snprintk(temp_buf, sizeof(temp_buf), "nan");
	} else {
		return rc;
	}

	for (size_t i = 0; i < 3; i++) {
		format_micro_value(accel_buf[i], sizeof(accel_buf[i]),
				   sensor_value_to_micro(&accel[i]));
		format_micro_value(gyro_buf[i], sizeof(gyro_buf[i]),
				   sensor_value_to_micro(&gyro[i]));
	}

	lsm6dsl_log_get_path(log_path, sizeof(log_path));
	rc = fs_stat(log_path, &entry);
	if (rc == 0) {
		if (entry.type != FS_DIR_ENTRY_FILE) {
			return -EISDIR;
		}
		write_header = (entry.size == 0U);
	} else if (rc == -ENOENT) {
		write_header = true;
	} else {
		return rc;
	}

	fs_file_t_init(&file);
	rc = fs_open(&file, log_path, FS_O_CREATE | FS_O_WRITE);
	if (rc < 0) {
		return rc;
	}

	rc = fs_seek(&file, 0, FS_SEEK_END);
	if (rc < 0) {
		fs_close(&file);
		return rc;
	}

	if (write_header) {
		rc = fs_write(&file, header, sizeof(header) - 1);
		if (rc < 0) {
			fs_close(&file);
			return rc;
		}
	}

	rc = snprintk(line, sizeof(line),
		      "%" PRId64 ",%s,%s,%s,%s,%s,%s,%s\n",
		      time_now_ms(),
		      accel_buf[0], accel_buf[1], accel_buf[2],
		      gyro_buf[0], gyro_buf[1], gyro_buf[2],
		      temp_buf);
	if (rc < 0) {
		fs_close(&file);
		return rc;
	}

	rc = fs_write(&file, line, rc);
	if (rc < 0) {
		fs_close(&file);
		return rc;
	}

	fs_close(&file);
	return 0;
}

#if DT_HAS_COMPAT_STATUS_OKAY(maxim_max17048)
static int max17048_log_write_entry(void)
{
	static const char header[] =
		"timestamp_ms,voltage_mv,soc_pct,time_to_empty_min,time_to_full_min,alert_count\n";
	struct fs_file_t file;
	struct fs_dirent entry;
	union fuel_gauge_prop_val val;
	char line[128];
	char log_path[MAX17048_LOG_PATH_MAX];
	uint32_t voltage_mv;
	uint8_t soc;
	uint32_t time_empty;
	uint32_t time_full;
	int alert_count = -1;
	bool write_header = false;
	int rc;

	if (!device_is_ready(max17048_dev)) {
		return -ENODEV;
	}

	rc = ensure_littlefs_ready();
	if (rc < 0) {
		return rc;
	}

	rc = ensure_max17048_log_path();
	if (rc < 0) {
		return rc;
	}

	rc = fuel_gauge_get_prop(max17048_dev, FUEL_GAUGE_VOLTAGE, &val);
	if (rc < 0) {
		return rc;
	}
	voltage_mv = (uint32_t)val.voltage / 1000U;

	rc = fuel_gauge_get_prop(max17048_dev, FUEL_GAUGE_RELATIVE_STATE_OF_CHARGE, &val);
	if (rc < 0) {
		return rc;
	}
	soc = val.relative_state_of_charge;

	rc = fuel_gauge_get_prop(max17048_dev, FUEL_GAUGE_RUNTIME_TO_EMPTY, &val);
	if (rc < 0) {
		return rc;
	}
	time_empty = val.runtime_to_empty;

	rc = fuel_gauge_get_prop(max17048_dev, FUEL_GAUGE_RUNTIME_TO_FULL, &val);
	if (rc < 0) {
		return rc;
	}
	time_full = val.runtime_to_full;

#if MAX17048_ALERT_AVAILABLE
	alert_count = atomic_get(&max17048_alert_count);
#endif

	max17048_log_get_path(log_path, sizeof(log_path));
	rc = fs_stat(log_path, &entry);
	if (rc == 0) {
		if (entry.type != FS_DIR_ENTRY_FILE) {
			return -EISDIR;
		}
		write_header = (entry.size == 0U);
	} else if (rc == -ENOENT) {
		write_header = true;
	} else {
		return rc;
	}

	fs_file_t_init(&file);
	rc = fs_open(&file, log_path, FS_O_CREATE | FS_O_WRITE);
	if (rc < 0) {
		return rc;
	}

	rc = fs_seek(&file, 0, FS_SEEK_END);
	if (rc < 0) {
		fs_close(&file);
		return rc;
	}

	if (write_header) {
		rc = fs_write(&file, header, sizeof(header) - 1);
		if (rc < 0) {
			fs_close(&file);
			return rc;
		}
	}

	rc = snprintk(line, sizeof(line), "%" PRId64 ",%u,%u,%u,%u,%d\n",
		      time_now_ms(), voltage_mv, soc, time_empty, time_full, alert_count);
	if (rc < 0) {
		fs_close(&file);
		return rc;
	}

	rc = fs_write(&file, line, rc);
	if (rc < 0) {
		fs_close(&file);
		return rc;
	}

	fs_close(&file);
	return 0;
}
#endif

static int bme_print_config(const struct shell *shell)
{
	uint8_t ctrl_hum;
	uint8_t ctrl_meas;
	uint8_t config;
	uint8_t ctrl_gas;
	uint8_t res_heat;
	uint8_t gas_wait;
	int hum_osr;
	int temp_osr;
	int press_osr;
	int filter;
	int rc;

	rc = bme680_reg_read(BME680_REG_CTRL_HUM, &ctrl_hum);
	if (rc < 0) {
		shell_error(shell, "Read CTRL_HUM failed (%d).", rc);
		return rc;
	}
	rc = bme680_reg_read(BME680_REG_CTRL_MEAS, &ctrl_meas);
	if (rc < 0) {
		shell_error(shell, "Read CTRL_MEAS failed (%d).", rc);
		return rc;
	}
	rc = bme680_reg_read(BME680_REG_CONFIG, &config);
	if (rc < 0) {
		shell_error(shell, "Read CONFIG failed (%d).", rc);
		return rc;
	}
	rc = bme680_reg_read(BME680_REG_CTRL_GAS_1, &ctrl_gas);
	if (rc < 0) {
		shell_error(shell, "Read CTRL_GAS_1 failed (%d).", rc);
		return rc;
	}
	rc = bme680_reg_read(BME680_REG_RES_HEAT0, &res_heat);
	if (rc < 0) {
		shell_error(shell, "Read RES_HEAT0 failed (%d).", rc);
		return rc;
	}
	rc = bme680_reg_read(BME680_REG_GAS_WAIT0, &gas_wait);
	if (rc < 0) {
		shell_error(shell, "Read GAS_WAIT0 failed (%d).", rc);
		return rc;
	}

	hum_osr = bme680_osr_to_val(ctrl_hum & 0x07);
	temp_osr = bme680_osr_to_val((ctrl_meas >> 5) & 0x07);
	press_osr = bme680_osr_to_val((ctrl_meas >> 2) & 0x07);
	filter = bme680_filter_to_val((config >> 2) & 0x07);

	shell_print(shell, "OSR temp=%d press=%d hum=%d", temp_osr, press_osr, hum_osr);
	shell_print(shell, "Filter=%d", filter);
	shell_print(shell, "Heater: %s (res_heat0=0x%02x gas_wait0=0x%02x)",
		    (ctrl_gas & BME680_CTRL_GAS_RUN) ? "on" : "off",
		    res_heat, gas_wait);
	if (bme_heater_configured) {
		shell_print(shell, "Heater last set: %u C, %u ms",
			    bme_heater_temp_c, bme_heater_dur_ms);
	}

	return 0;
}

static int cmd_bme_config(const struct shell *shell, size_t argc, char **argv)
{
	int rc;

	if (argc < 2) {
		shell_print(shell,
			    "Usage: bme config <show|osr <temp|press|hum> <0|1|2|4|8|16>|filter <off|2|4|8|16|32|64|128>|heater <temp_c> <dur_ms>|heater off>");
		return -EINVAL;
	}

	rc = bme680_i2c_ready();
	if (rc < 0) {
		shell_error(shell, "BME680 I2C bus not ready.");
		return rc;
	}
	if (!device_is_ready(bme680)) {
		shell_error(shell, "BME680 device not ready.");
		return -ENODEV;
	}

	if (strcmp(argv[1], "show") == 0) {
		return bme_print_config(shell);
	}

	if (strcmp(argv[1], "osr") == 0) {
		const char *chan;
		int osr_val;
		uint8_t reg_val;
		char *end = NULL;

		if (argc < 4) {
			shell_error(shell, "Usage: bme config osr <temp|press|hum> <0|1|2|4|8|16>");
			return -EINVAL;
		}

		chan = argv[2];
		osr_val = (int)strtol(argv[3], &end, 0);
		if (end == argv[3] || *end != '\0') {
			shell_error(shell, "Invalid oversampling value.");
			return -EINVAL;
		}

		rc = bme680_osr_from_val(osr_val, &reg_val);
		if (rc < 0) {
			shell_error(shell, "Unsupported oversampling value.");
			return rc;
		}

		if (strcmp(chan, "hum") == 0) {
			uint8_t ctrl_hum;
			uint8_t ctrl_meas;

			rc = bme680_reg_read(BME680_REG_CTRL_HUM, &ctrl_hum);
			if (rc < 0) {
				shell_error(shell, "Read CTRL_HUM failed (%d).", rc);
				return rc;
			}
			ctrl_hum = (ctrl_hum & ~0x07) | (reg_val & 0x07);
			rc = bme680_reg_write(BME680_REG_CTRL_HUM, ctrl_hum);
			if (rc < 0) {
				shell_error(shell, "Write CTRL_HUM failed (%d).", rc);
				return rc;
			}
			rc = bme680_reg_read(BME680_REG_CTRL_MEAS, &ctrl_meas);
			if (rc == 0) {
				(void)bme680_reg_write(BME680_REG_CTRL_MEAS, ctrl_meas);
			}
			shell_print(shell, "Humidity oversampling set to %dx.", osr_val);
			return 0;
		}

		if (strcmp(chan, "temp") == 0 || strcmp(chan, "press") == 0) {
			uint8_t ctrl_meas;
			uint8_t mask;
			uint8_t shift;

			mask = (strcmp(chan, "temp") == 0) ? 0xE0 : 0x1C;
			shift = (strcmp(chan, "temp") == 0) ? 5 : 2;

			rc = bme680_reg_read(BME680_REG_CTRL_MEAS, &ctrl_meas);
			if (rc < 0) {
				shell_error(shell, "Read CTRL_MEAS failed (%d).", rc);
				return rc;
			}
			ctrl_meas = (ctrl_meas & ~mask) | ((reg_val << shift) & mask);
			rc = bme680_reg_write(BME680_REG_CTRL_MEAS, ctrl_meas);
			if (rc < 0) {
				shell_error(shell, "Write CTRL_MEAS failed (%d).", rc);
				return rc;
			}
			shell_print(shell, "%s oversampling set to %dx.",
				    strcmp(chan, "temp") == 0 ? "Temperature" : "Pressure",
				    osr_val);
			shell_print(shell,
				    "Note: driver uses build-time temp/press oversampling for reads/logs.");
			return 0;
		}

		shell_error(shell, "Unknown channel. Use: temp|press|hum");
		return -EINVAL;
	}

	if (strcmp(argv[1], "filter") == 0) {
		uint8_t reg_val;
		uint8_t config;
		const char *arg;
		char *end = NULL;
		int val;

		if (argc < 3) {
			shell_error(shell, "Usage: bme config filter <off|2|4|8|16|32|64|128>");
			return -EINVAL;
		}

		arg = argv[2];
		if (strcmp(arg, "off") == 0) {
			val = 0;
		} else {
			val = (int)strtol(arg, &end, 0);
			if (end == arg || *end != '\0') {
				shell_error(shell, "Invalid filter value.");
				return -EINVAL;
			}
		}

		rc = bme680_filter_from_val(val, &reg_val);
		if (rc < 0) {
			shell_error(shell, "Unsupported filter value.");
			return rc;
		}

		rc = bme680_reg_read(BME680_REG_CONFIG, &config);
		if (rc < 0) {
			shell_error(shell, "Read CONFIG failed (%d).", rc);
			return rc;
		}

		config = (config & ~0x1C) | ((reg_val << 2) & 0x1C);
		rc = bme680_reg_write(BME680_REG_CONFIG, config);
		if (rc < 0) {
			shell_error(shell, "Write CONFIG failed (%d).", rc);
			return rc;
		}

		shell_print(shell, "Filter set to %s.", val == 0 ? "off" : argv[2]);
		return 0;
	}

	if (strcmp(argv[1], "heater") == 0) {
		if (argc < 3) {
			shell_error(shell, "Usage: bme config heater <temp_c> <dur_ms>|off");
			return -EINVAL;
		}

		if (strcmp(argv[2], "off") == 0) {
			uint8_t ctrl_gas;

			rc = bme680_reg_read(BME680_REG_CTRL_GAS_1, &ctrl_gas);
			if (rc < 0) {
				shell_error(shell, "Read CTRL_GAS_1 failed (%d).", rc);
				return rc;
			}
			ctrl_gas &= (uint8_t)~BME680_CTRL_GAS_RUN;
			rc = bme680_reg_write(BME680_REG_CTRL_GAS_1, ctrl_gas);
			if (rc < 0) {
				shell_error(shell, "Write CTRL_GAS_1 failed (%d).", rc);
				return rc;
			}
			bme_heater_configured = false;
			shell_print(shell, "Gas heater disabled.");
			return 0;
		}

		if (argc < 4) {
			shell_error(shell, "Usage: bme config heater <temp_c> <dur_ms>");
			return -EINVAL;
		}

		char *end = NULL;
		int temp_c = (int)strtol(argv[2], &end, 0);
		if (end == argv[2] || *end != '\0') {
			shell_error(shell, "Invalid temperature.");
			return -EINVAL;
		}

		end = NULL;
		int dur_ms = (int)strtol(argv[3], &end, 0);
		if (end == argv[3] || *end != '\0') {
			shell_error(shell, "Invalid duration.");
			return -EINVAL;
		}

		if (temp_c < 0 || temp_c > 400 || dur_ms < 0) {
			shell_error(shell, "Temperature must be 0..400 C, duration >= 0.");
			return -EINVAL;
		}

		rc = bme680_read_heater_calib(&bme680_heater_calib);
		if (rc < 0) {
			shell_error(shell, "Read heater calibration failed (%d).", rc);
			return rc;
		}

		uint8_t res_heat = bme680_calc_res_heat(&bme680_heater_calib, (uint16_t)temp_c);
		uint8_t gas_wait = bme680_calc_gas_wait((uint16_t)dur_ms);
		uint8_t ctrl_gas;

		rc = bme680_reg_write(BME680_REG_RES_HEAT0, res_heat);
		if (rc < 0) {
			shell_error(shell, "Write RES_HEAT0 failed (%d).", rc);
			return rc;
		}
		rc = bme680_reg_write(BME680_REG_GAS_WAIT0, gas_wait);
		if (rc < 0) {
			shell_error(shell, "Write GAS_WAIT0 failed (%d).", rc);
			return rc;
		}
		rc = bme680_reg_read(BME680_REG_CTRL_GAS_1, &ctrl_gas);
		if (rc < 0) {
			shell_error(shell, "Read CTRL_GAS_1 failed (%d).", rc);
			return rc;
		}
		ctrl_gas |= BME680_CTRL_GAS_RUN;
		rc = bme680_reg_write(BME680_REG_CTRL_GAS_1, ctrl_gas);
		if (rc < 0) {
			shell_error(shell, "Write CTRL_GAS_1 failed (%d).", rc);
			return rc;
		}

		bme_heater_configured = true;
		bme_heater_temp_c = (uint16_t)temp_c;
		bme_heater_dur_ms = (uint16_t)dur_ms;
		shell_print(shell, "Gas heater set to %d C for %d ms.", temp_c, dur_ms);
		return 0;
	}

	shell_error(shell, "Unknown config command.");
	return -EINVAL;
}

static void bme_log_thread_fn(void *p1, void *p2, void *p3)
{
	(void)p1;
	(void)p2;
	(void)p3;

	while (true) {
		uint32_t events;
		int interval;
		int rc;

		if (!atomic_get(&bme_log_running)) {
			k_event_wait(&bme_log_event, BME_LOG_EVENT_START, false, K_FOREVER);
			k_event_clear(&bme_log_event, BME_LOG_EVENT_START);
			continue;
		}

		rc = bme_log_write_entry();
		if (rc < 0) {
			boot_log_write("bme_log: write error (%d)", rc);
		}

		interval = atomic_get(&bme_log_interval_s);
		if (interval < 1) {
			interval = 1;
		}

		events = k_event_wait(&bme_log_event,
				      BME_LOG_EVENT_STOP | BME_LOG_EVENT_INTERVAL,
				      false, K_SECONDS(interval));
		if (events & BME_LOG_EVENT_STOP) {
			atomic_set(&bme_log_running, 0);
			k_event_clear(&bme_log_event, BME_LOG_EVENT_STOP);
		}
		if (events & BME_LOG_EVENT_INTERVAL) {
			k_event_clear(&bme_log_event, BME_LOG_EVENT_INTERVAL);
		}
	}
}

static void ltr_log_thread_fn(void *p1, void *p2, void *p3)
{
	(void)p1;
	(void)p2;
	(void)p3;

	while (true) {
		uint32_t events;
		int interval;
		int rc;

		if (!atomic_get(&ltr_log_running)) {
			k_event_wait(&ltr_log_event, LTR_LOG_EVENT_START, false, K_FOREVER);
			k_event_clear(&ltr_log_event, LTR_LOG_EVENT_START);
			continue;
		}

		rc = ltr_log_write_entry();
		if (rc < 0) {
			boot_log_write("ltr_log: write error (%d)", rc);
		}

		interval = atomic_get(&ltr_log_interval_s);
		if (interval < 1) {
			interval = 1;
		}

		events = k_event_wait(&ltr_log_event,
				      LTR_LOG_EVENT_STOP | LTR_LOG_EVENT_INTERVAL,
				      false, K_SECONDS(interval));
		if (events & LTR_LOG_EVENT_STOP) {
			atomic_set(&ltr_log_running, 0);
			k_event_clear(&ltr_log_event, LTR_LOG_EVENT_STOP);
		}
		if (events & LTR_LOG_EVENT_INTERVAL) {
			k_event_clear(&ltr_log_event, LTR_LOG_EVENT_INTERVAL);
		}
	}
}

static void lsm6dsl_log_thread_fn(void *p1, void *p2, void *p3)
{
	(void)p1;
	(void)p2;
	(void)p3;

	while (true) {
		uint32_t events;
		int interval;
		int rc;

		if (!atomic_get(&lsm6dsl_log_running)) {
			k_event_wait(&lsm6dsl_log_event, LSM6DSL_LOG_EVENT_START,
				     false, K_FOREVER);
			k_event_clear(&lsm6dsl_log_event, LSM6DSL_LOG_EVENT_START);
			continue;
		}

		rc = lsm6dsl_log_write_entry();
		if (rc < 0) {
			boot_log_write("lsm6dsl_log: write error (%d)", rc);
		}

		interval = atomic_get(&lsm6dsl_log_interval_s);
		if (interval < 1) {
			interval = 1;
		}

		events = k_event_wait(&lsm6dsl_log_event,
				      LSM6DSL_LOG_EVENT_STOP | LSM6DSL_LOG_EVENT_INTERVAL,
				      false, K_SECONDS(interval));
		if (events & LSM6DSL_LOG_EVENT_STOP) {
			atomic_set(&lsm6dsl_log_running, 0);
			k_event_clear(&lsm6dsl_log_event, LSM6DSL_LOG_EVENT_STOP);
		}
		if (events & LSM6DSL_LOG_EVENT_INTERVAL) {
			k_event_clear(&lsm6dsl_log_event, LSM6DSL_LOG_EVENT_INTERVAL);
		}
	}
}

#if DT_HAS_COMPAT_STATUS_OKAY(maxim_max17048)
static void max17048_log_thread_fn(void *p1, void *p2, void *p3)
{
	(void)p1;
	(void)p2;
	(void)p3;

	while (true) {
		uint32_t events;
		int interval;
		int rc;

		if (!atomic_get(&max17048_log_running)) {
			k_event_wait(&max17048_log_event, MAX17048_LOG_EVENT_START,
				     false, K_FOREVER);
			k_event_clear(&max17048_log_event, MAX17048_LOG_EVENT_START);
			continue;
		}

		rc = max17048_log_write_entry();
		if (rc < 0) {
			boot_log_write("max17048_log: write error (%d)", rc);
		}

		interval = atomic_get(&max17048_log_interval_s);
		if (interval < 1) {
			interval = 1;
		}

		events = k_event_wait(&max17048_log_event,
				      MAX17048_LOG_EVENT_STOP | MAX17048_LOG_EVENT_INTERVAL,
				      false, K_SECONDS(interval));
		if (events & MAX17048_LOG_EVENT_STOP) {
			atomic_set(&max17048_log_running, 0);
			k_event_clear(&max17048_log_event, MAX17048_LOG_EVENT_STOP);
		}
		if (events & MAX17048_LOG_EVENT_INTERVAL) {
			k_event_clear(&max17048_log_event, MAX17048_LOG_EVENT_INTERVAL);
		}
	}
}
#endif

static int cmd_bme_log(const struct shell *shell, size_t argc, char **argv)
{
	int interval;
	const char *path_arg = NULL;
	char path[BME_LOG_PATH_MAX];
	int rc;

	if (argc < 2) {
		shell_print(shell,
			    "Usage: bme log <start [interval_s] [path]|stop|interval <s>|status>");
		return -EINVAL;
	}

	if (strcmp(argv[1], "start") == 0) {
		interval = atomic_get(&bme_log_interval_s);

		if (argc >= 3) {
			char *end = NULL;
			long val = strtol(argv[2], &end, 0);

			if (end != argv[2] && *end == '\0') {
				interval = (int)val;
			} else {
				path_arg = argv[2];
			}
		}

		if (argc >= 4) {
			if (path_arg) {
				shell_error(shell, "Too many arguments.");
				return -EINVAL;
			}
			path_arg = argv[3];
		}

		if (argc > 4) {
			shell_error(shell, "Too many arguments.");
			return -EINVAL;
		}

		if (interval < 1) {
			shell_error(shell, "Interval must be >= 1 second.");
			return -EINVAL;
		}

		if (path_arg) {
			rc = bme_log_set_path(path_arg);
			if (rc < 0) {
				shell_error(shell, "Invalid log path (%d).", rc);
				return rc;
			}
		}

		if (!device_is_ready(bme680)) {
			shell_error(shell, "BME680 device not ready.");
			return -ENODEV;
		}

		rc = ensure_littlefs_ready();
		if (rc < 0) {
			shell_error(shell, "LittleFS not mounted (%d).", rc);
			return rc;
		}

		rc = ensure_bme_log_path();
		if (rc < 0) {
			shell_error(shell, "Failed to prepare log path (%d).", rc);
			return rc;
		}

		atomic_set(&bme_log_interval_s, interval);
		k_event_post(&bme_log_event, BME_LOG_EVENT_INTERVAL);
		atomic_set(&bme_log_running, 1);
		k_event_post(&bme_log_event, BME_LOG_EVENT_START);
		bme_log_get_path(path, sizeof(path));
		shell_print(shell, "BME680 logging started (interval %d s).", interval);
		shell_print(shell, "Log file: %s", path);
		boot_log_write("bme_log: started interval %d s", interval);
		return 0;
	}

	if (strcmp(argv[1], "stop") == 0) {
		atomic_set(&bme_log_running, 0);
		k_event_post(&bme_log_event, BME_LOG_EVENT_STOP);
		shell_print(shell, "BME680 logging stopped.");
		boot_log_write("bme_log: stopped");
		return 0;
	}

	if (strcmp(argv[1], "interval") == 0) {
		if (argc < 3) {
			shell_error(shell, "Missing interval in seconds.");
			return -EINVAL;
		}

		interval = (int)strtol(argv[2], NULL, 0);
		if (interval < 1) {
			shell_error(shell, "Interval must be >= 1 second.");
			return -EINVAL;
		}

		atomic_set(&bme_log_interval_s, interval);
		k_event_post(&bme_log_event, BME_LOG_EVENT_INTERVAL);
		shell_print(shell, "BME680 log interval set to %d s.", interval);
		boot_log_write("bme_log: interval set %d s", interval);
		return 0;
	}

	if (strcmp(argv[1], "status") == 0) {
		bme_log_get_path(path, sizeof(path));
		shell_print(shell, "BME680 logging %s, interval %d s.",
			    atomic_get(&bme_log_running) ? "on" : "off",
			    (int)atomic_get(&bme_log_interval_s));
		shell_print(shell, "Log file: %s", path);
		boot_log_write("bme_log: status queried");
		return 0;
	}

	shell_error(shell, "Unknown command. Use: start|stop|interval|status");
	return -EINVAL;
}

static void bme_print_help(const struct shell *shell)
{
	shell_print(shell, "Usage:");
	shell_print(shell, "  bme read");
	shell_print(shell, "  bme status");
	shell_print(shell, "  bme log <start [interval_s] [path]|stop|interval <s>|status>");
	shell_print(shell, "  bme config <osr|filter|heater>");
}

static int cmd_bme(const struct shell *shell, size_t argc, char **argv)
{
	struct sensor_value temp;
	struct sensor_value press;
	struct sensor_value humidity;
	struct sensor_value gas;
	char temp_buf[20];
	char press_buf[20];
	char hum_buf[20];
	char gas_buf[20];
	int rc;

	if (argc < 2) {
		bme_print_help(shell);
		return -EINVAL;
	}

	if (strcmp(argv[1], "status") == 0) {
		if (!device_is_ready(bme680)) {
			shell_print(shell, "BME680 not ready.");
			return -ENODEV;
		}

		shell_print(shell, "BME680 ready.");
		rc = bme680_i2c_ready();
		if (rc < 0) {
			shell_error(shell, "BME680 I2C bus not ready.");
			return rc;
		}

		return bme_print_config(shell);
	}

	if (strcmp(argv[1], "log") == 0) {
		return cmd_bme_log(shell, argc - 1, &argv[1]);
	}

	if (strcmp(argv[1], "config") == 0) {
		return cmd_bme_config(shell, argc - 1, &argv[1]);
	}

	if (strcmp(argv[1], "read") != 0) {
		shell_error(shell, "Unknown command. Use: read|status|log|config");
		return -EINVAL;
	}

	if (!device_is_ready(bme680)) {
		shell_error(shell, "BME680 device not ready.");
		return -ENODEV;
	}

	rc = sensor_sample_fetch(bme680);
	if (rc < 0) {
		shell_error(shell, "BME680 fetch failed (%d).", rc);
		return rc;
	}

	rc = sensor_channel_get(bme680, SENSOR_CHAN_AMBIENT_TEMP, &temp);
	if (rc < 0) {
		shell_error(shell, "BME680 temp read failed (%d).", rc);
		return rc;
	}

	rc = sensor_channel_get(bme680, SENSOR_CHAN_PRESS, &press);
	if (rc < 0) {
		shell_error(shell, "BME680 pressure read failed (%d).", rc);
		return rc;
	}

	rc = sensor_channel_get(bme680, SENSOR_CHAN_HUMIDITY, &humidity);
	if (rc < 0) {
		shell_error(shell, "BME680 humidity read failed (%d).", rc);
		return rc;
	}

	rc = sensor_channel_get(bme680, SENSOR_CHAN_GAS_RES, &gas);
	if (rc < 0) {
		shell_error(shell, "BME680 gas read failed (%d).", rc);
		return rc;
	}

	format_micro_value(temp_buf, sizeof(temp_buf), sensor_value_to_micro(&temp));
	format_micro_value(press_buf, sizeof(press_buf), sensor_value_to_micro(&press));
	format_micro_value(hum_buf, sizeof(hum_buf), sensor_value_to_micro(&humidity));
	format_micro_value(gas_buf, sizeof(gas_buf), sensor_value_to_micro(&gas));

	shell_print(shell, "Temp: %s C", temp_buf);
	shell_print(shell, "Press: %s kPa", press_buf);
	shell_print(shell, "Humidity: %s %%", hum_buf);
	shell_print(shell, "Gas: %s ohm", gas_buf);
	return 0;
}

SHELL_CMD_REGISTER(bme, NULL, "BME680 sensor control.", cmd_bme);

#define LTR303_NODE DT_NODELABEL(ltr303)
#define LTR303_STARTUP_MS 100
#define LTR303_WAKEUP_MS 10

#define LTR303_REG_ALS_CONTR 0x80
#define LTR303_REG_MEAS_RATE 0x85
#define LTR303_REG_PART_ID 0x86
#define LTR303_REG_MANUF_ID 0x87
#define LTR303_REG_ALS_DATA_CH1_0 0x88
#define LTR303_REG_ALS_DATA_CH1_1 0x89
#define LTR303_REG_ALS_DATA_CH0_0 0x8A
#define LTR303_REG_ALS_DATA_CH0_1 0x8B
#define LTR303_REG_ALS_STATUS 0x8C
#define LTR303_REG_INTERRUPT 0x8F
#define LTR303_REG_THRES_UP_0 0x97
#define LTR303_REG_THRES_UP_1 0x98
#define LTR303_REG_THRES_LOW_0 0x99
#define LTR303_REG_THRES_LOW_1 0x9A

#define LTR303_ALS_CONTR_MODE_MASK BIT(0)
#define LTR303_ALS_CONTR_MODE_SHIFT 0
#define LTR303_ALS_CONTR_GAIN_MASK GENMASK(4, 2)
#define LTR303_ALS_CONTR_GAIN_SHIFT 2

#define LTR303_MEAS_RATE_REPEAT_MASK GENMASK(2, 0)
#define LTR303_MEAS_RATE_REPEAT_SHIFT 0
#define LTR303_MEAS_RATE_INT_TIME_MASK GENMASK(5, 3)
#define LTR303_MEAS_RATE_INT_TIME_SHIFT 3

#define LTR303_ALS_STATUS_DATA_READY_MASK BIT(2)

#define LTR303_REG_SET(reg, field, value) \
	(((value) << reg##_##field##_SHIFT) & reg##_##field##_MASK)
#define LTR303_REG_GET(reg, field, value) \
	(((value) & reg##_##field##_MASK) >> reg##_##field##_SHIFT)

#define LTR303_DEFAULT_GAIN DT_PROP_OR(LTR303_NODE, gain, 0)
#define LTR303_DEFAULT_INTEGRATION_TIME DT_PROP_OR(LTR303_NODE, integration_time, 0)
#define LTR303_DEFAULT_MEAS_RATE DT_PROP_OR(LTR303_NODE, measurement_rate, 3)

static const struct i2c_dt_spec ltr303_bus = I2C_DT_SPEC_GET(LTR303_NODE);
static const struct gpio_dt_spec ltr303_int_gpio =
	GPIO_DT_SPEC_GET_OR(LTR303_NODE, int_gpios, {0});
static struct gpio_callback ltr303_int_cb;
static atomic_t ltr303_int_count = ATOMIC_INIT(0);
static bool ltr303_int_enabled;
static bool ltr303_int_callback_added;
bool ltr303_configured;
bool ltr303_enabled;
static uint8_t ltr303_gain = LTR303_DEFAULT_GAIN;
static uint8_t ltr303_integration_time = LTR303_DEFAULT_INTEGRATION_TIME;
static uint8_t ltr303_measurement_rate = LTR303_DEFAULT_MEAS_RATE;

int ltr303_check_bus(void)
{
	if (!i2c_is_ready_dt(&ltr303_bus)) {
		return -ENODEV;
	}

	return 0;
}

static int ltr303_reg_read(uint8_t reg, uint8_t *val)
{
	return i2c_reg_read_byte_dt(&ltr303_bus, reg, val);
}

static int ltr303_reg_write(uint8_t reg, uint8_t val)
{
	return i2c_reg_write_byte_dt(&ltr303_bus, reg, val);
}

static int ltr303_get_mapped_gain(uint8_t reg_val, uint8_t *output)
{
	static const uint8_t gain_lux_calc[] = {1, 2, 4, 8, 0, 0, 48, 96};

	if (reg_val < ARRAY_SIZE(gain_lux_calc)) {
		*output = gain_lux_calc[reg_val];
		return (*output == 0) ? -EINVAL : 0;
	}

	return -EINVAL;
}

static int ltr303_get_mapped_int_time(uint8_t reg_val, uint8_t *output)
{
	static const uint8_t int_time_lux_calc[] = {10, 5, 20, 40, 15, 25, 30, 35};

	if (reg_val < ARRAY_SIZE(int_time_lux_calc)) {
		*output = int_time_lux_calc[reg_val];
		return 0;
	}

	*output = 0;
	return -EINVAL;
}

static int ltr303_validate_gain(uint8_t gain)
{
	uint8_t mapped;

	return ltr303_get_mapped_gain(gain, &mapped);
}

static int ltr303_validate_integration_time(uint8_t integration_time)
{
	uint8_t mapped;

	return ltr303_get_mapped_int_time(integration_time, &mapped);
}

static int ltr303_validate_measurement_rate(uint8_t rate)
{
	return (rate <= 5U) ? 0 : -EINVAL;
}

static int ltr303_apply_config(bool enable)
{
	uint8_t ctrl = LTR303_REG_SET(LTR303_ALS_CONTR, MODE, enable ? 1U : 0U) |
		       LTR303_REG_SET(LTR303_ALS_CONTR, GAIN, ltr303_gain);
	uint8_t meas = LTR303_REG_SET(LTR303_MEAS_RATE, REPEAT, ltr303_measurement_rate) |
		       LTR303_REG_SET(LTR303_MEAS_RATE, INT_TIME, ltr303_integration_time);
	int rc;

	rc = ltr303_reg_write(LTR303_REG_ALS_CONTR, ctrl);
	if (rc < 0) {
		return rc;
	}

	rc = ltr303_reg_write(LTR303_REG_MEAS_RATE, meas);
	if (rc < 0) {
		return rc;
	}

	ltr303_enabled = enable;
	ltr303_configured = true;

	if (enable) {
		k_sleep(K_MSEC(LTR303_WAKEUP_MS));
	}

	return 0;
}

static int ltr303_read_id(uint8_t *part_id, uint8_t *manuf_id)
{
	int rc;

	rc = ltr303_reg_read(LTR303_REG_PART_ID, part_id);
	if (rc < 0) {
		return rc;
	}

	rc = ltr303_reg_read(LTR303_REG_MANUF_ID, manuf_id);
	if (rc < 0) {
		return rc;
	}

	return 0;
}

int ltr303_init_device(void)
{
	int rc;

	rc = ltr303_check_bus();
	if (rc < 0) {
		return rc;
	}

	rc = ltr303_validate_gain(ltr303_gain);
	if (rc < 0) {
		return rc;
	}

	rc = ltr303_validate_integration_time(ltr303_integration_time);
	if (rc < 0) {
		return rc;
	}

	rc = ltr303_validate_measurement_rate(ltr303_measurement_rate);
	if (rc < 0) {
		return rc;
	}

	k_sleep(K_MSEC(LTR303_STARTUP_MS));

	rc = ltr303_apply_config(true);
	if (rc < 0) {
		return rc;
	}

	return 0;
}

static int ltr303_data_ready(void)
{
	uint8_t status;
	int rc = ltr303_reg_read(LTR303_REG_ALS_STATUS, &status);

	if (rc < 0) {
		return rc;
	}

	if ((status & LTR303_ALS_STATUS_DATA_READY_MASK) == 0U) {
		return -EBUSY;
	}

	return 0;
}

int ltr303_wait_data_ready(uint32_t timeout_ms)
{
	const uint32_t step_ms = 20;
	uint32_t waited = 0;

	while (waited < timeout_ms) {
		int rc = ltr303_data_ready();

		if (rc == 0) {
			return 0;
		}
		if (rc != -EBUSY) {
			return rc;
		}

		k_sleep(K_MSEC(step_ms));
		waited += step_ms;
	}

	return -ETIMEDOUT;
}

int ltr303_read_channels(uint16_t *ch0, uint16_t *ch1)
{
	uint8_t buf[4];
	int rc = i2c_burst_read_dt(&ltr303_bus, LTR303_REG_ALS_DATA_CH1_0,
			   buf, sizeof(buf));

	if (rc < 0) {
		return rc;
	}

	*ch1 = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
	*ch0 = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);

	return 0;
}

int ltr303_calc_lux(uint16_t ch0, uint16_t ch1, struct sensor_value *val)
{
	uint8_t gain_value;
	uint8_t integration_time_value;
	uint64_t lux;
	uint64_t scaled_ratio;

	if (ch0 == 0U && ch1 == 0U) {
		return -EINVAL;
	}

	if (ltr303_get_mapped_gain(ltr303_gain, &gain_value) != 0) {
		return -EINVAL;
	}

	if (ltr303_get_mapped_int_time(ltr303_integration_time,
				       &integration_time_value) != 0) {
		return -EINVAL;
	}

	scaled_ratio = (uint64_t)ch1 * UINT64_C(1000000) / (uint64_t)(ch0 + ch1);

	if (scaled_ratio < UINT64_C(450000)) {
		lux = (UINT64_C(1774300) * ch0 + UINT64_C(1105900) * ch1);
	} else if (scaled_ratio < UINT64_C(640000)) {
		lux = (UINT64_C(4278500) * ch0 - UINT64_C(1954800) * ch1);
	} else if (scaled_ratio < UINT64_C(850000)) {
		lux = (UINT64_C(592600) * ch0 + UINT64_C(118500) * ch1);
	} else {
		return -EINVAL;
	}

	lux = (lux * 10U) / ((uint64_t)gain_value * integration_time_value);

	val->val1 = (int32_t)(lux / UINT64_C(1000000));
	val->val2 = (int32_t)(lux % UINT64_C(1000000));

	return 0;
}

static void ltr303_int_handler(const struct device *port,
			       struct gpio_callback *cb,
			       uint32_t pins)
{
	(void)port;
	(void)cb;
	(void)pins;

	atomic_inc(&ltr303_int_count);
}

static int ltr303_int_configure(bool enable)
{
	int rc;

	if (!ltr303_int_gpio.port) {
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&ltr303_int_gpio)) {
		return -ENODEV;
	}

	rc = gpio_pin_configure_dt(&ltr303_int_gpio, GPIO_INPUT);
	if (rc < 0) {
		return rc;
	}

	if (!ltr303_int_callback_added) {
		gpio_init_callback(&ltr303_int_cb, ltr303_int_handler,
				   BIT(ltr303_int_gpio.pin));
		rc = gpio_add_callback(ltr303_int_gpio.port, &ltr303_int_cb);
		if (rc < 0) {
			return rc;
		}
		ltr303_int_callback_added = true;
	}

	if (enable) {
		rc = gpio_pin_interrupt_configure_dt(&ltr303_int_gpio,
				     GPIO_INT_EDGE_TO_ACTIVE);
		if (rc < 0) {
			return rc;
		}
		ltr303_int_enabled = true;
		return 0;
	}

	rc = gpio_pin_interrupt_configure_dt(&ltr303_int_gpio, GPIO_INT_DISABLE);
	if (rc < 0) {
		return rc;
	}

	ltr303_int_enabled = false;
	return 0;
}

static int ltr303_parse_u8(const struct shell *shell, const char *arg, uint8_t *out)
{
	char *end = NULL;
	unsigned long val = strtoul(arg, &end, 0);

	if (end == arg || *end != '\0' || val > 0xFFUL) {
		shell_error(shell, "Invalid value: %s", arg);
		return -EINVAL;
	}

	*out = (uint8_t)val;
	return 0;
}

static int ltr303_parse_u16(const struct shell *shell, const char *arg, uint16_t *out)
{
	char *end = NULL;
	unsigned long val = strtoul(arg, &end, 0);

	if (end == arg || *end != '\0' || val > 0xFFFFUL) {
		shell_error(shell, "Invalid value: %s", arg);
		return -EINVAL;
	}

	*out = (uint16_t)val;
	return 0;
}

static int cmd_ltr_log(const struct shell *shell, size_t argc, char **argv)
{
	int interval;
	const char *path_arg = NULL;
	char path[LTR_LOG_PATH_MAX];
	int rc;

	if (argc < 2) {
		shell_print(shell,
			    "Usage: ltr log <start [interval_s] [path]|stop|interval <s>|status>");
		return -EINVAL;
	}

	if (strcmp(argv[1], "start") == 0) {
		interval = atomic_get(&ltr_log_interval_s);

		if (argc >= 3) {
			char *end = NULL;
			long val = strtol(argv[2], &end, 0);

			if (end != argv[2] && *end == '\0') {
				interval = (int)val;
			} else {
				path_arg = argv[2];
			}
		}

		if (argc >= 4) {
			if (path_arg) {
				shell_error(shell, "Too many arguments.");
				return -EINVAL;
			}
			path_arg = argv[3];
		}

		if (argc > 4) {
			shell_error(shell, "Too many arguments.");
			return -EINVAL;
		}

		if (interval < 1) {
			shell_error(shell, "Interval must be >= 1 second.");
			return -EINVAL;
		}

		if (path_arg) {
			rc = ltr_log_set_path(path_arg);
			if (rc < 0) {
				shell_error(shell, "Invalid log path (%d).", rc);
				return rc;
			}
		}

		rc = ltr303_check_bus();
		if (rc < 0) {
			shell_error(shell, "LTR303 I2C bus not ready.");
			return rc;
		}

		if (!ltr303_configured) {
			rc = ltr303_init_device();
			if (rc < 0) {
				shell_error(shell, "LTR303 init failed (%d).", rc);
				return rc;
			}
		}

		rc = ensure_littlefs_ready();
		if (rc < 0) {
			shell_error(shell, "LittleFS not mounted (%d).", rc);
			return rc;
		}

		rc = ensure_ltr_log_path();
		if (rc < 0) {
			shell_error(shell, "Failed to prepare log path (%d).", rc);
			return rc;
		}

		atomic_set(&ltr_log_interval_s, interval);
		k_event_post(&ltr_log_event, LTR_LOG_EVENT_INTERVAL);
		atomic_set(&ltr_log_running, 1);
		k_event_post(&ltr_log_event, LTR_LOG_EVENT_START);
		ltr_log_get_path(path, sizeof(path));
		shell_print(shell, "LTR303 logging started (interval %d s).", interval);
		shell_print(shell, "Log file: %s", path);
		boot_log_write("ltr_log: started interval %d s", interval);
		return 0;
	}

	if (strcmp(argv[1], "stop") == 0) {
		atomic_set(&ltr_log_running, 0);
		k_event_post(&ltr_log_event, LTR_LOG_EVENT_STOP);
		shell_print(shell, "LTR303 logging stopped.");
		boot_log_write("ltr_log: stopped");
		return 0;
	}

	if (strcmp(argv[1], "interval") == 0) {
		if (argc < 3) {
			shell_error(shell, "Missing interval in seconds.");
			return -EINVAL;
		}

		interval = (int)strtol(argv[2], NULL, 0);
		if (interval < 1) {
			shell_error(shell, "Interval must be >= 1 second.");
			return -EINVAL;
		}

		atomic_set(&ltr_log_interval_s, interval);
		k_event_post(&ltr_log_event, LTR_LOG_EVENT_INTERVAL);
		shell_print(shell, "LTR303 log interval set to %d s.", interval);
		boot_log_write("ltr_log: interval set %d s", interval);
		return 0;
	}

	if (strcmp(argv[1], "status") == 0) {
		ltr_log_get_path(path, sizeof(path));
		shell_print(shell, "LTR303 logging %s, interval %d s.",
			    atomic_get(&ltr_log_running) ? "on" : "off",
			    (int)atomic_get(&ltr_log_interval_s));
		shell_print(shell, "Log file: %s", path);
		boot_log_write("ltr_log: status queried");
		return 0;
	}

	shell_error(shell, "Unknown command. Use: start|stop|interval|status");
	return -EINVAL;
}

static void ltr_print_help(const struct shell *shell)
{
	shell_print(shell, "Usage:");
	shell_print(shell, "  ltr init|read|enable|disable");
	shell_print(shell, "  ltr gain <val>|integration <val>|rate <val>");
	shell_print(shell, "  ltr status|id");
	shell_print(shell, "  ltr log <start [interval_s] [path]|stop|interval <s>|status>");
	shell_print(shell, "  ltr int <on|off|status|clear>");
	shell_print(shell, "  ltr reg <read|write> <addr> [val]");
	shell_print(shell, "  ltr threshold <low> <high>");
}

static int cmd_ltr303(const struct shell *shell, size_t argc, char **argv)
{
	int rc;

	if (argc < 2) {
		ltr_print_help(shell);
		return -EINVAL;
	}

	rc = ltr303_check_bus();
	if (rc < 0) {
		shell_error(shell, "LTR303 I2C bus not ready.");
		return rc;
	}

	if (strcmp(argv[1], "init") == 0) {
		uint8_t part_id = 0;
		uint8_t manuf_id = 0;

		rc = ltr303_init_device();
		if (rc < 0) {
			shell_error(shell, "LTR303 init failed (%d).", rc);
			return rc;
		}

		rc = ltr303_read_id(&part_id, &manuf_id);
		if (rc == 0) {
			shell_print(shell, "LTR303 ready. PART_ID=0x%02x MANUF_ID=0x%02x",
				    part_id, manuf_id);
		} else {
			shell_print(shell, "LTR303 ready (ID read failed %d).", rc);
		}
		boot_log_write("ltr303: init ok");
		return 0;
	}

	if (strcmp(argv[1], "read") == 0) {
		uint16_t ch0;
		uint16_t ch1;
		struct sensor_value lux;

		if (!ltr303_configured) {
			rc = ltr303_init_device();
			if (rc < 0) {
				shell_error(shell, "LTR303 init failed (%d).", rc);
				return rc;
			}
		}

		rc = ltr303_wait_data_ready(500);
		if (rc < 0 && rc != -ETIMEDOUT) {
			shell_error(shell, "LTR303 data not ready (%d).", rc);
			return rc;
		}

		rc = ltr303_read_channels(&ch0, &ch1);
		if (rc < 0) {
			shell_error(shell, "LTR303 read failed (%d).", rc);
			return rc;
		}

		shell_print(shell, "LTR303 CH0=%u CH1=%u", ch0, ch1);
		rc = ltr303_calc_lux(ch0, ch1, &lux);
		if (rc == 0) {
			shell_print(shell, "Lux: %d.%06d", lux.val1,
				    (int)abs(lux.val2));
		} else {
			shell_print(shell, "Lux: n/a");
		}

		return 0;
	}

	if (strcmp(argv[1], "log") == 0) {
		return cmd_ltr_log(shell, argc - 1, &argv[1]);
	}

	if (strcmp(argv[1], "enable") == 0 || strcmp(argv[1], "disable") == 0) {
		bool enable = (strcmp(argv[1], "enable") == 0);

		rc = ltr303_apply_config(enable);
		if (rc < 0) {
			shell_error(shell, "LTR303 %s failed (%d).",
				    enable ? "enable" : "disable", rc);
			return rc;
		}

		shell_print(shell, "LTR303 %s.", enable ? "enabled" : "disabled");
		return 0;
	}

	if (strcmp(argv[1], "gain") == 0 || strcmp(argv[1], "integration") == 0 ||
	    strcmp(argv[1], "rate") == 0) {
		uint8_t val;

		if (argc < 3) {
			shell_error(shell, "Missing value.");
			return -EINVAL;
		}

		rc = ltr303_parse_u8(shell, argv[2], &val);
		if (rc < 0) {
			return rc;
		}

		if (strcmp(argv[1], "gain") == 0) {
			if (ltr303_validate_gain(val) < 0) {
				shell_error(shell, "Invalid gain value.");
				return -EINVAL;
			}
			ltr303_gain = val;
		} else if (strcmp(argv[1], "integration") == 0) {
			if (ltr303_validate_integration_time(val) < 0) {
				shell_error(shell, "Invalid integration value.");
				return -EINVAL;
			}
			ltr303_integration_time = val;
		} else {
			if (ltr303_validate_measurement_rate(val) < 0) {
				shell_error(shell, "Invalid measurement rate.");
				return -EINVAL;
			}
			ltr303_measurement_rate = val;
		}

		if (!ltr303_configured) {
			rc = ltr303_init_device();
		} else {
			rc = ltr303_apply_config(ltr303_enabled);
		}
		if (rc < 0) {
			shell_error(shell, "LTR303 config write failed (%d).", rc);
			return rc;
		}

		shell_print(shell, "LTR303 config updated.");
		return 0;
	}

	if (strcmp(argv[1], "status") == 0) {
		int level = -ENODEV;

		shell_print(shell, "LTR303 %s",
			    ltr303_enabled ? "enabled" : "disabled");
		shell_print(shell, "Gain %u, integration %u, rate %u",
			    ltr303_gain, ltr303_integration_time,
			    ltr303_measurement_rate);
		if (ltr303_int_gpio.port && gpio_is_ready_dt(&ltr303_int_gpio)) {
			level = gpio_pin_get_dt(&ltr303_int_gpio);
			if (level < 0) {
				shell_print(shell, "INT: %s, count %d (level err %d)",
					    ltr303_int_enabled ? "on" : "off",
					    (int)atomic_get(&ltr303_int_count),
					    level);
			} else {
				shell_print(shell, "INT: %s, count %d, level %d",
					    ltr303_int_enabled ? "on" : "off",
					    (int)atomic_get(&ltr303_int_count),
					    level);
			}
		} else {
			shell_print(shell, "INT: not available");
		}
		return 0;
	}

	if (strcmp(argv[1], "id") == 0) {
		uint8_t part_id = 0;
		uint8_t manuf_id = 0;

		rc = ltr303_read_id(&part_id, &manuf_id);
		if (rc < 0) {
			shell_error(shell, "LTR303 ID read failed (%d).", rc);
			return rc;
		}

		shell_print(shell, "PART_ID=0x%02x MANUF_ID=0x%02x",
			    part_id, manuf_id);
		return 0;
	}

	if (strcmp(argv[1], "int") == 0) {
		if (argc < 3) {
			shell_error(shell, "Usage: ltr int <on|off|status|clear>");
			return -EINVAL;
		}

		if (strcmp(argv[2], "on") == 0) {
			rc = ltr303_int_configure(true);
			if (rc < 0) {
				shell_error(shell, "INT enable failed (%d).", rc);
				return rc;
			}
			shell_print(shell, "LTR303 INT enabled.");
			return 0;
		}

		if (strcmp(argv[2], "off") == 0) {
			rc = ltr303_int_configure(false);
			if (rc < 0) {
				shell_error(shell, "INT disable failed (%d).", rc);
				return rc;
			}
			shell_print(shell, "LTR303 INT disabled.");
			return 0;
		}

		if (strcmp(argv[2], "clear") == 0) {
			atomic_set(&ltr303_int_count, 0);
			shell_print(shell, "LTR303 INT count cleared.");
			return 0;
		}

		if (strcmp(argv[2], "status") == 0) {
			shell_print(shell, "LTR303 INT %s, count %d",
				    ltr303_int_enabled ? "on" : "off",
				    (int)atomic_get(&ltr303_int_count));
			return 0;
		}

		shell_error(shell, "Unknown int command.");
		return -EINVAL;
	}

	if (strcmp(argv[1], "reg") == 0) {
		uint8_t reg;

		if (argc < 4) {
			shell_error(shell, "Usage: ltr reg <read|write> <addr> [value]");
			return -EINVAL;
		}

		rc = ltr303_parse_u8(shell, argv[3], &reg);
		if (rc < 0) {
			return rc;
		}

		if (strcmp(argv[2], "read") == 0) {
			uint8_t val;

			rc = ltr303_reg_read(reg, &val);
			if (rc < 0) {
				shell_error(shell, "Reg read failed (%d).", rc);
				return rc;
			}
			shell_print(shell, "Reg 0x%02x = 0x%02x", reg, val);
			return 0;
		}

		if (strcmp(argv[2], "write") == 0) {
			uint8_t val;

			if (argc < 5) {
				shell_error(shell, "Missing value.");
				return -EINVAL;
			}

			rc = ltr303_parse_u8(shell, argv[4], &val);
			if (rc < 0) {
				return rc;
			}

			rc = ltr303_reg_write(reg, val);
			if (rc < 0) {
				shell_error(shell, "Reg write failed (%d).", rc);
				return rc;
			}
			shell_print(shell, "Reg 0x%02x <- 0x%02x", reg, val);
			return 0;
		}

		shell_error(shell, "Unknown reg command.");
		return -EINVAL;
	}

	if (strcmp(argv[1], "threshold") == 0) {
		uint16_t low;
		uint16_t high;

		if (argc < 4) {
			shell_error(shell, "Usage: ltr threshold <low> <high>");
			return -EINVAL;
		}

		rc = ltr303_parse_u16(shell, argv[2], &low);
		if (rc < 0) {
			return rc;
		}

		rc = ltr303_parse_u16(shell, argv[3], &high);
		if (rc < 0) {
			return rc;
		}

		rc = ltr303_reg_write(LTR303_REG_THRES_LOW_0, low & 0xFF);
		if (rc == 0) {
			rc = ltr303_reg_write(LTR303_REG_THRES_LOW_1,
				      (low >> 8) & 0xFF);
		}
		if (rc == 0) {
			rc = ltr303_reg_write(LTR303_REG_THRES_UP_0, high & 0xFF);
		}
		if (rc == 0) {
			rc = ltr303_reg_write(LTR303_REG_THRES_UP_1,
				      (high >> 8) & 0xFF);
		}
		if (rc < 0) {
			shell_error(shell, "Threshold write failed (%d).", rc);
			return rc;
		}

		shell_print(shell, "LTR303 thresholds set low=%u high=%u", low, high);
		return 0;
	}

	shell_error(shell, "Unknown command.");
	return -EINVAL;
}

SHELL_CMD_REGISTER(ltr, NULL, "LTR303 light sensor control.", cmd_ltr303);

static bool lsm6dsl_odr_supported(int hz)
{
	switch (hz) {
	case 1:
	case 12:
	case 26:
	case 52:
	case 104:
	case 208:
	case 416:
	case 833:
	case 1666:
	case 3332:
	case 6664:
		return true;
	default:
		return false;
	}
}

static bool lsm6dsl_accel_range_supported(int g)
{
	switch (g) {
	case 2:
	case 4:
	case 8:
	case 16:
		return true;
	default:
		return false;
	}
}

static bool lsm6dsl_gyro_range_supported(int dps)
{
	switch (dps) {
	case 125:
	case 250:
	case 500:
	case 1000:
	case 2000:
		return true;
	default:
		return false;
	}
}

static int lsm6dsl_set_accel_odr(int hz)
{
	struct sensor_value val = { .val1 = hz, .val2 = 0 };
	int rc;

	if (!lsm6dsl_odr_supported(hz)) {
		return -EINVAL;
	}

	rc = sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ,
			     SENSOR_ATTR_SAMPLING_FREQUENCY, &val);
	if (rc == 0) {
		lsm6dsl_accel_odr_hz = hz;
		lsm6dsl_powered = true;
	}

	return rc;
}

static int lsm6dsl_set_gyro_odr(int hz)
{
	struct sensor_value val = { .val1 = hz, .val2 = 0 };
	int rc;

	if (!lsm6dsl_odr_supported(hz)) {
		return -EINVAL;
	}

	rc = sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ,
			     SENSOR_ATTR_SAMPLING_FREQUENCY, &val);
	if (rc == 0) {
		lsm6dsl_gyro_odr_hz = hz;
		lsm6dsl_powered = true;
	}

	return rc;
}

static int lsm6dsl_set_accel_range(int g)
{
	struct sensor_value val;
	int rc;

	if (!lsm6dsl_accel_range_supported(g)) {
		return -EINVAL;
	}

	sensor_g_to_ms2(g, &val);
	rc = sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ,
			     SENSOR_ATTR_FULL_SCALE, &val);
	if (rc == 0) {
		lsm6dsl_accel_range_g = g;
	}

	return rc;
}

static int lsm6dsl_set_gyro_range(int dps)
{
	struct sensor_value val;
	int rc;

	if (!lsm6dsl_gyro_range_supported(dps)) {
		return -EINVAL;
	}

	sensor_degrees_to_rad(dps, &val);
	rc = sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ,
			     SENSOR_ATTR_FULL_SCALE, &val);
	if (rc == 0) {
		lsm6dsl_gyro_range_dps = dps;
	}

	return rc;
}

static void lsm6dsl_apply_defaults(void)
{
	int rc;
	int rc_accel;
	int rc_gyro;

	if (!device_is_ready(lsm6dsl_dev)) {
		boot_log_write("lsm6dsl: not ready");
		return;
	}

	rc_accel = lsm6dsl_set_accel_odr(LSM6DSL_DEFAULT_ACCEL_ODR_HZ);
	if (rc_accel < 0) {
		boot_log_write("lsm6dsl: accel ODR set failed (%d)", rc_accel);
	}

	rc_gyro = lsm6dsl_set_gyro_odr(LSM6DSL_DEFAULT_GYRO_ODR_HZ);
	if (rc_gyro < 0) {
		boot_log_write("lsm6dsl: gyro ODR set failed (%d)", rc_gyro);
	}

	rc = lsm6dsl_set_accel_range(LSM6DSL_DEFAULT_ACCEL_RANGE_G);
	if (rc < 0) {
		boot_log_write("lsm6dsl: accel range set failed (%d)", rc);
	}

	rc = lsm6dsl_set_gyro_range(LSM6DSL_DEFAULT_GYRO_RANGE_DPS);
	if (rc < 0) {
		boot_log_write("lsm6dsl: gyro range set failed (%d)", rc);
	}

	if (rc_accel == 0 && rc_gyro == 0) {
		lsm6dsl_powered = true;
	} else {
		lsm6dsl_powered = false;
	}
}

static int lsm6dsl_set_power(bool on)
{
	struct sensor_value val = { .val1 = 0, .val2 = 0 };
	int rc_accel;
	int rc_gyro;

	if (!device_is_ready(lsm6dsl_dev)) {
		return -ENODEV;
	}

	if (on) {
		rc_accel = lsm6dsl_set_accel_odr(lsm6dsl_accel_odr_hz);
		rc_gyro = lsm6dsl_set_gyro_odr(lsm6dsl_gyro_odr_hz);
	} else {
		rc_accel = sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ,
				   SENSOR_ATTR_SAMPLING_FREQUENCY, &val);
		rc_gyro = sensor_attr_set(lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ,
				  SENSOR_ATTR_SAMPLING_FREQUENCY, &val);
	}

	if (rc_accel < 0) {
		return rc_accel;
	}
	if (rc_gyro < 0) {
		return rc_gyro;
	}

	lsm6dsl_powered = on;
	return 0;
}

static void lsm6dsl_int_handler(const struct device *port,
				struct gpio_callback *cb,
				uint32_t pins)
{
	(void)port;
	(void)cb;
	(void)pins;

	atomic_inc(&lsm6dsl_int_count);
}

static int lsm6dsl_int_configure(bool enable)
{
	int rc;

	if (!lsm6dsl_int_gpio.port) {
		return -ENODEV;
	}

	if (!gpio_is_ready_dt(&lsm6dsl_int_gpio)) {
		return -ENODEV;
	}

	rc = gpio_pin_configure_dt(&lsm6dsl_int_gpio, GPIO_INPUT);
	if (rc < 0) {
		return rc;
	}

	if (!lsm6dsl_int_callback_added) {
		gpio_init_callback(&lsm6dsl_int_cb, lsm6dsl_int_handler,
				   BIT(lsm6dsl_int_gpio.pin));
		rc = gpio_add_callback(lsm6dsl_int_gpio.port, &lsm6dsl_int_cb);
		if (rc < 0) {
			return rc;
		}
		lsm6dsl_int_callback_added = true;
	}

	if (enable) {
		rc = gpio_pin_interrupt_configure_dt(&lsm6dsl_int_gpio,
				     GPIO_INT_EDGE_TO_ACTIVE);
		if (rc < 0) {
			return rc;
		}
		lsm6dsl_int_enabled = true;
		return 0;
	}

	rc = gpio_pin_interrupt_configure_dt(&lsm6dsl_int_gpio, GPIO_INT_DISABLE);
	if (rc < 0) {
		return rc;
	}

	lsm6dsl_int_enabled = false;
	return 0;
}

static void lsm6dsl_print_help(const struct shell *shell)
{
	shell_print(shell, "Usage:");
	shell_print(shell, "  lsm6dsl status|read");
	shell_print(shell, "  lsm6dsl accel <odr|range|status> [value]");
	shell_print(shell, "  lsm6dsl gyro <odr|range|status> [value]");
	shell_print(shell, "  lsm6dsl power <on|off|status>");
	shell_print(shell, "  lsm6dsl log <start [interval_s] [path]|stop|interval <s>|status>");
	shell_print(shell, "  lsm6dsl int <on|off|status|clear>");
}

static int lsm6dsl_parse_int(const struct shell *shell, const char *arg, int *out)
{
	char *end = NULL;
	long val = strtol(arg, &end, 0);

	if (end == arg || *end != '\0') {
		shell_error(shell, "Invalid value: %s", arg);
		return -EINVAL;
	}

	*out = (int)val;
	return 0;
}

static int cmd_lsm6dsl_log(const struct shell *shell, size_t argc, char **argv)
{
	int interval;
	const char *path_arg = NULL;
	char path[LSM6DSL_LOG_PATH_MAX];
	int rc;

	if (argc < 2) {
		shell_print(shell,
			    "Usage: lsm6dsl log <start [interval_s] [path]|stop|interval <s>|status>");
		return -EINVAL;
	}

	if (strcmp(argv[1], "start") == 0) {
		interval = atomic_get(&lsm6dsl_log_interval_s);

		if (argc >= 3) {
			char *end = NULL;
			long val = strtol(argv[2], &end, 0);

			if (end != argv[2] && *end == '\0') {
				interval = (int)val;
			} else {
				path_arg = argv[2];
			}
		}

		if (argc >= 4) {
			if (path_arg) {
				shell_error(shell, "Too many arguments.");
				return -EINVAL;
			}
			path_arg = argv[3];
		}

		if (argc > 4) {
			shell_error(shell, "Too many arguments.");
			return -EINVAL;
		}

		if (interval < 1) {
			shell_error(shell, "Interval must be >= 1 second.");
			return -EINVAL;
		}

		if (path_arg) {
			rc = lsm6dsl_log_set_path(path_arg);
			if (rc < 0) {
				shell_error(shell, "Invalid log path (%d).", rc);
				return rc;
			}
		}

		if (!device_is_ready(lsm6dsl_dev)) {
			shell_error(shell, "LSM6DSL device not ready.");
			return -ENODEV;
		}

		rc = ensure_littlefs_ready();
		if (rc < 0) {
			shell_error(shell, "LittleFS not mounted (%d).", rc);
			return rc;
		}

		rc = ensure_lsm6dsl_log_path();
		if (rc < 0) {
			shell_error(shell, "Failed to prepare log path (%d).", rc);
			return rc;
		}

		atomic_set(&lsm6dsl_log_interval_s, interval);
		k_event_post(&lsm6dsl_log_event, LSM6DSL_LOG_EVENT_INTERVAL);
		atomic_set(&lsm6dsl_log_running, 1);
		k_event_post(&lsm6dsl_log_event, LSM6DSL_LOG_EVENT_START);
		lsm6dsl_log_get_path(path, sizeof(path));
		shell_print(shell, "LSM6DSL logging started (interval %d s).", interval);
		shell_print(shell, "Log file: %s", path);
		boot_log_write("lsm6dsl_log: started interval %d s", interval);
		return 0;
	}

	if (strcmp(argv[1], "stop") == 0) {
		atomic_set(&lsm6dsl_log_running, 0);
		k_event_post(&lsm6dsl_log_event, LSM6DSL_LOG_EVENT_STOP);
		shell_print(shell, "LSM6DSL logging stopped.");
		boot_log_write("lsm6dsl_log: stopped");
		return 0;
	}

	if (strcmp(argv[1], "interval") == 0) {
		if (argc < 3) {
			shell_error(shell, "Missing interval in seconds.");
			return -EINVAL;
		}

		interval = (int)strtol(argv[2], NULL, 0);
		if (interval < 1) {
			shell_error(shell, "Interval must be >= 1 second.");
			return -EINVAL;
		}

		atomic_set(&lsm6dsl_log_interval_s, interval);
		k_event_post(&lsm6dsl_log_event, LSM6DSL_LOG_EVENT_INTERVAL);
		shell_print(shell, "LSM6DSL log interval set to %d s.", interval);
		boot_log_write("lsm6dsl_log: interval set %d s", interval);
		return 0;
	}

	if (strcmp(argv[1], "status") == 0) {
		lsm6dsl_log_get_path(path, sizeof(path));
		shell_print(shell, "LSM6DSL logging %s, interval %d s.",
			    atomic_get(&lsm6dsl_log_running) ? "on" : "off",
			    (int)atomic_get(&lsm6dsl_log_interval_s));
		shell_print(shell, "Log file: %s", path);
		boot_log_write("lsm6dsl_log: status queried");
		return 0;
	}

	shell_error(shell, "Unknown command. Use: start|stop|interval|status");
	return -EINVAL;
}

static int cmd_lsm6dsl(const struct shell *shell, size_t argc, char **argv)
{
	int rc;

	if (argc < 2) {
		lsm6dsl_print_help(shell);
		return -EINVAL;
	}

	if (strcmp(argv[1], "status") == 0) {
		int level = -1;

		if (!device_is_ready(lsm6dsl_dev)) {
			shell_print(shell, "LSM6DSL not ready.");
			return -ENODEV;
		}

		shell_print(shell, "LSM6DSL ready.");
		shell_print(shell, "Power: %s", lsm6dsl_powered ? "on" : "off");
		shell_print(shell, "Accel ODR: %d Hz, range: %d g",
			    lsm6dsl_accel_odr_hz, lsm6dsl_accel_range_g);
		shell_print(shell, "Gyro ODR: %d Hz, range: %d dps",
			    lsm6dsl_gyro_odr_hz, lsm6dsl_gyro_range_dps);
		if (lsm6dsl_int_gpio.port && gpio_is_ready_dt(&lsm6dsl_int_gpio)) {
			level = gpio_pin_get_dt(&lsm6dsl_int_gpio);
			shell_print(shell, "INT1: %s, count %d, level %d",
				    lsm6dsl_int_enabled ? "on" : "off",
				    (int)atomic_get(&lsm6dsl_int_count),
				    level);
		} else {
			shell_print(shell, "INT1: not configured");
		}
		return 0;
	}

	if (strcmp(argv[1], "read") == 0) {
		struct sensor_value accel[3];
		struct sensor_value gyro[3];
		struct sensor_value temp;
		char accel_buf[3][20];
		char gyro_buf[3][20];
		char temp_buf[20];

		if (!device_is_ready(lsm6dsl_dev)) {
			shell_error(shell, "LSM6DSL device not ready.");
			return -ENODEV;
		}

		rc = sensor_sample_fetch(lsm6dsl_dev);
		if (rc < 0) {
			shell_error(shell, "Sample fetch failed (%d).", rc);
			return rc;
		}

		rc = sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
		if (rc < 0) {
			shell_error(shell, "Accel read failed (%d).", rc);
			return rc;
		}

		rc = sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_GYRO_XYZ, gyro);
		if (rc < 0) {
			shell_error(shell, "Gyro read failed (%d).", rc);
			return rc;
		}

		rc = sensor_channel_get(lsm6dsl_dev, SENSOR_CHAN_DIE_TEMP, &temp);
		if (rc == 0) {
			format_micro_value(temp_buf, sizeof(temp_buf),
					   sensor_value_to_micro(&temp));
		} else if (rc == -ENOTSUP) {
			snprintk(temp_buf, sizeof(temp_buf), "n/a");
		} else {
			shell_error(shell, "Temp read failed (%d).", rc);
			return rc;
		}

		for (size_t i = 0; i < 3; i++) {
			format_micro_value(accel_buf[i], sizeof(accel_buf[i]),
					   sensor_value_to_micro(&accel[i]));
			format_micro_value(gyro_buf[i], sizeof(gyro_buf[i]),
					   sensor_value_to_micro(&gyro[i]));
		}

		shell_print(shell, "Accel (m/s^2): x=%s y=%s z=%s",
			    accel_buf[0], accel_buf[1], accel_buf[2]);
		shell_print(shell, "Gyro (rad/s): x=%s y=%s z=%s",
			    gyro_buf[0], gyro_buf[1], gyro_buf[2]);
		shell_print(shell, "Temp: %s C", temp_buf);
		return 0;
	}

	if (strcmp(argv[1], "accel") == 0) {
		int val;

		if (argc < 3) {
			shell_error(shell, "Usage: lsm6dsl accel <odr|range|status> [value]");
			return -EINVAL;
		}

		if (strcmp(argv[2], "status") == 0) {
			shell_print(shell, "Accel ODR: %d Hz, range: %d g",
				    lsm6dsl_accel_odr_hz, lsm6dsl_accel_range_g);
			return 0;
		}

		if (argc < 4) {
			shell_error(shell, "Missing value.");
			return -EINVAL;
		}

		rc = lsm6dsl_parse_int(shell, argv[3], &val);
		if (rc < 0) {
			return rc;
		}

		if (!device_is_ready(lsm6dsl_dev)) {
			shell_error(shell, "LSM6DSL device not ready.");
			return -ENODEV;
		}

		if (strcmp(argv[2], "odr") == 0) {
			rc = lsm6dsl_set_accel_odr(val);
			if (rc == -EINVAL) {
				shell_error(shell,
					    "Unsupported ODR. Use: 1,12,26,52,104,208,416,833,1666,3332,6664.");
				return rc;
			}
			if (rc < 0) {
				shell_error(shell, "Accel ODR set failed (%d).", rc);
				return rc;
			}
			shell_print(shell, "Accel ODR set to %d Hz.", val);
			return 0;
		}

		if (strcmp(argv[2], "range") == 0) {
			rc = lsm6dsl_set_accel_range(val);
			if (rc == -EINVAL) {
				shell_error(shell, "Unsupported range. Use: 2,4,8,16.");
				return rc;
			}
			if (rc < 0) {
				shell_error(shell, "Accel range set failed (%d).", rc);
				return rc;
			}
			shell_print(shell, "Accel range set to %d g.", val);
			return 0;
		}

		shell_error(shell, "Unknown accel command.");
		return -EINVAL;
	}

	if (strcmp(argv[1], "gyro") == 0) {
		int val;

		if (argc < 3) {
			shell_error(shell, "Usage: lsm6dsl gyro <odr|range|status> [value]");
			return -EINVAL;
		}

		if (strcmp(argv[2], "status") == 0) {
			shell_print(shell, "Gyro ODR: %d Hz, range: %d dps",
				    lsm6dsl_gyro_odr_hz, lsm6dsl_gyro_range_dps);
			return 0;
		}

		if (argc < 4) {
			shell_error(shell, "Missing value.");
			return -EINVAL;
		}

		rc = lsm6dsl_parse_int(shell, argv[3], &val);
		if (rc < 0) {
			return rc;
		}

		if (!device_is_ready(lsm6dsl_dev)) {
			shell_error(shell, "LSM6DSL device not ready.");
			return -ENODEV;
		}

		if (strcmp(argv[2], "odr") == 0) {
			rc = lsm6dsl_set_gyro_odr(val);
			if (rc == -EINVAL) {
				shell_error(shell,
					    "Unsupported ODR. Use: 1,12,26,52,104,208,416,833,1666,3332,6664.");
				return rc;
			}
			if (rc < 0) {
				shell_error(shell, "Gyro ODR set failed (%d).", rc);
				return rc;
			}
			shell_print(shell, "Gyro ODR set to %d Hz.", val);
			return 0;
		}

		if (strcmp(argv[2], "range") == 0) {
			rc = lsm6dsl_set_gyro_range(val);
			if (rc == -EINVAL) {
				shell_error(shell, "Unsupported range. Use: 125,250,500,1000,2000.");
				return rc;
			}
			if (rc < 0) {
				shell_error(shell, "Gyro range set failed (%d).", rc);
				return rc;
			}
			shell_print(shell, "Gyro range set to %d dps.", val);
			return 0;
		}

		shell_error(shell, "Unknown gyro command.");
		return -EINVAL;
	}

	if (strcmp(argv[1], "power") == 0) {
		if (argc < 3) {
			shell_error(shell, "Usage: lsm6dsl power <on|off|status>");
			return -EINVAL;
		}

		if (strcmp(argv[2], "status") == 0) {
			shell_print(shell, "Power: %s", lsm6dsl_powered ? "on" : "off");
			return 0;
		}

		if (strcmp(argv[2], "on") == 0) {
			rc = lsm6dsl_set_power(true);
			if (rc < 0) {
				shell_error(shell, "Power on failed (%d).", rc);
				return rc;
			}
			shell_print(shell, "LSM6DSL powered on.");
			return 0;
		}

		if (strcmp(argv[2], "off") == 0) {
			rc = lsm6dsl_set_power(false);
			if (rc < 0) {
				shell_error(shell, "Power off failed (%d).", rc);
				return rc;
			}
			shell_print(shell, "LSM6DSL powered off.");
			return 0;
		}

		shell_error(shell, "Unknown power command.");
		return -EINVAL;
	}

	if (strcmp(argv[1], "log") == 0) {
		return cmd_lsm6dsl_log(shell, argc - 1, &argv[1]);
	}

	if (strcmp(argv[1], "int") == 0) {
		if (argc < 3) {
			shell_error(shell, "Usage: lsm6dsl int <on|off|status|clear>");
			return -EINVAL;
		}

		if (strcmp(argv[2], "on") == 0) {
			rc = lsm6dsl_int_configure(true);
			if (rc < 0) {
				shell_error(shell, "INT1 enable failed (%d).", rc);
				return rc;
			}
			shell_print(shell, "INT1 enabled.");
			return 0;
		}

		if (strcmp(argv[2], "off") == 0) {
			rc = lsm6dsl_int_configure(false);
			if (rc < 0) {
				shell_error(shell, "INT1 disable failed (%d).", rc);
				return rc;
			}
			shell_print(shell, "INT1 disabled.");
			return 0;
		}

		if (strcmp(argv[2], "status") == 0) {
			int level;

			if (!lsm6dsl_int_gpio.port || !gpio_is_ready_dt(&lsm6dsl_int_gpio)) {
				shell_print(shell, "INT1: not configured");
				return 0;
			}

			level = gpio_pin_get_dt(&lsm6dsl_int_gpio);
			shell_print(shell, "INT1: %s, count %d, level %d",
				    lsm6dsl_int_enabled ? "on" : "off",
				    (int)atomic_get(&lsm6dsl_int_count),
				    level);
			return 0;
		}

		if (strcmp(argv[2], "clear") == 0) {
			atomic_set(&lsm6dsl_int_count, 0);
			shell_print(shell, "INT1 count cleared.");
			return 0;
		}

		shell_error(shell, "Unknown int command.");
		return -EINVAL;
	}

	shell_error(shell, "Unknown command.");
	return -EINVAL;
}

SHELL_CMD_REGISTER(lsm6dsl, NULL, "LSM6DSL IMU control.", cmd_lsm6dsl);

#if DT_HAS_COMPAT_STATUS_OKAY(maxim_max17048)
#if MAX17048_ALERT_AVAILABLE
static void max17048_alert_handler(const struct device *dev, struct gpio_callback *cb,
				   uint32_t pins)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(cb);
	ARG_UNUSED(pins);

	atomic_inc(&max17048_alert_count);
}

static int max17048_alert_init(void)
{
	int rc;

	if (!device_is_ready(max17048_alert.port)) {
		return -ENODEV;
	}

	rc = gpio_pin_configure_dt(&max17048_alert, GPIO_INPUT);
	if (rc < 0) {
		return rc;
	}

	gpio_init_callback(&max17048_alert_cb, max17048_alert_handler,
			   BIT(max17048_alert.pin));
	rc = gpio_add_callback(max17048_alert.port, &max17048_alert_cb);
	if (rc < 0) {
		return rc;
	}

	rc = gpio_pin_interrupt_configure_dt(&max17048_alert, GPIO_INT_EDGE_TO_ACTIVE);
	if (rc < 0) {
		gpio_remove_callback(max17048_alert.port, &max17048_alert_cb);
		return rc;
	}

	return 0;
}
#endif

static int max17048_read_prop(fuel_gauge_prop_t prop, union fuel_gauge_prop_val *val)
{
	if (!device_is_ready(max17048_dev)) {
		return -ENODEV;
	}

	return fuel_gauge_get_prop(max17048_dev, prop, val);
}

static void max17048_print_prop_error(const struct shell *shell, const char *name, int rc)
{
	if (rc == -ENOTSUP) {
		shell_print(shell, "%s: not supported", name);
	} else {
		shell_error(shell, "%s: read failed (%d).", name, rc);
	}
}

static void max17048_print_help(const struct shell *shell)
{
	shell_print(shell, "Usage:");
	shell_print(shell, "  max17048 status");
	shell_print(shell, "  max17048 alert [clear]");
	shell_print(shell, "  max17048 log <start [interval_s] [path]|stop|interval <s>|status>");
}

static int cmd_max17048_log(const struct shell *shell, size_t argc, char **argv)
{
	int interval;
	const char *path_arg = NULL;
	char path[MAX17048_LOG_PATH_MAX];
	int rc;

	if (argc < 2) {
		shell_print(shell,
			    "Usage: max17048 log <start [interval_s] [path]|stop|interval <s>|status>");
		return -EINVAL;
	}

	if (strcmp(argv[1], "start") == 0) {
		interval = atomic_get(&max17048_log_interval_s);

		if (argc >= 3) {
			char *end = NULL;
			long val = strtol(argv[2], &end, 0);

			if (end != argv[2] && *end == '\0') {
				interval = (int)val;
			} else {
				path_arg = argv[2];
			}
		}

		if (argc >= 4) {
			if (path_arg) {
				shell_error(shell, "Too many arguments.");
				return -EINVAL;
			}
			path_arg = argv[3];
		}

		if (argc > 4) {
			shell_error(shell, "Too many arguments.");
			return -EINVAL;
		}

		if (interval < 1) {
			shell_error(shell, "Interval must be >= 1 second.");
			return -EINVAL;
		}

		if (path_arg) {
			rc = max17048_log_set_path(path_arg);
			if (rc < 0) {
				shell_error(shell, "Invalid log path (%d).", rc);
				return rc;
			}
		}

		if (!device_is_ready(max17048_dev)) {
			shell_error(shell, "MAX17048 device not ready.");
			return -ENODEV;
		}

		rc = ensure_littlefs_ready();
		if (rc < 0) {
			shell_error(shell, "LittleFS not mounted (%d).", rc);
			return rc;
		}

		rc = ensure_max17048_log_path();
		if (rc < 0) {
			shell_error(shell, "Failed to prepare log path (%d).", rc);
			return rc;
		}

		atomic_set(&max17048_log_interval_s, interval);
		k_event_post(&max17048_log_event, MAX17048_LOG_EVENT_INTERVAL);
		atomic_set(&max17048_log_running, 1);
		k_event_post(&max17048_log_event, MAX17048_LOG_EVENT_START);
		max17048_log_get_path(path, sizeof(path));
		shell_print(shell, "MAX17048 logging started (interval %d s).", interval);
	    shell_print(shell, "Log file: %s", path);
		boot_log_write("max17048_log: started interval %d s", interval);
		return 0;
	}

	if (strcmp(argv[1], "stop") == 0) {
		atomic_set(&max17048_log_running, 0);
		k_event_post(&max17048_log_event, MAX17048_LOG_EVENT_STOP);
		shell_print(shell, "MAX17048 logging stopped.");
		boot_log_write("max17048_log: stopped");
		return 0;
	}

	if (strcmp(argv[1], "interval") == 0) {
		if (argc < 3) {
			shell_error(shell, "Missing interval in seconds.");
			return -EINVAL;
		}

		interval = (int)strtol(argv[2], NULL, 0);
		if (interval < 1) {
			shell_error(shell, "Interval must be >= 1 second.");
			return -EINVAL;
		}

		atomic_set(&max17048_log_interval_s, interval);
		k_event_post(&max17048_log_event, MAX17048_LOG_EVENT_INTERVAL);
		shell_print(shell, "MAX17048 log interval set to %d s.", interval);
		boot_log_write("max17048_log: interval set %d s", interval);
		return 0;
	}

	if (strcmp(argv[1], "status") == 0) {
		max17048_log_get_path(path, sizeof(path));
		shell_print(shell, "MAX17048 logging %s, interval %d s.",
			    atomic_get(&max17048_log_running) ? "on" : "off",
			    (int)atomic_get(&max17048_log_interval_s));
		shell_print(shell, "Log file: %s", path);
		boot_log_write("max17048_log: status queried");
		return 0;
	}

	shell_error(shell, "Unknown command. Use: start|stop|interval|status");
	return -EINVAL;
}

static int cmd_max17048(const struct shell *shell, size_t argc, char **argv)
{
	union fuel_gauge_prop_val val;
	int rc;

	if (!device_is_ready(max17048_dev)) {
		shell_error(shell, "MAX17048 device not ready.");
		return -ENODEV;
	}

	if (argc < 2) {
		max17048_print_help(shell);
		return 0;
	}

	if (strcmp(argv[1], "log") == 0) {
		return cmd_max17048_log(shell, argc - 1, &argv[1]);
	}

	if (strcmp(argv[1], "status") == 0) {
		shell_print(shell, "Device: %s", max17048_dev->name);

		rc = max17048_read_prop(FUEL_GAUGE_VOLTAGE, &val);
		if (rc == 0) {
			shell_print(shell, "Voltage: %u mV",
				    (unsigned int)(val.voltage / 1000U));
		} else {
			max17048_print_prop_error(shell, "Voltage", rc);
		}

		rc = max17048_read_prop(FUEL_GAUGE_RELATIVE_STATE_OF_CHARGE, &val);
		if (rc == 0) {
			shell_print(shell, "SOC: %u %%", val.relative_state_of_charge);
		} else {
			max17048_print_prop_error(shell, "SOC", rc);
		}

		rc = max17048_read_prop(FUEL_GAUGE_RUNTIME_TO_EMPTY, &val);
		if (rc == 0) {
			shell_print(shell, "Time to empty: %u min",
				    (unsigned int)val.runtime_to_empty);
		} else {
			max17048_print_prop_error(shell, "Time to empty", rc);
		}

		rc = max17048_read_prop(FUEL_GAUGE_RUNTIME_TO_FULL, &val);
		if (rc == 0) {
			shell_print(shell, "Time to full: %u min",
				    (unsigned int)val.runtime_to_full);
		} else {
			max17048_print_prop_error(shell, "Time to full", rc);
		}

#if MAX17048_ALERT_AVAILABLE
		rc = gpio_pin_get_dt(&max17048_alert);
		if (rc >= 0) {
			shell_print(shell, "Alert: %s (count %d)",
				    rc ? "active" : "inactive",
				    (int)atomic_get(&max17048_alert_count));
		} else {
			shell_error(shell, "Alert: read failed (%d).", rc);
		}
#else
		shell_print(shell, "Alert: not configured");
#endif
		return 0;
	}

	if (strcmp(argv[1], "alert") == 0) {
#if MAX17048_ALERT_AVAILABLE
		if (argc > 2 && strcmp(argv[2], "clear") == 0) {
			atomic_set(&max17048_alert_count, 0);
			shell_print(shell, "Alert count cleared.");
			return 0;
		}

		rc = gpio_pin_get_dt(&max17048_alert);
		if (rc < 0) {
			shell_error(shell, "Alert: read failed (%d).", rc);
			return rc;
		}

		shell_print(shell, "Alert: %s (count %d)",
			    rc ? "active" : "inactive",
			    (int)atomic_get(&max17048_alert_count));
		return 0;
#else
		shell_error(shell, "Alert not configured.");
		return -ENOTSUP;
#endif
	}

	shell_error(shell, "Unknown command.");
	return -EINVAL;
}
#else
static int cmd_max17048(const struct shell *shell, size_t argc, char **argv)
{
	(void)argc;
	(void)argv;

	shell_error(shell, "MAX17048 not configured in devicetree.");
	return -ENODEV;
}
#endif

SHELL_CMD_REGISTER(max17048, NULL, "MAX17048 fuel gauge control.", cmd_max17048);

#if DT_HAS_COMPAT_STATUS_OKAY(maxim_max17048)
static void max17048_setup(void)
{
#if MAX17048_ALERT_AVAILABLE
	int alert_rc;
#endif

	if (!device_is_ready(max17048_dev)) {
		printk("MAX17048 not ready.\n");
		boot_log_write("max17048: not ready");
		return;
	}

	boot_log_write("max17048: init ok");

#if MAX17048_ALERT_AVAILABLE
	alert_rc = max17048_alert_init();
	if (alert_rc == 0) {
		boot_log_write("max17048: alert configured");
	} else {
		boot_log_write("max17048: alert init failed (%d)", alert_rc);
	}
#else
	boot_log_write("max17048: alert not configured");
#endif
}
#endif

void sensors_init(void)
{
	int rc;

	k_mutex_init(&bme_log_path_lock);
	k_mutex_init(&ltr_log_path_lock);
	k_mutex_init(&lsm6dsl_log_path_lock);
#if DT_HAS_COMPAT_STATUS_OKAY(maxim_max17048)
	k_mutex_init(&max17048_log_path_lock);
#endif

	rc = ltr303_init_device();
	if (rc == 0) {
		boot_log_write("ltr303: init ok");
	} else {
		boot_log_write("ltr303: init failed (%d)", rc);
	}

	if (device_is_ready(lsm6dsl_dev)) {
		boot_log_write("lsm6dsl: init ok");
		lsm6dsl_apply_defaults();
	} else {
		boot_log_write("lsm6dsl: not ready");
	}

#if DT_HAS_COMPAT_STATUS_OKAY(maxim_max17048)
	max17048_setup();
	k_thread_create(&max17048_log_thread, max17048_log_stack,
			K_THREAD_STACK_SIZEOF(max17048_log_stack),
			max17048_log_thread_fn, NULL, NULL, NULL,
			MAX17048_LOG_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&max17048_log_thread, "max17048_log");
	boot_log_write("boot: max17048_log thread started");
#endif
	k_thread_create(&bme_log_thread, bme_log_stack,
			K_THREAD_STACK_SIZEOF(bme_log_stack),
			bme_log_thread_fn, NULL, NULL, NULL,
			BME_LOG_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&bme_log_thread, "bme_log");
	boot_log_write("boot: bme_log thread started");
	k_thread_create(&ltr_log_thread, ltr_log_stack,
			K_THREAD_STACK_SIZEOF(ltr_log_stack),
			ltr_log_thread_fn, NULL, NULL, NULL,
			LTR_LOG_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&ltr_log_thread, "ltr_log");
	boot_log_write("boot: ltr_log thread started");
	k_thread_create(&lsm6dsl_log_thread, lsm6dsl_log_stack,
			K_THREAD_STACK_SIZEOF(lsm6dsl_log_stack),
			lsm6dsl_log_thread_fn, NULL, NULL, NULL,
			LSM6DSL_LOG_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&lsm6dsl_log_thread, "lsm6dsl_log");
	boot_log_write("boot: lsm6dsl_log thread started");
}
