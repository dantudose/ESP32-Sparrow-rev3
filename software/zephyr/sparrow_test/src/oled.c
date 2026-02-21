#include <ctype.h>
#include <errno.h>
#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/display/cfb.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/fs/fs.h>
#include <zephyr/kernel.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/util.h>

#include "sensors.h"
#include "net_time.h"

#if DT_NODE_HAS_STATUS(DT_CHOSEN(zephyr_display), okay)
#define OLED_NODE DT_CHOSEN(zephyr_display)
#else
#define OLED_NODE DT_INVALID_NODE
#endif

#define OLED_MOUNT_POINT "/lfs"

#if DT_NODE_HAS_STATUS(OLED_NODE, okay)
static const struct device *const oled_dev = DEVICE_DT_GET(OLED_NODE);
static bool oled_cfb_ready;
static bool oled_blanked = true;
static bool oled_inverted;
static uint8_t oled_font_idx;
static uint8_t oled_font_width;
static uint8_t oled_font_height;
static atomic_t oled_info_interval_s = ATOMIC_INIT(0);
static struct k_work_delayable oled_info_work;
static bool oled_info_work_ready;

static int oled_parse_int(const struct shell *shell, const char *arg, int *out)
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

static void oled_info_work_handler(struct k_work *work);
static int oled_init(void);

static int oled_resolve_path(const char *path, char *out, size_t len)
{
	if (!path || path[0] == '\0') {
		return -EINVAL;
	}

	if (path[0] == '/') {
		if (strlen(path) >= len) {
			return -ENAMETOOLONG;
		}
		if (strncmp(path, OLED_MOUNT_POINT "/", strlen(OLED_MOUNT_POINT) + 1) != 0) {
			return -EINVAL;
		}
		snprintk(out, len, "%s", path);
		return 0;
	}

	if (snprintk(out, len, "%s/%s", OLED_MOUNT_POINT, path) >= (int)len) {
		return -ENAMETOOLONG;
	}

	return 0;
}

static int pbm_read_token(struct fs_file_t *file, char *buf, size_t len)
{
	size_t pos = 0;
	bool in_token = false;

	if (!buf || len == 0U) {
		return -EINVAL;
	}

	while (true) {
		uint8_t ch;
		int rc = fs_read(file, &ch, 1);

		if (rc < 0) {
			return rc;
		}
		if (rc == 0) {
			if (in_token) {
				buf[pos] = '\0';
				return 0;
			}
			return -EINVAL;
		}

		if (ch == '#') {
			do {
				rc = fs_read(file, &ch, 1);
				if (rc < 0) {
					return rc;
				}
				if (rc == 0) {
					return -EINVAL;
				}
			} while (ch != '\n' && ch != '\r');
			continue;
		}

		if (isspace((int)ch)) {
			if (in_token) {
				buf[pos] = '\0';
				return 0;
			}
			continue;
		}

		if (pos + 1 >= len) {
			return -ENAMETOOLONG;
		}

		buf[pos++] = (char)ch;
		in_token = true;
	}
}

static int pbm_skip_to_data(struct fs_file_t *file)
{
	while (true) {
		uint8_t ch;
		int rc = fs_read(file, &ch, 1);

		if (rc < 0) {
			return rc;
		}
		if (rc == 0) {
			return -EINVAL;
		}

		if (ch == '#') {
			do {
				rc = fs_read(file, &ch, 1);
				if (rc < 0) {
					return rc;
				}
				if (rc == 0) {
					return -EINVAL;
				}
			} while (ch != '\n' && ch != '\r');
			continue;
		}

		if (isspace((int)ch)) {
			continue;
		}

		rc = fs_seek(file, -1, FS_SEEK_CUR);
		if (rc < 0) {
			return rc;
		}
		return 0;
	}
}

static int oled_draw_pbm(const char *path, int x_off, int y_off)
{
	struct fs_file_t file;
	struct cfb_position pos;
	char token[16];
	char resolved[128];
	int width;
	int height;
	int rc;

	rc = oled_resolve_path(path, resolved, sizeof(resolved));
	if (rc < 0) {
		return rc;
	}

	rc = oled_init();
	if (rc < 0) {
		return rc;
	}

	fs_file_t_init(&file);
	rc = fs_open(&file, resolved, FS_O_READ);
	if (rc < 0) {
		return rc;
	}

	rc = pbm_read_token(&file, token, sizeof(token));
	if (rc < 0) {
		fs_close(&file);
		return rc;
	}

	bool is_p1 = strcmp(token, "P1") == 0;
	bool is_p4 = strcmp(token, "P4") == 0;
	if (!is_p1 && !is_p4) {
		fs_close(&file);
		return -EINVAL;
	}

	rc = pbm_read_token(&file, token, sizeof(token));
	if (rc < 0) {
		fs_close(&file);
		return rc;
	}
	width = (int)strtol(token, NULL, 0);

	rc = pbm_read_token(&file, token, sizeof(token));
	if (rc < 0) {
		fs_close(&file);
		return rc;
	}
	height = (int)strtol(token, NULL, 0);

	if (width <= 0 || height <= 0) {
		fs_close(&file);
		return -EINVAL;
	}

	uint16_t disp_w = cfb_get_display_parameter(oled_dev, CFB_DISPLAY_WIDTH);
	uint16_t disp_h = cfb_get_display_parameter(oled_dev, CFB_DISPLAY_HEIGHT);
	if (x_off < 0 || y_off < 0 ||
	    x_off + width > (int)disp_w ||
	    y_off + height > (int)disp_h) {
		fs_close(&file);
		return -ERANGE;
	}

	rc = cfb_framebuffer_clear(oled_dev, false);
	if (rc != 0) {
		fs_close(&file);
		return rc;
	}

	if (is_p1) {
		for (int y = 0; y < height; y++) {
			for (int x = 0; x < width; x++) {
				rc = pbm_read_token(&file, token, sizeof(token));
				if (rc < 0) {
					fs_close(&file);
					return rc;
				}

				if (token[0] == '1') {
					pos.x = (uint16_t)(x_off + x);
					pos.y = (uint16_t)(y_off + y);
					rc = cfb_draw_point(oled_dev, &pos);
					if (rc != 0) {
						fs_close(&file);
						return rc;
					}
				}
			}
		}
	} else {
		int row_bytes = (width + 7) / 8;
		uint8_t row[32];

		if (row_bytes > (int)sizeof(row)) {
			fs_close(&file);
			return -ENOSPC;
		}

		rc = pbm_skip_to_data(&file);
		if (rc < 0) {
			fs_close(&file);
			return rc;
		}

		for (int y = 0; y < height; y++) {
			int got = fs_read(&file, row, row_bytes);
			if (got < 0) {
				fs_close(&file);
				return got;
			}
			if (got != row_bytes) {
				fs_close(&file);
				return -EINVAL;
			}

			for (int x = 0; x < width; x++) {
				uint8_t byte = row[x / 8];
				uint8_t mask = (uint8_t)(1U << (7 - (x % 8)));

				if (byte & mask) {
					pos.x = (uint16_t)(x_off + x);
					pos.y = (uint16_t)(y_off + y);
					rc = cfb_draw_point(oled_dev, &pos);
					if (rc != 0) {
						fs_close(&file);
						return rc;
					}
				}
			}
		}
	}

	fs_close(&file);

	rc = cfb_framebuffer_finalize(oled_dev);
	if (rc != 0) {
		return rc;
	}

	if (oled_blanked) {
		display_blanking_off(oled_dev);
		oled_blanked = false;
	}

	return 0;
}

static int oled_select_font(uint8_t idx)
{
	uint8_t width;
	uint8_t height;

	if (cfb_get_font_size(oled_dev, idx, &width, &height)) {
		return -EINVAL;
	}

	cfb_framebuffer_set_font(oled_dev, idx);
	oled_font_idx = idx;
	oled_font_width = width;
	oled_font_height = height;
	return 0;
}

static int oled_find_font(uint8_t width, uint8_t height, uint8_t *out_idx)
{
	uint8_t count;

	if (!out_idx) {
		return -EINVAL;
	}

	count = (uint8_t)cfb_get_numof_fonts(oled_dev);
	for (uint8_t idx = 0; idx < count; idx++) {
		uint8_t w;
		uint8_t h;

		if (cfb_get_font_size(oled_dev, idx, &w, &h) == 0 &&
		    w == width && h == height) {
			*out_idx = idx;
			return 0;
		}
	}

	return -ENOENT;
}

static int oled_init(void)
{
	int rc;

	if (!oled_info_work_ready) {
		k_work_init_delayable(&oled_info_work, oled_info_work_handler);
		oled_info_work_ready = true;
	}

	if (oled_cfb_ready) {
		return 0;
	}

	if (!device_is_ready(oled_dev)) {
		return -ENODEV;
	}

	rc = display_set_pixel_format(oled_dev, PIXEL_FORMAT_MONO10);
	if (rc != 0) {
		rc = display_set_pixel_format(oled_dev, PIXEL_FORMAT_MONO01);
	}
	if (rc != 0) {
		return rc;
	}

	rc = cfb_framebuffer_init(oled_dev);
	if (rc != 0) {
		return rc;
	}

	rc = cfb_framebuffer_invert(oled_dev);
	if (rc != 0) {
		return rc;
	}
	oled_inverted = true;

	rc = oled_select_font(0);
	if (rc < 0) {
		return rc;
	}

	rc = cfb_framebuffer_clear(oled_dev, true);
	if (rc != 0) {
		return rc;
	}

	display_blanking_off(oled_dev);
	oled_blanked = false;
	oled_cfb_ready = true;
	return 0;
}

static void oled_fit_text(char *out, size_t out_len, const char *src, size_t cols, bool keep_tail)
{
	size_t src_len;
	size_t max_len;

	if (!out || out_len == 0) {
		return;
	}

	if (!src) {
		out[0] = '\0';
		return;
	}

	if (cols == 0) {
		out[0] = '\0';
		return;
	}

	src_len = strlen(src);
	max_len = MIN(cols, out_len - 1);
	if (src_len <= max_len) {
		if (out != src) {
			memcpy(out, src, src_len);
		}
		out[src_len] = '\0';
		return;
	}

	if (keep_tail && src_len > max_len) {
		src += (src_len - max_len);
	}

	if (out == src) {
		memmove(out, src, max_len);
	} else {
		memcpy(out, src, max_len);
	}
	out[max_len] = '\0';
}

static void oled_format_time(char *buf, size_t len)
{
	int64_t now_ms = time_now_ms();
	int64_t now_s = now_ms / 1000;
	int hour;
	int min;
	int sec;

	if (!buf || len == 0) {
		return;
	}

	if (now_s < 0) {
		now_s = 0;
	}

	sec = (int)(now_s % 60);
	min = (int)((now_s / 60) % 60);
	hour = (int)((now_s / 3600) % 24);

	if (time_is_synced()) {
		snprintk(buf, len, "%02d:%02d:%02d", hour, min, sec);
	} else {
		snprintk(buf, len, "!%02d:%02d:%02d", hour, min, sec);
	}
}

static bool oled_get_ipv4(char *buf, size_t len)
{
	struct net_if *iface = time_get_wifi_iface();
	struct net_in_addr *addr;

	if (!buf || len == 0) {
		return false;
	}

	buf[0] = '\0';
	if (!iface) {
		return false;
	}

	addr = net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED);
	if (!addr) {
		return false;
	}

	return net_addr_ntop(NET_AF_INET, addr, buf, len) != NULL;
}

static void oled_format_ip_line(const char *ip, char *out, size_t len, size_t cols)
{
	const char *text = ip ? ip : "";
	const char *dot;

	if (!out || len == 0) {
		return;
	}

	out[0] = '\0';
	if (cols == 0) {
		return;
	}

	if (strlen(text) > cols) {
		dot = strchr(text, '.');
		if (dot && dot[1] != '\0') {
			text = dot + 1;
		}
	}

	if (strlen(text) + 3 <= cols) {
		snprintk(out, len, "IP %s", text);
		return;
	}

	oled_fit_text(out, len, text, cols, true);
	if (out[0] == '.' && out[1] != '\0') {
		memmove(out, out + 1, strlen(out));
	}
}

static void oled_format_1dp(char *buf, size_t len, int64_t micro)
{
	int64_t abs_val = llabs(micro);
	int64_t whole = abs_val / 1000000;
	int64_t frac = (abs_val % 1000000 + 50000) / 100000;

	if (frac >= 10) {
		whole += 1;
		frac = 0;
	}

	if (micro < 0) {
		snprintk(buf, len, "-%" PRId64 ".%" PRId64, whole, frac);
	} else {
		snprintk(buf, len, "%" PRId64 ".%" PRId64, whole, frac);
	}
}

static bool oled_read_bme(struct sensor_value *temp,
			  struct sensor_value *hum,
			  struct sensor_value *press)
{
	int rc;

	if (!device_is_ready(bme680)) {
		return false;
	}

	rc = sensor_sample_fetch(bme680);
	if (rc < 0) {
		return false;
	}

	rc = sensor_channel_get(bme680, SENSOR_CHAN_AMBIENT_TEMP, temp);
	if (rc < 0) {
		return false;
	}

	rc = sensor_channel_get(bme680, SENSOR_CHAN_HUMIDITY, hum);
	if (rc < 0) {
		return false;
	}

	rc = sensor_channel_get(bme680, SENSOR_CHAN_PRESS, press);
	if (rc < 0) {
		return false;
	}

	return true;
}

static bool oled_read_lux(struct sensor_value *lux, bool *enabled_out)
{
	int rc;
	uint16_t ch0;
	uint16_t ch1;

	if (enabled_out) {
		*enabled_out = ltr303_enabled;
	}

	rc = ltr303_check_bus();
	if (rc < 0) {
		return false;
	}

	if (!ltr303_configured) {
		rc = ltr303_init_device();
		if (rc < 0) {
			return false;
		}
	}

	if (enabled_out) {
		*enabled_out = ltr303_enabled;
	}

	if (!ltr303_enabled) {
		return false;
	}

	rc = ltr303_wait_data_ready(500);
	if (rc < 0) {
		return false;
	}

	rc = ltr303_read_channels(&ch0, &ch1);
	if (rc < 0) {
		return false;
	}

	rc = ltr303_calc_lux(ch0, ch1, lux);
	if (rc < 0) {
		return false;
	}

	return true;
}

static void oled_format_lux(char *buf, size_t len, int64_t micro)
{
	int64_t lux;
	int64_t whole;
	int64_t frac;

	if (micro < 0) {
		micro = 0;
	}

	lux = (micro + 500000) / 1000000;
	if (lux >= 10000) {
		whole = lux / 1000;
		frac = (lux % 1000 + 50) / 100;
		if (frac >= 10) {
			whole += 1;
			frac = 0;
		}
		snprintk(buf, len, "L:%" PRId64 ".%" PRId64 "K", whole, frac);
	} else {
		snprintk(buf, len, "L:%" PRId64, lux);
	}
}

static int oled_render_info(const struct shell *shell, bool report_shell)
{
	char time_buf[16];
	char ip_buf[NET_IPV4_ADDR_LEN];
	char line_time[32];
	char line_ip[32];
	char temp_buf[16];
	char hum_buf[16];
	char press_buf[16];
	char lux_buf[16];
	char line_sense1[32];
	char line_sense2[32];
	struct sensor_value temp;
	struct sensor_value hum;
	struct sensor_value press;
	struct sensor_value lux;
	uint8_t prev_font = oled_font_idx;
	uint8_t info_font = prev_font;
	bool font_changed = false;
	bool have_bme;
	bool have_lux;
	bool lux_enabled = false;
	int rows;
	int cols;
	int rc;
	int rc_font;
	bool have_ip;
	bool show_sensors;

	rc = oled_init();
	if (rc < 0) {
		if (report_shell && shell) {
			shell_error(shell, "OLED init failed (%d).", rc);
		}
		return rc;
	}

	rc_font = oled_find_font(5, 8, &info_font);
	if (rc_font == 0 && info_font != prev_font) {
		rc_font = oled_select_font(info_font);
		if (rc_font < 0) {
			if (report_shell && shell) {
				shell_error(shell, "Small font not available.");
			}
			return rc_font;
		}
		font_changed = true;
	}

	rows = cfb_get_display_parameter(oled_dev, CFB_DISPLAY_HEIGHT) / oled_font_height;
	cols = cfb_get_display_parameter(oled_dev, CFB_DISPLAY_WIDTH) / oled_font_width;
	show_sensors = rows >= 4;
	if (rows < 2) {
		if (font_changed) {
			oled_select_font(prev_font);
		}
		if (report_shell && shell) {
			shell_error(shell, "Display too small for info view.");
		}
		return -ENOSPC;
	}

	oled_format_time(time_buf, sizeof(time_buf));
	oled_fit_text(line_time, sizeof(line_time), time_buf, cols, false);

	have_ip = oled_get_ipv4(ip_buf, sizeof(ip_buf));
	if (!have_ip) {
		oled_fit_text(line_ip, sizeof(line_ip), "NOIP", cols, false);
	} else {
		oled_format_ip_line(ip_buf, line_ip, sizeof(line_ip), cols);
	}

	if (show_sensors) {
		have_bme = oled_read_bme(&temp, &hum, &press);
		if (have_bme) {
			oled_format_1dp(temp_buf, sizeof(temp_buf),
					sensor_value_to_micro(&temp));
			oled_format_1dp(hum_buf, sizeof(hum_buf),
					sensor_value_to_micro(&hum));
			oled_format_1dp(press_buf, sizeof(press_buf),
					sensor_value_to_micro(&press));
		} else {
			snprintk(temp_buf, sizeof(temp_buf), "--.-");
			snprintk(hum_buf, sizeof(hum_buf), "--.-");
			snprintk(press_buf, sizeof(press_buf), "---.-");
		}

		have_lux = oled_read_lux(&lux, &lux_enabled);
		if (have_lux) {
			oled_format_lux(lux_buf, sizeof(lux_buf),
					sensor_value_to_micro(&lux));
		} else if (!lux_enabled) {
			snprintk(lux_buf, sizeof(lux_buf), "L:OFF");
		} else {
			snprintk(lux_buf, sizeof(lux_buf), "L:--");
		}

		snprintk(line_sense1, sizeof(line_sense1), "T:%.8sC H:%.8s",
			 temp_buf, hum_buf);
		oled_fit_text(line_sense1, sizeof(line_sense1), line_sense1,
			      cols, false);

		snprintk(line_sense2, sizeof(line_sense2), "P:%.8sK %.8s",
			 press_buf, lux_buf);
		oled_fit_text(line_sense2, sizeof(line_sense2), line_sense2,
			      cols, false);
	}

	rc = cfb_framebuffer_clear(oled_dev, false);
	if (rc != 0) {
		goto oled_info_done;
	}

	rc = cfb_print(oled_dev, line_time, 0, 0);
	if (rc != 0) {
		goto oled_info_done;
	}

	rc = cfb_print(oled_dev, line_ip, 0, oled_font_height);
	if (rc != 0) {
		goto oled_info_done;
	}

	if (show_sensors) {
		rc = cfb_print(oled_dev, line_sense1, 0,
			       oled_font_height * 2);
		if (rc != 0) {
			goto oled_info_done;
		}

		rc = cfb_print(oled_dev, line_sense2, 0,
			       oled_font_height * 3);
		if (rc != 0) {
			goto oled_info_done;
		}
	}

	rc = cfb_framebuffer_finalize(oled_dev);
	if (rc != 0) {
		goto oled_info_done;
	}

	if (oled_blanked) {
		display_blanking_off(oled_dev);
		oled_blanked = false;
	}

oled_info_done:
	if (font_changed) {
		oled_select_font(prev_font);
	}

	if (rc != 0 && report_shell && shell) {
		shell_error(shell, "Info render failed (%d).", rc);
	}
	return rc;
}

static void oled_info_work_handler(struct k_work *work)
{
	int interval_s = (int)atomic_get(&oled_info_interval_s);

	(void)work;

	if (interval_s <= 0) {
		return;
	}

	(void)oled_render_info(NULL, false);

	interval_s = (int)atomic_get(&oled_info_interval_s);
	if (interval_s > 0) {
		k_work_schedule(&oled_info_work, K_SECONDS(interval_s));
	}
}

static void oled_print_help(const struct shell *shell)
{
	shell_print(shell, "Usage:");
	shell_print(shell, "  oled status");
	shell_print(shell, "  oled clear");
	shell_print(shell, "  oled print <row> <col> <text...>");
	shell_print(shell, "  oled time [row col]");
	shell_print(shell, "  oled ip [row col]");
	shell_print(shell, "  oled info [seconds|off]");
	shell_print(shell, "  oled bmp <path> [x y]");
	shell_print(shell, "  oled on|off");
	shell_print(shell, "  oled invert <on|off|toggle|status>");
	shell_print(shell, "  oled contrast <0-255>");
	shell_print(shell, "  oled font <idx>");
}

static int cmd_oled(const struct shell *shell, size_t argc, char **argv)
{
	int rc;

	if (argc < 2) {
		oled_print_help(shell);
		return -EINVAL;
	}

	if (strcmp(argv[1], "status") == 0) {
		uint16_t rows;
		uint16_t cols;
		uint16_t width;
		uint16_t height;

		rc = oled_init();
		if (rc < 0) {
			shell_error(shell, "OLED init failed (%d).", rc);
			return rc;
		}

		width = cfb_get_display_parameter(oled_dev, CFB_DISPLAY_WIDTH);
		height = cfb_get_display_parameter(oled_dev, CFB_DISPLAY_HEIGHT);
		rows = cfb_get_display_parameter(oled_dev, CFB_DISPLAY_ROWS);
		cols = cfb_get_display_parameter(oled_dev, CFB_DISPLAY_COLS);
		shell_print(shell, "OLED: %s", oled_dev->name);
		shell_print(shell, "Res: %ux%u, rows %u, cols %u", width, height, rows, cols);
		shell_print(shell, "Font: %u (%ux%u)", oled_font_idx,
			    oled_font_width, oled_font_height);
		shell_print(shell, "Inverted: %s", oled_inverted ? "yes" : "no");
		shell_print(shell, "Blanked: %s", oled_blanked ? "yes" : "no");
		return 0;
	}

	if (strcmp(argv[1], "clear") == 0) {
		rc = oled_init();
		if (rc < 0) {
			shell_error(shell, "OLED init failed (%d).", rc);
			return rc;
		}

		rc = cfb_framebuffer_clear(oled_dev, true);
		if (rc != 0) {
			shell_error(shell, "Clear failed (%d).", rc);
			return rc;
		}

		shell_print(shell, "OLED cleared.");
		return 0;
	}

	if (strcmp(argv[1], "print") == 0) {
		int row;
		int col;
		int rows;
		int cols;
		int x;
		int y;
		char text[128];
		size_t used = 0;

		if (argc < 5) {
			shell_error(shell, "Usage: oled print <row> <col> <text...>");
			return -EINVAL;
		}

		rc = oled_parse_int(shell, argv[2], &row);
		if (rc < 0) {
			return rc;
		}

		rc = oled_parse_int(shell, argv[3], &col);
		if (rc < 0) {
			return rc;
		}

		rc = oled_init();
		if (rc < 0) {
			shell_error(shell, "OLED init failed (%d).", rc);
			return rc;
		}

		rows = cfb_get_display_parameter(oled_dev, CFB_DISPLAY_HEIGHT) / oled_font_height;
		cols = cfb_get_display_parameter(oled_dev, CFB_DISPLAY_WIDTH) / oled_font_width;
		if (row < 0 || row >= rows || col < 0 || col >= cols) {
			shell_error(shell, "Row/col out of range (rows %d, cols %d).",
				    rows, cols);
			return -ERANGE;
		}

		for (int i = 4; i < (int)argc; i++) {
			size_t arg_len = strlen(argv[i]);
			size_t need = arg_len + ((i > 4) ? 1U : 0U);

			if (used + need >= sizeof(text)) {
				shell_error(shell, "Text too long.");
				return -ENAMETOOLONG;
			}

			if (i > 4) {
				text[used++] = ' ';
			}

			memcpy(&text[used], argv[i], arg_len);
			used += arg_len;
		}
		text[used] = '\0';

		x = col * oled_font_width;
		y = row * oled_font_height;
		rc = cfb_print(oled_dev, text, x, y);
		if (rc != 0) {
			shell_error(shell, "Print failed (%d).", rc);
			return rc;
		}

		rc = cfb_framebuffer_finalize(oled_dev);
		if (rc != 0) {
			shell_error(shell, "Finalize failed (%d).", rc);
			return rc;
		}

		if (oled_blanked) {
			display_blanking_off(oled_dev);
			oled_blanked = false;
		}

		return 0;
	}

	if (strcmp(argv[1], "time") == 0 || strcmp(argv[1], "ip") == 0) {
		int row = 0;
		int col = 0;
		int rows;
		int cols;
		char text[32];
		char line[32];
		int x;
		int y;
		bool want_time = strcmp(argv[1], "time") == 0;

		if (argc >= 3) {
			rc = oled_parse_int(shell, argv[2], &row);
			if (rc < 0) {
				return rc;
			}
		}

		if (argc >= 4) {
			rc = oled_parse_int(shell, argv[3], &col);
			if (rc < 0) {
				return rc;
			}
		}

		rc = oled_init();
		if (rc < 0) {
			shell_error(shell, "OLED init failed (%d).", rc);
			return rc;
		}

		rows = cfb_get_display_parameter(oled_dev, CFB_DISPLAY_HEIGHT) / oled_font_height;
		cols = cfb_get_display_parameter(oled_dev, CFB_DISPLAY_WIDTH) / oled_font_width;
		if (row < 0 || row >= rows || col < 0 || col >= cols) {
			shell_error(shell, "Row/col out of range (rows %d, cols %d).",
				    rows, cols);
			return -ERANGE;
		}

		if (want_time) {
			oled_format_time(text, sizeof(text));
			oled_fit_text(line, sizeof(line), text, cols - col, false);
		} else {
			char ip_buf[NET_IPV4_ADDR_LEN];
			bool have_ip = oled_get_ipv4(ip_buf, sizeof(ip_buf));

			if (!have_ip) {
				oled_fit_text(line, sizeof(line), "NOIP", cols - col, false);
			} else {
				oled_format_ip_line(ip_buf, line, sizeof(line), cols - col);
			}
		}

		x = col * oled_font_width;
		y = row * oled_font_height;
		rc = cfb_print(oled_dev, line, x, y);
		if (rc != 0) {
			shell_error(shell, "Print failed (%d).", rc);
			return rc;
		}

		rc = cfb_framebuffer_finalize(oled_dev);
		if (rc != 0) {
			shell_error(shell, "Finalize failed (%d).", rc);
			return rc;
		}

		if (oled_blanked) {
			display_blanking_off(oled_dev);
			oled_blanked = false;
		}

		return 0;
	}

	if (strcmp(argv[1], "info") == 0) {
		int interval_s = -1;
		bool have_interval = false;
		bool stop_refresh = false;
		struct k_work_sync sync;

		if (argc >= 3) {
			if (strcmp(argv[2], "off") == 0 || strcmp(argv[2], "stop") == 0) {
				stop_refresh = true;
			} else {
				rc = oled_parse_int(shell, argv[2], &interval_s);
				if (rc < 0) {
					return rc;
				}
				have_interval = true;
			}
		}

		if (have_interval && interval_s < 0) {
			shell_error(shell, "Interval must be >= 0.");
			return -ERANGE;
		}

		rc = oled_render_info(shell, true);
		if (stop_refresh || (have_interval && interval_s == 0)) {
			atomic_set(&oled_info_interval_s, 0);
			(void)k_work_cancel_delayable_sync(&oled_info_work, &sync);
			shell_print(shell, "OLED info refresh off.");
			return rc;
		}

		if (have_interval && interval_s > 0) {
			atomic_set(&oled_info_interval_s, interval_s);
			(void)k_work_cancel_delayable(&oled_info_work);
			k_work_schedule(&oled_info_work, K_SECONDS(interval_s));
			shell_print(shell, "OLED info refresh %d s.", interval_s);
		}

		return rc;
	}

	if (strcmp(argv[1], "bmp") == 0) {
		int x = 0;
		int y = 0;

		if (argc < 3) {
			shell_error(shell, "Usage: oled bmp <path> [x y]");
			return -EINVAL;
		}

		if (argc >= 4) {
			rc = oled_parse_int(shell, argv[3], &x);
			if (rc < 0) {
				return rc;
			}
		}

		if (argc >= 5) {
			rc = oled_parse_int(shell, argv[4], &y);
			if (rc < 0) {
				return rc;
			}
		}

		rc = oled_draw_pbm(argv[2], x, y);
		if (rc < 0) {
			if (rc == -ERANGE) {
				shell_error(shell, "Bitmap does not fit on display.");
			} else if (rc == -ENAMETOOLONG) {
				shell_error(shell, "Bitmap path too long.");
			} else {
				shell_error(shell, "Bitmap draw failed (%d).", rc);
			}
			return rc;
		}

		return 0;
	}

	if (strcmp(argv[1], "on") == 0 || strcmp(argv[1], "off") == 0) {
		rc = oled_init();
		if (rc < 0) {
			shell_error(shell, "OLED init failed (%d).", rc);
			return rc;
		}

		if (strcmp(argv[1], "off") == 0) {
			display_blanking_on(oled_dev);
			oled_blanked = true;
			shell_print(shell, "OLED blanked.");
		} else {
			display_blanking_off(oled_dev);
			oled_blanked = false;
			shell_print(shell, "OLED unblanked.");
		}

		return 0;
	}

	if (strcmp(argv[1], "contrast") == 0) {
		int val;

		if (argc < 3) {
			shell_error(shell, "Usage: oled contrast <0-255>");
			return -EINVAL;
		}

		rc = oled_parse_int(shell, argv[2], &val);
		if (rc < 0) {
			return rc;
		}

		if (val < 0 || val > 255) {
			shell_error(shell, "Contrast must be 0..255.");
			return -ERANGE;
		}

		rc = oled_init();
		if (rc < 0) {
			shell_error(shell, "OLED init failed (%d).", rc);
			return rc;
		}

		rc = display_set_contrast(oled_dev, (uint8_t)val);
		if (rc != 0) {
			shell_error(shell, "Contrast set failed (%d).", rc);
			return rc;
		}

		shell_print(shell, "Contrast set to %d.", val);
		return 0;
	}

	if (strcmp(argv[1], "invert") == 0) {
		if (argc < 3) {
			shell_error(shell, "Usage: oled invert <on|off|toggle|status>");
			return -EINVAL;
		}

		if (strcmp(argv[2], "status") == 0) {
			shell_print(shell, "Inverted: %s", oled_inverted ? "yes" : "no");
			return 0;
		}

		rc = oled_init();
		if (rc < 0) {
			shell_error(shell, "OLED init failed (%d).", rc);
			return rc;
		}

		if (strcmp(argv[2], "toggle") == 0) {
			rc = cfb_framebuffer_invert(oled_dev);
			if (rc != 0) {
				shell_error(shell, "Invert failed (%d).", rc);
				return rc;
			}
			oled_inverted = !oled_inverted;
			shell_print(shell, "Inverted: %s", oled_inverted ? "yes" : "no");
			return 0;
		}

		if (strcmp(argv[2], "on") == 0 && !oled_inverted) {
			rc = cfb_framebuffer_invert(oled_dev);
			if (rc != 0) {
				shell_error(shell, "Invert failed (%d).", rc);
				return rc;
			}
			oled_inverted = true;
			shell_print(shell, "Inverted: yes");
			return 0;
		}

		if (strcmp(argv[2], "off") == 0 && oled_inverted) {
			rc = cfb_framebuffer_invert(oled_dev);
			if (rc != 0) {
				shell_error(shell, "Invert failed (%d).", rc);
				return rc;
			}
			oled_inverted = false;
			shell_print(shell, "Inverted: no");
			return 0;
		}

		if (strcmp(argv[2], "on") == 0 || strcmp(argv[2], "off") == 0) {
			shell_print(shell, "Inverted: %s", oled_inverted ? "yes" : "no");
			return 0;
		}

		shell_error(shell, "Unknown invert command.");
		return -EINVAL;
	}

	if (strcmp(argv[1], "font") == 0) {
		int idx;

		if (argc < 3) {
			shell_error(shell, "Usage: oled font <idx>");
			return -EINVAL;
		}

		rc = oled_parse_int(shell, argv[2], &idx);
		if (rc < 0) {
			return rc;
		}

		if (idx < 0 || idx > 255) {
			shell_error(shell, "Font index out of range.");
			return -ERANGE;
		}

		rc = oled_init();
		if (rc < 0) {
			shell_error(shell, "OLED init failed (%d).", rc);
			return rc;
		}

		rc = oled_select_font((uint8_t)idx);
		if (rc < 0) {
			shell_error(shell, "Font %d not available.", idx);
			return rc;
		}

		shell_print(shell, "Font set to %d (%ux%u).", oled_font_idx,
			    oled_font_width, oled_font_height);
		return 0;
	}

	shell_error(shell, "Unknown command.");
	return -EINVAL;
}
#else
static int cmd_oled(const struct shell *shell, size_t argc, char **argv)
{
	(void)argc;
	(void)argv;

	shell_error(shell, "OLED not configured in devicetree.");
	return -ENODEV;
}
#endif

SHELL_CMD_REGISTER(oled, NULL, "OLED display control.", cmd_oled);
