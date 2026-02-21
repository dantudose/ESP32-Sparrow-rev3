#include <inttypes.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/drivers/hwinfo.h>
#include <zephyr/fs/fs.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "app_paths.h"
#include "boot_log.h"
#include "fs_utils.h"
#include "net_time.h"

#define BOOT_LOG_PATH BME_LOG_DIR "/boot.log"
#define BOOT_COUNT_PATH BME_LOG_DIR "/boot.count"
#define BOOT_LOG_MSG_MAX 128
#define BOOT_LOG_LINE_MAX 192
#define BOOT_DIAG_THREAD_STACK_SIZE 2048
#define BOOT_DIAG_THREAD_PRIORITY 7
#define BOOT_DIAG_DELAY_MS 1500

static struct k_mutex boot_log_lock;
static bool boot_log_ready;
static uint32_t boot_count;
static struct k_thread boot_diag_thread;
static K_THREAD_STACK_DEFINE(boot_diag_stack, BOOT_DIAG_THREAD_STACK_SIZE);

void boot_log_write(const char *fmt, ...)
{
	struct fs_file_t file;
	char msg[BOOT_LOG_MSG_MAX];
	char line[BOOT_LOG_LINE_MAX];
	va_list args;
	int rc;
	size_t len;

	if (!boot_log_ready) {
		return;
	}

	va_start(args, fmt);
	vsnprintk(msg, sizeof(msg), fmt, args);
	va_end(args);

	rc = snprintk(line, sizeof(line), "%" PRId64 " %s\n",
		      time_now_ms(), msg);
	if (rc < 0) {
		return;
	}

	len = MIN((size_t)rc, sizeof(line) - 1U);

	k_mutex_lock(&boot_log_lock, K_FOREVER);
	fs_file_t_init(&file);
	rc = fs_open(&file, BOOT_LOG_PATH, FS_O_CREATE | FS_O_WRITE);
	if (rc == 0) {
		rc = fs_seek(&file, 0, FS_SEEK_END);
		if (rc == 0) {
			rc = fs_write(&file, line, len);
		}
		fs_close(&file);
	}
	k_mutex_unlock(&boot_log_lock);

	if (rc < 0) {
		printk("Boot log write failed (%d)\n", rc);
	}
}

static void reset_cause_format(uint32_t cause, char *buf, size_t len)
{
	struct reset_cause_name {
		uint32_t flag;
		const char *name;
	};
	static const struct reset_cause_name names[] = {
		{ RESET_PIN, "pin" },
		{ RESET_SOFTWARE, "software" },
		{ RESET_BROWNOUT, "brownout" },
		{ RESET_POR, "por" },
		{ RESET_WATCHDOG, "watchdog" },
		{ RESET_DEBUG, "debug" },
		{ RESET_SECURITY, "security" },
		{ RESET_LOW_POWER_WAKE, "low_power" },
		{ RESET_CPU_LOCKUP, "cpu_lockup" },
		{ RESET_PARITY, "parity" },
		{ RESET_PLL, "pll" },
		{ RESET_CLOCK, "clock" },
		{ RESET_HARDWARE, "hardware" },
		{ RESET_USER, "user" },
		{ RESET_TEMPERATURE, "temperature" },
		{ RESET_BOOTLOADER, "bootloader" },
		{ RESET_FLASH, "flash" },
	};
	size_t pos = 0;

	if (!buf || len == 0) {
		return;
	}

	buf[0] = '\0';
	if (cause == 0) {
		snprintk(buf, len, "unknown");
		return;
	}

	for (size_t i = 0; i < ARRAY_SIZE(names); i++) {
		if (!(cause & names[i].flag)) {
			continue;
		}

		int rc = snprintk(buf + pos, len - pos, pos ? "|%s" : "%s", names[i].name);

		if (rc < 0 || (size_t)rc >= (len - pos)) {
			buf[len - 1] = '\0';
			return;
		}
		pos += (size_t)rc;
	}

	if (pos == 0) {
		snprintk(buf, len, "unknown");
	}
}

static int boot_count_update(void)
{
	struct fs_file_t file;
	struct fs_dirent entry;
	char buf[16];
	ssize_t read_len;
	uint32_t count = 0;
	int rc;

	rc = fs_stat(BOOT_COUNT_PATH, &entry);
	if (rc == 0) {
		if (entry.type != FS_DIR_ENTRY_FILE) {
			return -EINVAL;
		}
		fs_file_t_init(&file);
		rc = fs_open(&file, BOOT_COUNT_PATH, FS_O_READ);
		if (rc < 0) {
			return rc;
		}
		read_len = fs_read(&file, buf, sizeof(buf) - 1);
		if (read_len > 0) {
			buf[read_len] = '\0';
			count = (uint32_t)strtoul(buf, NULL, 10);
		}
		fs_close(&file);
	} else if (rc != -ENOENT) {
		return rc;
	}

	count++;
	fs_file_t_init(&file);
	rc = fs_open(&file, BOOT_COUNT_PATH, FS_O_CREATE | FS_O_WRITE | FS_O_TRUNC);
	if (rc == 0) {
		int len = snprintk(buf, sizeof(buf), "%" PRIu32 "\n", count);

		if (len > 0) {
			ssize_t wr = fs_write(&file, buf, len);

			if (wr < 0) {
				rc = (int)wr;
			}
		} else {
			rc = -EIO;
		}
		fs_close(&file);
	}

	if (rc < 0) {
		return rc;
	}

	boot_count = count;
	return 0;
}

static void boot_log_reset_cause(void)
{
	uint32_t cause = 0;
	char desc[96];
	int rc;

	rc = hwinfo_get_reset_cause(&cause);
	if (rc < 0) {
		boot_log_write("boot: reset cause unavailable (%d)", rc);
		return;
	}

	reset_cause_format(cause, desc, sizeof(desc));
	boot_log_write("boot: reset cause 0x%08x (%s)", cause, desc);
}

static void boot_diag_thread_fn(void *p1, void *p2, void *p3)
{
	(void)p1;
	(void)p2;
	(void)p3;

	k_sleep(K_MSEC(BOOT_DIAG_DELAY_MS));

	if (!boot_log_ready) {
		return;
	}

	{
		int rc = boot_count_update();

		if (rc == 0) {
			boot_log_write("boot: count %" PRIu32, boot_count);
		} else {
			boot_log_write("boot: count unavailable (%d)", rc);
		}
	}

	boot_log_reset_cause();
}

void boot_log_init(void)
{
	int rc;

	k_mutex_init(&boot_log_lock);

	rc = ensure_littlefs_ready();
	if (rc < 0) {
		printk("Boot log: LittleFS not ready (%d)\n", rc);
		return;
	}

	rc = ensure_log_dir();
	if (rc < 0) {
		printk("Boot log: failed to create %s (%d)\n", BME_LOG_DIR, rc);
		return;
	}

	boot_log_ready = true;
	boot_log_write("boot: log started");
}

void boot_diag_start(void)
{
	if (!boot_log_ready) {
		return;
	}

	k_thread_create(&boot_diag_thread, boot_diag_stack,
			K_THREAD_STACK_SIZEOF(boot_diag_stack),
			boot_diag_thread_fn, NULL, NULL, NULL,
			BOOT_DIAG_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&boot_diag_thread, "boot_diag");
}
