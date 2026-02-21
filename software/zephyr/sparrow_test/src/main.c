#include <ctype.h>
#include <errno.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_uart.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "app_paths.h"
#include "boot_log.h"
#include "fs_utils.h"
#include "net_time.h"
#include "sensors.h"
#include "startup.h"

int main(void)
{
	int rc;

	printk("Zephyr console ready (ESP32-C6 Sparrow).\n");
	time_init();
	rc = mount_littlefs();
	if (rc == 0) {
		boot_log_init();
		boot_log_write("boot: littlefs mounted at %s", LITTLEFS_MOUNT_POINT);
		boot_diag_start();
	} else {
		printk("Boot: LittleFS unavailable, skipping HTTP server\n");
	}
	net_set_default_wifi();
	net_ipv4_diag_init();
	net_log_diagnostics("boot");
	time_sync_start();
	sensors_init();
	startup_script_start();

	while (true) {
		k_sleep(K_SECONDS(1));
	}

	return 0;
}
