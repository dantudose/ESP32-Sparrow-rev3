#include "startup.h"

#include <ctype.h>
#include <errno.h>
#include <string.h>
#include <zephyr/fs/fs.h>
#include <zephyr/kernel.h>
#include <zephyr/shell/shell.h>
#include <zephyr/shell/shell_uart.h>
#include <zephyr/sys/printk.h>

#include "boot_log.h"

#define STARTUP_SCRIPT_PATH "/lfs/startup.txt"
#define STARTUP_SCRIPT_WAIT_MS 3000
#define STARTUP_SCRIPT_MAX_LINE CONFIG_SHELL_CMD_BUFF_SIZE
#define STARTUP_SCRIPT_THREAD_STACK_SIZE 4096
#define STARTUP_SCRIPT_THREAD_PRIORITY 7

static char *trim_startup_line(char *line)
{
	while (*line != '\0' && isspace((unsigned char)*line)) {
		line++;
	}

	char *end = line + strlen(line);
	while (end > line && isspace((unsigned char)end[-1])) {
		end--;
	}
	*end = '\0';

	return line;
}

static void execute_startup_line(const struct shell *sh, char *line)
{
	char *cmd = trim_startup_line(line);

	if (cmd[0] == '\0' || cmd[0] == '#') {
		return;
	}

	boot_log_write("startup: cmd \"%s\"", cmd);
	int rc = shell_execute_cmd(sh, cmd);
	if (rc < 0) {
		boot_log_write("startup: cmd failed (%d)", rc);
	} else {
		boot_log_write("startup: cmd ok");
	}
}

static void run_startup_script(const struct shell *sh)
{
	struct fs_file_t file;
	char line[STARTUP_SCRIPT_MAX_LINE];
	size_t len = 0U;
	bool drop_line = false;
	int rc;

	if (!sh) {
		printk("Startup script: shell backend unavailable\n");
		return;
	}

	fs_file_t_init(&file);
	rc = fs_open(&file, STARTUP_SCRIPT_PATH, FS_O_READ);
	if (rc < 0) {
		if (rc != -ENOENT) {
			boot_log_write("startup: open %s failed (%d)",
				       STARTUP_SCRIPT_PATH, rc);
		} else {
			boot_log_write("startup: %s missing",
				       STARTUP_SCRIPT_PATH);
		}
		return;
	}

	boot_log_write("startup: begin");
	while (true) {
		char ch;

		rc = fs_read(&file, &ch, 1);
		if (rc < 0) {
			boot_log_write("startup: read error (%d)", rc);
			break;
		}

		if (rc == 0) {
			if (!drop_line && len > 0U) {
				line[len] = '\0';
				execute_startup_line(sh, line);
			}
			break;
		}

		if (ch == '\r') {
			continue;
		}

		if (ch == '\n') {
			if (!drop_line && len > 0U) {
				line[len] = '\0';
				execute_startup_line(sh, line);
			}
			len = 0U;
			drop_line = false;
			continue;
		}

		if (drop_line) {
			continue;
		}

		if (len < sizeof(line) - 1U) {
			line[len++] = ch;
		} else {
			drop_line = true;
			boot_log_write("startup: line too long");
		}
	}

	fs_close(&file);
	boot_log_write("startup: done");
}

static struct k_thread startup_script_thread;
static K_THREAD_STACK_DEFINE(startup_script_stack, STARTUP_SCRIPT_THREAD_STACK_SIZE);

static void startup_script_thread_fn(void *p1, void *p2, void *p3)
{
	const struct shell *sh = shell_backend_uart_get_ptr();
	bool stopped = false;
	bool ready = false;

	(void)p1;
	(void)p2;
	(void)p3;

	if (!sh) {
		printk("Startup script: shell backend unavailable\n");
		return;
	}

	for (int i = 0; i < STARTUP_SCRIPT_WAIT_MS / 100; i++) {
		if (shell_ready(sh)) {
			ready = true;
			if (shell_stop(sh) == 0) {
				stopped = true;
				boot_log_write("startup: shell stopped");
			}
			break;
		}
		k_sleep(K_MSEC(100));
	}

	if (!ready) {
		boot_log_write("startup: shell not ready");
	}

	run_startup_script(sh);

	if (stopped) {
		if (shell_start(sh) == 0) {
			boot_log_write("startup: shell restarted");
		} else {
			boot_log_write("startup: shell restart failed");
		}
	}
}

void startup_script_start(void)
{
	k_thread_create(&startup_script_thread, startup_script_stack,
			K_THREAD_STACK_SIZEOF(startup_script_stack),
			startup_script_thread_fn, NULL, NULL, NULL,
			STARTUP_SCRIPT_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&startup_script_thread, "startup_script");
	boot_log_write("boot: startup script thread started");
}
