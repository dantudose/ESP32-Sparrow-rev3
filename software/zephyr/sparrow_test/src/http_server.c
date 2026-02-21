#include "http_server.h"

#include <errno.h>
#include <string.h>
#include <zephyr/fs/fs.h>
#include <zephyr/net/http/server.h>
#include <zephyr/net/http/service.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>

#include "app_paths.h"
#include "boot_log.h"
#include "sensors.h"

#define BME_LOG_HTTP_PATH "/bme_log"

static int ensure_http_dir(void)
{
	struct fs_dirent entry;
	int rc = fs_stat(LITTLEFS_HTTP_ROOT, &entry);

	if (rc == 0) {
		if (entry.type != FS_DIR_ENTRY_DIR) {
			return -ENOTDIR;
		}
		return 0;
	}

	if (rc != -ENOENT) {
		return rc;
	}

	rc = fs_mkdir(LITTLEFS_HTTP_ROOT);
	if (rc < 0 && rc != -EEXIST) {
		return rc;
	}

	return 0;
}

static void warn_if_http_index_missing(void)
{
	struct fs_dirent entry;
	int rc = fs_stat(LITTLEFS_HTTP_INDEX, &entry);

	if (rc == -ENOENT) {
		boot_log_write("http: index missing (%s)", LITTLEFS_HTTP_INDEX);
		return;
	}

	if (rc < 0) {
		boot_log_write("http: stat %s failed (%d)",
			       LITTLEFS_HTTP_INDEX, rc);
		return;
	}

	if (entry.type != FS_DIR_ENTRY_FILE) {
		boot_log_write("http: index not file (%s)", LITTLEFS_HTTP_INDEX);
	}
}

static struct http_resource_detail_static_fs littlefs_http_detail = {
	.common = {
		.type = HTTP_RESOURCE_TYPE_STATIC_FS,
		.bitmask_of_supported_http_methods = BIT(HTTP_GET),
	},
	.fs_path = LITTLEFS_HTTP_ROOT,
};

static uint16_t littlefs_http_port = LITTLEFS_HTTP_PORT;
static atomic_t http_server_running = ATOMIC_INIT(0);

HTTP_SERVICE_DEFINE(littlefs_http_service, NULL, &littlefs_http_port,
		    2, 10, NULL, &littlefs_http_detail.common, NULL);

struct bme_log_http_state {
	struct fs_file_t file;
	char path[BME_LOG_PATH_MAX];
	bool active;
	bool file_open;
};

static struct bme_log_http_state bme_log_http_state;
static uint8_t bme_log_http_buf[256];

static int bme_log_http_handler(struct http_client_ctx *client,
				enum http_data_status status,
				const struct http_request_ctx *request_ctx,
				struct http_response_ctx *response_ctx,
				void *user_data)
{
	struct bme_log_http_state *state = user_data;
	static const char not_found[] = "Log file not found.\n";
	static const char read_err[] = "Log file read error.\n";
	(void)client;
	(void)request_ctx;

	if (status == HTTP_SERVER_DATA_ABORTED) {
		if (state->file_open) {
			fs_close(&state->file);
		}
		memset(state, 0, sizeof(*state));
		return 0;
	}

	if (!state->active) {
		memset(state, 0, sizeof(*state));
		state->active = true;
		bme_log_get_path(state->path, sizeof(state->path));
		fs_file_t_init(&state->file);
		if (fs_open(&state->file, state->path, FS_O_READ) == 0) {
			state->file_open = true;
		}
	}

	if (!state->file_open) {
		response_ctx->status = 404;
		response_ctx->body = (const uint8_t *)not_found;
		response_ctx->body_len = sizeof(not_found) - 1;
		response_ctx->final_chunk = true;
		state->active = false;
		return 0;
	}

	ssize_t rc = fs_read(&state->file, bme_log_http_buf,
			     sizeof(bme_log_http_buf));
	if (rc < 0) {
		response_ctx->status = 500;
		response_ctx->body = (const uint8_t *)read_err;
		response_ctx->body_len = sizeof(read_err) - 1;
		response_ctx->final_chunk = true;
		fs_close(&state->file);
		state->file_open = false;
		state->active = false;
		return 0;
	}

	if (rc == 0) {
		fs_close(&state->file);
		state->file_open = false;
		response_ctx->final_chunk = true;
		state->active = false;
		return 0;
	}

	response_ctx->body = bme_log_http_buf;
	response_ctx->body_len = rc;
	response_ctx->final_chunk = false;
	return 0;
}

static struct http_resource_detail_dynamic bme_log_http_detail = {
	.common = {
		.type = HTTP_RESOURCE_TYPE_DYNAMIC,
		.bitmask_of_supported_http_methods = BIT(HTTP_GET),
		.content_type = "text/csv",
	},
	.cb = bme_log_http_handler,
	.user_data = &bme_log_http_state,
};

HTTP_RESOURCE_DEFINE(bme_log_http_resource, littlefs_http_service,
		     BME_LOG_HTTP_PATH, &bme_log_http_detail);

bool http_server_is_running(void)
{
	return atomic_get(&http_server_running) != 0;
}

int start_littlefs_http_server(void)
{
	int rc;

	rc = ensure_http_dir();
	if (rc < 0) {
		boot_log_write("http: prepare %s failed (%d)",
			       LITTLEFS_HTTP_ROOT, rc);
		return rc;
	}

	warn_if_http_index_missing();

	rc = http_server_start();
	if (rc == 0) {
		boot_log_write("http: server started port %u root %s",
			       littlefs_http_port, LITTLEFS_HTTP_ROOT);
		atomic_set(&http_server_running, 1);
		return 0;
	}

	if (rc == -EALREADY) {
		atomic_set(&http_server_running, 1);
		return rc;
	}

	boot_log_write("http: start failed (%d)", rc);
	return rc;
}

int stop_littlefs_http_server(void)
{
	int rc = http_server_stop();

	if (rc == 0) {
		boot_log_write("http: server stopped");
		atomic_set(&http_server_running, 0);
		return 0;
	}

	if (rc == -EALREADY) {
		atomic_set(&http_server_running, 0);
		return rc;
	}

	boot_log_write("http: stop failed (%d)", rc);
	return rc;
}

static void http_print_help(const struct shell *shell)
{
	shell_print(shell, "Usage:");
	shell_print(shell, "  http status");
	shell_print(shell, "  http start");
	shell_print(shell, "  http stop");
}

static int cmd_http(const struct shell *shell, size_t argc, char **argv)
{
	int rc;

	if (argc < 2) {
		http_print_help(shell);
		return -EINVAL;
	}

	if (strcmp(argv[1], "status") == 0) {
		shell_print(shell, "HTTP server: %s",
			    http_server_is_running() ? "on" : "off");
		shell_print(shell, "Port: %u", littlefs_http_port);
		shell_print(shell, "Root: %s", LITTLEFS_HTTP_ROOT);
		return 0;
	}

	if (strcmp(argv[1], "start") == 0) {
		rc = start_littlefs_http_server();
		if (rc == 0) {
			shell_print(shell, "HTTP server started (port %u).",
				    littlefs_http_port);
			return 0;
		}
		if (rc == -EALREADY) {
			shell_print(shell, "HTTP server already running.");
			return 0;
		}
		shell_error(shell, "HTTP server start failed (%d).", rc);
		return rc;
	}

	if (strcmp(argv[1], "stop") == 0) {
		rc = stop_littlefs_http_server();
		if (rc == 0) {
			shell_print(shell, "HTTP server stopped.");
			return 0;
		}
		if (rc == -EALREADY) {
			shell_print(shell, "HTTP server already stopped.");
			return 0;
		}
		shell_error(shell, "HTTP server stop failed (%d).", rc);
		return rc;
	}

	shell_error(shell, "Unknown command.");
	return -EINVAL;
}

SHELL_CMD_REGISTER(http, NULL, "HTTP server control.", cmd_http);
