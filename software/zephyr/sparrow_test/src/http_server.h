#pragma once

#include <stdbool.h>

bool http_server_is_running(void);
int start_littlefs_http_server(void);
int stop_littlefs_http_server(void);
