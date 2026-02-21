#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/net/net_if.h>

void time_init(void);
void time_sync_start(void);
int64_t time_now_ms(void);
bool time_is_synced(void);
struct net_if *time_get_wifi_iface(void);

void net_set_default_wifi(void);
void net_ipv4_diag_init(void);
void net_log_diagnostics(const char *tag);
