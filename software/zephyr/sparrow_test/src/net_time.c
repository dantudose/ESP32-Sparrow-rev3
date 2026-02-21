#include "net_time.h"

#include <errno.h>
#include <inttypes.h>
#include <stdarg.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/net/dhcpv4.h>
#include <zephyr/net/dns_resolve.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_ip.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/sntp.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>

#include "boot_log.h"

#define NET_DIAG_MSG_MAX 128
#define TIME_SYNC_THREAD_STACK_SIZE 2048
#define TIME_SYNC_THREAD_PRIORITY 7
#define TIME_SYNC_RETRY_S 10
#define TIME_SYNC_TIMEOUT_MS 4000
#define TIME_FALLBACK_EPOCH_SEC 449132400LL
#define TIME_SYNC_SERVER_PRIMARY "pool.ntp.org"
#define TIME_SYNC_SERVER_FALLBACK "129.6.15.28"

static struct k_thread time_sync_thread;
static K_THREAD_STACK_DEFINE(time_sync_stack, TIME_SYNC_THREAD_STACK_SIZE);
static struct k_mutex time_lock;
static int64_t time_offset_ms;
static bool time_synced;

static atomic_t ipv4_diag_logged = ATOMIC_INIT(0);
static struct net_mgmt_event_callback ipv4_event_cb;

static void time_set_offset_ms(int64_t offset_ms, bool synced)
{
	k_mutex_lock(&time_lock, K_FOREVER);
	time_offset_ms = offset_ms;
	time_synced = synced;
	k_mutex_unlock(&time_lock);
}

int64_t time_now_ms(void)
{
	int64_t offset;

	k_mutex_lock(&time_lock, K_FOREVER);
	offset = time_offset_ms;
	k_mutex_unlock(&time_lock);

	return (int64_t)k_uptime_get() + offset;
}

bool time_is_synced(void)
{
	bool synced;

	k_mutex_lock(&time_lock, K_FOREVER);
	synced = time_synced;
	k_mutex_unlock(&time_lock);

	return synced;
}

static void net_diag_log(const char *fmt, ...)
{
	char msg[NET_DIAG_MSG_MAX];
	va_list args;

	va_start(args, fmt);
	vsnprintk(msg, sizeof(msg), fmt, args);
	va_end(args);

	boot_log_write("%s", msg);
}

static void net_format_ll_addr(const struct net_linkaddr *ll, char *buf, size_t len)
{
	size_t pos = 0;

	if (!buf || len == 0) {
		return;
	}

	buf[0] = '\0';
	if (!ll || ll->len == 0) {
		return;
	}

	for (uint8_t i = 0; i < ll->len; i++) {
		int rc = snprintk(buf + pos, len - pos, i ? ":%02x" : "%02x", ll->addr[i]);

		if (rc < 0 || (size_t)rc >= (len - pos)) {
			buf[len - 1] = '\0';
			break;
		}
		pos += (size_t)rc;
	}
}

static void net_log_dns_servers(const char *tag)
{
#if defined(CONFIG_DNS_RESOLVER)
	struct dns_resolve_context *ctx = dns_resolve_get_default();
	const char *use_tag = tag ? tag : "boot";
	bool any = false;

	if (!ctx) {
		net_diag_log("net diag(%s): dns context unavailable", use_tag);
		return;
	}

	k_mutex_lock(&ctx->lock, K_FOREVER);
	for (size_t i = 0; i < ARRAY_SIZE(ctx->servers); i++) {
		const struct dns_server *srv = &ctx->servers[i];
		char addr[NET_INET6_ADDRSTRLEN];
		const char *addr_str = NULL;

		if (srv->dns_server.sa_family == 0) {
			continue;
		}

		if (srv->dns_server.sa_family == NET_AF_INET) {
			addr_str = net_addr_ntop(NET_AF_INET,
						 &net_sin(&srv->dns_server)->sin_addr,
						 addr, sizeof(addr));
		} else if (srv->dns_server.sa_family == NET_AF_INET6) {
			addr_str = net_addr_ntop(NET_AF_INET6,
						 &net_sin6(&srv->dns_server)->sin6_addr,
						 addr, sizeof(addr));
		}

		if (!addr_str) {
			addr_str = "unknown";
		}

		net_diag_log("net diag(%s): dns[%zu] %s source=%s if=%d mdns=%d llmnr=%d",
			     use_tag, i, addr_str, dns_get_source_str(srv->source),
			     srv->if_index, srv->is_mdns, srv->is_llmnr);
		any = true;
	}
	k_mutex_unlock(&ctx->lock);

	if (!any) {
		net_diag_log("net diag(%s): dns servers none", use_tag);
	}
#else
	ARG_UNUSED(tag);
#endif
}

struct net_if *time_get_wifi_iface(void)
{
	struct net_if *wifi = net_if_get_first_wifi();

	if (!wifi) {
		wifi = net_if_get_default();
	}

	return wifi;
}

static bool time_get_dhcp_ntp_server(char *buf, size_t len)
{
#if defined(CONFIG_NET_DHCPV4) && defined(CONFIG_NET_DHCPV4_OPTION_NTP_SERVER)
	struct net_if *iface = time_get_wifi_iface();
	const struct net_in_addr *ntp_addr;

	if (!iface) {
		return false;
	}

	ntp_addr = &iface->config.dhcpv4.ntp_addr;
	if (net_ipv4_is_addr_unspecified(ntp_addr)) {
		return false;
	}

	if (!net_addr_ntop(NET_AF_INET, ntp_addr, buf, len)) {
		return false;
	}

	return true;
#else
	ARG_UNUSED(buf);
	ARG_UNUSED(len);
	return false;
#endif
}

void net_log_diagnostics(const char *tag)
{
	struct net_if *iface = time_get_wifi_iface();
	const char *use_tag = tag ? tag : "boot";
	char ifname[NET_IFNAMSIZ] = "";
	const struct device *dev;
	const struct net_linkaddr *ll;
	char ll_buf[NET_LINK_ADDR_MAX_LENGTH * 3];
	bool up;
	int ifindex;

	if (!iface) {
		net_diag_log("net diag(%s): no network interface", use_tag);
		net_log_dns_servers(use_tag);
		return;
	}

	ifindex = net_if_get_by_iface(iface);
	dev = net_if_get_device(iface);
	up = net_if_is_up(iface);
	(void)net_if_get_name(iface, ifname, sizeof(ifname));
	ll = net_if_get_link_addr(iface);
	net_format_ll_addr(ll, ll_buf, sizeof(ll_buf));

	net_diag_log("net diag(%s): iface %d name=%s up=%d dev=%s ready=%d mac=%s",
		     use_tag, ifindex,
		     ifname[0] != '\0' ? ifname : "n/a",
		     up,
		     dev ? dev->name : "n/a",
		     dev ? device_is_ready(dev) : 0,
		     ll_buf[0] != '\0' ? ll_buf : "n/a");

	if (IS_ENABLED(CONFIG_NET_IPV4)) {
		struct net_in_addr *addr;
		char ip_buf[NET_IPV4_ADDR_LEN];
		char mask_buf[NET_IPV4_ADDR_LEN];
		char gw_buf[NET_IPV4_ADDR_LEN];

		addr = net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED);
		if (addr &&
		    net_addr_ntop(NET_AF_INET, addr, ip_buf, sizeof(ip_buf))) {
			struct net_in_addr mask = net_if_ipv4_get_netmask_by_addr(iface, addr);
			struct net_in_addr gw = net_if_ipv4_get_gw(iface);

			(void)net_addr_ntop(NET_AF_INET, &mask, mask_buf, sizeof(mask_buf));
			(void)net_addr_ntop(NET_AF_INET, &gw, gw_buf, sizeof(gw_buf));
			net_diag_log("net diag(%s): ipv4 addr=%s mask=%s gw=%s",
				     use_tag, ip_buf, mask_buf, gw_buf);
		} else {
			net_diag_log("net diag(%s): ipv4 addr=none", use_tag);
		}
	}

#if defined(CONFIG_NET_DHCPV4)
	net_diag_log("net diag(%s): dhcp state=%s lease=%u",
		     use_tag, net_dhcpv4_state_name(iface->config.dhcpv4.state),
		     iface->config.dhcpv4.lease_time);
#endif

	{
		char ntp_server[NET_IPV4_ADDR_LEN];

		if (time_get_dhcp_ntp_server(ntp_server, sizeof(ntp_server))) {
			net_diag_log("net diag(%s): dhcp ntp=%s", use_tag, ntp_server);
		} else {
			net_diag_log("net diag(%s): dhcp ntp=none", use_tag);
		}
	}

	net_log_dns_servers(use_tag);
}

static void net_ipv4_event_handler(struct net_mgmt_event_callback *cb, uint64_t mgmt_event,
				   struct net_if *iface)
{
	struct net_if *wifi = time_get_wifi_iface();

	ARG_UNUSED(cb);

	if (wifi && iface && iface != wifi) {
		return;
	}

	if (mgmt_event == NET_EVENT_IPV4_ADDR_ADD) {
		if (atomic_cas(&ipv4_diag_logged, 0, 1)) {
			net_log_diagnostics("ipv4");
		}
	} else if (mgmt_event == NET_EVENT_IPV4_ADDR_DEL) {
		atomic_set(&ipv4_diag_logged, 0);
	}
}

void net_ipv4_diag_init(void)
{
	net_mgmt_init_event_callback(&ipv4_event_cb, net_ipv4_event_handler,
				     NET_EVENT_IPV4_ADDR_ADD | NET_EVENT_IPV4_ADDR_DEL);
	net_mgmt_add_event_callback(&ipv4_event_cb);
}

void net_set_default_wifi(void)
{
	struct net_if *wifi = net_if_get_first_wifi();

	if (!wifi) {
		printk("Net: no Wi-Fi interface found\n");
		boot_log_write("net: no wifi iface found");
		return;
	}

	net_if_set_default(wifi);
	printk("Net: default interface set to Wi-Fi (idx %d)\n",
	       net_if_get_by_iface(wifi));
	boot_log_write("net: default iface wifi idx %d",
		       net_if_get_by_iface(wifi));
}

void time_init(void)
{
	int64_t offset_ms = (TIME_FALLBACK_EPOCH_SEC * 1000LL) - (int64_t)k_uptime_get();

	k_mutex_init(&time_lock);
	time_set_offset_ms(offset_ms, false);
}

static bool time_net_ready(void)
{
	struct net_if *iface = time_get_wifi_iface();

	if (!iface) {
		return false;
	}

	if (!net_if_is_up(iface)) {
		return false;
	}

	return net_if_ipv4_get_global_addr(iface, NET_ADDR_PREFERRED) != NULL;
}

static int time_sync_attempt(const char *server, struct sntp_time *ts, int *err_out)
{
	int rc;
	struct net_sockaddr_in addr;

	errno = 0;
	if (net_addr_pton(NET_AF_INET, server, &addr.sin_addr) == 0) {
		addr.sin_family = NET_AF_INET;
		addr.sin_port = 0;
		rc = sntp_simple_addr((struct net_sockaddr *)&addr,
				      sizeof(addr), TIME_SYNC_TIMEOUT_MS, ts);
	} else {
		rc = sntp_simple(server, TIME_SYNC_TIMEOUT_MS, ts);
	}
	if (rc < 0 && err_out) {
		*err_out = errno;
	}
	return rc;
}

static int time_sync_once(const char *dhcp_ntp, bool have_dhcp_ntp,
			  char *server_used, size_t server_used_len,
			  int *rc_primary, int *err_primary,
			  int *rc_dhcp, int *err_dhcp,
			  int *rc_fallback, int *err_fallback)
{
	struct sntp_time ts;
	int rc;
	int err;

	if (rc_primary) {
		*rc_primary = 0;
	}
	if (err_primary) {
		*err_primary = 0;
	}
	if (rc_dhcp) {
		*rc_dhcp = 0;
	}
	if (err_dhcp) {
		*err_dhcp = 0;
	}
	if (rc_fallback) {
		*rc_fallback = 0;
	}
	if (err_fallback) {
		*err_fallback = 0;
	}

	rc = time_sync_attempt(TIME_SYNC_SERVER_PRIMARY, &ts, &err);
	if (rc == 0) {
		if (server_used && server_used_len > 0) {
			snprintk(server_used, server_used_len, "%s", TIME_SYNC_SERVER_PRIMARY);
		}
		goto synced;
	}

	if (rc_primary) {
		*rc_primary = rc;
	}
	if (err_primary) {
		*err_primary = err;
	}

	if (have_dhcp_ntp) {
		rc = time_sync_attempt(dhcp_ntp, &ts, &err);
		if (rc == 0) {
			if (server_used && server_used_len > 0) {
				snprintk(server_used, server_used_len, "%s", dhcp_ntp);
			}
			goto synced;
		}
		if (rc_dhcp) {
			*rc_dhcp = rc;
		}
		if (err_dhcp) {
			*err_dhcp = err;
		}
	}

	rc = time_sync_attempt(TIME_SYNC_SERVER_FALLBACK, &ts, &err);
	if (rc < 0) {
		if (rc_fallback) {
			*rc_fallback = rc;
		}
		if (err_fallback) {
			*err_fallback = err;
		}
		return rc;
	}
	if (server_used && server_used_len > 0) {
		snprintk(server_used, server_used_len, "%s", TIME_SYNC_SERVER_FALLBACK);
	}

synced:
	{
		uint64_t frac_ms = ((uint64_t)ts.fraction * 1000U) >> 32;
		int64_t epoch_ms = (int64_t)(ts.seconds * 1000U + frac_ms);
		time_set_offset_ms(epoch_ms - (int64_t)k_uptime_get(), true);
	}
	return 0;
}

static void time_sync_thread_fn(void *p1, void *p2, void *p3)
{
	bool logged_wait = false;
	bool logged_net = false;
	bool logged_diag = false;
	int fail_count = 0;

	(void)p1;
	(void)p2;
	(void)p3;

	while (true) {
		int rc;
		int rc_primary = 0;
		int rc_fallback = 0;
		int rc_dhcp = 0;
		char server[NET_IPV4_ADDR_LEN] = "";
		char dhcp_ntp[NET_IPV4_ADDR_LEN] = "";
		bool have_dhcp_ntp;
		int err_primary = 0;
		int err_dhcp = 0;
		int err_fallback = 0;

		if (time_is_synced()) {
			k_sleep(K_MINUTES(10));
			continue;
		}

		if (!time_net_ready()) {
			if (!logged_net) {
				boot_log_write("time: waiting for IPv4 address");
				logged_net = true;
			}
			k_sleep(K_SECONDS(TIME_SYNC_RETRY_S));
			continue;
		}

		logged_net = false;
		if (!logged_diag) {
			if (atomic_cas(&ipv4_diag_logged, 0, 1)) {
				net_log_diagnostics("ipv4");
			}
			logged_diag = true;
		}
		have_dhcp_ntp = time_get_dhcp_ntp_server(dhcp_ntp, sizeof(dhcp_ntp));
		rc = time_sync_once(dhcp_ntp, have_dhcp_ntp,
				    server, sizeof(server),
				    &rc_primary, &err_primary,
				    &rc_dhcp, &err_dhcp,
				    &rc_fallback, &err_fallback);
		if (rc == 0) {
			const char *server_name = server[0] != '\0' ? server : "unknown";

			boot_log_write("time: synced via %s", server_name);
			logged_wait = false;
			fail_count = 0;
		} else {
			fail_count++;
			if (!logged_wait || (fail_count % 6) == 0) {
				if (have_dhcp_ntp) {
					boot_log_write("time: SNTP failed (%s rc=%d errno=%d, %s rc=%d errno=%d, %s rc=%d errno=%d)",
						       TIME_SYNC_SERVER_PRIMARY, rc_primary, err_primary,
						       dhcp_ntp, rc_dhcp, err_dhcp,
						       TIME_SYNC_SERVER_FALLBACK, rc_fallback, err_fallback);
				} else {
					boot_log_write("time: SNTP failed (%s rc=%d errno=%d, %s rc=%d errno=%d)",
						       TIME_SYNC_SERVER_PRIMARY, rc_primary, err_primary,
						       TIME_SYNC_SERVER_FALLBACK, rc_fallback, err_fallback);
				}
				logged_wait = true;
			}
		}

		k_sleep(K_SECONDS(TIME_SYNC_RETRY_S));
	}
}

void time_sync_start(void)
{
	k_thread_create(&time_sync_thread, time_sync_stack,
			K_THREAD_STACK_SIZEOF(time_sync_stack),
			time_sync_thread_fn, NULL, NULL, NULL,
			TIME_SYNC_THREAD_PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&time_sync_thread, "time_sync");
	boot_log_write("boot: time sync thread started");
}

static int cmd_time(const struct shell *shell, size_t argc, char **argv)
{
	int64_t now_ms;

	(void)argc;
	(void)argv;

	now_ms = time_now_ms();
	shell_print(shell, "Epoch ms: %" PRId64, now_ms);
	shell_print(shell, "Epoch s: %" PRId64, now_ms / 1000);
	shell_print(shell, "Synced: %s", time_is_synced() ? "yes" : "no (fallback)");
	return 0;
}

SHELL_CMD_REGISTER(time, NULL, "Show current epoch time.", cmd_time);
