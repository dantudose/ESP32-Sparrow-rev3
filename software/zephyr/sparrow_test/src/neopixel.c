#include <errno.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/shell/shell.h>
#include <zephyr/sys/util.h>

#if DT_HAS_ALIAS(neopixel)
#define NEOPIXEL_NODE DT_ALIAS(neopixel)
#else
#define NEOPIXEL_NODE DT_INVALID_NODE
#endif

#if DT_NODE_HAS_STATUS(NEOPIXEL_NODE, okay)
#define NEOPIXEL_COUNT DT_PROP(NEOPIXEL_NODE, chain_length)

static const struct device *const neopixel_dev = DEVICE_DT_GET(NEOPIXEL_NODE);
static struct led_rgb neopixel_state[NEOPIXEL_COUNT];

static int neopixel_parse_u8(const char *arg, uint8_t *out)
{
	char *end = NULL;
	unsigned long val;

	if (!arg || !out) {
		return -EINVAL;
	}

	errno = 0;
	val = strtoul(arg, &end, 0);
	if (errno || end == arg || *end != '\0' || val > UINT8_MAX) {
		return -EINVAL;
	}

	*out = (uint8_t)val;
	return 0;
}

static int neopixel_parse_index(const char *arg, size_t *out)
{
	char *end = NULL;
	unsigned long val;

	if (!arg || !out) {
		return -EINVAL;
	}

	errno = 0;
	val = strtoul(arg, &end, 0);
	if (errno || end == arg || *end != '\0') {
		return -EINVAL;
	}

	if (val >= NEOPIXEL_COUNT) {
		return -ERANGE;
	}

	*out = (size_t)val;
	return 0;
}

static int neopixel_apply(const struct shell *shell)
{
	int rc = led_strip_update_rgb(neopixel_dev, neopixel_state, NEOPIXEL_COUNT);

	if (rc < 0) {
		shell_error(shell, "Neopixel update failed (%d).", rc);
	}

	return rc;
}

static int cmd_neopixel(const struct shell *shell, size_t argc, char **argv)
{
	uint8_t r;
	uint8_t g;
	uint8_t b;
	size_t idx = 0;
	int rc;

	if (!device_is_ready(neopixel_dev)) {
		shell_error(shell, "Neopixel device not ready.");
		return -ENODEV;
	}

	if (argc < 2) {
		shell_print(shell, "Usage:");
		shell_print(shell, "  neopixel status");
		shell_print(shell, "  neopixel set <r> <g> <b> [index]");
		shell_print(shell, "  neopixel fill <r> <g> <b>");
		shell_print(shell, "  neopixel off");
		return 0;
	}

	if (strcmp(argv[1], "status") == 0) {
		shell_print(shell, "Neopixel %s length %u.",
			    neopixel_dev->name, (unsigned int)NEOPIXEL_COUNT);
		shell_print(shell, "Pixel[0] r=%u g=%u b=%u.",
			    neopixel_state[0].r, neopixel_state[0].g,
			    neopixel_state[0].b);
		return 0;
	}

	if (strcmp(argv[1], "off") == 0) {
		memset(neopixel_state, 0, sizeof(neopixel_state));
		return neopixel_apply(shell);
	}

	if (strcmp(argv[1], "fill") == 0) {
		if (argc != 5) {
			shell_error(shell, "Usage: neopixel fill <r> <g> <b>");
			return -EINVAL;
		}

		rc = neopixel_parse_u8(argv[2], &r);
		rc |= neopixel_parse_u8(argv[3], &g);
		rc |= neopixel_parse_u8(argv[4], &b);
		if (rc < 0) {
			shell_error(shell, "Invalid color value.");
			return -EINVAL;
		}

		for (size_t i = 0; i < NEOPIXEL_COUNT; i++) {
			neopixel_state[i].r = r;
			neopixel_state[i].g = g;
			neopixel_state[i].b = b;
		}

		return neopixel_apply(shell);
	}

	if (strcmp(argv[1], "set") == 0) {
		if (argc != 5 && argc != 6) {
			shell_error(shell, "Usage: neopixel set <r> <g> <b> [index]");
			return -EINVAL;
		}

		rc = neopixel_parse_u8(argv[2], &r);
		rc |= neopixel_parse_u8(argv[3], &g);
		rc |= neopixel_parse_u8(argv[4], &b);
		if (rc < 0) {
			shell_error(shell, "Invalid color value.");
			return -EINVAL;
		}

		if (argc == 6) {
			rc = neopixel_parse_index(argv[5], &idx);
			if (rc == -ERANGE) {
				shell_error(shell, "Index out of range (0-%u).",
					    (unsigned int)(NEOPIXEL_COUNT - 1));
				return -ERANGE;
			}
			if (rc < 0) {
				shell_error(shell, "Invalid index.");
				return -EINVAL;
			}
		}

		neopixel_state[idx].r = r;
		neopixel_state[idx].g = g;
		neopixel_state[idx].b = b;
		return neopixel_apply(shell);
	}

	shell_error(shell, "Unknown command.");
	return -EINVAL;
}
#else
static int cmd_neopixel(const struct shell *shell, size_t argc, char **argv)
{
	(void)argc;
	(void)argv;

	shell_error(shell, "Neopixel not configured in devicetree.");
	return -ENODEV;
}
#endif

SHELL_CMD_REGISTER(neopixel, NULL, "Neopixel control (set/fill/off/status).", cmd_neopixel);
