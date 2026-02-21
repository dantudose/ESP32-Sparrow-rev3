#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/shell/shell.h>

#define I2C_NODE DT_NODELABEL(i2c0)

static int cmd_i2c_scan(const struct shell *shell, size_t argc, char **argv)
{
	const struct device *i2c = DEVICE_DT_GET(I2C_NODE);
	uint8_t count = 0;
	uint8_t first = 0x04;
	uint8_t last = 0x77;

	(void)argc;
	(void)argv;

	if (!device_is_ready(i2c)) {
		shell_error(shell, "I2C device not ready.");
		return -ENODEV;
	}

	shell_print(shell, "     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f");
	for (uint8_t base = 0; base <= last; base += 16) {
		shell_fprintf_normal(shell, "%02x: ", base);
		for (uint8_t offset = 0; offset < 16; offset++) {
			uint8_t addr = base + offset;
			struct i2c_msg msg;
			uint8_t dummy;

			if (addr < first || addr > last) {
				shell_fprintf_normal(shell, "   ");
				continue;
			}

			msg.buf = &dummy;
			msg.len = 0U;
			msg.flags = I2C_MSG_WRITE | I2C_MSG_STOP;
			if (i2c_transfer(i2c, &msg, 1, addr) == 0) {
				shell_fprintf_normal(shell, "%02x ", addr);
				count++;
			} else {
				shell_fprintf_normal(shell, "-- ");
			}
		}
		shell_print(shell, "");
	}

	shell_print(shell, "%u devices found on %s", count, i2c->name);
	return 0;
}

SHELL_CMD_REGISTER(i2c_scan, NULL, "Scan the I2C bus (i2c0).", cmd_i2c_scan);
