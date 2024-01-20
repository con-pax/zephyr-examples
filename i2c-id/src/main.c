/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>

#if DT_NODE_HAS_STATUS(DT_ALIAS(i2c1), okay)
#define I2C_DEV_NODE_CONTROLLER DT_ALIAS(i2c1)
#endif

#define BUFFER_SIZE         32u
#define REFRESH_REG         0x00
#define PID_REG             0xFD
#define MID_REG             0xFE
#define REV_REG             0xFF
#define PID                 0x5B
#define MID                 0x5D
#define RID                 0x03

int read_pac1934_ids(const struct device *dev_i2c, uint16_t i2c_addr);

int main(void)
{
	const struct device *const i2c_dev_controller = DEVICE_DT_GET(I2C_DEV_NODE_CONTROLLER);

	uint8_t product_id_buf[3];
	uint8_t read_product_id_buf[3];
	uint8_t compare_buf[3];

	compare_buf[0] = PID;
	compare_buf[1] = MID;
	compare_buf[2] = RID;

	uint32_t i2c_cfg = I2C_SPEED_SET(I2C_SPEED_FAST) | I2C_MODE_CONTROLLER;

	if (!device_is_ready(i2c_dev_controller)) {
		printk("Failed! %s\n", "hello");
	}

	if (i2c_configure(i2c_dev_controller, i2c_cfg)) {
		printk("I2C config failed\n");
	}

	product_id_buf[0] = PID_REG;
	product_id_buf[1] = MID_REG;
	product_id_buf[2] = REV_REG;

	printk("\nStarting write_read test:\n");

	if (i2c_write_read(i2c_dev_controller, 0x10, product_id_buf, 0x01, read_product_id_buf,
			   0x03)) {
		printk("failed to do write read");
	}

	read_pac1934_ids(i2c_dev_controller, 0x10);

	return 0;
}

int read_pac1934_ids(const struct device *dev_i2c, uint16_t i2c_addr)
{

	uint8_t product_id_buf[3];
	uint8_t read_product_id_buf[3];
	uint8_t compare_reg[3];

	product_id_buf[0] = PID_REG;
	product_id_buf[1] = MID_REG;
	product_id_buf[2] = REV_REG;

	compare_reg[0] = PID;
	compare_reg[1] = MID;
	compare_reg[2] = RID;

	printk("\nRead pac1934 ID Reg's\n");

	if (i2c_write_read(dev_i2c, i2c_addr, product_id_buf, 0x01, read_product_id_buf, 0x03)) {
		printk("\nfailed to do write read\n");
	} else {
		for (int cnt = 0; cnt < 20000; cnt++)
			;
	}

	for (int i = 0; i < 3; i++) {
		if (read_product_id_buf[i] != compare_reg[i]) {
			printk("\nfailed to read correct value\n");
		} else {
			printk("\nread_product_id_buf [%d] value %x\n", i, read_product_id_buf[i]);
		}
	}
	return 0;
}
