/*
 * Copyright The Zephyr Project Contributors
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/sys/printk.h>

#define FLASH_NODE DT_NODELABEL(spi_flash)

static const uint8_t expected_id[] = {0x9d, 0x40, 0x12};

static void raw_read_id(void)
{
	struct spi_dt_spec spi = SPI_DT_SPEC_GET(FLASH_NODE, SPI_WORD_SET(8));
	uint8_t opcode = 0x9f; /* RDID */
	uint8_t id[4];
	struct spi_buf tx_buf = { .buf = &opcode, .len = 1 };
	struct spi_buf rx_buf = { .buf = id, .len = 4 };
	const struct spi_buf_set tx = { .buffers = &tx_buf, .count = 1 };
	const struct spi_buf_set rx = { .buffers = &rx_buf, .count = 1 };
	int rc;

	if (!spi_is_ready_dt(&spi)) {
		printk("raw RDID: FAIL, SPI bus not ready\n");
		return;
	}

	rc = spi_transceive_dt(&spi, &tx, &rx);
	if (rc != 0) {
		printk("raw RDID: FAIL, spi_transceive_dt = %d\n", rc);
		return;
	}

	/* id[0] is a dummy; the JEDEC ID is id[1..3]. */
	printk("raw RDID: %02x %02x %02x %02x\n", id[0], id[1], id[2], id[3]);
}

int main(void)
{
	const struct device *flash = DEVICE_DT_GET(FLASH_NODE);
	const struct device *spi = DEVICE_DT_GET(DT_BUS(FLASH_NODE));
	uint8_t id[3];
	int rc;

	printk("SPI flash JEDEC ID test\n");
	printk("usart1 (SPI) ready: %d\n", device_is_ready(spi));
	printk("spi_flash ready: %d\n", device_is_ready(flash));

	/* Bypass the flash driver and read the ID straight off the bus. */
	raw_read_id();

	if (!device_is_ready(flash)) {
		printk("FAIL: spi_flash driver did not initialize\n");
		return 0;
	}

	rc = flash_read_jedec_id(flash, id);
	if (rc != 0) {
		printk("FAIL: flash_read_jedec_id() = %d\n", rc);
		return 0;
	}

	printk("flash_read_jedec_id: %02x %02x %02x\n", id[0], id[1], id[2]);

	if (id[0] == expected_id[0] &&
	    id[1] == expected_id[1] &&
	    id[2] == expected_id[2]) {
		printk("PASS: matches IS25LQ020B (9d 40 12)\n");
	} else {
		printk("MISMATCH: expected 9d 40 12\n");
	}

	return 0;
}
