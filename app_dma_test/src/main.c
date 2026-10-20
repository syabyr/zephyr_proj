/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/device.h>
#include <zephyr/drivers/dma.h>
#include <string.h>

/*
 * uDMA memory-to-memory test: copy a 256-byte pattern through the
 * software request channel and verify the result plus the completion
 * callback.
 */

#define TRANSFER_SIZE 256U

static const struct device *const dma_dev = DEVICE_DT_GET_ONE(ti_tiva_c_udma);

static struct dma_config dma_cfg;
static struct dma_block_config dma_block;

static uint8_t src_buf[TRANSFER_SIZE] __aligned(4);
static uint8_t dst_buf[TRANSFER_SIZE] __aligned(4);

static volatile int dma_done;
static volatile int dma_status;

static void dma_callback(const struct device *dev, void *user_data,
			 uint32_t channel, int status)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(user_data);

	dma_status = status;
	dma_done = 1;
}

int main(void)
{
	int ret;
	uint32_t chan;

	printk("=== uDMA mem-to-mem test ===\n");

	if (!device_is_ready(dma_dev)) {
		printk("ERROR: DMA device not ready\n");
		return 0;
	}
	printk("DMA device: %s\n", dma_dev->name);

	/* Fill the source with a recognizable pattern */
	for (uint32_t i = 0; i < TRANSFER_SIZE; i++) {
		src_buf[i] = (uint8_t)(i ^ 0xA5);
	}
	memset(dst_buf, 0, sizeof(dst_buf));

	/* Request a software channel */
	chan = dma_request_channel(dma_dev, NULL);
	if (chan == UINT32_MAX) {
		printk("ERROR: no free DMA channel\n");
		return 0;
	}
	printk("got channel %u\n", chan);

	/* Configure the transfer */
	memset(&dma_cfg, 0, sizeof(dma_cfg));
	memset(&dma_block, 0, sizeof(dma_block));

	dma_block.source_address = (uint32_t)src_buf;
	dma_block.dest_address = (uint32_t)dst_buf;
	dma_block.block_size = TRANSFER_SIZE;
	dma_block.source_addr_adj = 0;  /* increment */
	dma_block.dest_addr_adj = 0;    /* increment */

	dma_cfg.channel_direction = MEMORY_TO_MEMORY;
	dma_cfg.source_data_size = 1U;
	dma_cfg.dest_data_size = 1U;
	dma_cfg.source_burst_length = 4U;
	dma_cfg.dest_burst_length = 4U;
	dma_cfg.complete_callback_en = 0;
	dma_cfg.dma_callback = dma_callback;
	dma_cfg.block_count = 1;
	dma_cfg.head_block = &dma_block;

	ret = dma_config(dma_dev, chan, &dma_cfg);
	if (ret != 0) {
		printk("dma_config failed: %d\n", ret);
		return 0;
	}

	dma_done = 0;
	ret = dma_start(dma_dev, chan);
	if (ret != 0) {
		printk("dma_start failed: %d\n", ret);
		return 0;
	}
	printk("transfer started, waiting...\n");

	/* AUTO mode completes in microseconds; poll with timeout */
	for (int i = 0; i < 100 && !dma_done; i++) {
		k_sleep(K_MSEC(10));
	}

	if (!dma_done) {
		printk("ERROR: completion callback never fired\n");
		return 0;
	}

	if (dma_status != 0) {
		printk("ERROR: callback reported status %d\n", dma_status);
		return 0;
	}

	/* Verify the data */
	if (memcmp(src_buf, dst_buf, TRANSFER_SIZE) == 0) {
		printk("*** PASS: %u bytes copied correctly, callback OK ***\n",
		       TRANSFER_SIZE);
	} else {
		printk("*** FAIL: data mismatch ***\n");
		uint32_t first_bad = TRANSFER_SIZE;

		for (uint32_t i = 0; i < TRANSFER_SIZE; i++) {
			if (src_buf[i] != dst_buf[i]) {
				first_bad = i;
				break;
			}
		}
		printk("first mismatch at %u: src=%02x dst=%02x\n",
		       first_bad, src_buf[first_bad], dst_buf[first_bad]);
	}

	dma_release_channel(dma_dev, chan);

	while (1) {
		k_sleep(K_SECONDS(5));
	}

	return 0;
}
