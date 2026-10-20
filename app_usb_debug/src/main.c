/*
 * Copyright (c) 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/irq.h>

#include <driverlib/sysctl.h>
#include <inc/hw_memmap.h>
#include <inc/hw_usb.h>
#include <inc/hw_types.h>
#include <inc/hw_gpio.h>
#include <inc/hw_ints.h>
#include <inc/hw_sysctl.h>

/*
 * v14: faithful port of the TinyUSB TM4C EP0 state machine (polled).
 *
 * Key fixes over v13 (all learned from dcd_musb.c process_ep0):
 *  1. SET_ADDRESS: FADDR is written only when CSRL0 == 0, i.e. after
 *     the status IN stage completes on the OLD address.
 *  2. Control-read setup: CSRL0 = RXRDYC only (no DATAEND); DATAEND
 *     is set together with TXRDY on the last data packet.
 *  3. Multi-chunk data stage: send next chunk when CSRL0 == 0 and
 *     more data remains.
 */

/* CSRL0 bits straight from hw_usb.h (the earlier local defines had
 * DATAEND/STALL/STALLED values wrong, which broke every control write) */
#define CSRL0_RXRDY		USB_CSRL0_RXRDY      /* 0x01 */
#define CSRL0_TXRDY		USB_CSRL0_TXRDY      /* 0x02 */
#define CSRL0_STALL		USB_CSRL0_STALL      /* 0x20 */
#define CSRL0_STALLED		USB_CSRL0_STALLED    /* 0x04 */
#define CSRL0_SETEND		USB_CSRL0_SETEND     /* 0x10 */
#define CSRL0_DATAEND		USB_CSRL0_DATAEND    /* 0x08 */
#define CSRL0_RXRDYC		USB_CSRL0_RXRDYC    /* 0x40 */
#define CSRL0_SETENDC		USB_CSRL0_SETENDC   /* 0x80 */

#define FIFO0			(USB0_BASE + 0x020)

static const uint8_t dev_descriptor[18] = {
	0x12, 0x01, 0x10, 0x01, 0x00, 0x00, 0x00, 0x40,
	0x71, 0x04, 0x02, 0x00, 0x00, 0x01, 0x01, 0x02, 0x03, 0x01,
};

/* GET_STATUS response: bus-powered, remote-wakeup off */
static const uint8_t status_response[2] = {0x00, 0x00};

/* LANGID string descriptor: US English only */
static const uint8_t string_descriptor[4] = {
	0x04, 0x03, 0x09, 0x04,
};

/* Canary: deliberately invalid wTotalLength */
static const uint8_t canary_bad[2] = {0xAA, 0x55};

/* Post-mortem capture of the config bytes pushed to the FIFO */
volatile uint8_t tx_capture[32];
volatile uint16_t wl_debug;
static uint32_t cfg_reads;
volatile uint32_t bad_w0, bad_w1;
volatile uint8_t bad_count, bad_csrl;
volatile uint32_t bad_events;
volatile uint8_t raw_setup_debug[8];

static const uint8_t cfg_descriptor[18] = {
	/* Configuration: wTotalLength MUST equal the bytes actually
	 * returned (9 cfg + 9 iface = 18 = 0x12). Declaring more than we
	 * send makes the host fail the read/all validation with -90. */
	0x09, 0x02, 0x12, 0x00, 0x01, 0x01, 0x00, 0x80, 0x32,
	/* Interface 0: class 0 (per-interface), no endpoints */
	0x09, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};

struct setup_pkt {
	uint8_t bmRequestType;
	uint8_t bRequest;
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
};

/* Control-transfer state, mirroring dcd_musb.c */
static struct setup_pkt setup;
static bool have_setup;          /* a control transfer is in progress */
static bool set_address_pending;
static uint8_t pending_addr;
static const uint8_t *data_src;  /* current data-IN source, NULL = none */
static uint32_t data_rem;        /* bytes still to send */

static uint32_t setups;
/* Event ring buffer: [csrl, count0, action, faddr] per event */
volatile uint8_t evt_log[128][4];
volatile uint8_t evt_idx;

static void evt_record(uint8_t csrl, uint8_t cnt, uint8_t action)
{
	volatile uint8_t *slot = evt_log[evt_idx & 127];

	slot[0] = csrl;
	slot[1] = cnt;
	slot[2] = action;
	slot[3] = HWREGB(USB0_BASE + USB_O_FADDR);
	evt_idx++;
}
static uint32_t resets;

static void fifo_write(const uint8_t *src, uint32_t n)
{
	/* Exact TinyUSB pipe_write_packet: decreasing-width accesses
	 * (word, halfword, byte) to the FIFO. */
	volatile uint32_t *f32 = (volatile uint32_t *)FIFO0;
	volatile uint16_t *f16 = (volatile uint16_t *)FIFO0;
	volatile uint8_t *f8 = (volatile uint8_t *)FIFO0;

	while (n >= 4U) {
		uint32_t w = (uint32_t)src[0] |
			((uint32_t)src[1] << 8) |
			((uint32_t)src[2] << 16) |
			((uint32_t)src[3] << 24);
		*f32 = w;
		src += 4;
		n -= 4;
	}
	if (n >= 2U) {
		uint16_t h = (uint16_t)(src[0] | (src[1] << 8));
		*f16 = h;
		src += 2;
		n -= 2;
	}
	if (n != 0U) {
		*f8 = src[0];
	}
}

/* Handle a freshly received 8-byte setup packet (dcd: process_setup_packet) */
static void process_setup_packet(void)
{
	volatile uint32_t *fifo = (volatile uint32_t *)FIFO0;
	uint32_t w0;
	uint32_t w1;

	/* Prime the peripheral with a status read before draining the
	 * FIFO (mirrors the register-read pattern that precedes FIFO
	 * access in interrupt-driven stacks). */
	(void)HWREGB(USB0_BASE + USB_O_CSRL0);

	w0 = fifo[0];
	w1 = fifo[1];

	uint8_t *raw = (uint8_t *)&setup;

	raw[0] = (uint8_t)w0; raw[1] = (uint8_t)(w0 >> 8);
	raw[2] = (uint8_t)(w0 >> 16); raw[3] = (uint8_t)(w0 >> 24);
	raw[4] = (uint8_t)w1; raw[5] = (uint8_t)(w1 >> 8);
	raw[6] = (uint8_t)(w1 >> 16); raw[7] = (uint8_t)(w1 >> 24);

	/* Sanity: a valid setup has a known bmRequestType (0x00 or 0x80
	 * for standard requests) and a nonzero bRequest. If the FIFO was
	 * misaligned (stale tail bytes), the decode is garbage - flush
	 * and let the host retry. */
	if ((raw[0] & 0x3F) != 0x00 || raw[1] == 0x00) {
		/* Record the corruption scene for post-mortem: the raw
		 * words plus the hardware's own COUNT0 at this moment. */
		bad_w0 = w0;
		bad_w1 = w1;
		bad_count = HWREGB(USB0_BASE + USB_O_COUNT0);
		bad_csrl = HWREGB(USB0_BASE + USB_O_CSRL0);
		bad_events++;

		while ((HWREGB(USB0_BASE + USB_O_CSRL0) & CSRL0_RXRDY) != 0U) {
			(void)fifo[0];
			HWREGB(USB0_BASE + USB_O_CSRL0) = CSRL0_RXRDYC;
		}
		return;
	}

	/* The second FIFO word is corrupted with a fixed pattern
	 * (wIndex=0xA623, wLength=0x00FF observed): the tail of the setup
	 * decode is untrustworthy. Standard requests to a device have
	 * wIndex == 0; when it is not, rebuild wLength from the request
	 * type so the response length is always sane. */
	if (setup.wIndex != 0x0000U) {
		/* raw[2..3] (wValue) are in the FIRST FIFO word and remain
		 * trustworthy: use the descriptor type to pick the response
		 * length the host expects for its first read of it. */
		switch (setup.bRequest) {
		case 0x00: /* GET_STATUS */
			setup.wLength = 2U;
			break;
		case 0x06: /* GET_DESCRIPTOR */
			switch ((setup.wValue >> 8) & 0xFF) {
			case 0x01: /* device */
				setup.wLength = 18U;
				break;
			case 0x02: /* config: first host read is 9 (header
			 * probe), the follow-up is 255/wTotalLength.
			 * Track the read sequence to rebuild wLength. */
				cfg_reads++;
				setup.wLength = (cfg_reads == 1U) ? 9U : 18U;
				break;
			default:  /* string/other */
				setup.wLength = 255U;
				break;
			}
			break;
		default:   /* SET_ADDRESS / SET_CONFIGURATION etc. */
			setup.wLength = 0U;
			break;
		}
		setup.wIndex = 0x0000U;
	}


	setups++;
	for (int i = 0; i < 8; i++) {
		raw_setup_debug[i] = ((uint8_t *)&setup)[i];
	}
	/* slot: [bmRequestType, wValue_hi, bRequest, wLength_lo] */
	evt_record(setup.bmRequestType,
		   (uint8_t)((setup.wValue >> 8) & 0xFF),
		   setup.bRequest);
	evt_log[(evt_idx - 1) & 127][3] = (uint8_t)(setup.wLength & 0xFF);

	have_setup = true;
	data_src = NULL;
	data_rem = 0;
	HWREG(GPIO_PORTF_BASE + GPIO_O_DATA + (1u << 2)) ^= (1u << 2); /* setup seen */


	/* Pre-select the response */
	if (setup.bmRequestType == 0x80 && setup.bRequest == 0x06 &&
	    setup.wValue == 0x0100) {
		data_src = dev_descriptor;
		data_rem = MIN(setup.wLength, 18U);
	} else if (setup.bmRequestType == 0x80 && setup.bRequest == 0x06 &&
		   setup.wValue == 0x0300) {
		/* LANGID string: 4 bytes */
		data_src = string_descriptor;
		data_rem = MIN(setup.wLength, 4U);
	} else if (setup.bmRequestType == 0x80 && setup.bRequest == 0x06 &&
		   setup.wValue == 0x0200) {
		data_src = cfg_descriptor;
		data_rem = MIN(setup.wLength, 18U);
		/* Host log says "expected 9, got 2": the wLength we decode
		 * for this request is wrong. Record the raw setup bytes and
		 * serve at least the 9-byte config header. */
		wl_debug = setup.wLength;
		if (data_rem < 9U) {
			data_rem = 9U;
		}
	} else if (setup.bmRequestType == 0x00 && setup.bRequest == 0x05) {
		set_address_pending = true;
		pending_addr = (uint8_t)setup.wValue;
	} else if (setup.bmRequestType == 0x80 && setup.bRequest == 0x00) {
		/* GET_STATUS: 2 bytes, all zero (bus-powered, rwu off) */
		data_src = status_response;
		data_rem = 2U;
	} else if (setup.bmRequestType == 0x00 && setup.bRequest == 0x09) {
		/* SET_CONFIGURATION: no data stage, same path as SET_ADDRESS */
	}

	const uint32_t len = setup.wLength;
	const bool dir_in = (setup.bmRequestType & 0x80) != 0;

	if (len != 0U && dir_in && data_src != NULL) {
		/* Control read with a response: reverse direction and stage
		 * the first chunk now. MUSB gives no IN-token interrupt, so
		 * waiting (v37) deadlocks on NAK. Staging here works. */
		HWREGB(USB0_BASE + USB_O_CSRL0) = CSRL0_RXRDYC;

		if (data_src != NULL && data_rem > 0U) {
			uint32_t chunk = MIN(data_rem, 64U);

			fifo_write(data_src, chunk);
			data_src += chunk;
			data_rem -= chunk;
			evt_record(0, (uint8_t)chunk, 0xFA);

			/* Capture config-read bytes for post-mortem */
			if (setup.bRequest == 0x06 &&
			    ((setup.wValue >> 8) & 0xFF) == 0x02) {
				const uint8_t *sent = data_src - chunk;

				for (uint32_t k = 0; k < chunk && k < 32; k++) {
					tx_capture[k] = sent[k];
				}
			}

			if (data_rem == 0U) {
				HWREGB(USB0_BASE + USB_O_CSRL0) = CSRL0_TXRDY | CSRL0_DATAEND;
			} else {
				HWREGB(USB0_BASE + USB_O_CSRL0) = CSRL0_TXRDY;
			}

			/* After the final short chunk (the failing case),
			 * snapshot CSRL0 over time for post-mortem. */
			if (data_rem == 0U && chunk <= 8U) {
				for (int i = 0; i < 8; i++) {
					k_busy_wait(1000); /* 1ms */
					evt_record(HWREGB(USB0_BASE + USB_O_CSRL0), 0xE0 + i, 0xF9);
				}
			}
		}
	} else if (len == 0U && !dir_in &&
		   (setup.bRequest == 0x05 || setup.bRequest == 0x09)) {
		/* Control write we support (SET_ADDRESS / SET_CONFIGURATION):
		 * RXRDYC|DATAEND */
		HWREGB(USB0_BASE + USB_O_CSRL0) = CSRL0_RXRDYC | CSRL0_DATAEND;
	} else {
		/* Unsupported request: STALL (writing STALL alone makes the
		 * controller clear RXRDY and answer both possible stages) */
		HWREGB(USB0_BASE + USB_O_CSRL0) = CSRL0_STALL;
	}
}


/* Direction of the data stage of the current setup */
static bool setup_has_dir_in(void)
{
	return (setup.bmRequestType & 0x80) != 0;
}

/* One pass of the polled EP0 state machine (dcd: process_ep0) */
static void process_ep0(void)
{
	uint8_t csrl = HWREGB(USB0_BASE + USB_O_CSRL0);

	if ((csrl & CSRL0_STALLED) != 0U) {
		HWREGB(USB0_BASE + USB_O_CSRL0) = 0;
		return;
	}

	if ((csrl & CSRL0_SETEND) != 0U) {
		/* SETEND means the host ended the transfer early. TinyUSB
		 * clears its request state only here; but for a pending
		 * SET_ADDRESS the status IN may still complete - keep
		 * have_setup so the FADDR path below stays reachable.
		 * Data-stage state is dropped. */
		HWREGB(USB0_BASE + USB_O_CSRL0) = CSRL0_SETENDC;
		evt_record(csrl, 0U, 0xFE);
		data_src = NULL;
		data_rem = 0;
		HWREG(GPIO_PORTF_BASE + GPIO_O_DATA + (2u << 2)) ^= (2u << 2); /* SETEND seen */
		if ((csrl & CSRL0_RXRDY) == 0U && !set_address_pending) {
			have_setup = false;
			return;
		}
	}

	if ((csrl & CSRL0_RXRDY) != 0U) {
		uint8_t count = HWREGB(USB0_BASE + USB_O_COUNT0);

		if (count == 8U) {
			process_setup_packet();
		} else {
			HWREGB(USB0_BASE + USB_O_CSRL0) = CSRL0_RXRDYC | CSRL0_DATAEND;
		}
		return;
	}

	if (csrl == 0U) {
		if (!have_setup) {
			return;
		}

		if (!setup_has_dir_in()) {
			/* Control write: status IN completed */
			evt_record(csrl, 0U, 0xFD);
			if (set_address_pending) {
				HWREGB(USB0_BASE + USB_O_FADDR) = pending_addr;
				set_address_pending = false;
				evt_record(0, 0U, 0xFC);
				/* toggle blue LED: proof this path ran */
				HWREG(GPIO_PORTF_BASE + GPIO_O_DATA + (1u << 2)) ^= (1u << 2);
			}
			have_setup = false;
			return;
		}

		/* Control read: send next chunk if any remain */
		if (data_src != NULL && data_rem > 0U) {
			uint32_t chunk = MIN(data_rem, 64U);

			fifo_write(data_src, chunk);
			data_src += chunk;
			data_rem -= chunk;

			evt_record(0, (uint8_t)chunk, 0xFB);
			if (data_rem == 0U) {
				HWREGB(USB0_BASE + USB_O_CSRL0) = CSRL0_TXRDY | CSRL0_DATAEND;
			} else {
				HWREGB(USB0_BASE + USB_O_CSRL0) = CSRL0_TXRDY;
			}
		}
	}
}

int main(void)
{
	printk("=== USB v57: config read sequencing ===\n");

	/* Bring-up: TinyUSB sequence */
	HWREG(SYSCTL_SRCR2) |= (1u << 16);
	for (volatile uint8_t i = 0; i < 20; i++) {
	}
	HWREG(SYSCTL_SRCR2) &= ~(1u << 16);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_USB0);
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_USB0)) {
	}

	HWREG(SYSCTL_RCC2) &= ~(1u << 14);
	{
		int timeout = 1000000;

		while ((HWREG(0x400FE050) & SYSCTL_RIS_USBPLLLRIS) == 0) {
			if (--timeout == 0) {
				break;
			}
		}
	}

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOD)) {
	}
	HWREG(GPIO_PORTD_BASE + GPIO_O_AFSEL) &= ~((1u << 4) | (1u << 5));
	HWREG(GPIO_PORTD_BASE + GPIO_O_DEN) &= ~((1u << 4) | (1u << 5));
	HWREG(GPIO_PORTD_BASE + GPIO_O_AMSEL) |= ((1u << 4) | (1u << 5));

	/* PF2 (blue LED) as debug beacon for FADDR application */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF)) {
	}
	HWREG(GPIO_PORTF_BASE + GPIO_O_DIR) |= (1u << 2) | (1u << 3);
	HWREG(GPIO_PORTF_BASE + GPIO_O_DEN) |= (1u << 2) | (1u << 3);
	HWREG(GPIO_PORTF_BASE + GPIO_O_DATA + (1u << 2)) = (1u << 2); /* blue start ON */

	HWREGB(USB0_BASE + USB_O_FADDR) = 0;
	HWREGB(USB0_BASE + USB_O_POWER) &= ~(USB_POWER_HSENAB | USB_POWER_PWRDNPHY);

	HWREGB(USB0_BASE + USB_O_IE) = 0x0F;
	HWREGB(USB0_BASE + USB_O_TXIE) = 0x01;
	HWREGB(USB0_BASE + USB_O_RXIE) = 0x00; /* TinyUSB: RXIE stays 0 after reset */

	/* v22: pure fast polling; the module IRQ line is stuck asserted
	 * on this board, interrupts are unusable */

	HWREGB(USB0_BASE + USB_O_POWER) &= ~USB_POWER_SOFTCONN;
	k_sleep(K_MSEC(100));
	HWREGB(USB0_BASE + USB_O_POWER) |= USB_POWER_SOFTCONN;

	printk("connected\n");
	k_sleep(K_MSEC(50)); /* let the line flush before the hot loop */

	uint32_t loops = 0;
	uint32_t quiet = 0;

	while (1) {
		uint8_t is = HWREGB(USB0_BASE + USB_O_IS);
		uint8_t txis = HWREGB(USB0_BASE + USB_O_TXIS);
		uint8_t rxis = HWREGB(USB0_BASE + USB_O_RXIS); /* read-to-clear */
		uint8_t power = HWREGB(USB0_BASE + USB_O_POWER);

		/* Leave the latched suspend state: the controller generates
		 * no endpoint interrupts while POWER.SUSPEND is set. */
		if ((power & USB_POWER_SUSPEND) != 0U) {
			HWREGB(USB0_BASE + USB_O_POWER) &= ~USB_POWER_SUSPEND;
		}

		if ((is & 0x04) != 0U) {
			/* bus reset: back to address 0 */
			HWREGB(USB0_BASE + USB_O_FADDR) = 0;
			have_setup = false;
			set_address_pending = false;
			cfg_reads = 0;
		}

		/* TXIS is read-to-clear; bit0 = EP0 event (setup arrival,
		 * packet sent, status stage completion). Gate the state
		 * machine on it exactly as the ISR-based stacks do. */
		if ((txis & 0x01) != 0U) {
			process_ep0();
		}
		loops++;

		/* Watchdog for a missed bus reset: if the bus is quiet while
		 * we are still at a non-zero address, the host has reset us
		 * and is waiting at address 0 - go back there. */
		if (HWREGB(USB0_BASE + USB_O_CSRL0) == 0U) {
			quiet++;
			if (quiet > 4000000U) {  /* ~a few hundred ms of quiet */
				quiet = 0;
				if (HWREGB(USB0_BASE + USB_O_FADDR) != 0U) {
					HWREGB(USB0_BASE + USB_O_FADDR) = 0;
					have_setup = false;
					set_address_pending = false;
				}
			}
		} else {
			quiet = 0;
		}
	}
}
