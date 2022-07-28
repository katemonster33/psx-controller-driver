// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Driver for Broadcom BCM2835 SPI Controllers
 *
 * Copyright (C) 2012 Chris Boot
 * Copyright (C) 2013 Stephen Warren
 * Copyright (C) 2015 Martin Sperl
 *
 * This driver is inspired by:
 * spi-ath79.c, Copyright (C) 2009-2011 Gabor Juhos <juhosg@openwrt.org>
 * spi-atmel.c, Copyright (C) 2006 Atmel Corporation
 */

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/gpio.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/machine.h> /* FIXME: using chip internals */
#include <linux/gpio/driver.h> /* FIXME: using chip internals */
#include <linux/of_irq.h>
#include <linux/spi/spi.h>

#define ACK_PIN				(25)

/* SPI register offsets */
#define BCM2835_SPI_CS			0x00
#define BCM2835_SPI_FIFO		0x04
#define BCM2835_SPI_CLK			0x08
#define BCM2835_SPI_DLEN		0x0c
#define BCM2835_SPI_LTOH		0x10
#define BCM2835_SPI_DC			0x14

/* Bitfields in CS */
#define BCM2835_SPI_CS_LEN_LONG		0x02000000
#define BCM2835_SPI_CS_DMA_LEN		0x01000000
#define BCM2835_SPI_CS_CSPOL2		0x00800000
#define BCM2835_SPI_CS_CSPOL1		0x00400000
#define BCM2835_SPI_CS_CSPOL0		0x00200000
#define BCM2835_SPI_CS_RXF		0x00100000
#define BCM2835_SPI_CS_RXR		0x00080000
#define BCM2835_SPI_CS_TXD		0x00040000
#define BCM2835_SPI_CS_RXD		0x00020000
#define BCM2835_SPI_CS_DONE		0x00010000
#define BCM2835_SPI_CS_LEN		0x00002000
#define BCM2835_SPI_CS_REN		0x00001000
#define BCM2835_SPI_CS_ADCS		0x00000800
#define BCM2835_SPI_CS_INTR		0x00000400
#define BCM2835_SPI_CS_INTD		0x00000200
#define BCM2835_SPI_CS_DMAEN		0x00000100
#define BCM2835_SPI_CS_TA		0x00000080
#define BCM2835_SPI_CS_CSPOL		0x00000040
#define BCM2835_SPI_CS_CLEAR_RX		0x00000020
#define BCM2835_SPI_CS_CLEAR_TX		0x00000010
#define BCM2835_SPI_CS_CPOL		0x00000008
#define BCM2835_SPI_CS_CPHA		0x00000004
#define BCM2835_SPI_CS_CS_10		0x00000002
#define BCM2835_SPI_CS_CS_01		0x00000001

#define BCM2835_SPI_FIFO_SIZE		64
#define BCM2835_SPI_FIFO_SIZE_3_4	48
#define BCM2835_SPI_DMA_MIN_LENGTH	96
#define BCM2835_SPI_MODE_BITS	(SPI_CPOL | SPI_CPHA | SPI_CS_HIGH \
				| SPI_NO_CS | SPI_3WIRE)

#define DRV_NAME	"spipsx-bcm2835"

/* define polling limits */
static unsigned int polling_limit_us = 30;
module_param(polling_limit_us, uint, 0664);
MODULE_PARM_DESC(polling_limit_us,
		 "time in us to run a transfer in polling mode\n");

/**
 * struct bcm2835_spi - BCM2835 SPI controller
 * @regs: base address of register map
 * @clk: core clock, divided to calculate serial clock
 * @clk_hz: core clock cached speed
 * @irq: interrupt, signals TX FIFO empty or RX FIFO ¾ full
 * @tfr: SPI transfer currently processed
 * @ctlr: SPI controller reverse lookup
 * @tx_buf: pointer whence next transmitted byte is read
 * @rx_buf: pointer where next received byte is written
 * @tx_len: remaining bytes to transmit
 * @rx_len: remaining bytes to receive
 * @tx_prologue: bytes transmitted without DMA if first TX sglist entry's
 *	length is not a multiple of 4 (to overcome hardware limitation)
 * @rx_prologue: bytes received without DMA if first RX sglist entry's
 *	length is not a multiple of 4 (to overcome hardware limitation)
 * @tx_spillover: whether @tx_prologue spills over to second TX sglist entry
 * @debugfs_dir: the debugfs directory - neede to remove debugfs when
 *      unloading the module
 * @count_transfer_polling: count of how often polling mode is used
 * @count_transfer_irq: count of how often interrupt mode is used
 * @count_transfer_irq_after_polling: count of how often we fall back to
 *      interrupt mode after starting in polling mode.
 *      These are counted as well in @count_transfer_polling and
 *      @count_transfer_irq
 * @count_transfer_dma: count how often dma mode is used
 * @slv: SPI slave currently selected
 *	(used by bcm2835_spi_dma_tx_done() to write @clear_rx_cs)
 * @tx_dma_active: whether a TX DMA descriptor is in progress
 * @rx_dma_active: whether a RX DMA descriptor is in progress
 *	(used by bcm2835_spi_dma_tx_done() to handle a race)
 * @fill_tx_desc: preallocated TX DMA descriptor used for RX-only transfers
 *	(cyclically copies from zero page to TX FIFO)
 * @fill_tx_addr: bus address of zero page
 */
struct bcm2835_spi {
	void __iomem *regs;
	struct clk *clk;
	unsigned long clk_hz;
	int irq;
	struct spi_transfer *tfr;
	struct spi_controller *ctlr;
	const u8 *tx_buf;
	u8 *rx_buf;
	int tx_len;
	int rx_len;
	int tx_prologue;
	int rx_prologue;
	unsigned int tx_spillover;

	struct dentry *debugfs_dir;
	u64 count_transfer_polling;
	u64 count_transfer_irq;
	u64 count_transfer_irq_after_polling;

	struct bcm2835_spidev *slv;
	int gpioIrq;
};

/**
 * struct bcm2835_spidev - BCM2835 SPI slave
 * @prepare_cs: precalculated CS register value for ->prepare_message()
 *	(uses slave-specific clock polarity and phase settings)
 * @clear_rx_desc: preallocated RX DMA descriptor used for TX-only transfers
 *	(cyclically clears RX FIFO by writing @clear_rx_cs to CS register)
 * @clear_rx_addr: bus address of @clear_rx_cs
 * @clear_rx_cs: precalculated CS register value to clear RX FIFO
 *	(uses slave-specific clock polarity and phase settings)
 */
struct bcm2835_spidev {
	u32 prepare_cs;
	struct dma_async_tx_descriptor *clear_rx_desc;
	dma_addr_t clear_rx_addr;
	u32 clear_rx_cs ____cacheline_aligned;
};

#if defined(CONFIG_DEBUG_FS)
static void bcm2835_debugfs_create(struct bcm2835_spi *bs,
				   const char *dname)
{
	char name[64];
	struct dentry *dir;

	/* get full name */
	snprintf(name, sizeof(name), "spi-bcm2835-%s", dname);

	/* the base directory */
	dir = debugfs_create_dir(name, NULL);
	bs->debugfs_dir = dir;

	/* the counters */
	debugfs_create_u64("count_transfer_polling", 0444, dir,
			   &bs->count_transfer_polling);
	debugfs_create_u64("count_transfer_irq", 0444, dir,
			   &bs->count_transfer_irq);
	debugfs_create_u64("count_transfer_irq_after_polling", 0444, dir,
			   &bs->count_transfer_irq_after_polling);
}

static void bcm2835_debugfs_remove(struct bcm2835_spi *bs)
{
	debugfs_remove_recursive(bs->debugfs_dir);
	bs->debugfs_dir = NULL;
}
#else
static void bcm2835_debugfs_create(struct bcm2835_spi *bs,
				   const char *dname)
{
}

static void bcm2835_debugfs_remove(struct bcm2835_spi *bs)
{
}
#endif /* CONFIG_DEBUG_FS */

static inline u32 bcm2835_rd(struct bcm2835_spi *bs, unsigned int reg)
{
	return readl(bs->regs + reg);
}

static inline void bcm2835_wr(struct bcm2835_spi *bs, unsigned int reg, u32 val)
{
	writel(val, bs->regs + reg);
}

static inline void bcm2835_rd_fifo(struct bcm2835_spi *bs)
{
	u8 byte;

	while ((bs->rx_len) &&
	       (bcm2835_rd(bs, BCM2835_SPI_CS) & BCM2835_SPI_CS_RXD)) {
		byte = bcm2835_rd(bs, BCM2835_SPI_FIFO);
		if (bs->rx_buf)
			*bs->rx_buf++ = byte;
		bs->rx_len--;
	}
}

static inline void bcm2835_wr_fifo(struct bcm2835_spi *bs)
{
	u8 byte;

	while ((bs->tx_len) &&
	       (bcm2835_rd(bs, BCM2835_SPI_CS) & BCM2835_SPI_CS_TXD)) {
		byte = bs->tx_buf ? *bs->tx_buf++ : 0;
		bcm2835_wr(bs, BCM2835_SPI_FIFO, byte);
		bs->tx_len--;
	}
}

/**
 * bcm2835_rd_fifo_count() - blindly read exactly @count bytes from RX FIFO
 * @bs: BCM2835 SPI controller
 * @count: bytes to read from RX FIFO
 *
 * The caller must ensure that @bs->rx_len is greater than or equal to @count,
 * that the RX FIFO contains at least @count bytes and that the DMA Enable flag
 * in the CS register is set (such that a read from the FIFO register receives
 * 32-bit instead of just 8-bit).  Moreover @bs->rx_buf must not be %NULL.
 */
static inline void bcm2835_rd_fifo_count(struct bcm2835_spi *bs, int count)
{
	u32 val;
	int len;

	bs->rx_len -= count;

	do {
		val = bcm2835_rd(bs, BCM2835_SPI_FIFO);
		len = min(count, 4);
		memcpy(bs->rx_buf, &val, len);
		bs->rx_buf += len;
		count -= 4;
	} while (count > 0);
}

/**
 * bcm2835_wr_fifo_count() - blindly write exactly @count bytes to TX FIFO
 * @bs: BCM2835 SPI controller
 * @count: bytes to write to TX FIFO
 *
 * The caller must ensure that @bs->tx_len is greater than or equal to @count,
 * that the TX FIFO can accommodate @count bytes and that the DMA Enable flag
 * in the CS register is set (such that a write to the FIFO register transmits
 * 32-bit instead of just 8-bit).
 */
static inline void bcm2835_wr_fifo_count(struct bcm2835_spi *bs, int count)
{
	u32 val;
	int len;

	bs->tx_len -= count;

	do {
		if (bs->tx_buf) {
			len = min(count, 4);
			memcpy(&val, bs->tx_buf, len);
			bs->tx_buf += len;
		} else {
			val = 0;
		}
		bcm2835_wr(bs, BCM2835_SPI_FIFO, val);
		count -= 4;
	} while (count > 0);
}

/**
 * bcm2835_wait_tx_fifo_empty() - busy-wait for TX FIFO to empty
 * @bs: BCM2835 SPI controller
 *
 * The caller must ensure that the RX FIFO can accommodate as many bytes
 * as have been written to the TX FIFO:  Transmission is halted once the
 * RX FIFO is full, causing this function to spin forever.
 */
static inline void bcm2835_wait_tx_fifo_empty(struct bcm2835_spi *bs)
{
	while (!(bcm2835_rd(bs, BCM2835_SPI_CS) & BCM2835_SPI_CS_DONE))
		cpu_relax();
}

/**
 * bcm2835_rd_fifo_blind() - blindly read up to @count bytes from RX FIFO
 * @bs: BCM2835 SPI controller
 * @count: bytes available for reading in RX FIFO
 */
static inline void bcm2835_rd_fifo_blind(struct bcm2835_spi *bs, int count)
{
	u8 val;

	count = min(count, bs->rx_len);
	bs->rx_len -= count;

	do {
		val = bcm2835_rd(bs, BCM2835_SPI_FIFO);
		if (bs->rx_buf)
			*bs->rx_buf++ = val;
	} while (--count);
}

/**
 * bcm2835_wr_fifo_blind() - blindly write up to @count bytes to TX FIFO
 * @bs: BCM2835 SPI controller
 * @count: bytes available for writing in TX FIFO
 */
static inline void bcm2835_wr_fifo_blind(struct bcm2835_spi *bs, int count)
{
	u8 val;

	count = min(count, bs->tx_len);
	bs->tx_len -= count;

	do {
		val = bs->tx_buf ? *bs->tx_buf++ : 0;
		bcm2835_wr(bs, BCM2835_SPI_FIFO, val);
	} while (--count);
}

static void bcm2835_spi_reset_hw(struct bcm2835_spi *bs)
{
	u32 cs = bcm2835_rd(bs, BCM2835_SPI_CS);

	/* Disable SPI interrupts and transfer */
	cs &= ~(BCM2835_SPI_CS_INTR |
		BCM2835_SPI_CS_INTD |
		BCM2835_SPI_CS_DMAEN |
		BCM2835_SPI_CS_TA);
	/*
	 * Transmission sometimes breaks unless the DONE bit is written at the
	 * end of every transfer.  The spec says it's a RO bit.  Either the
	 * spec is wrong and the bit is actually of type RW1C, or it's a
	 * hardware erratum.
	 */
	cs |= BCM2835_SPI_CS_DONE;
	/* and reset RX/TX FIFOS */
	cs |= BCM2835_SPI_CS_CLEAR_RX | BCM2835_SPI_CS_CLEAR_TX;

	/* and reset the SPI_HW */
	bcm2835_wr(bs, BCM2835_SPI_CS, cs);
	/* as well as DLEN */
	bcm2835_wr(bs, BCM2835_SPI_DLEN, 0);
}

static irqreturn_t pin_ack_irq_handler(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

static irqreturn_t bcm2835_spi_interrupt(int irq, void *dev_id)
{
	struct bcm2835_spi *bs = dev_id;
	u32 cs = bcm2835_rd(bs, BCM2835_SPI_CS);

	/*
	 * An interrupt is signaled either if DONE is set (TX FIFO empty)
	 * or if RXR is set (RX FIFO >= ¾ full).
	 */
	if (cs & BCM2835_SPI_CS_RXF)
		bcm2835_rd_fifo_blind(bs, BCM2835_SPI_FIFO_SIZE);
	else if (cs & BCM2835_SPI_CS_RXR)
		bcm2835_rd_fifo_blind(bs, BCM2835_SPI_FIFO_SIZE_3_4);

	if (bs->tx_len && cs & BCM2835_SPI_CS_DONE)
		bcm2835_wr_fifo_blind(bs, BCM2835_SPI_FIFO_SIZE);

	/* Read as many bytes as possible from FIFO */
	bcm2835_rd_fifo(bs);
	/* Write as many bytes as possible to FIFO */
	bcm2835_wr_fifo(bs);

	if (!bs->rx_len) {
		/* Transfer complete - reset SPI HW */
		bcm2835_spi_reset_hw(bs);
		/* wake up the framework */
		spi_finalize_current_transfer(bs->ctlr);
	}

	return IRQ_HANDLED;
}

static int bcm2835_spi_transfer_one_irq(struct spi_controller *ctlr,
					struct spi_device *spi,
					struct spi_transfer *tfr,
					u32 cs, bool fifo_empty)
{
	struct bcm2835_spi *bs = spi_controller_get_devdata(ctlr);

	/* update usage statistics */
	bs->count_transfer_irq++;

	/*
	 * Enable HW block, but with interrupts still disabled.
	 * Otherwise the empty TX FIFO would immediately trigger an interrupt.
	 */
	bcm2835_wr(bs, BCM2835_SPI_CS, cs | BCM2835_SPI_CS_TA);

	/* fill TX FIFO as much as possible */
	if (fifo_empty)
		bcm2835_wr_fifo_blind(bs, BCM2835_SPI_FIFO_SIZE);
	bcm2835_wr_fifo(bs);

	/* enable interrupts */
	cs |= BCM2835_SPI_CS_INTR | BCM2835_SPI_CS_INTD | BCM2835_SPI_CS_TA;
	bcm2835_wr(bs, BCM2835_SPI_CS, cs);

	/* signal that we need to wait for completion */
	return 1;
}

static int bcm2835_spi_transfer_one_poll(struct spi_controller *ctlr,
					 struct spi_device *spi,
					 struct spi_transfer *tfr,
					 u32 cs)
{
	struct bcm2835_spi *bs = spi_controller_get_devdata(ctlr);
	unsigned long timeout;

	/* update usage statistics */
	bs->count_transfer_polling++;

	/* enable HW block without interrupts */
	bcm2835_wr(bs, BCM2835_SPI_CS, cs | BCM2835_SPI_CS_TA);

	/* fill in the fifo before timeout calculations
	 * if we are interrupted here, then the data is
	 * getting transferred by the HW while we are interrupted
	 */
	bcm2835_wr_fifo_blind(bs, BCM2835_SPI_FIFO_SIZE);

	/* set the timeout to at least 2 jiffies */
	timeout = jiffies + 2 + HZ * polling_limit_us / 1000000;

	/* loop until finished the transfer */
	while (bs->rx_len) {
		/* fill in tx fifo with remaining data */
		bcm2835_wr_fifo(bs);

		/* read from fifo as much as possible */
		bcm2835_rd_fifo(bs);

		/* if there is still data pending to read
		 * then check the timeout
		 */
		if (bs->rx_len && time_after(jiffies, timeout)) {
			dev_dbg_ratelimited(&spi->dev,
					    "timeout period reached: jiffies: %lu remaining tx/rx: %d/%d - falling back to interrupt mode\n",
					    jiffies - timeout,
					    bs->tx_len, bs->rx_len);
			/* fall back to interrupt mode */

			/* update usage statistics */
			bs->count_transfer_irq_after_polling++;

			return bcm2835_spi_transfer_one_irq(ctlr, spi,
							    tfr, cs, false);
		}
	}

	/* Transfer complete - reset SPI HW */
	bcm2835_spi_reset_hw(bs);
	/* and return without waiting for completion */
	return 0;
}

static int bcm2835_spi_transfer_one(struct spi_controller *ctlr,
				    struct spi_device *spi,
				    struct spi_transfer *tfr)
{
	struct bcm2835_spi *bs = spi_controller_get_devdata(ctlr);
	struct bcm2835_spidev *slv = spi_get_ctldata(spi);
	unsigned long spi_hz, cdiv;
	unsigned long hz_per_byte, byte_limit;
	u32 cs = slv->prepare_cs;

	/* set clock */
	spi_hz = tfr->speed_hz;

	if (spi_hz >= bs->clk_hz / 2) {
		cdiv = 2; /* clk_hz/2 is the fastest we can go */
	} else if (spi_hz) {
		/* CDIV must be a multiple of two */
		cdiv = DIV_ROUND_UP(bs->clk_hz, spi_hz);
		cdiv += (cdiv % 2);

		if (cdiv >= 65536)
			cdiv = 0; /* 0 is the slowest we can go */
	} else {
		cdiv = 0; /* 0 is the slowest we can go */
	}
	tfr->effective_speed_hz = cdiv ? (bs->clk_hz / cdiv) : (bs->clk_hz / 65536);
	bcm2835_wr(bs, BCM2835_SPI_CLK, cdiv);

	/* handle all the 3-wire mode */
	if (spi->mode & SPI_3WIRE && tfr->rx_buf)
		cs |= BCM2835_SPI_CS_REN;

	/* set transmit buffers and length */
	bs->tx_buf = tfr->tx_buf;
	bs->rx_buf = tfr->rx_buf;
	bs->tx_len = tfr->len;
	bs->rx_len = tfr->len;

	/* Calculate the estimated time in us the transfer runs.  Note that
	 * there is 1 idle clocks cycles after each byte getting transferred
	 * so we have 9 cycles/byte.  This is used to find the number of Hz
	 * per byte per polling limit.  E.g., we can transfer 1 byte in 30 us
	 * per 300,000 Hz of bus clock.
	 */
	hz_per_byte = polling_limit_us ? (9 * 1000000) / polling_limit_us : 0;
	byte_limit = hz_per_byte ? tfr->effective_speed_hz / hz_per_byte : 1;

	/* run in polling mode for short transfers */
	if (tfr->len < byte_limit)
		return bcm2835_spi_transfer_one_poll(ctlr, spi, tfr, cs);

	/* run in interrupt-mode */
	return bcm2835_spi_transfer_one_irq(ctlr, spi, tfr, cs, true);
}

static int bcm2835_spi_prepare_message(struct spi_controller *ctlr,
				       struct spi_message *msg)
{
	struct spi_device *spi = msg->spi;
	struct bcm2835_spi *bs = spi_controller_get_devdata(ctlr);
	struct bcm2835_spidev *slv = spi_get_ctldata(spi);
	int ret;

	if (ctlr->can_dma) {
		/*
		 * DMA transfers are limited to 16 bit (0 to 65535 bytes) by
		 * the SPI HW due to DLEN. Split up transfers (32-bit FIFO
		 * aligned) if the limit is exceeded.
		 */
		ret = spi_split_transfers_maxsize(ctlr, msg, 65532,
						  GFP_KERNEL | GFP_DMA);
		if (ret)
			return ret;
	}

	/*
	 * Set up clock polarity before spi_transfer_one_message() asserts
	 * chip select to avoid a gratuitous clock signal edge.
	 */
	bcm2835_wr(bs, BCM2835_SPI_CS, slv->prepare_cs);

	return 0;
}

static void bcm2835_spi_handle_err(struct spi_controller *ctlr,
				   struct spi_message *msg)
{
	struct bcm2835_spi *bs = spi_controller_get_devdata(ctlr);

	/* and reset */
	bcm2835_spi_reset_hw(bs);
}

static int chip_match_name(struct gpio_chip *chip, void *data)
{
	return !strcmp(chip->label, data);
}

static void bcm2835_spi_cleanup(struct spi_device *spi)
{
	struct bcm2835_spidev *slv = spi_get_ctldata(spi);
	struct spi_controller *ctlr = spi->controller;

	if (slv->clear_rx_desc)
		dmaengine_desc_free(slv->clear_rx_desc);

	if (slv->clear_rx_addr)
		dma_unmap_single(ctlr->dma_rx->device->dev,
				 slv->clear_rx_addr,
				 sizeof(u32),
				 DMA_TO_DEVICE);

	kfree(slv);
}

static int bcm2835_spi_setup_dma(struct spi_controller *ctlr,
				 struct spi_device *spi,
				 struct bcm2835_spi *bs,
				 struct bcm2835_spidev *slv)
{
	int ret;

	if (!ctlr->dma_rx)
		return 0;

	slv->clear_rx_addr = dma_map_single(ctlr->dma_rx->device->dev,
					    &slv->clear_rx_cs,
					    sizeof(u32),
					    DMA_TO_DEVICE);
	if (dma_mapping_error(ctlr->dma_rx->device->dev, slv->clear_rx_addr)) {
		dev_err(&spi->dev, "cannot map clear_rx_cs\n");
		slv->clear_rx_addr = 0;
		return -ENOMEM;
	}

	slv->clear_rx_desc = dmaengine_prep_dma_cyclic(ctlr->dma_rx,
						       slv->clear_rx_addr,
						       sizeof(u32), 0,
						       DMA_MEM_TO_DEV, 0);
	if (!slv->clear_rx_desc) {
		dev_err(&spi->dev, "cannot prepare clear_rx_desc\n");
		return -ENOMEM;
	}

	ret = dmaengine_desc_set_reuse(slv->clear_rx_desc);
	if (ret) {
		dev_err(&spi->dev, "cannot reuse clear_rx_desc\n");
		return ret;
	}

	return 0;
}

static int bcm2835_spi_setup(struct spi_device *spi)
{
	struct spi_controller *ctlr = spi->controller;
	struct bcm2835_spi *bs = spi_controller_get_devdata(ctlr);
	struct bcm2835_spidev *slv = spi_get_ctldata(spi);
	struct gpio_chip *chip;
	int ret;
	u32 cs;

	if (!slv) {
		slv = kzalloc(ALIGN(sizeof(*slv), dma_get_cache_alignment()),
			      GFP_KERNEL);
		if (!slv)
			return -ENOMEM;

		spi_set_ctldata(spi, slv);

		ret = bcm2835_spi_setup_dma(ctlr, spi, bs, slv);
		if (ret)
			goto err_cleanup;
	}

	/*
	 * Precalculate SPI slave's CS register value for ->prepare_message():
	 * The driver always uses software-controlled GPIO chip select, hence
	 * set the hardware-controlled native chip select to an invalid value
	 * to prevent it from interfering.
	 */
	cs = BCM2835_SPI_CS_CS_10 | BCM2835_SPI_CS_CS_01;
	if (spi->mode & SPI_CPOL)
		cs |= BCM2835_SPI_CS_CPOL;
	if (spi->mode & SPI_CPHA)
		cs |= BCM2835_SPI_CS_CPHA;
	slv->prepare_cs = cs;

	/*
	 * Precalculate SPI slave's CS register value to clear RX FIFO
	 * in case of a TX-only DMA transfer.
	 */
	if (ctlr->dma_rx) {
		slv->clear_rx_cs = cs | BCM2835_SPI_CS_TA |
					BCM2835_SPI_CS_DMAEN |
					BCM2835_SPI_CS_CLEAR_RX;
		dma_sync_single_for_device(ctlr->dma_rx->device->dev,
					   slv->clear_rx_addr,
					   sizeof(u32),
					   DMA_TO_DEVICE);
	}

	/*
	 * sanity checking the native-chipselects
	 */
	if (spi->mode & SPI_NO_CS)
		return 0;
	/*
	 * The SPI core has successfully requested the CS GPIO line from the
	 * device tree, so we are done.
	 */
	if (spi->cs_gpiod)
		return 0;
	if (spi->chip_select > 1) {
		/* error in the case of native CS requested with CS > 1
		 * officially there is a CS2, but it is not documented
		 * which GPIO is connected with that...
		 */
		dev_err(&spi->dev,
			"setup: only two native chip-selects are supported\n");
		ret = -EINVAL;
		goto err_cleanup;
	}

	/*
	 * Translate native CS to GPIO
	 *
	 * FIXME: poking around in the gpiolib internals like this is
	 * not very good practice. Find a way to locate the real problem
	 * and fix it. Why is the GPIO descriptor in spi->cs_gpiod
	 * sometimes not assigned correctly? Erroneous device trees?
	 */

	/* get the gpio chip for the base */
	chip = gpiochip_find("pinctrl-bcm2835", chip_match_name);
	if (!chip)
		return 0;

	spi->cs_gpiod = gpiochip_request_own_desc(chip, 8 - spi->chip_select,
						  DRV_NAME,
						  GPIO_LOOKUP_FLAGS_DEFAULT,
						  GPIOD_OUT_LOW);
	if (IS_ERR(spi->cs_gpiod)) {
		ret = PTR_ERR(spi->cs_gpiod);
		goto err_cleanup;
	}

	/* and set up the "mode" and level */
	dev_info(&spi->dev, "setting up native-CS%i to use GPIO\n",
		 spi->chip_select);

	return 0;

err_cleanup:
	bcm2835_spi_cleanup(spi);
	return ret;
}

static int bcm2835_spi_probe(struct platform_device *pdev)
{
	struct spi_controller *ctlr;
	struct bcm2835_spi *bs;
	int err;

	ctlr = devm_spi_alloc_master(&pdev->dev, sizeof(*bs));
	if (!ctlr)
		return -ENOMEM;

	platform_set_drvdata(pdev, ctlr);

	ctlr->use_gpio_descriptors = true;
	ctlr->mode_bits = BCM2835_SPI_MODE_BITS;
	ctlr->bits_per_word_mask = SPI_BPW_MASK(8);
	ctlr->num_chipselect = 3;
	ctlr->setup = bcm2835_spi_setup;
	ctlr->cleanup = bcm2835_spi_cleanup;
	ctlr->transfer_one = bcm2835_spi_transfer_one;
	ctlr->handle_err = bcm2835_spi_handle_err;
	ctlr->prepare_message = bcm2835_spi_prepare_message;
	ctlr->dev.of_node = pdev->dev.of_node;

	bs = spi_controller_get_devdata(ctlr);
	bs->ctlr = ctlr;

	bs->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(bs->regs))
		return PTR_ERR(bs->regs);

	bs->clk = devm_clk_get(&pdev->dev, NULL);
	if (IS_ERR(bs->clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(bs->clk),
				     "could not get clk\n");

	ctlr->max_speed_hz = clk_get_rate(bs->clk) / 2;

	bs->irq = platform_get_irq(pdev, 0);
	if (bs->irq <= 0)
		return bs->irq ? bs->irq : -ENODEV;

	clk_prepare_enable(bs->clk);
	bs->clk_hz = clk_get_rate(bs->clk);

	/* initialise the hardware with the default polarities */
	bcm2835_wr(bs, BCM2835_SPI_CS,
		   BCM2835_SPI_CS_CLEAR_RX | BCM2835_SPI_CS_CLEAR_TX);

	err = gpio_request(ACK_PIN, "GPIO_25_IN");

	if (err) {
		dev_err(&pdev->dev, "GPIO %d could not be requested\n", ACK_PIN);
		goto out_clk_disable;
	}

	gpio_direction_input(ACK_PIN);

	bs->gpioIrq = gpio_to_irq(ACK_PIN);

	err = request_irq(bs->gpioIrq, (void*)pin_ack_irq_handler, IRQF_TRIGGER_RISING, dev_name(&pdev->dev), NULL);
	if(err) {
		dev_err(&pdev->dev, "Could not request IRQ %d\n", bs->gpioIrq);
		goto pin_ack_release;
	}

	err = devm_request_irq(&pdev->dev, bs->irq, bcm2835_spi_interrupt, 0,
			       dev_name(&pdev->dev), bs);
	if (err) {
		dev_err(&pdev->dev, "could not request IRQ: %d\n", err);
		goto pin_ack_release;
	}

	err = spi_register_controller(ctlr);
	if (err) {
		dev_err(&pdev->dev, "could not register SPI controller: %d\n",
			err);
		goto pin_ack_release;
	}

	bcm2835_debugfs_create(bs, dev_name(&pdev->dev));

	return 0;

pin_ack_release:
	gpio_free(ACK_PIN);
out_clk_disable:
	clk_disable_unprepare(bs->clk);
	return err;
}

static int bcm2835_spi_remove(struct platform_device *pdev)
{
	struct spi_controller *ctlr = platform_get_drvdata(pdev);
	struct bcm2835_spi *bs = spi_controller_get_devdata(ctlr);

	bcm2835_debugfs_remove(bs);

	spi_unregister_controller(ctlr);

	/* Clear FIFOs, and disable the HW block */
	bcm2835_wr(bs, BCM2835_SPI_CS,
		   BCM2835_SPI_CS_CLEAR_RX | BCM2835_SPI_CS_CLEAR_TX);

        gpio_free(ACK_PIN);

	clk_disable_unprepare(bs->clk);

	return 0;
}

static void bcm2835_spi_shutdown(struct platform_device *pdev)
{
	int ret;

	ret = bcm2835_spi_remove(pdev);
	if (ret)
		dev_err(&pdev->dev, "failed to shutdown\n");
}

static const struct of_device_id bcm2835_spi_match[] = {
	{ .compatible = "brcm,bcm2835-spi", },
	{}
};
MODULE_DEVICE_TABLE(of, bcm2835_spi_match);

static struct platform_driver bcm2835_spi_driver = {
	.driver		= {
		.name		= DRV_NAME,
		.of_match_table	= bcm2835_spi_match,
	},
	.probe		= bcm2835_spi_probe,
	.remove		= bcm2835_spi_remove,
	.shutdown	= bcm2835_spi_shutdown,
};
module_platform_driver(bcm2835_spi_driver);

MODULE_DESCRIPTION("SPI controller driver for Broadcom BCM2835");
MODULE_AUTHOR("Chris Boot <bootc@bootc.net>");
MODULE_LICENSE("GPL");
