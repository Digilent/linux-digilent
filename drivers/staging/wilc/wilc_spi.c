// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2012 - 2018 Microchip Technology Inc., and its subsidiaries.
 * All rights reserved.
 */

#include <linux/spi/spi.h>
#include <linux/module.h>

#include "wilc_wfi_netdevice.h"
#include "wilc_wfi_cfgoperations.h"

struct wilc_spi {
	int crc_off;
	int nint;
	bool is_init;
};

static const struct wilc_hif_func wilc_hif_spi;

static int wilc_spi_rx(struct wilc *wilc, u8 *rb, u32 rlen);
static int wilc_spi_reset(struct wilc *wilc);

/********************************************
 *
 *      Crc7
 *
 ********************************************/

static const u8 crc7_syndrome_table[256] = {
	0x00, 0x09, 0x12, 0x1b, 0x24, 0x2d, 0x36, 0x3f,
	0x48, 0x41, 0x5a, 0x53, 0x6c, 0x65, 0x7e, 0x77,
	0x19, 0x10, 0x0b, 0x02, 0x3d, 0x34, 0x2f, 0x26,
	0x51, 0x58, 0x43, 0x4a, 0x75, 0x7c, 0x67, 0x6e,
	0x32, 0x3b, 0x20, 0x29, 0x16, 0x1f, 0x04, 0x0d,
	0x7a, 0x73, 0x68, 0x61, 0x5e, 0x57, 0x4c, 0x45,
	0x2b, 0x22, 0x39, 0x30, 0x0f, 0x06, 0x1d, 0x14,
	0x63, 0x6a, 0x71, 0x78, 0x47, 0x4e, 0x55, 0x5c,
	0x64, 0x6d, 0x76, 0x7f, 0x40, 0x49, 0x52, 0x5b,
	0x2c, 0x25, 0x3e, 0x37, 0x08, 0x01, 0x1a, 0x13,
	0x7d, 0x74, 0x6f, 0x66, 0x59, 0x50, 0x4b, 0x42,
	0x35, 0x3c, 0x27, 0x2e, 0x11, 0x18, 0x03, 0x0a,
	0x56, 0x5f, 0x44, 0x4d, 0x72, 0x7b, 0x60, 0x69,
	0x1e, 0x17, 0x0c, 0x05, 0x3a, 0x33, 0x28, 0x21,
	0x4f, 0x46, 0x5d, 0x54, 0x6b, 0x62, 0x79, 0x70,
	0x07, 0x0e, 0x15, 0x1c, 0x23, 0x2a, 0x31, 0x38,
	0x41, 0x48, 0x53, 0x5a, 0x65, 0x6c, 0x77, 0x7e,
	0x09, 0x00, 0x1b, 0x12, 0x2d, 0x24, 0x3f, 0x36,
	0x58, 0x51, 0x4a, 0x43, 0x7c, 0x75, 0x6e, 0x67,
	0x10, 0x19, 0x02, 0x0b, 0x34, 0x3d, 0x26, 0x2f,
	0x73, 0x7a, 0x61, 0x68, 0x57, 0x5e, 0x45, 0x4c,
	0x3b, 0x32, 0x29, 0x20, 0x1f, 0x16, 0x0d, 0x04,
	0x6a, 0x63, 0x78, 0x71, 0x4e, 0x47, 0x5c, 0x55,
	0x22, 0x2b, 0x30, 0x39, 0x06, 0x0f, 0x14, 0x1d,
	0x25, 0x2c, 0x37, 0x3e, 0x01, 0x08, 0x13, 0x1a,
	0x6d, 0x64, 0x7f, 0x76, 0x49, 0x40, 0x5b, 0x52,
	0x3c, 0x35, 0x2e, 0x27, 0x18, 0x11, 0x0a, 0x03,
	0x74, 0x7d, 0x66, 0x6f, 0x50, 0x59, 0x42, 0x4b,
	0x17, 0x1e, 0x05, 0x0c, 0x33, 0x3a, 0x21, 0x28,
	0x5f, 0x56, 0x4d, 0x44, 0x7b, 0x72, 0x69, 0x60,
	0x0e, 0x07, 0x1c, 0x15, 0x2a, 0x23, 0x38, 0x31,
	0x46, 0x4f, 0x54, 0x5d, 0x62, 0x6b, 0x70, 0x79
};

static u8 crc7_byte(u8 crc, u8 data)
{
	return crc7_syndrome_table[(crc << 1) ^ data];
}

static u8 crc7(u8 crc, const u8 *buffer, u32 len)
{
	while (len--)
		crc = crc7_byte(crc, *buffer++);
	return crc;
}

/********************************************
 *
 *      Spi protocol Function
 *
 ********************************************/

#define CMD_DMA_WRITE				0xc1
#define CMD_DMA_READ				0xc2
#define CMD_INTERNAL_WRITE			0xc3
#define CMD_INTERNAL_READ			0xc4
#define CMD_TERMINATE				0xc5
#define CMD_REPEAT				0xc6
#define CMD_DMA_EXT_WRITE			0xc7
#define CMD_DMA_EXT_READ			0xc8
#define CMD_SINGLE_WRITE			0xc9
#define CMD_SINGLE_READ				0xca
#define CMD_RESET				0xcf

#define N_OK					1
#define N_FAIL					0
#define N_RESET					-1
#define N_RETRY					-2

#define SPI_RESP_RETRY_COUNT			(10)
#define SPI_RETRY_COUNT				(10)
#define DATA_PKT_SZ_256				256
#define DATA_PKT_SZ_512				512
#define DATA_PKT_SZ_1K				1024
#define DATA_PKT_SZ_2K				(2 * 1024)
#define DATA_PKT_SZ_4K				(4 * 1024)
#define DATA_PKT_SZ_8K				(8 * 1024)
#define DATA_PKT_SZ				DATA_PKT_SZ_8K

#define USE_SPI_DMA				0

static int wilc_bus_probe(struct spi_device *spi)
{
	int ret;
	static bool init_power;
	struct wilc *wilc;
	struct device *dev = &spi->dev;
	struct wilc_spi *spi_priv;

	dev_info(&spi->dev, "spiModalias: %s, spiMax-Speed: %d\n",
			spi->modalias, spi->max_speed_hz);

	spi_priv = kzalloc(sizeof(*spi_priv), GFP_KERNEL);
	if (!spi_priv)
		return -ENOMEM;

	ret = wilc_cfg80211_init(&wilc, dev, WILC_HIF_SPI, &wilc_hif_spi);
	if (ret) {
		kfree(spi_priv);
		return ret;
	}

	spi_set_drvdata(spi, wilc);
	wilc->dev = &spi->dev;
	wilc->bus_data = spi_priv;
	wilc->dt_dev = &spi->dev;


	if (!init_power) {
		wilc_wlan_power_on_sequence(wilc);
		init_power = 1;
	}

	wilc_bt_init(wilc);

	dev_info(dev, "WILC SPI probe success\n");
	return 0;
}

static int wilc_bus_remove(struct spi_device *spi)
{
	struct wilc *wilc = spi_get_drvdata(spi);

	wilc_netdev_cleanup(wilc);
	wilc_bt_deinit();
	return 0;
}

static int wilc_spi_suspend(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct wilc *wilc = spi_get_drvdata(spi);

	dev_info(&spi->dev, "\n\n << SUSPEND >>\n\n");
	mutex_lock(&wilc->hif_cs);
	chip_wakeup(wilc, 0);

	if (mutex_is_locked(&wilc->hif_cs))
		mutex_unlock(&wilc->hif_cs);

	/*notify the chip that host will sleep*/
	host_sleep_notify(wilc, 0);
	chip_allow_sleep(wilc, 0);
	mutex_lock(&wilc->hif_cs);

	return 0;
}

static int wilc_spi_resume(struct device *dev)
{
	struct spi_device *spi = to_spi_device(dev);
	struct wilc *wilc = spi_get_drvdata(spi);

	dev_info(&spi->dev, "\n\n  <<RESUME>>\n\n");

	/*wake the chip to compelete the re-intialization*/
	chip_wakeup(wilc, 0);

	if (mutex_is_locked(&wilc->hif_cs))
		mutex_unlock(&wilc->hif_cs);

	host_wakeup_notify(wilc, 0);

	mutex_lock(&wilc->hif_cs);

	chip_allow_sleep(wilc, 0);

	if (mutex_is_locked(&wilc->hif_cs))
		mutex_unlock(&wilc->hif_cs);

	return 0;
}

static const struct of_device_id wilc_of_match[] = {
	{ .compatible = "microchip,wilc1000", },
	{ .compatible = "microchip,wilc3000", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, wilc_of_match);
static const struct dev_pm_ops wilc_spi_pm_ops = {
	.suspend = wilc_spi_suspend,
	.resume = wilc_spi_resume,
};

static struct spi_driver wilc_spi_driver = {
	.driver = {
		.name = MODALIAS,
		.of_match_table = wilc_of_match,
		.pm = &wilc_spi_pm_ops,
	},
	.probe =  wilc_bus_probe,
	.remove = wilc_bus_remove,
};
module_spi_driver(wilc_spi_driver);
MODULE_LICENSE("GPL");

static int spi_data_rsp(struct wilc *wilc, u8 cmd)
{
	struct spi_device *spi = to_spi_device(wilc->dev);
	struct wilc_spi *spi_priv = wilc->bus_data;
	u8 len;
	u8 rsp[3];
	int result = N_OK;

	if (!spi_priv->crc_off)
		len = 2;
	else
		len = 3;

	if (wilc_spi_rx(wilc, &rsp[0], len)) {
		dev_err(&spi->dev, "Failed bus error...\n");
		result = N_FAIL;
		goto fail;
	}

	if ((rsp[len-1] != 0) || (rsp[len-2] != 0xC3)) {
		dev_err(&spi->dev, "Failed data response read, %x %x %x\n",
			rsp[0], rsp[1], rsp[2]);
		result = N_FAIL;
		goto fail;
	}

fail:
	return result;
}

static int wilc_spi_tx(struct wilc *wilc, u8 *b, u32 len)
{
	struct spi_device *spi = to_spi_device(wilc->dev);
	int ret;
	struct spi_message msg;

	if (len > 0 && b) {
		struct spi_transfer tr = {
			.tx_buf = b,
			.len = len,
			.delay_usecs = 0,
		};
		char *r_buffer = kzalloc(len, GFP_KERNEL);

		if (!r_buffer)
			return -ENOMEM;

		tr.rx_buf = r_buffer;
		dev_dbg(&spi->dev, "Request writing %d bytes\n", len);

		memset(&msg, 0, sizeof(msg));
		spi_message_init(&msg);
		msg.spi = spi;
		msg.is_dma_mapped = USE_SPI_DMA;
		spi_message_add_tail(&tr, &msg);

		ret = spi_sync(spi, &msg);
		if (ret < 0)
			dev_err(&spi->dev, "SPI transaction failed\n");

		kfree(r_buffer);
	} else {
		dev_err(&spi->dev,
			"can't write data with the following length: %d\n",
			len);
		ret = -EINVAL;
	}

	return ret;
}

static int wilc_spi_rx(struct wilc *wilc, u8 *rb, u32 rlen)
{
	struct spi_device *spi = to_spi_device(wilc->dev);
	int ret;

	if (rlen > 0) {
		struct spi_message msg;
		struct spi_transfer tr = {
			.rx_buf = rb,
			.len = rlen,
			.delay_usecs = 0,

		};
		char *t_buffer = kzalloc(rlen, GFP_KERNEL);

		if (!t_buffer)
			return -ENOMEM;

		tr.tx_buf = t_buffer;

		memset(&msg, 0, sizeof(msg));
		spi_message_init(&msg);
		msg.spi = spi;
		msg.is_dma_mapped = USE_SPI_DMA;
		spi_message_add_tail(&tr, &msg);

		ret = spi_sync(spi, &msg);
		if (ret < 0)
			dev_err(&spi->dev, "SPI transaction failed\n");
		kfree(t_buffer);
	} else {
		dev_err(&spi->dev,
			"can't read data with the following length: %u\n",
			rlen);
		ret = -EINVAL;
	}

	return ret;
}

static int wilc_spi_tx_rx(struct wilc *wilc, u8 *wb, u8 *rb, u32 rlen)
{
	struct spi_device *spi = to_spi_device(wilc->dev);
	int ret;

	if (rlen > 0) {
		struct spi_message msg;
		struct spi_transfer tr = {
			.rx_buf = rb,
			.tx_buf = wb,
			.len = rlen,
			.bits_per_word = 8,
			.delay_usecs = 0,

		};

		memset(&msg, 0, sizeof(msg));
		spi_message_init(&msg);
		msg.spi = spi;
		msg.is_dma_mapped = USE_SPI_DMA;

		spi_message_add_tail(&tr, &msg);
		ret = spi_sync(spi, &msg);
		if (ret < 0)
			dev_err(&spi->dev, "SPI transaction failed\n");
	} else {
		dev_err(&spi->dev,
			"can't read data with the following length: %u\n",
			rlen);
		ret = -EINVAL;
	}

	return ret;
}

static int spi_cmd_complete(struct wilc *wilc, u8 cmd, u32 adr, u8 *b, u32 sz,
			    u8 clockless)
{
	struct spi_device *spi = to_spi_device(wilc->dev);
	struct wilc_spi *spi_priv = wilc->bus_data;
	u8 wb[32], rb[32];
	u8 wix, rix;
	u32 len2;
	u8 rsp;
	int len = 0;
	int result = N_OK;
	int retry;
	u8 crc[2];

	wb[0] = cmd;
	switch (cmd) {
	case CMD_SINGLE_READ: /* single word (4 bytes) read */
		wb[1] = (u8)(adr >> 16);
		wb[2] = (u8)(adr >> 8);
		wb[3] = (u8)adr;
		len = 5;
		break;

	case CMD_INTERNAL_READ: /* internal register read */
		wb[1] = (u8)(adr >> 8);
		if (clockless == 1)
			wb[1] |= BIT(7);
		wb[2] = (u8)adr;
		wb[3] = 0x00;
		len = 5;
		break;

	case CMD_TERMINATE:
		wb[1] = 0x00;
		wb[2] = 0x00;
		wb[3] = 0x00;
		len = 5;
		break;

	case CMD_REPEAT:
		wb[1] = 0x00;
		wb[2] = 0x00;
		wb[3] = 0x00;
		len = 5;
		break;

	case CMD_RESET:
		wb[1] = 0xff;
		wb[2] = 0xff;
		wb[3] = 0xff;
		len = 5;
		break;

	case CMD_DMA_WRITE: /* dma write */
	case CMD_DMA_READ:  /* dma read */
		wb[1] = (u8)(adr >> 16);
		wb[2] = (u8)(adr >> 8);
		wb[3] = (u8)adr;
		wb[4] = (u8)(sz >> 8);
		wb[5] = (u8)(sz);
		len = 7;
		break;

	case CMD_DMA_EXT_WRITE: /* dma extended write */
	case CMD_DMA_EXT_READ:  /* dma extended read */
		wb[1] = (u8)(adr >> 16);
		wb[2] = (u8)(adr >> 8);
		wb[3] = (u8)adr;
		wb[4] = (u8)(sz >> 16);
		wb[5] = (u8)(sz >> 8);
		wb[6] = (u8)(sz);
		len = 8;
		break;

	case CMD_INTERNAL_WRITE: /* internal register write */
		wb[1] = (u8)(adr >> 8);
		if (clockless == 1)
			wb[1] |= BIT(7);
		wb[2] = (u8)(adr);
		wb[3] = b[3];
		wb[4] = b[2];
		wb[5] = b[1];
		wb[6] = b[0];
		len = 8;
		break;

	case CMD_SINGLE_WRITE: /* single word write */
		wb[1] = (u8)(adr >> 16);
		wb[2] = (u8)(adr >> 8);
		wb[3] = (u8)(adr);
		wb[4] = b[3];
		wb[5] = b[2];
		wb[6] = b[1];
		wb[7] = b[0];
		len = 9;
		break;

	default:
		result = N_FAIL;
		break;
	}

	if (result != N_OK)
		return result;

	if (!spi_priv->crc_off)
		wb[len - 1] = (crc7(0x7f, (const u8 *)&wb[0], len - 1)) << 1;
	else
		len -= 1;

#define NUM_SKIP_BYTES (1)
#define NUM_RSP_BYTES (2)
#define NUM_DATA_HDR_BYTES (1)
#define NUM_DATA_BYTES (4)
#define NUM_CRC_BYTES (2)
#define NUM_DUMMY_BYTES (3)
	if (cmd == CMD_RESET ||
	    cmd == CMD_TERMINATE ||
	    cmd == CMD_REPEAT) {
		len2 = len + (NUM_SKIP_BYTES + NUM_RSP_BYTES + NUM_DUMMY_BYTES);
	} else if (cmd == CMD_INTERNAL_READ || cmd == CMD_SINGLE_READ) {
		int tmp = NUM_RSP_BYTES + NUM_DATA_HDR_BYTES + NUM_DATA_BYTES
			+ NUM_DUMMY_BYTES;
		if (!spi_priv->crc_off)
			len2 = len + tmp + NUM_CRC_BYTES;
		else
			len2 = len + tmp;
	} else {
		len2 = len + (NUM_RSP_BYTES + NUM_DUMMY_BYTES);
	}
#undef NUM_DUMMY_BYTES

	if (len2 > ARRAY_SIZE(wb)) {
		dev_err(&spi->dev, "spi buffer size too small (%d) (%zu)\n",
			len2, ARRAY_SIZE(wb));
		return N_FAIL;
	}
	/* zero spi write buffers. */
	for (wix = len; wix < len2; wix++)
		wb[wix] = 0;
	rix = len;

	if (wilc_spi_tx_rx(wilc, wb, rb, len2)) {
		dev_err(&spi->dev, "Failed cmd write, bus error...\n");
		return N_FAIL;
	}

	/*
	 * Command/Control response
	 */
	if (cmd == CMD_RESET || cmd == CMD_TERMINATE || cmd == CMD_REPEAT)
		rix++; /* skip 1 byte */

	rsp = rb[rix++];

	if (rsp != cmd) {
		dev_err(&spi->dev,
			"Failed cmd response, cmd (%02x), resp (%02x)\n",
			cmd, rsp);
		return N_FAIL;
	}

	/*
	 * State response
	 */
	rsp = rb[rix++];
	if (rsp != 0x00) {
		dev_err(&spi->dev, "Failed cmd state response state (%02x)\n",
			rsp);
		return N_FAIL;
	}

	if (cmd == CMD_INTERNAL_READ || cmd == CMD_SINGLE_READ ||
	    cmd == CMD_DMA_READ || cmd == CMD_DMA_EXT_READ) {
		/*
		 * Data Respnose header
		 */
		retry = SPI_RESP_RETRY_COUNT;
		do {
			/*
			 * ensure there is room in buffer later
			 * to read data and crc
			 */
			if (rix < len2) {
				rsp = rb[rix++];
			} else {
				retry = 0;
				break;
			}
			if (((rsp >> 4) & 0xf) == 0xf)
				break;
		} while (retry--);

		if (retry <= 0) {
			dev_err(&spi->dev,
				"Error, data read response (%02x)\n", rsp);
			return N_RESET;
		}
	}

	if (cmd == CMD_INTERNAL_READ || cmd == CMD_SINGLE_READ) {
		/*
		 * Read bytes
		 */
		if ((rix + 3) < len2) {
			b[0] = rb[rix++];
			b[1] = rb[rix++];
			b[2] = rb[rix++];
			b[3] = rb[rix++];
		} else {
			dev_err(&spi->dev,
				"buffer overrun when reading data.\n");
			return N_FAIL;
		}

		if (!spi_priv->crc_off) {
			/*
			 * Read Crc
			 */
			if ((rix + 1) < len2) {
				crc[0] = rb[rix++];
				crc[1] = rb[rix++];
			} else {
				dev_err(&spi->dev,
					"buffer overrun when reading crc.\n");
				return N_FAIL;
			}
		}
	} else if ((cmd == CMD_DMA_READ) || (cmd == CMD_DMA_EXT_READ)) {
		int ix;

		/* some data may be read in response to dummy bytes. */
		for (ix = 0; (rix < len2) && (ix < sz); )
			b[ix++] = rb[rix++];

		sz -= ix;

		if (sz > 0) {
			int nbytes;

			if (sz <= (DATA_PKT_SZ - ix))
				nbytes = sz;
			else
				nbytes = DATA_PKT_SZ - ix;

			/*
			 * Read bytes
			 */
			if (wilc_spi_rx(wilc, &b[ix], nbytes)) {
				dev_err(&spi->dev,
					"Failed block read, bus err\n");
				return N_FAIL;
			}

			/*
			 * Read Crc
			 */
			if (!spi_priv->crc_off && wilc_spi_rx(wilc, crc, 2)) {
				dev_err(&spi->dev,
					"Failed block crc read, bus err\n");
				return N_FAIL;
			}

			ix += nbytes;
			sz -= nbytes;
		}

		/*
		 * if any data in left unread,
		 * then read the rest using normal DMA code.
		 */
		while (sz > 0) {
			int nbytes;

			if (sz <= DATA_PKT_SZ)
				nbytes = sz;
			else
				nbytes = DATA_PKT_SZ;

			/*
			 * read data response only on the next DMA cycles not
			 * the first DMA since data response header is already
			 * handled above for the first DMA.
			 */
			/*
			 * Data Respnose header
			 */
			retry = SPI_RESP_RETRY_COUNT;
			do {
				if (wilc_spi_rx(wilc, &rsp, 1)) {
					dev_err(&spi->dev,
						"Failed resp read, bus err\n");
					result = N_FAIL;
					break;
				}
				if (((rsp >> 4) & 0xf) == 0xf)
					break;
			} while (retry--);

			if (result == N_FAIL)
				break;

			/*
			 * Read bytes
			 */
			if (wilc_spi_rx(wilc, &b[ix], nbytes)) {
				dev_err(&spi->dev,
					"Failed block read, bus err\n");
				result = N_FAIL;
				break;
			}

			/*
			 * Read Crc
			 */
			if (!spi_priv->crc_off && wilc_spi_rx(wilc, crc, 2)) {
				dev_err(&spi->dev,
					"Failed block crc read, bus err\n");
				result = N_FAIL;
				break;
			}

			ix += nbytes;
			sz -= nbytes;
		}
	}
	return result;
}

static int spi_data_write(struct wilc *wilc, u8 *b, u32 sz)
{
	struct spi_device *spi = to_spi_device(wilc->dev);
	struct wilc_spi *spi_priv = wilc->bus_data;
	int ix, nbytes;
	int result = 1;
	u8 cmd, order, crc[2] = {0};

	/*
	 * Data
	 */
	ix = 0;
	do {
		if (sz <= DATA_PKT_SZ) {
			nbytes = sz;
			order = 0x3;
		} else {
			nbytes = DATA_PKT_SZ;
			if (ix == 0)
				order = 0x1;
			else
				order = 0x02;
		}

		/*
		 * Write command
		 */
		cmd = 0xf0;
		cmd |= order;

		if (wilc_spi_tx(wilc, &cmd, 1)) {
			dev_err(&spi->dev,
				"Failed data block cmd write, bus error...\n");
			result = N_FAIL;
			break;
		}

		/*
		 * Write data
		 */
		if (wilc_spi_tx(wilc, &b[ix], nbytes)) {
			dev_err(&spi->dev,
				"Failed data block write, bus error...\n");
			result = N_FAIL;
			break;
		}

		/*
		 * Write Crc
		 */
		if (!spi_priv->crc_off) {
			if (wilc_spi_tx(wilc, crc, 2)) {
				dev_err(&spi->dev, "Failed data block crc write, bus error...\n");
				result = N_FAIL;
				break;
			}
		}

		/*
		 * No need to wait for response
		 */
		ix += nbytes;
		sz -= nbytes;
	} while (sz);

	return result;
}

/********************************************
 *
 *      Spi Internal Read/Write Function
 *
 ********************************************/

static int spi_internal_write(struct wilc *wilc, u32 adr, u32 dat)
{
	struct spi_device *spi = to_spi_device(wilc->dev);
	int result;
	u8 retry = SPI_RETRY_COUNT;

retry:
	cpu_to_le32s(&dat);
	result = spi_cmd_complete(wilc, CMD_INTERNAL_WRITE, adr, (u8 *)&dat, 4,
				  0);
	if (result != N_OK) {
		dev_err(&spi->dev, "Failed internal write cmd...\n");
		goto fail;
	}

fail:
	if (result != N_OK) {
		usleep_range(1000, 1100);
		wilc_spi_reset(wilc);
		dev_err(&spi->dev, "Reset and retry %d %x\n", retry, adr);
		usleep_range(1000, 1100);
		retry--;
		if (retry)
			goto retry;
	}
	return result;
}

static int spi_internal_read(struct wilc *wilc, u32 adr, u32 *data)
{
	struct spi_device *spi = to_spi_device(wilc->dev);
	int result = N_OK;
	u8 retry = SPI_RETRY_COUNT;

retry:
	result = spi_cmd_complete(wilc, CMD_INTERNAL_READ, adr, (u8 *)data, 4,
				  0);
	if (result != N_OK) {
		dev_err(&spi->dev, "Failed internal read cmd...\n");
		goto fail;
	}

	le32_to_cpus(data);

fail:
	if (result != N_OK) {
		usleep_range(1000, 1100);
		wilc_spi_reset(wilc);
		dev_err(&spi->dev, "Reset and retry %d %x\n", retry, adr);
		usleep_range(1000, 1100);
		retry--;
		if (retry)
			goto retry;
	}
	return result;
}

/********************************************
 *
 *      Spi interfaces
 *
 ********************************************/

static int wilc_spi_write_reg(struct wilc *wilc, u32 addr, u32 data)
{
	struct spi_device *spi = to_spi_device(wilc->dev);
	u8 retry = SPI_RETRY_COUNT;
	int result = N_OK;
	u8 cmd = CMD_SINGLE_WRITE;
	u8 clockless = 0;

	cpu_to_le32s(&data);
_RETRY_:
	if (addr <= 0x30) {
		/* Clockless register */
		cmd = CMD_INTERNAL_WRITE;
		clockless = 1;
	} else {
		cmd = CMD_SINGLE_WRITE;
		clockless = 0;
	}

	result = spi_cmd_complete(wilc, cmd, addr, (u8 *)&data, 4, clockless);
	if (result != N_OK) {
		dev_err(&spi->dev, "Failed cmd, write reg (%08x)...\n", addr);
		goto fail;
	}

fail:
	if (result != N_OK) {
		usleep_range(1000, 1100);
		wilc_spi_reset(wilc);
		dev_err(&spi->dev,
			"Reset and retry %d %x %d\n", retry, addr, data);
		usleep_range(1000, 1100);
		retry--;
		if (retry)
			goto _RETRY_;
	}
	return result;
}

static int wilc_spi_write(struct wilc *wilc, u32 addr, u8 *buf, u32 size)
{
	struct spi_device *spi = to_spi_device(wilc->dev);
	int result;
	u8 retry = SPI_RETRY_COUNT;

	/*
	 * has to be greated than 4
	 */
	if (size <= 4)
		return 0;

retry:
	result = spi_cmd_complete(wilc, CMD_DMA_EXT_WRITE, addr, NULL, size, 0);
	if (result != N_OK) {
		dev_err(&spi->dev,
			"Failed cmd, write block (%08x)...\n", addr);
		goto fail;
	}

	/*
	 * Data
	 */
	result = spi_data_write(wilc, buf, size);
	if (result != N_OK) {
		dev_err(&spi->dev, "Failed block data write...\n");
		goto fail;
	}
	/*
	 * Data RESP
	 */
	result = spi_data_rsp(wilc, CMD_DMA_EXT_WRITE);
	if (result != N_OK) {
		dev_err(&spi->dev, "Failed block data write...\n");
		goto fail;
	}

fail:
	if (result != N_OK) {
		usleep_range(1000, 1100);
		wilc_spi_reset(wilc);
		dev_err(&spi->dev,
			"Reset and retry %d %x %d\n", retry, addr, size);
		usleep_range(1000, 1100);
		retry--;
		if (retry)
			goto retry;
	}
	return result;
}

static int wilc_spi_read_reg(struct wilc *wilc, u32 addr, u32 *data)
{
	struct spi_device *spi = to_spi_device(wilc->dev);
	u8 retry = SPI_RETRY_COUNT;
	int result = N_OK;
	u8 cmd = CMD_SINGLE_READ;
	u8 clockless = 0;

retry:
	if (addr <= 0x30) {
		/* Clockless register */
		cmd = CMD_INTERNAL_READ;
		clockless = 1;
	} else {
		cmd = CMD_SINGLE_READ;
		clockless = 0;
	}

	result = spi_cmd_complete(wilc, cmd, addr, (u8 *)data, 4, clockless);
	if (result != N_OK) {
		dev_err(&spi->dev, "Failed cmd, read reg (%08x)...\n", addr);
		goto fail;
	}

	le32_to_cpus(data);

fail:
	if (result != N_OK) {
		usleep_range(1000, 1100);
		wilc_spi_reset(wilc);
		dev_warn(&spi->dev, "Reset and retry %d %x\n", retry, addr);
		usleep_range(1000, 1100);
		retry--;
		if (retry)
			goto retry;
	}
	return result;
}

static int wilc_spi_read(struct wilc *wilc, u32 addr, u8 *buf, u32 size)
{
	struct spi_device *spi = to_spi_device(wilc->dev);
	int result;
	u8 retry = SPI_RETRY_COUNT;

	if (size <= 4)
		return 0;

retry:
	result = spi_cmd_complete(wilc, CMD_DMA_EXT_READ, addr, buf, size, 0);
	if (result != N_OK) {
		dev_err(&spi->dev, "Failed cmd, read block (%08x)...\n", addr);
		goto fail;
	}

fail:
	if (result != N_OK) {
		usleep_range(1000, 1100);
		wilc_spi_reset(wilc);
		dev_warn(&spi->dev, "Reset and retry %d %x %d\n", retry, addr,
			 size);
		usleep_range(1000, 1100);
		retry--;
		if (retry)
			goto retry;
	}
	return result;
}

/********************************************
 *
 *      Bus interfaces
 *
 ********************************************/

int wilc_spi_reset(struct wilc *wilc)
{
	struct spi_device *spi = to_spi_device(wilc->dev);
	int result = N_OK;

	result = spi_cmd_complete(wilc, CMD_RESET, 0, 0, 0, 0);
	if (result != N_OK) {
		dev_err(&spi->dev, "Failed cmd reset\n");
		return 0;
	}

	return 1;
}

static bool wilc_spi_is_init(struct wilc *wilc)
{
	struct wilc_spi *spi_priv = wilc->bus_data;

	return spi_priv->is_init;
}

static int wilc_spi_deinit(struct wilc *wilc)
{
	struct wilc_spi *spi_priv = wilc->bus_data;

	/*
	 * TODO:
	 */
	spi_priv->is_init = false;

	return 1;
}

static int wilc_spi_init(struct wilc *wilc, bool resume)
{
	struct spi_device *spi = to_spi_device(wilc->dev);
	struct wilc_spi *spi_priv = wilc->bus_data;
	u32 reg;
	u32 chipid;

	if (spi_priv->is_init) {
		if (!wilc_spi_read_reg(wilc, 0x1000, &chipid)) {
			dev_err(&spi->dev, "Fail cmd read chip id...\n");
			return 0;
		}
		return 1;
	}

	/*
	 * configure protocol
	 */

	/*
	 * TODO: We can remove the CRC trials if there is a definite
	 * way to reset
	 */
	/* the SPI to it's initial value. */
	if (!spi_internal_read(wilc, WILC_SPI_PROTOCOL_OFFSET, &reg)) {
		/*
		 * Read failed. Try with CRC off. This might happen when module
		 * is removed but chip isn't reset
		 */
		spi_priv->crc_off = 1;
		dev_err(&spi->dev,
			"Failed read with CRC on, retrying with CRC off\n");
		if (!spi_internal_read(wilc, WILC_SPI_PROTOCOL_OFFSET, &reg)) {
			/*
			 * Read failed with both CRC on and off,
			 * something went bad
			 */
			dev_err(&spi->dev, "Failed internal read protocol...\n");
			return 0;
		}
	}
	if (spi_priv->crc_off == 0) {
		reg &= ~0xc; /* disable crc checking */
		reg &= ~0x70;
		reg |= (0x5 << 4);
		if (!spi_internal_write(wilc, WILC_SPI_PROTOCOL_OFFSET, reg)) {
			dev_err(&spi->dev,
				"[wilc spi %d]: Failed internal write reg\n",
				__LINE__);
			return 0;
		}
		spi_priv->crc_off = 1;
	}

	/*
	 * make sure can read back chip id correctly
	 */
	if (!wilc_spi_read_reg(wilc, 0x1000, &chipid)) {
		dev_err(&spi->dev, "Fail cmd read chip id...\n");
		return 0;
	}

	if (!resume) {
		chipid = wilc_get_chipid(wilc, true);
		if (is_wilc3000(chipid)) {
			wilc->chip = WILC_3000;
			goto pass;
		} else if (is_wilc1000(chipid)) {
			wilc->chip = WILC_1000;
			goto pass;
		} else {
			dev_err(&spi->dev, "Unsupported chipid: %x\n", chipid);
			goto fail;
		}
		dev_dbg(&spi->dev, "chipid %08x\n", chipid);
	}

pass:
	spi_priv->is_init = true;
	return 1;

fail:
	return 0;
}

static int wilc_spi_read_size(struct wilc *wilc, u32 *size)
{
	int ret;

	ret = spi_internal_read(wilc, 0xe840 - WILC_SPI_REG_BASE,
				size);
	*size = *size  & IRQ_DMA_WD_CNT_MASK;

	return ret;
}

static int wilc_spi_read_int(struct wilc *wilc, u32 *int_status)
{
	return spi_internal_read(wilc, 0xe840 - WILC_SPI_REG_BASE,
				int_status);
}

static int wilc_spi_clear_int_ext(struct wilc *wilc, u32 val)
{
	return spi_internal_write(wilc, 0xe844 - WILC_SPI_REG_BASE, val);
}

static int wilc_spi_sync_ext(struct wilc *wilc, int nint)
{
	struct spi_device *spi = to_spi_device(wilc->dev);
	struct wilc_spi *spi_priv = wilc->bus_data;
	u32 reg;
	int ret, i;

	if (nint > MAX_NUM_INT) {
		dev_err(&spi->dev, "Too many interrupts (%d)...\n", nint);
		return 0;
	}

	spi_priv->nint = nint;

	/*
	 * interrupt pin mux select
	 */
	ret = wilc_spi_read_reg(wilc, WILC_PIN_MUX_0, &reg);
	if (!ret) {
		dev_err(&spi->dev, "Failed read reg (%08x)...\n",
			WILC_PIN_MUX_0);
		return 0;
	}
	reg |= BIT(8);
	ret = wilc_spi_write_reg(wilc, WILC_PIN_MUX_0, reg);
	if (!ret) {
		dev_err(&spi->dev, "Failed write reg (%08x)...\n",
			WILC_PIN_MUX_0);
		return 0;
	}

	/*
	 * interrupt enable
	 */
	ret = wilc_spi_read_reg(wilc, WILC_INTR_ENABLE, &reg);
	if (!ret) {
		dev_err(&spi->dev, "Failed read reg (%08x)...\n",
			WILC_INTR_ENABLE);
		return 0;
	}

	for (i = 0; (i < 5) && (nint > 0); i++, nint--)
		reg |= (BIT((27 + i)));

	ret = wilc_spi_write_reg(wilc, WILC_INTR_ENABLE, reg);
	if (!ret) {
		dev_err(&spi->dev, "Failed write reg (%08x)...\n",
			WILC_INTR_ENABLE);
		return 0;
	}
	if (nint) {
		ret = wilc_spi_read_reg(wilc, WILC_INTR2_ENABLE, &reg);
		if (!ret) {
			dev_err(&spi->dev, "Failed read reg (%08x)...\n",
				WILC_INTR2_ENABLE);
			return 0;
		}

		for (i = 0; (i < 3) && (nint > 0); i++, nint--)
			reg |= BIT(i);

		ret = wilc_spi_write_reg(wilc, WILC_INTR2_ENABLE, reg);
		if (!ret) {
			dev_err(&spi->dev, "Failed write reg (%08x)...\n",
				WILC_INTR2_ENABLE);
			return 0;
		}
	}

	return 1;
}

/* Global spi HIF function table */
static const struct wilc_hif_func wilc_hif_spi = {
	.hif_init = wilc_spi_init,
	.hif_deinit = wilc_spi_deinit,
	.hif_read_reg = wilc_spi_read_reg,
	.hif_write_reg = wilc_spi_write_reg,
	.hif_block_rx = wilc_spi_read,
	.hif_block_tx = wilc_spi_write,
	.hif_read_int = wilc_spi_read_int,
	.hif_clear_int_ext = wilc_spi_clear_int_ext,
	.hif_read_size = wilc_spi_read_size,
	.hif_block_tx_ext = wilc_spi_write,
	.hif_block_rx_ext = wilc_spi_read,
	.hif_sync_ext = wilc_spi_sync_ext,
	.hif_reset = wilc_spi_reset,
	.hif_is_init = wilc_spi_is_init,
};
