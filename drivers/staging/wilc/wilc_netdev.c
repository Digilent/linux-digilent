// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2012 - 2018 Microchip Technology Inc., and its subsidiaries.
 * All rights reserved.
 */

#include <linux/irq.h>
#include <linux/kthread.h>
#include <linux/firmware.h>
#include <linux/init.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/interrupt.h>
#include <net/ip.h>
#include <linux/module.h>

#include "wilc_netdev.h"
#include "wilc_wfi_cfgoperations.h"

#define WILC_MULTICAST_TABLE_SIZE	8

static int wilc_mac_open(struct net_device *ndev);
static int wilc_mac_close(struct net_device *ndev);

int debug_running;
int recovery_on;
int wait_for_recovery;
static int debug_thread(void *arg)
{
	struct wilc *wl = arg;
	struct wilc_vif *vif = wilc_get_interface(wl);
	signed long timeout;
	struct host_if_drv *hif_drv;
	int i = 0;

	if (!vif)
		return -1;

	hif_drv = vif->priv.hif_drv;

	complete(&wl->debug_thread_started);

	while (1) {
		if (!wl->initialized && !kthread_should_stop()) {
			msleep(1000);
			continue;
		} else if (!wl->initialized) {
			break;
		}

		if (wait_for_completion_timeout(&wl->debug_thread_started,
						msecs_to_jiffies(6000))) {
			while (!kthread_should_stop())
				schedule();
			pr_info("Exit debug thread\n");
			return 0;
		}

		if (!debug_running)
			continue;
		PRINT_INFO(wl->vif[0]->ndev, GENERIC_DBG,
			   "*** Debug Thread Running ***\n");
		if (cfg_packet_timeout < 5)
			continue;

		PRINT_INFO(wl->vif[0]->ndev, GENERIC_DBG,
			   "<Recover>\n");
		cfg_packet_timeout = 0;
		timeout = 10;
		recovery_on = 1;
		wait_for_recovery = 1;
		for (i = 0; i < WILC_NUM_CONCURRENT_IFC; i++)
			wilc_mac_close(wl->vif[i]->ndev);
		for (i = WILC_NUM_CONCURRENT_IFC; i > 0; i--) {
			while (wilc_mac_open(wl->vif[i-1]->ndev) && --timeout)
				msleep(100);

			if (timeout == 0)
				PRINT_WRN(wl->vif[0]->ndev, GENERIC_DBG,
					  "Couldn't restart ifc %d\n", i);
		}
		if (hif_drv->hif_state == HOST_IF_CONNECTED) {
			struct wilc_conn_info *conn_info = &hif_drv->conn_info;

			PRINT_INFO(wl->vif[0]->ndev, GENERIC_DBG,
				   "notify the user with the Disconnection\n");
			if (hif_drv->usr_scan_req.scan_result) {
				PRINT_INFO(wl->vif[0]->ndev, GENERIC_DBG,
					   "Abort the running OBSS Scan\n");
				del_timer(&hif_drv->scan_timer);
				handle_scan_done(vif, SCAN_EVENT_ABORTED);
			}
			if (conn_info->conn_result) {
				conn_info->conn_result(EVENT_DISCONN_NOTIF,
						       0, conn_info->arg);
			} else {
				PRINT_ER(wl->vif[0]->ndev,
					 "Connect result NULL\n");
			}
			eth_zero_addr(hif_drv->assoc_bssid);

			conn_info->req_ies_len = 0;
			kfree(conn_info->req_ies);
			conn_info->req_ies = NULL;

			hif_drv->hif_state = HOST_IF_IDLE;
		}
		recovery_on = 0;
	}
	return 0;
}

void wilc_disable_irq(struct wilc *wilc, int wait)
{
	if (wait) {
		pr_info("%s Disabling IRQ ...\n", __func__);
		disable_irq(wilc->dev_irq_num);
	} else {
		pr_info("%s Disabling IRQ ...\n", __func__);
		disable_irq_nosync(wilc->dev_irq_num);
	}
}

static irqreturn_t host_wakeup_isr(int irq, void *user_data)
{
	return IRQ_HANDLED;
}

static irqreturn_t isr_uh_routine(int irq, void *user_data)
{
	struct wilc *wilc = (struct wilc *)user_data;

	pr_info("%s: Interrupt received UH\n", __func__);

	if (wilc->close) {
		pr_err("%s: Can't handle UH interrupt\n", __func__);
		return IRQ_HANDLED;
	}
	return IRQ_WAKE_THREAD;
}

static irqreturn_t isr_bh_routine(int irq, void *userdata)
{
	struct wilc *wilc = (struct wilc *)userdata;

	if (wilc->close) {
		pr_err("%s: Can't handle BH interrupt\n", __func__);
		return IRQ_HANDLED;
	}

	pr_info("%s: Interrupt received BH\n", __func__);
	wilc_handle_isr(wilc);

	return IRQ_HANDLED;
}

static int init_irq(struct net_device *dev)
{
	int ret = 0;
	struct wilc_vif *vif = netdev_priv(dev);
	struct wilc *wl = vif->wilc;

#if KERNEL_VERSION(3, 13, 0) < LINUX_VERSION_CODE

	wl->gpio_irq = gpiod_get(wl->dt_dev, "irq", GPIOD_IN);
	if (IS_ERR(wl->gpio_irq)) {
		dev_warn(wl->dev, "failed to get IRQ GPIO, load default\r\n");
		wl->gpio_irq = gpio_to_desc(GPIO_NUM);
		if (!wl->gpio_irq) {
			dev_warn(wl->dev, "failed to load default irq\r\n");
			return -1;
		}
	} else {
		dev_info(wl->dev, "got gpio_irq successfully\r\n");
	}

	ret = gpiod_direction_input(wl->gpio_irq);
	if (ret) {
		PRINT_ER(dev, "could not obtain gpio for WILC_INTR\n");
		return ret;
	}

	wl->dev_irq_num = gpiod_to_irq(wl->gpio_irq);
	if (wl->dev_irq_num < 0) {
		PRINT_ER(dev, "could not the IRQ\n");
		goto free_gpio;
	}
#else
	wl->gpio_irq = of_get_named_gpio_flags(wl->dt_dev->of_node,
					       "irq-gpios", 0, NULL);
	if (wl->gpio_irq < 0) {
		wl->gpio_irq = GPIO_NUM;
		dev_warn(wl->dev, "failed to get IRQ GPIO, load default\r\n");
	}

	if ((gpio_request(wl->gpio_irq, "WILC_INTR") == 0) &&
	    (gpio_direction_input(wl->gpio_irq) == 0)) {
		wl->dev_irq_num = gpio_to_irq(wl->gpio_irq);
	} else {
		dev_err(wl->dev, "could not obtain gpio for WILC_INTR\n");
		wl->gpio_irq = 0;
		return -1;
	}

#endif

	if (wl->io_type == WILC_HIF_SPI ||
		wl->io_type == WILC_HIF_SDIO_GPIO_IRQ) {
		if (request_threaded_irq(wl->dev_irq_num, isr_uh_routine,
					 isr_bh_routine, IRQF_TRIGGER_LOW |
							IRQF_ONESHOT |
							IRQF_NO_SUSPEND,
					 "WILC_IRQ", wl) < 0) {
			PRINT_ER(dev, "Failed to request IRQ\n");
			goto free_gpio;
		}
	} else {
		if (request_irq(wl->dev_irq_num, host_wakeup_isr,
					     IRQF_TRIGGER_FALLING |
					     IRQF_NO_SUSPEND,
					     "WILC_IRQ", wl) < 0) {
			PRINT_ER(dev, "Failed to request IRQ\n");
			goto free_gpio;
		}
	}

	PRINT_INFO(dev, GENERIC_DBG, "IRQ request succeeded IRQ-NUM= %d\n",
		   wl->dev_irq_num);
	enable_irq_wake(wl->dev_irq_num);
	return ret;

free_gpio:
#if KERNEL_VERSION(3, 13, 0) < LINUX_VERSION_CODE
	gpiod_put(wl->gpio_irq);
	wl->gpio_irq = NULL;
#else
	gpio_free(wl->gpio_irq);
	wl->gpio_irq = 0;
#endif
	return -1;
}

static void deinit_irq(struct net_device *dev)
{
	struct wilc_vif *vif = netdev_priv(dev);
	struct wilc *wilc = vif->wilc;

	/* Deinitialize IRQ */
	if (wilc->dev_irq_num > 0) {
		free_irq(wilc->dev_irq_num, wilc);
		wilc->dev_irq_num = -1;
	}

#if KERNEL_VERSION(3, 13, 0) < LINUX_VERSION_CODE
	if (wilc->gpio_irq) {
		gpiod_put(wilc->gpio_irq);
		wilc->gpio_irq = NULL;
	}
#else
	if (wilc->gpio_irq > 0) {
		gpio_free(wilc->gpio_irq);
		wilc->gpio_irq = 0;
	}

#endif

}

void wilc_mac_indicate(struct wilc *wilc)
{
	s8 status;

	cfg_get_val(wilc, WID_STATUS, &status, 1);
	if (wilc->mac_status == WILC_MAC_STATUS_INIT) {
		wilc->mac_status = status;
		complete(&wilc->sync_event);
	} else {
		wilc->mac_status = status;
	}
}

void wilc_frmw_to_host(struct wilc_vif *vif, u8 *buff, u32 size,
		       u32 pkt_offset, u8 status)
{
	unsigned int frame_len = 0;
	int stats;
	unsigned char *buff_to_send = NULL;
	struct sk_buff *skb;
	struct wilc_priv *priv;
	u8 null_bssid[ETH_ALEN] = {0};

	buff += pkt_offset;
	priv = &vif->priv;

	if (size == 0) {
		PRINT_ER(vif->ndev,
			 "Discard sending packet with len = %d\n", size);
		return;
	}

	frame_len = size;
	buff_to_send = buff;

	if (status == PKT_STATUS_NEW && buff_to_send[12] == 0x88 &&
	   buff_to_send[13] == 0x8e &&
	   (vif->iftype == WILC_STATION_MODE ||
	    vif->iftype == WILC_CLIENT_MODE) &&
	   ether_addr_equal_unaligned(priv->associated_bss, null_bssid)) {
		if (!priv->buffered_eap) {
			priv->buffered_eap = kmalloc(sizeof(struct
							    wilc_buffered_eap),
						     GFP_ATOMIC);
			if (priv->buffered_eap) {
				priv->buffered_eap->buff = NULL;
				priv->buffered_eap->size = 0;
				priv->buffered_eap->pkt_offset = 0;
			} else {
				PRINT_ER(vif->ndev,
					 "failed to alloc buffered_eap\n");
				return;
			}
		} else {
			kfree(priv->buffered_eap->buff);
		}
		priv->buffered_eap->buff = kmalloc(size + pkt_offset,
						   GFP_ATOMIC);
		priv->buffered_eap->size = size;
		priv->buffered_eap->pkt_offset = pkt_offset;
		memcpy(priv->buffered_eap->buff, buff -
		       pkt_offset, size + pkt_offset);
	#if KERNEL_VERSION(4, 15, 0) > LINUX_VERSION_CODE
		priv->eap_buff_timer.data = (unsigned long) priv;
	#endif
		mod_timer(&priv->eap_buff_timer, (jiffies +
			  msecs_to_jiffies(10)));
		return;
	}
	skb = dev_alloc_skb(frame_len);
	if (!skb) {
		PRINT_ER(vif->ndev, "Low memory - packet droped\n");
		return;
	}

	skb->dev = vif->ndev;
#if KERNEL_VERSION(4, 13, 0) <= LINUX_VERSION_CODE
	skb_put_data(skb, buff_to_send, frame_len);
#else
	memcpy(skb_put(skb, frame_len), buff_to_send, frame_len);
#endif

	skb->protocol = eth_type_trans(skb, vif->ndev);
	vif->netstats.rx_packets++;
	vif->netstats.rx_bytes += frame_len;
	skb->ip_summed = CHECKSUM_UNNECESSARY;
	stats = netif_rx(skb);
	PRINT_D(vif->ndev, RX_DBG, "netif_rx ret value: %d\n", stats);
}

void free_eap_buff_params(void *vp)
{
	struct wilc_priv *priv;

	priv = (struct wilc_priv *)vp;

	if (priv->buffered_eap) {
		kfree(priv->buffered_eap->buff);
		priv->buffered_eap->buff = NULL;

		kfree(priv->buffered_eap);
		priv->buffered_eap = NULL;
	}
}

#if KERNEL_VERSION(4, 15, 0) <= LINUX_VERSION_CODE
void eap_buff_timeout(struct timer_list *t)
#else
void eap_buff_timeout(unsigned long user)
#endif
{
	u8 null_bssid[ETH_ALEN] = {0};
	u8 *assoc_bss;
	static u8 timeout = 5;
	int status = -1;
#if KERNEL_VERSION(4, 15, 0) <= LINUX_VERSION_CODE
	struct wilc_priv *priv = from_timer(priv, t, eap_buff_timer);
#else
	struct wilc_priv *priv = (struct wilc_priv *)user;
#endif
	struct wilc_vif *vif = netdev_priv(priv->dev);

	assoc_bss = priv->associated_bss;
	if (!(memcmp(assoc_bss, null_bssid, ETH_ALEN)) && (timeout-- > 0)) {
		mod_timer(&priv->eap_buff_timer,
			  (jiffies + msecs_to_jiffies(10)));
		return;
	}
	del_timer(&priv->eap_buff_timer);
	timeout = 5;

	status = wilc_send_buffered_eap(vif, wilc_frmw_to_host,
					free_eap_buff_params,
					priv->buffered_eap->buff,
					priv->buffered_eap->size,
					priv->buffered_eap->pkt_offset,
					(void *)priv);
	if (status)
		PRINT_ER(vif->ndev, "Failed so send buffered eap\n");
}

void wilc_wlan_set_bssid(struct net_device *wilc_netdev, u8 *bssid, u8 mode)
{
	struct wilc_vif *vif = netdev_priv(wilc_netdev);
	struct wilc *wilc = vif->wilc;
	u8 i = 0;

	//TODO: Do we really need this iteration?
	mutex_lock(&wilc->vif_mutex);
	PRINT_INFO(vif->ndev, GENERIC_DBG, "set bssid on[%p]\n", wilc_netdev);
	for (i = 0; i < wilc->vif_num; i++) {
		if (wilc_netdev == wilc->vif[i]->ndev) {
			if (bssid)
				ether_addr_copy(wilc->vif[i]->bssid, bssid);
			else
				eth_zero_addr(wilc->vif[i]->bssid);
			PRINT_INFO(vif->ndev, GENERIC_DBG,
				   "set bssid [%pM]\n", wilc->vif[i]->bssid);
			wilc->vif[i]->iftype = mode;
		}
	}
	mutex_unlock(&wilc->vif_mutex);
}

struct net_device *wilc_get_if_netdev(struct wilc *wilc, uint8_t ifc)
{
	if (wilc->vif[ifc])
		return wilc->vif[ifc]->ndev;
	else
		return NULL;
}

struct host_if_drv *get_drv_hndl_by_ifc(struct wilc *wilc, uint8_t ifc)
{
	if (wilc->vif[ifc])
		return wilc->vif[ifc]->hif_drv;
	else
		return NULL;
}

#define TX_BACKOFF_WEIGHT_INCR_STEP (1)
#define TX_BACKOFF_WEIGHT_DECR_STEP (1)
#define TX_BACKOFF_WEIGHT_MAX (0)
#define TX_BACKOFF_WEIGHT_MIN (0)
#define TX_BCKOFF_WGHT_MS (1)

static int wilc_txq_task(void *vp)
{
	int ret;
	u32 txq_count;
	int backoff_weight = TX_BACKOFF_WEIGHT_MIN;
	signed long timeout;
	struct wilc *wl = vp;

	complete(&wl->txq_thread_started);
	while (1) {
		struct wilc_vif *vif = wilc_get_interface(wl);
		struct net_device *ndev = vif->ndev;

		PRINT_INFO(ndev, TX_DBG, "txq_task Taking a nap\n");
		wait_for_completion(&wl->txq_event);
		PRINT_INFO(ndev, TX_DBG, "txq_task Who waked me up\n");
		if (wl->close) {
			complete(&wl->txq_thread_started);

			while (!kthread_should_stop())
				schedule();
			PRINT_INFO(ndev, TX_DBG, "TX thread stopped\n");
			break;
		}
		PRINT_INFO(ndev, TX_DBG, "handle the tx packet\n");
		do {
			ret = wilc_wlan_handle_txq(wl, &txq_count);
			if (txq_count < FLOW_CTRL_LOW_THRESHLD) {
				int i;
				struct wilc_vif *ifc;

				mutex_lock(&wl->vif_mutex);
				PRINT_INFO(ndev, TX_DBG, "Waking up queue\n");
				for (i = 0; i < wl->vif_num; i++) {
					ifc = wl->vif[i];
					if (ifc->mac_opened &&
					    netif_queue_stopped(ifc->ndev))
						netif_wake_queue(ifc->ndev);
				}
				mutex_unlock(&wl->vif_mutex);
			}

			if (ret == -ENOBUFS) {
				timeout = msecs_to_jiffies(TX_BCKOFF_WGHT_MS <<
							   backoff_weight);
				do {
			/* Back off from sending packets for some time.
			 * schedule_timeout will allow RX task to run and free
			 * buffers. Setting state to TASK_INTERRUPTIBLE will
			 * put the thread back to CPU running queue when it's
			 * signaled even if 'timeout' isn't elapsed. This gives
			 * faster chance for reserved SK buffers to be freed
			 */
					set_current_state(TASK_INTERRUPTIBLE);
					timeout = schedule_timeout(timeout);
					} while (/*timeout*/0);
				backoff_weight += TX_BACKOFF_WEIGHT_INCR_STEP;
				if (backoff_weight > TX_BACKOFF_WEIGHT_MAX)
					backoff_weight = TX_BACKOFF_WEIGHT_MAX;
			} else if (backoff_weight > TX_BACKOFF_WEIGHT_MIN) {
				backoff_weight -= TX_BACKOFF_WEIGHT_DECR_STEP;
				if (backoff_weight < TX_BACKOFF_WEIGHT_MIN)
					backoff_weight = TX_BACKOFF_WEIGHT_MIN;
			}
		} while (ret == -ENOBUFS && !wl->close);
	}
	return 0;
}

static int wilc_wlan_get_firmware(struct net_device *dev)
{
	struct wilc_vif *vif = netdev_priv(dev);
	struct wilc *wilc = vif->wilc;
	int ret = 0;
	const struct firmware *wilc_firmware;
	char *firmware;

	if (wilc->chip == WILC_3000) {
		PRINT_INFO(dev, INIT_DBG, "Detect chip WILC3000\n");
		firmware = FW_WILC3000_WIFI;
	} else if (wilc->chip == WILC_1000) {
		PRINT_INFO(dev, INIT_DBG, "Detect chip WILC1000\n");
		firmware = FW_WILC1000_WIFi;
	} else {
		return -1;
	}

	PRINT_INFO(dev, INIT_DBG, "loading firmware %s\n", firmware);

	if (!(&vif->ndev->dev)) {
		PRINT_ER(dev, "Dev  is NULL\n");
		goto fail;
	}

	PRINT_INFO(vif->ndev, INIT_DBG, "WLAN firmware: %s\n", firmware);
	if (request_firmware(&wilc_firmware, firmware, wilc->dev) != 0) {
		PRINT_ER(dev, "%s - firmware not available\n", firmware);
		ret = -1;
		goto fail;
	}
	wilc->firmware = wilc_firmware;

fail:

	return ret;
}

static int wilc_start_firmware(struct net_device *dev)
{
	struct wilc_vif *vif = netdev_priv(dev);
	struct wilc *wilc = vif->wilc;
	int ret = 0;

	PRINT_INFO(vif->ndev, INIT_DBG, "Starting Firmware ...\n");

	ret = wilc_wlan_start(wilc);
	if (ret < 0) {
		PRINT_ER(dev, "Failed to start Firmware\n");
		return ret;
	}
	PRINT_INFO(vif->ndev, INIT_DBG, "Waiting for FW to get ready ...\n");

	if (!wait_for_completion_timeout(&wilc->sync_event,
					 msecs_to_jiffies(500))) {
		PRINT_INFO(vif->ndev, INIT_DBG, "Firmware start timed out\n");
		return -ETIME;
	}
	PRINT_INFO(vif->ndev, INIT_DBG, "Firmware successfully started\n");

	return 0;
}

static int wilc_firmware_download(struct net_device *dev)
{
	struct wilc_vif *vif = netdev_priv(dev);
	struct wilc *wilc = vif->wilc;
	int ret = 0;

	if (!wilc->firmware) {
		PRINT_ER(dev, "Firmware buffer is NULL\n");
		ret = -ENOBUFS;
	}
	PRINT_INFO(vif->ndev, INIT_DBG, "Downloading Firmware ...\n");
	ret = wilc_wlan_firmware_download(wilc, wilc->firmware->data,
					  wilc->firmware->size);
	if (ret < 0)
		goto fail;

	PRINT_INFO(vif->ndev, INIT_DBG, "Download Succeeded\n");

fail:
	release_firmware(wilc->firmware);
	wilc->firmware = NULL;

	return ret;
}

static int wilc_init_fw_config(struct net_device *dev, struct wilc_vif *vif)
{
	struct wilc_priv *priv = &vif->priv;
	struct host_if_drv *hif_drv;
	u8 b;
	u16 hw;
	u32 w;

	PRINT_INFO(vif->ndev, INIT_DBG, "Start configuring Firmware\n");
	hif_drv = (struct host_if_drv *)priv->hif_drv;
	PRINT_D(vif->ndev, INIT_DBG, "Host = %p\n", hif_drv);

	w = vif->iftype;
	cpu_to_le32s(&w);
	if (!cfg_set(vif, 1, WID_SET_OPERATION_MODE, (u8 *)&w, 4, 0, 0))
		goto fail;

	b = WILC_FW_BSS_TYPE_INFRA;
	if (!cfg_set(vif, 0, WID_BSS_TYPE, &b, 1, 0, 0))
		goto fail;

	b = WILC_FW_TX_RATE_AUTO;
	if (!cfg_set(vif, 0, WID_CURRENT_TX_RATE, &b, 1, 0, 0))
		goto fail;

	b = WILC_FW_OPER_MODE_G_MIXED_11B_2;
	if (!cfg_set(vif, 0, WID_11G_OPERATING_MODE, &b, 1, 0,
			       0))
		goto fail;

	b = WILC_FW_PREAMBLE_AUTO;
	if (!cfg_set(vif, 0, WID_PREAMBLE, &b, 1, 0, 0))
		goto fail;

	b = WILC_FW_11N_PROT_AUTO;
	if (!cfg_set(vif, 0, WID_11N_PROT_MECH, &b, 1, 0, 0))
		goto fail;

	b = WILC_FW_ACTIVE_SCAN;
	if (!cfg_set(vif, 0, WID_SCAN_TYPE, &b, 1, 0, 0))
		goto fail;

	b = WILC_FW_SITE_SURVEY_OFF;
	if (!cfg_set(vif, 0, WID_SITE_SURVEY, &b, 1, 0, 0))
		goto fail;

	hw = 0xffff;
	cpu_to_le16s(&hw);
	if (!cfg_set(vif, 0, WID_RTS_THRESHOLD, (u8 *)&hw, 2, 0, 0))
		goto fail;

	hw = 2346;
	cpu_to_le16s(&hw);
	if (!cfg_set(vif, 0, WID_FRAG_THRESHOLD, (u8 *)&hw, 2, 0, 0))
		goto fail;

	b = 0;
	if (!cfg_set(vif, 0, WID_BCAST_SSID, &b, 1, 0, 0))
		goto fail;

	b = 1;
	if (!cfg_set(vif, 0, WID_QOS_ENABLE, &b, 1, 0, 0))
		goto fail;

	b = WILC_FW_NO_POWERSAVE;
	if (!cfg_set(vif, 0, WID_POWER_MANAGEMENT, &b, 1, 0, 0))
		goto fail;

	b = WILC_FW_SEC_NO;
	if (!cfg_set(vif, 0, WID_11I_MODE, &b, 1, 0, 0))
		goto fail;

	b = WILC_FW_AUTH_OPEN_SYSTEM;
	if (!cfg_set(vif, 0, WID_AUTH_TYPE, &b, 1, 0, 0))
		goto fail;

	b = 3;
	if (!cfg_set(vif, 0, WID_LISTEN_INTERVAL, &b, 1, 0, 0))
		goto fail;

	b = 3;
	if (!cfg_set(vif, 0, WID_DTIM_PERIOD, &b, 1, 0, 0))
		goto fail;

	b = WILC_FW_ACK_POLICY_NORMAL;
	if (!cfg_set(vif, 0, WID_ACK_POLICY, &b, 1, 0, 0))
		goto fail;

	b = 0;
	if (!cfg_set(vif, 0, WID_USER_CONTROL_ON_TX_POWER, &b, 1,
			       0, 0))
		goto fail;

	b = 48;
	if (!cfg_set(vif, 0, WID_TX_POWER_LEVEL_11A, &b, 1, 0,
			       0))
		goto fail;

	b = 28;
	if (!cfg_set(vif, 0, WID_TX_POWER_LEVEL_11B, &b, 1, 0,
			       0))
		goto fail;

	hw = 100;
	cpu_to_le16s(&hw);
	if (!cfg_set(vif, 0, WID_BEACON_INTERVAL, (u8 *)&hw, 2, 0, 0))
		goto fail;

	b = WILC_FW_REKEY_POLICY_DISABLE;
	if (!cfg_set(vif, 0, WID_REKEY_POLICY, &b, 1, 0, 0))
		goto fail;

	w = 84600;
	cpu_to_le32s(&w);
	if (!cfg_set(vif, 0, WID_REKEY_PERIOD, (u8 *)&w, 4, 0, 0))
		goto fail;

	w = 500;
	cpu_to_le32s(&w);
	if (!cfg_set(vif, 0, WID_REKEY_PACKET_COUNT, (u8 *)&w, 4, 0,
			       0))
		goto fail;

	b = 1;
	if (!cfg_set(vif, 0, WID_SHORT_SLOT_ALLOWED, &b, 1, 0,
			       0))
		goto fail;

	b = WILC_FW_ERP_PROT_SELF_CTS;
	if (!cfg_set(vif, 0, WID_11N_ERP_PROT_TYPE, &b, 1, 0, 0))
		goto fail;

	b = 1;
	if (!cfg_set(vif, 0, WID_11N_ENABLE, &b, 1, 0, 0))
		goto fail;

	b = WILC_FW_11N_OP_MODE_HT_MIXED;
	if (!cfg_set(vif, 0, WID_11N_OPERATING_MODE, &b, 1, 0,
			       0))
		goto fail;

	b = 1;
	if (!cfg_set(vif, 0, WID_11N_TXOP_PROT_DISABLE, &b, 1, 0,
			       0))
		goto fail;

	b = WILC_FW_OBBS_NONHT_DETECT_PROTECT_REPORT;
	if (!cfg_set(vif, 0, WID_11N_OBSS_NONHT_DETECTION, &b, 1,
			       0, 0))
		goto fail;

	b = WILC_FW_HT_PROT_RTS_CTS_NONHT;
	if (!cfg_set(vif, 0, WID_11N_HT_PROT_TYPE, &b, 1, 0, 0))
		goto fail;

	b = 0;
	if (!cfg_set(vif, 0, WID_11N_RIFS_PROT_ENABLE, &b, 1, 0,
			       0))
		goto fail;

	b = 7;
	if (!cfg_set(vif, 0, WID_11N_CURRENT_TX_MCS, &b, 1, 0,
			       0))
		goto fail;

	b = 1;
	if (!cfg_set(vif, 0, WID_11N_IMMEDIATE_BA_ENABLED, &b, 1,
			       1, 0))
		goto fail;

	return 0;

fail:
	return -1;
}

static void wlan_deinitialize_threads(struct net_device *dev)
{
	struct wilc_vif *vif = netdev_priv(dev);
	struct wilc *wl = vif->wilc;

	PRINT_INFO(vif->ndev, INIT_DBG, "Deinitializing Threads\n");
	if (!recovery_on) {
		PRINT_INFO(vif->ndev, INIT_DBG, "Deinit debug Thread\n");
		debug_running = false;
		if (&wl->debug_thread_started)
			complete(&wl->debug_thread_started);
		if (wl->debug_thread) {
			kthread_stop(wl->debug_thread);
			wl->debug_thread = NULL;
		}
	}

	wl->close = 1;
	PRINT_INFO(vif->ndev, INIT_DBG, "Deinitializing Threads\n");

	complete(&wl->txq_event);

	if (wl->txq_thread) {
		kthread_stop(wl->txq_thread);
		wl->txq_thread = NULL;
	}
}

static void wilc_wlan_deinitialize(struct net_device *dev)
{
	int ret;
	struct wilc_vif *vif = netdev_priv(dev);
	struct wilc *wl = vif->wilc;

	if (wl->initialized) {
		PRINT_INFO(vif->ndev, INIT_DBG, "Deinitializing wilc  ...\n");

		if (!wl) {
			PRINT_ER(dev, "wl is NULL\n");
			return;
		}

		PRINT_D(vif->ndev, INIT_DBG, "destroy aging timer\n");

		PRINT_INFO(vif->ndev, INIT_DBG, "Disabling IRQ\n");
		if (wl->io_type == WILC_HIF_SPI ||
			wl->io_type == WILC_HIF_SDIO_GPIO_IRQ) {
			wilc_disable_irq(wl, 1);
		} else {
			if (wl->hif_func->disable_interrupt) {
				mutex_lock(&wl->hif_cs);
				wl->hif_func->disable_interrupt(wl);
				mutex_unlock(&wl->hif_cs);
			}
		}
		complete(&wl->txq_event);

		PRINT_INFO(vif->ndev, INIT_DBG, "Deinitializing Threads\n");
		wlan_deinitialize_threads(dev);
		PRINT_INFO(vif->ndev, INIT_DBG, "Deinitializing IRQ\n");
		deinit_irq(dev);

		ret = wilc_wlan_stop(wl, vif);
		if (ret == 0)
			PRINT_ER(dev, "failed in wlan_stop\n");

		PRINT_INFO(vif->ndev, INIT_DBG, "Deinitializing WILC Wlan\n");
		wilc_wlan_cleanup(dev);

		wl->initialized = false;

		PRINT_INFO(dev, INIT_DBG, "wilc deinitialization Done\n");
	} else {
		PRINT_INFO(dev, INIT_DBG, "wilc is not initialized\n");
	}
}

static int wlan_initialize_threads(struct net_device *dev)
{
	struct wilc_vif *vif = netdev_priv(dev);
	struct wilc *wilc = vif->wilc;

	PRINT_INFO(vif->ndev, INIT_DBG, "Initializing Threads ...\n");
	PRINT_INFO(vif->ndev, INIT_DBG, "Creating kthread for transmission\n");
	wilc->txq_thread = kthread_run(wilc_txq_task, (void *)wilc,
				       "K_TXQ_TASK");
	if (IS_ERR(wilc->txq_thread)) {
		PRINT_ER(dev, "couldn't create TXQ thread\n");
		wilc->close = 1;
		return PTR_ERR(wilc->txq_thread);
	}
	wait_for_completion(&wilc->txq_thread_started);

	if (!debug_running) {
		PRINT_INFO(vif->ndev, INIT_DBG,
			   "Creating kthread for Debugging\n");
		wilc->debug_thread = kthread_run(debug_thread, (void *)wilc,
						 "WILC_DEBUG");
		if (IS_ERR(wilc->debug_thread)) {
			PRINT_ER(dev, "couldn't create debug thread\n");
			wilc->close = 1;
			kthread_stop(wilc->txq_thread);
			return PTR_ERR(wilc->debug_thread);
		}
		debug_running = true;
		wait_for_completion(&wilc->debug_thread_started);
	}

	return 0;
}

static int wilc_wlan_initialize(struct net_device *dev, struct wilc_vif *vif)
{
	int ret = 0;
	struct wilc *wl = vif->wilc;

	if (!wl->initialized) {
		wl->mac_status = WILC_MAC_STATUS_INIT;
		wl->close = 0;
		wl->initialized = 0;

		ret = wilc_wlan_init(dev);
		if (ret < 0) {
			PRINT_ER(dev, "Initializing WILC_Wlan FAILED\n");
			return -EIO;
		}
		PRINT_INFO(vif->ndev, GENERIC_DBG,
			   "WILC Initialization done\n");

		ret = wlan_initialize_threads(dev);
		if (ret < 0) {
			PRINT_ER(dev, "Initializing Threads FAILED\n");
			ret = -EIO;
			goto fail_wilc_wlan;
		}

		if (init_irq(dev)) {
			ret = -EIO;
			goto fail_threads;
		}

		if (wl->io_type == WILC_HIF_SDIO &&
		    wl->hif_func->enable_interrupt(wl)) {
			PRINT_ER(dev, "couldn't initialize IRQ\n");
			ret = -EIO;
			goto fail_irq_init;
		}

		if (wilc_wlan_get_firmware(dev)) {
			PRINT_ER(dev, "Can't get firmware\n");
			ret = -EIO;
			goto fail_irq_enable;
		}

		ret = wilc_firmware_download(dev);
		if (ret < 0) {
			PRINT_ER(dev, "Failed to download firmware\n");
			ret = -EIO;
			goto fail_irq_enable;
		}

		ret = wilc_start_firmware(dev);
		if (ret < 0) {
			PRINT_ER(dev, "Failed to start firmware\n");
			ret = -EIO;
			goto fail_irq_enable;
		}

		if (cfg_get(vif, 1, WID_FIRMWARE_VERSION, 1, 0)) {
			int size;
			char firmware_ver[50];

			size = cfg_get_val(wl, WID_FIRMWARE_VERSION,
						     firmware_ver,
						     sizeof(firmware_ver));
			firmware_ver[size] = '\0';
			PRINT_INFO(dev, INIT_DBG, "WILC Firmware Ver = %s\n",
				   firmware_ver);
		}

		ret = wilc_init_fw_config(dev, vif);
		if (ret < 0) {
			PRINT_ER(dev, "Failed to configure firmware\n");
			ret = -EIO;
			goto fail_fw_start;
		}

		wl->initialized = true;
		return 0;

fail_fw_start:
		wilc_wlan_stop(wl, vif);

fail_irq_enable:
		if (wl->io_type == WILC_HIF_SDIO)
			wl->hif_func->disable_interrupt(wl);
fail_irq_init:
		deinit_irq(dev);

fail_threads:
		wlan_deinitialize_threads(dev);
fail_wilc_wlan:
		wilc_wlan_cleanup(dev);
		PRINT_ER(dev, "WLAN initialization FAILED\n");
	} else {
		PRINT_WRN(vif->ndev, INIT_DBG, "wilc already initialized\n");
	}
	return ret;
}

static int mac_init_fn(struct net_device *ndev)
{
	netif_start_queue(ndev);
	netif_stop_queue(ndev);

	return 0;
}

static int wilc_mac_open(struct net_device *ndev)
{
	struct wilc_vif *vif = netdev_priv(ndev);
	struct wilc *wl = vif->wilc;
	struct wilc_priv *priv = &vif->priv;
	unsigned char mac_add[ETH_ALEN] = {0};
	int ret = 0;

	if (!wl || !wl->dev) {
		PRINT_ER(ndev, "device not ready\n");
		return -ENODEV;
	}

	PRINT_INFO(ndev, INIT_DBG, "MAC OPEN[%p] %s\n", ndev, ndev->name);

	if (wl->open_ifcs == 0)
		wilc_bt_power_up(wl, DEV_WIFI);

	if (!recovery_on) {
		ret = wilc_init_host_int(ndev);
		if (ret < 0) {
			PRINT_ER(ndev, "Failed to initialize host interface\n");
			return ret;
		}
	}

	PRINT_INFO(vif->ndev, INIT_DBG, "*** re-init ***\n");
	ret = wilc_wlan_initialize(ndev, vif);
	if (ret < 0) {
		PRINT_ER(ndev, "Failed to initialize wilc\n");
		if (!recovery_on)
			wilc_deinit_host_int(ndev);
		return ret;
	}

	wait_for_recovery = 0;
	wilc_set_operation_mode(vif, wilc_get_vif_idx(vif),
				 vif->iftype, vif->idx);
	wilc_get_mac_address(vif, mac_add);
	PRINT_INFO(vif->ndev, INIT_DBG, "Mac address: %pM\n", mac_add);

	if (!is_valid_ether_addr(mac_add)) {
		PRINT_ER(ndev, "Wrong MAC address\n");
		wilc_deinit_host_int(ndev);
		wilc_wlan_deinitialize(ndev);
		return -EINVAL;
	}
	ether_addr_copy(ndev->dev_addr, mac_add);

	wilc_mgmt_frame_register(vif->ndev->ieee80211_ptr->wiphy,
				 vif->ndev->ieee80211_ptr,
				 vif->frame_reg[0].type,
				 vif->frame_reg[0].reg);
	wilc_mgmt_frame_register(vif->ndev->ieee80211_ptr->wiphy,
				 vif->ndev->ieee80211_ptr,
				 vif->frame_reg[1].type,
				 vif->frame_reg[1].reg);
	netif_wake_queue(ndev);
	wl->open_ifcs++;
	priv->p2p.local_random = 0x01;
	vif->mac_opened = 1;
	return 0;
}

static struct net_device_stats *mac_stats(struct net_device *dev)
{
	struct wilc_vif *vif = netdev_priv(dev);

	return &vif->netstats;
}

static int wilc_set_mac_addr(struct net_device *dev, void *p)
{
	int result;
	struct wilc_vif *vif = netdev_priv(dev);
	struct sockaddr *addr = (struct sockaddr *)p;
	struct wilc *wilc = vif->wilc;
	unsigned char mac_addr[6] = {0};
	int i;

	if (!is_valid_ether_addr(addr->sa_data)) {
		PRINT_INFO(vif->ndev, INIT_DBG, "Invalid MAC address\n");
		return -EINVAL;
	}

	for (i = 0; i < wilc->vif_num; i++) {
		wilc_get_mac_address(wilc->vif[i], mac_addr);
		if (ether_addr_equal(addr->sa_data, mac_addr)) {
			if (vif != wilc->vif[i]) {
				PRINT_INFO(vif->ndev, INIT_DBG,
					   "MAC address is alredy in use\n");
				return -EINVAL;
			} else {
				return 0;
			}
		}
	}

	/* configure new MAC address */
	result = wilc_set_mac_address(vif, (u8 *)addr->sa_data);
	ether_addr_copy(vif->bssid, addr->sa_data);
	ether_addr_copy(vif->ndev->dev_addr, vif->bssid);

	return result;
}

static void wilc_set_multicast_list(struct net_device *dev)
{
	struct netdev_hw_addr *ha;
	struct wilc_vif *vif = netdev_priv(dev);
	int i;
	u8 *mc_list;
	u8 *cur_mc;

	PRINT_INFO(vif->ndev, INIT_DBG,
		   "Setting mcast List with count = %d.\n", dev->mc.count);
	if (dev->flags & IFF_PROMISC) {
		PRINT_INFO(vif->ndev, INIT_DBG,
			   "Set promiscuous mode ON retrive all pkts\n");
		return;
	}

	if (dev->flags & IFF_ALLMULTI ||
	    dev->mc.count > WILC_MULTICAST_TABLE_SIZE) {
		PRINT_INFO(vif->ndev, INIT_DBG,
			   "Disable mcast filter retrive multicast pkts\n");
		wilc_setup_multicast_filter(vif, 0, 0, NULL);
		return;
	}

	if (dev->mc.count == 0) {
		PRINT_INFO(vif->ndev, INIT_DBG,
			   "Enable mcast filter retrive directed pkts only\n");
		wilc_setup_multicast_filter(vif, 1, 0, NULL);
		return;
	}

	mc_list = kmalloc_array(dev->mc.count, ETH_ALEN, GFP_ATOMIC);
	if (!mc_list)
		return;

	cur_mc = mc_list;
	i = 0;
	netdev_for_each_mc_addr(ha, dev) {
		memcpy(cur_mc, ha->addr, ETH_ALEN);
		PRINT_INFO(vif->ndev, INIT_DBG, "Entry[%d]: %pM\n", i, cur_mc);
		i++;
		cur_mc += ETH_ALEN;
	}

	if (wilc_setup_multicast_filter(vif, 1, dev->mc.count, mc_list))
		kfree(mc_list);
}

static void wilc_tx_complete(void *priv, int status)
{
	struct tx_complete_data *pv_data = priv;

	if (status == 1)
		PRINT_INFO(pv_data->vif->ndev, TX_DBG,
			  "Packet sentSize= %d Add= %p SKB= %p\n",
			  pv_data->size, pv_data->buff, pv_data->skb);
	else
		PRINT_INFO(pv_data->vif->ndev, TX_DBG,
			   "Couldn't send pkt Size= %d Add= %p SKB= %p\n",
			   pv_data->size, pv_data->buff, pv_data->skb);
	dev_kfree_skb(pv_data->skb);
	kfree(pv_data);
}

netdev_tx_t wilc_mac_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct wilc_vif *vif = netdev_priv(ndev);
	struct wilc *wilc = vif->wilc;
	struct tx_complete_data *tx_data = NULL;
	int queue_count;

	PRINT_INFO(vif->ndev, TX_DBG,
		   "Sending packet just received from TCP/IP\n");
	if (skb->dev != ndev) {
		PRINT_ER(ndev, "Packet not destined to this device\n");
		return NETDEV_TX_OK;
	}

	tx_data = kmalloc(sizeof(*tx_data), GFP_ATOMIC);
	if (!tx_data) {
		PRINT_ER(ndev, "Failed to alloc memory for tx_data struct\n");
		dev_kfree_skb(skb);
		netif_wake_queue(ndev);
		return NETDEV_TX_OK;
	}

	tx_data->buff = skb->data;
	tx_data->size = skb->len;
	tx_data->skb  = skb;

	PRINT_D(vif->ndev, TX_DBG, "Sending pkt Size= %d Add= %p SKB= %p\n",
		tx_data->size, tx_data->buff, tx_data->skb);
	PRINT_D(vif->ndev, TX_DBG, "Adding tx pkt to TX Queue\n");
	vif->netstats.tx_packets++;
	vif->netstats.tx_bytes += tx_data->size;
	tx_data->vif = vif;
	queue_count = txq_add_net_pkt(ndev, (void *)tx_data,
				      tx_data->buff, tx_data->size,
				      wilc_tx_complete);

	if (queue_count > FLOW_CTRL_UP_THRESHLD) {
		int i;

		mutex_lock(&wilc->vif_mutex);
		for (i = 0; i < wilc->vif_num; i++) {
			if (wilc->vif[i] && wilc->vif[i]->mac_opened)
				netif_stop_queue(wilc->vif[i]->ndev);
		}
		mutex_unlock(&wilc->vif_mutex);
	}

	return NETDEV_TX_OK;
}

static int wilc_mac_close(struct net_device *ndev)
{
	struct wilc_vif *vif = netdev_priv(ndev);
	struct wilc *wl = vif->wilc;

	PRINT_INFO(vif->ndev, GENERIC_DBG, "Mac close\n");

	if (wl->open_ifcs > 0) {
		wl->open_ifcs--;
	} else {
		PRINT_ER(ndev, "MAC close called with no opened interfaces\n");
		return 0;
	}

	if (vif->ndev) {
		netif_stop_queue(vif->ndev);

	if (!recovery_on)
		wilc_deinit_host_int(vif->ndev);
	}

	if (wl->open_ifcs == 0) {
		PRINT_INFO(ndev, GENERIC_DBG, "Deinitializing wilc\n");
		wl->close = 1;

		wilc_wlan_deinitialize(ndev);
	}

	vif->mac_opened = 0;

	return 0;
}

void wilc_wfi_mgmt_rx(struct wilc *wilc, u8 *buff, u32 size)
{
	int i = 0;
	struct wilc_vif *vif;

	mutex_lock(&wilc->vif_mutex);
	for (i = 0; i < wilc->vif_num; i++) {
		u16 tp = le16_to_cpup((__le16 *)buff);
		struct wilc_priv *priv;

		vif = netdev_priv(wilc->vif[i]->ndev);
		priv = &vif->priv;

		if (((tp == vif->frame_reg[0].type && vif->frame_reg[0].reg) ||
		    (tp == vif->frame_reg[1].type && vif->frame_reg[1].reg)) &&
			    wilc->p2p_listen_state) {
			bool ret;

			ret = wilc_wfi_p2p_rx(vif, buff, size);
			if (ret) {
				mutex_unlock(&wilc->vif_mutex);
				return;
			}
		}

		if (vif->monitor_flag)
			wilc_wfi_monitor_rx(wilc->monitor_dev, buff, size);
	}
	mutex_unlock(&wilc->vif_mutex);
}

static const struct net_device_ops wilc_netdev_ops = {
	.ndo_init = mac_init_fn,
	.ndo_open = wilc_mac_open,
	.ndo_stop = wilc_mac_close,
	.ndo_set_mac_address = wilc_set_mac_addr,
	.ndo_start_xmit = wilc_mac_xmit,
	.ndo_get_stats = mac_stats,
	.ndo_set_rx_mode  = wilc_set_multicast_list,
};

void wilc_netdev_cleanup(struct wilc *wilc)
{
	int i;

	if (!wilc)
		return;

	if (wilc->firmware) {
		release_firmware(wilc->firmware);
		wilc->firmware = NULL;
	}

	for (i = 0; i < wilc->vif_num; i++) {
		if (wilc->vif[i] && wilc->vif[i]->ndev) {
			PRINT_INFO(wilc->vif[i]->ndev, INIT_DBG,
				   "Unregistering netdev %p\n",
				   wilc->vif[i]->ndev);
			unregister_netdev(wilc->vif[i]->ndev);
		}
	}

	wilc_wfi_deinit_mon_interface(wilc, false);

	flush_workqueue(wilc->hif_workqueue);
	destroy_workqueue(wilc->hif_workqueue);
	wilc->hif_workqueue = NULL;
	cfg_deinit(wilc);
#ifdef WILC_DEBUGFS
	wilc_debugfs_remove();
#endif
	wilc_sysfs_exit();
	wlan_deinit_locks(wilc);
	kfree(wilc->bus_data);
	wiphy_unregister(wilc->wiphy);
	pr_info("Freeing wiphy\n");
	wiphy_free(wilc->wiphy);
	pr_info("Module_exit Done.\n");
}


struct wilc_vif *wilc_netdev_ifc_init(struct wilc *wl, const char *name,
				      int iftype, enum nl80211_iftype type,
				      bool rtnl_locked)
{
	struct net_device *ndev;
	struct wilc_vif *vif;
	int ret;

	ndev = alloc_etherdev(sizeof(struct wilc_vif));
	if (!ndev)
		return ERR_PTR(-ENOMEM);

	vif = netdev_priv(ndev);

	ndev->ieee80211_ptr = &vif->priv.wdev;

	vif->wilc = wl;
	vif->ndev = ndev;
	ndev->ml_priv = vif;
	strcpy(ndev->name, name);
	ndev->netdev_ops = &wilc_netdev_ops;

	SET_NETDEV_DEV(ndev, wiphy_dev(wl->wiphy));

	vif->ndev->ml_priv = vif;
	vif->priv.wdev.wiphy = wl->wiphy;
	vif->priv.wdev.netdev = ndev;
	vif->priv.wdev.iftype = type;
	vif->priv.dev = ndev;

	vif->priv.dev = ndev;
	if (rtnl_locked)
		ret = register_netdevice(ndev);
	else
		ret = register_netdev(ndev);

	if (ret) {
		pr_err("Device couldn't be registered - %s\n", ndev->name);
		free_netdev(ndev);
		return ERR_PTR(-EFAULT);
	}
#if KERNEL_VERSION(4, 15, 0) <= LINUX_VERSION_CODE
	ndev->needs_free_netdev = true;
#else
	ndev->destructor = free_netdev;
#endif
	vif->iftype = iftype;
	vif->wilc->vif[wl->vif_num] = vif;
	vif->idx = wl->vif_num;
	wl->vif_num += 1;
	vif->mac_opened = 0;

	return vif;
}

#if KERNEL_VERSION(3, 13, 0) < LINUX_VERSION_CODE
static void wilc_wlan_power(struct wilc *wilc, int power)
{
	struct gpio_desc *gpio_reset;
	struct gpio_desc *gpio_chip_en;

	pr_info("wifi_pm : %d\n", power);

	gpio_reset = gpiod_get(wilc->dt_dev, "reset", GPIOD_ASIS);
	if (IS_ERR(gpio_reset)) {
		dev_warn(wilc->dev, "failed to get Reset GPIO, try default\r\n");
		gpio_reset = gpio_to_desc(GPIO_NUM_RESET);
		if (!gpio_reset) {
			dev_warn(wilc->dev,
				 "failed to get default Reset GPIO\r\n");
			return;
		}
	} else {
		dev_info(wilc->dev, "succesfully got gpio_reset\r\n");
	}

	gpio_chip_en = gpiod_get(wilc->dt_dev, "chip_en", GPIOD_ASIS);
	if (IS_ERR(gpio_chip_en)) {
		gpio_chip_en = gpio_to_desc(GPIO_NUM_CHIP_EN);
		if (!gpio_chip_en) {
			dev_warn(wilc->dev,
				 "failed to get default chip_en GPIO\r\n");
			gpiod_put(gpio_reset);
			return;
		}
	} else {
		dev_info(wilc->dev, "succesfully got gpio_chip_en\r\n");
	}

	if (power) {
		gpiod_direction_output(gpio_chip_en, 1);
		mdelay(5);
		gpiod_direction_output(gpio_reset, 1);
	} else {
		gpiod_direction_output(gpio_reset, 0);
		gpiod_direction_output(gpio_chip_en, 0);
	}
	gpiod_put(gpio_chip_en);
	gpiod_put(gpio_reset);
}
#else
static void wilc_wlan_power(struct wilc *wilc, int power)
{
	int gpio_reset;
	int gpio_chip_en;
	struct device_node *of_node = wilc->dt_dev->of_node;

	pr_info("wifi_pm : %d\n", power);

	gpio_reset = of_get_named_gpio_flags(of_node, "reset-gpios", 0, NULL);

	if (gpio_reset < 0) {
		gpio_reset = GPIO_NUM_RESET;
		pr_info("wifi_pm : load default reset GPIO %d\n", gpio_reset);
	}

	gpio_chip_en = of_get_named_gpio_flags(of_node, "chip_en-gpios", 0,
					       NULL);

	if (gpio_chip_en < 0) {
		gpio_chip_en = GPIO_NUM_CHIP_EN;
		pr_info("wifi_pm : load default chip_en GPIO %d\n",
			gpio_chip_en);
	}

	if (gpio_request(gpio_chip_en, "CHIP_EN") == 0 &&
	    gpio_request(gpio_reset, "RESET") == 0) {
		gpio_direction_output(gpio_chip_en, 0);
		gpio_direction_output(gpio_reset, 0);
		if (power) {
			gpio_set_value(gpio_chip_en, 1);
			mdelay(5);
			gpio_set_value(gpio_reset, 1);
		} else {
			gpio_set_value(gpio_reset, 0);
			gpio_set_value(gpio_chip_en, 0);
		}
		gpio_free(gpio_chip_en);
		gpio_free(gpio_reset);
	} else {
		dev_err(wilc->dev,
			"Error requesting GPIOs for CHIP_EN and RESET");
	}
}
#endif

void wilc_wlan_power_on_sequence(struct wilc *wilc)
{
	wilc_wlan_power(wilc, 0);
	wilc_wlan_power(wilc, 1);
}

void wilc_wlan_power_off_sequence(struct wilc *wilc)
{
	wilc_wlan_power(wilc, 0);
}

MODULE_LICENSE("GPL");
