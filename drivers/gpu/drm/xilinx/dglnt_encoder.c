/*
 * dglnt_encoder.c - DRM slave encoder for Video-out on Digilent boards
 *
 * Copyright (C) 2015 Digilent
 * Author: Sam Bobrowicz <sbobrowicz@digilentinc.com>
 *
 * Based on udl_encoder.c and udl_connector.c, Copyright (C) 2012 Red Hat.
 * Also based on xilinx_drm_dp.c, Copyright (C) 2014 Xilinx, Inc.
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 *
 */

#include <drm/drmP.h>
#include <drm/drm_edid.h>
#include <drm/drm_encoder_slave.h>

#include <linux/device.h>
#include <linux/module.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>

#define DGLNT_ENC_MAX_FREQ 150000
#define DGLNT_ENC_MAX_H 1920
#define DGLNT_ENC_MAX_V 1080
#define DGLNT_ENC_PREF_H 1280
#define DGLNT_ENC_PREF_V 720

struct dglnt_encoder {
	struct drm_encoder *encoder;
	struct i2c_adapter *i2c_bus;
   bool i2c_present;
};

static inline struct dglnt_encoder *to_dglnt_encoder(struct drm_encoder *encoder)
{
	return to_encoder_slave(encoder)->slave_priv;
}

static bool dglnt_mode_fixup(struct drm_encoder *encoder,
			   const struct drm_display_mode *mode,
			   struct drm_display_mode *adjusted_mode)
{
	return true;
}

static void dglnt_encoder_mode_set(struct drm_encoder *encoder,
				 struct drm_display_mode *mode,
				 struct drm_display_mode *adjusted_mode)
{
}

static void
dglnt_encoder_dpms(struct drm_encoder *encoder, int mode)
{
}

static void dglnt_encoder_save(struct drm_encoder *encoder)
{
}

static void dglnt_encoder_restore(struct drm_encoder *encoder)
{
}

static int dglnt_encoder_mode_valid(struct drm_encoder *encoder,
				    struct drm_display_mode *mode)
{
   if (mode && 
      !(mode->flags & ((DRM_MODE_FLAG_INTERLACE | DRM_MODE_FLAG_DBLCLK) | DRM_MODE_FLAG_3D_MASK)) &&
      (mode->clock <= DGLNT_ENC_MAX_FREQ) &&
      (mode->hdisplay <= DGLNT_ENC_MAX_H) && 
      (mode->vdisplay <= DGLNT_ENC_MAX_V)) 
         return MODE_OK;
   return MODE_BAD;
}

static int dglnt_encoder_get_modes(struct drm_encoder *encoder,
				   struct drm_connector *connector)
{
   struct dglnt_encoder *dglnt = to_dglnt_encoder(encoder);
	struct edid *edid;
   int num_modes = 0;
   
   if (dglnt->i2c_present)
   {
      edid = drm_get_edid(connector, dglnt->i2c_bus);
      drm_mode_connector_update_edid_property(connector, edid);
      if (edid) 
      {
         num_modes = drm_add_edid_modes(connector, edid);
         kfree(edid);
      }
   }
   else
   {
      num_modes = drm_add_modes_noedid(connector, DGLNT_ENC_MAX_H, DGLNT_ENC_MAX_V);
      drm_set_preferred_mode(connector, DGLNT_ENC_PREF_H, DGLNT_ENC_PREF_V);
   }   
	return num_modes;
}

static enum drm_connector_status dglnt_encoder_detect(struct drm_encoder *encoder,
		     struct drm_connector *connector)
{
   struct dglnt_encoder *dglnt = to_dglnt_encoder(encoder);

   if (dglnt->i2c_present)
   {
      if (drm_probe_ddc(dglnt->i2c_bus))
         return connector_status_connected;
      return connector_status_disconnected;
   }
   else
      return connector_status_unknown; 
}

static struct drm_encoder_slave_funcs dglnt_encoder_slave_funcs = {
	.dpms = dglnt_encoder_dpms,
	.save			= dglnt_encoder_save,
	.restore		= dglnt_encoder_restore,
	.mode_fixup = dglnt_mode_fixup,
	.mode_valid		= dglnt_encoder_mode_valid,
	.mode_set = dglnt_encoder_mode_set,
	.detect			= dglnt_encoder_detect,
	.get_modes		= dglnt_encoder_get_modes,
};

static int dglnt_encoder_encoder_init(struct platform_device *pdev,
				      struct drm_device *dev,
				      struct drm_encoder_slave *encoder)
{
	struct dglnt_encoder *dglnt = platform_get_drvdata(pdev);
	struct device_node *sub_node;

	encoder->slave_priv = dglnt;
	encoder->slave_funcs = &dglnt_encoder_slave_funcs;

	dglnt->encoder = &encoder->base;

    /* get i2c adapter for edid */
   dglnt->i2c_present = false;
	sub_node = of_parse_phandle(pdev->dev.of_node, "dglnt,edid-i2c", 0);
	if (sub_node) 
   {
	   dglnt->i2c_bus = of_find_i2c_adapter_by_node(sub_node);
      if (!dglnt->i2c_bus)
		   DRM_INFO("failed to get the edid i2c adapter, using default modes\n");
      else
         dglnt->i2c_present = true;
	   of_node_put(sub_node);
   }

	return 0;
}

static int dglnt_encoder_probe(struct platform_device *pdev)
{
	struct dglnt_encoder *dglnt;

	dglnt = devm_kzalloc(&pdev->dev, sizeof(*dglnt), GFP_KERNEL);
	if (!dglnt)
		return -ENOMEM;

	platform_set_drvdata(pdev, dglnt);

	return 0;
}

static int dglnt_encoder_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id dglnt_encoder_of_match[] = {
	{ .compatible = "dglnt,drm-encoder", },
	{ /* end of table */ },
};
MODULE_DEVICE_TABLE(of, dglnt_encoder_of_match);

static struct drm_platform_encoder_driver dglnt_encoder_driver = {
	.platform_driver = {
		.probe			= dglnt_encoder_probe,
		.remove			= dglnt_encoder_remove,
		.driver			= {
			.owner		= THIS_MODULE,
			.name		= "dglnt-drm-enc",
			.of_match_table	= dglnt_encoder_of_match,
		},
	},

	.encoder_init = dglnt_encoder_encoder_init,
};

static int __init dglnt_encoder_init(void)
{
	return platform_driver_register(&dglnt_encoder_driver.platform_driver);
}

static void __exit dglnt_encoder_exit(void)
{
	platform_driver_unregister(&dglnt_encoder_driver.platform_driver);
}

module_init(dglnt_encoder_init);
module_exit(dglnt_encoder_exit);

MODULE_AUTHOR("Digilent, Inc.");
MODULE_DESCRIPTION("DRM slave encoder for Video-out on Digilent boards");
MODULE_LICENSE("GPL v2");
