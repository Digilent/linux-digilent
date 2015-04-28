/*
 * digilent_encoder.c - DRM slave encoder for Video-out on Digilent boards
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

/*
 * Default frame maximums/prefs; can be set in devicetree
 */
#define DIGILENT_ENC_MAX_FREQ 150000  //KHz
#define DIGILENT_ENC_MAX_H 1920
#define DIGILENT_ENC_MAX_V 1080
#define DIGILENT_ENC_PREF_H 1280
#define DIGILENT_ENC_PREF_V 720

struct digilent_encoder {
	struct drm_encoder *encoder;
	struct i2c_adapter *i2c_bus;
   bool i2c_present;
   u32 fmax;
   u32 hmax;
   u32 vmax;
   u32 hpref;
   u32 vpref;
};

static inline struct digilent_encoder *to_digilent_encoder(struct drm_encoder *encoder)
{
	return to_encoder_slave(encoder)->slave_priv;
}

static bool digilent_mode_fixup(struct drm_encoder *encoder,
			   const struct drm_display_mode *mode,
			   struct drm_display_mode *adjusted_mode)
{
	return true;
}

static void digilent_encoder_mode_set(struct drm_encoder *encoder,
				 struct drm_display_mode *mode,
				 struct drm_display_mode *adjusted_mode)
{
}

static void
digilent_encoder_dpms(struct drm_encoder *encoder, int mode)
{
}

static void digilent_encoder_save(struct drm_encoder *encoder)
{
}

static void digilent_encoder_restore(struct drm_encoder *encoder)
{
}

static int digilent_encoder_mode_valid(struct drm_encoder *encoder,
				    struct drm_display_mode *mode)
{
   struct digilent_encoder *digilent = to_digilent_encoder(encoder);
   if (mode && 
      !(mode->flags & ((DRM_MODE_FLAG_INTERLACE | DRM_MODE_FLAG_DBLCLK) | DRM_MODE_FLAG_3D_MASK)) &&
      (mode->clock <= digilent->fmax) &&
      (mode->hdisplay <= digilent->hmax) && 
      (mode->vdisplay <= digilent->vmax)) 
         return MODE_OK;
   return MODE_BAD;
}

static int digilent_encoder_get_modes(struct drm_encoder *encoder,
				   struct drm_connector *connector)
{
   struct digilent_encoder *digilent = to_digilent_encoder(encoder);
	struct edid *edid;
   int num_modes = 0;
   
   if (digilent->i2c_present)
   {
      edid = drm_get_edid(connector, digilent->i2c_bus);

      /*
       *Other drivers tend to call update edid property after the call to 
       *drm_add_edid_modes. If problems with modesetting, this could be why.
       */
      drm_connector_update_edid_property(connector, edid);
      if (edid) 
      {
         num_modes = drm_add_edid_modes(connector, edid);
         kfree(edid);
      }
   }
   else
   {
      num_modes = drm_add_modes_noedid(connector, digilent->hmax, digilent->vmax);
      drm_set_preferred_mode(connector, digilent->hpref, digilent->vpref);
   }   
	return num_modes;
}

static enum drm_connector_status digilent_encoder_detect(struct drm_encoder *encoder,
		     struct drm_connector *connector)
{
   struct digilent_encoder *digilent = to_digilent_encoder(encoder);

   if (digilent->i2c_present)
   {
      if (drm_probe_ddc(digilent->i2c_bus))
         return connector_status_connected;
      return connector_status_disconnected;
   }
   else
      return connector_status_unknown; 
}

static struct drm_encoder_slave_funcs digilent_encoder_slave_funcs = {
	.dpms = digilent_encoder_dpms,
	.save			= digilent_encoder_save,
	.restore		= digilent_encoder_restore,
	.mode_fixup = digilent_mode_fixup,
	.mode_valid		= digilent_encoder_mode_valid,
	.mode_set = digilent_encoder_mode_set,
	.detect			= digilent_encoder_detect,
	.get_modes		= digilent_encoder_get_modes,
};

static int digilent_encoder_encoder_init(struct platform_device *pdev,
				      struct drm_device *dev,
				      struct drm_encoder_slave *encoder)
{
	struct digilent_encoder *digilent = platform_get_drvdata(pdev);
	struct device_node *sub_node;
   int ret;

	encoder->slave_priv = digilent;
	encoder->slave_funcs = &digilent_encoder_slave_funcs;

	digilent->encoder = &encoder->base;

    /* get i2c adapter for edid */
   digilent->i2c_present = false;

	sub_node = of_parse_phandle(pdev->dev.of_node, "digilent,edid-i2c", 0);
	if (sub_node) 
   {
	   digilent->i2c_bus = of_find_i2c_adapter_by_node(sub_node);
      if (!digilent->i2c_bus)
		   DRM_INFO("failed to get the edid i2c adapter, using default modes\n");
      else
         digilent->i2c_present = true;
	   of_node_put(sub_node);
   }

	ret = of_property_read_u32(pdev->dev.of_node, "digilent,fmax", &digilent->fmax);
	if (ret < 0) {
      digilent->fmax = DIGILENT_ENC_MAX_FREQ;
		DRM_INFO("No max frequency in DT, using default %dKHz\n", DIGILENT_ENC_MAX_FREQ);
	}

	ret = of_property_read_u32(pdev->dev.of_node, "digilent,hmax", &digilent->hmax);
	if (ret < 0) {
      digilent->hmax = DIGILENT_ENC_MAX_H;
		DRM_INFO("No max horizontal width in DT, using default %d\n", DIGILENT_ENC_MAX_H);
	}

	ret = of_property_read_u32(pdev->dev.of_node, "digilent,vmax", &digilent->vmax);
	if (ret < 0) {
      digilent->vmax = DIGILENT_ENC_MAX_V;
		DRM_INFO("No max vertical height in DT, using default %d\n", DIGILENT_ENC_MAX_V);
	}

	ret = of_property_read_u32(pdev->dev.of_node, "digilent,hpref", &digilent->hpref);
	if (ret < 0) {
      digilent->hpref = DIGILENT_ENC_PREF_H;
		if (!(digilent->i2c_present))
			DRM_INFO("No pref horizontal width in DT, using default %d\n", DIGILENT_ENC_PREF_H);
	}

	ret = of_property_read_u32(pdev->dev.of_node, "digilent,vpref", &digilent->vpref);
	if (ret < 0) {
      digilent->vpref = DIGILENT_ENC_PREF_V;
		if (!(digilent->i2c_present))
			DRM_INFO("No pref horizontal width in DT, using default %d\n", DIGILENT_ENC_PREF_V);
	}

	return 0;
}

static int digilent_encoder_probe(struct platform_device *pdev)
{
	struct digilent_encoder *digilent;

	digilent = devm_kzalloc(&pdev->dev, sizeof(*digilent), GFP_KERNEL);
	if (!digilent)
		return -ENOMEM;

	platform_set_drvdata(pdev, digilent);

	return 0;
}

static int digilent_encoder_remove(struct platform_device *pdev)
{
	return 0;
}

static const struct of_device_id digilent_encoder_of_match[] = {
	{ .compatible = "digilent,drm-encoder", },
	{ /* end of table */ },
};
MODULE_DEVICE_TABLE(of, digilent_encoder_of_match);

static struct drm_platform_encoder_driver digilent_encoder_driver = {
	.platform_driver = {
		.probe			= digilent_encoder_probe,
		.remove			= digilent_encoder_remove,
		.driver			= {
			.owner		= THIS_MODULE,
			.name		= "digilent-drm-enc",
			.of_match_table	= digilent_encoder_of_match,
		},
	},

	.encoder_init = digilent_encoder_encoder_init,
};

static int __init digilent_encoder_init(void)
{
	return platform_driver_register(&digilent_encoder_driver.platform_driver);
}

static void __exit digilent_encoder_exit(void)
{
	platform_driver_unregister(&digilent_encoder_driver.platform_driver);
}

module_init(digilent_encoder_init);
module_exit(digilent_encoder_exit);

MODULE_AUTHOR("Digilent, Inc.");
MODULE_DESCRIPTION("DRM slave encoder for Video-out on Digilent boards");
MODULE_LICENSE("GPL v2");
