/*
 *  Copyright (C) 2012-2013, Analog Devices Inc.
 *	Author: Lars-Peter Clausen <lars@metafoo.de>
 *
 * Licensed under the GPL-2.
 */

/*
 *  This driver is taken from:
 *	https://ez.analog.com/thread/80986?start=0&tstart=0
 *  It has modification to work on Digilient Zybo linux_bd design or HW
 *  that uses 12.288MHz as MCLK to SSM2603 codec.
 *
 *  Known issues:
 *	* 44100Hz is not supported due to fixed clock.
 *	* cancel while recording or playback may cause playback issue.
 *
 */

#include <linux/module.h>
#include <linux/timer.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include "../codecs/ssm2602.h"

static int zybo_ssm2603_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	unsigned int pll_rate;
	int ret;

	switch (params_rate(params)) {
	case 8000:
	case 12000:
	case 16000:
	case 24000:
	case 32000:
	case 48000:
	case 96000:
		pll_rate  = 12288000;
		break;
	default:
		return -EINVAL;
	}

	ret = snd_soc_dai_set_sysclk(codec_dai, SSM2602_SYSCLK, pll_rate,
			SND_SOC_CLOCK_IN);

	return ret;
}

static struct snd_soc_ops zybo_ssm2603_ops = {
	.hw_params = zybo_ssm2603_hw_params,
};

static struct snd_soc_dai_link zybo_ssm2603_dai_link = {
	.name = "ssm2602",
	.stream_name = "ssm2602",
	.codec_dai_name = "ssm2602-hifi",
	.dai_fmt = SND_SOC_DAIFMT_I2S |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
	.ops = &zybo_ssm2603_ops,
};

static struct snd_soc_card zybo_ssm2603_card = {
	.name = "ZYBO SSM2603",
	.owner = THIS_MODULE,
	.dai_link = &zybo_ssm2603_dai_link,
	.num_links = 1,
};

static int zybo_ssm2603_probe(struct platform_device *pdev)
{
	struct snd_soc_card *card = &zybo_ssm2603_card;
	struct device_node *of_node = pdev->dev.of_node;

	if (!of_node)
		return -ENXIO;

	card->dev = &pdev->dev;

	zybo_ssm2603_dai_link.codec_of_node = of_parse_phandle(of_node,
						"audio-codec", 0);
	zybo_ssm2603_dai_link.cpu_of_node = of_parse_phandle(of_node,
						"cpu-dai", 0);
	zybo_ssm2603_dai_link.platform_of_node =
					zybo_ssm2603_dai_link.cpu_of_node;

	if (!zybo_ssm2603_dai_link.codec_of_node ||
		!zybo_ssm2603_dai_link.cpu_of_node)
		return -ENXIO;

	return snd_soc_register_card(card);
}

static int zybo_ssm2603_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);

	snd_soc_unregister_card(card);

	return 0;
}

static const struct of_device_id zybo_ssm2603_of_match[] = {
	{ .compatible = "digilent,zybo-sound", },
	{},
};
MODULE_DEVICE_TABLE(of, zybo_ssm2603_of_match);

static struct platform_driver zybo_ssm2603_card_driver = {
	.driver = {
		.name = "zybo-ssm2603-snd",
		.owner = THIS_MODULE,
		.of_match_table = zybo_ssm2603_of_match,
		.pm = &snd_soc_pm_ops,
	},
	.probe = zybo_ssm2603_probe,
	.remove = zybo_ssm2603_remove,
};
module_platform_driver(zybo_ssm2603_card_driver);

MODULE_DESCRIPTION("ASoC ZYBO board SSM2603 driver");
MODULE_AUTHOR("origin Lars-Peter Clausen <lars@metafoo.de>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:zybo-ssm2603-snd");
