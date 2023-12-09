/*
 * es8388.c -- es8388 ALSA SoC audio driver
 *
 * Copyright (c) 2022 Rockchip Electronics Co. Ltd.
 *
 * Author: Will <will@everset-semi.com>
 * Author: Jianqun Xu <jay.xu@rock-chips.com>
 * Author: Nickey Yang <nickey.yang@rock-chips.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/regmap.h>
#include "es8388.h"

#define INVALID_GPIO -1

#define ES8388_CODEC_SET_SPK	1
#define ES8388_CODEC_SET_HP	2

/* codec private data */
struct es8388_priv {
	unsigned int sysclk;
	struct clk *mclk;
	struct regmap *regmap;
	int spk_ctl_gpio;
	int hp_det_gpio;
	struct i2c_client *i2c;

	bool muted;
	bool hp_inserted;
	bool spk_gpio_level;
	bool hp_det_level;

	struct delayed_work pcm_pop_work;
};

static struct es8388_priv *es8388_private;
static int es8388_set_gpio(int gpio, bool level)
{
	struct es8388_priv *es8388 = es8388_private;

	if (!es8388)
		return 0;

	// if ((gpio & ES8388_CODEC_SET_SPK) && es8388
	//     && es8388->spk_ctl_gpio != INVALID_GPIO) {
	// 	gpio_set_value(es8388->spk_ctl_gpio, level);
	// }

	return 0;
}
/*
static irqreturn_t hp_det_irq_handler(int irq, void *dev_id)
{
	struct es8388_priv *es8388 = es8388_private;

	if (gpio_get_value(es8388->hp_det_gpio))
		es8388->hp_inserted = 0;
	else
		es8388->hp_inserted = 1;

	if (es8388->muted == 0) {
		if (es8388->hp_det_level != es8388->hp_inserted)
			es8388_set_gpio(ES8388_CODEC_SET_SPK, !es8388->spk_gpio_level);
		else
			es8388_set_gpio(ES8388_CODEC_SET_SPK, es8388->spk_gpio_level);
	}
	return IRQ_HANDLED;
}
*/
static void pcm_pop_work_events(struct work_struct *work)
{
	struct es8388_priv *es8388 =
		container_of(work, struct es8388_priv, pcm_pop_work.work);
	/* open dac output here */
	regmap_write(es8388->regmap, 0x04, 0x3c);
}

/* dapm controls */
static const char *stereo_3d_txt[] = { "No 3D  ", "Level 1", "Level 2", "Level 3",
	"Level 4", "Level 5", "Level 6", "Level 7" };
static SOC_ENUM_SINGLE_DECL(stereo_3d_func,
			    ES8388_DACCONTROL7, 2, stereo_3d_txt);

static const char *const alc_func_txt[] = { "Off", "Right", "Left", "Stereo" };
static SOC_ENUM_SINGLE_DECL(alc_func,
			    ES8388_ADCCONTROL10, 6, alc_func_txt);
			    
static const char *const datasel_txt[] = { "DS=DSL_diffinput", "DSL=DSL_diffinput DSR=DSR_diffinput" };
static SOC_ENUM_SINGLE_DECL(datasel,
			    ES8388_ADCCONTROL2, 3, datasel_txt);
			    
static const char *const diffsel_txt[] = { "diff=lin1-rin1", "diff=lin2-rin2" };
static SOC_ENUM_SINGLE_DECL(dslsel,
			    ES8388_ADCCONTROL2, 2, diffsel_txt);			    
static SOC_ENUM_SINGLE_DECL(dsrsel,
			    ES8388_ADCCONTROL3, 7, diffsel_txt);

static const char *const ng_type_txt[] = { "Constant PGA Gain", "Mute ADC Output" };
static SOC_ENUM_SINGLE_DECL(alc_ng_type,
			    ES8388_ADCCONTROL14, 1, ng_type_txt);
			    
static const char *deemph_txt[] = {"None", "32Khz", "44.1Khz", "48Khz"};
static SOC_ENUM_SINGLE_DECL(deemph,
			    ES8388_DACCONTROL6, 6, deemph_txt);
			    
static const char *adcpol_txt[] = {"Normal", "L Invert", "R Invert",
				   "L + R Invert"};
static SOC_ENUM_SINGLE_DECL(adcpol,
			    ES8388_ADCCONTROL6, 6, adcpol_txt);			    

static const char *left_adc_mux_txt[] = { "Line 1L", "Line 2L", "NC", "DifferentialL"};
static SOC_ENUM_SINGLE_DECL(left_adc_mux,
			    ES8388_ADCCONTROL2, 6, left_adc_mux_txt);
static const struct snd_kcontrol_new es8388_left_adc_mux_controls =
	SOC_DAPM_ENUM("Route", left_adc_mux);

static const char *right_adc_mux_txt[] = { "Line 1R", "Line 2R", "NC", "DifferentialR" };
static SOC_ENUM_SINGLE_DECL(right_adc_mux,
			    ES8388_ADCCONTROL2, 4, right_adc_mux_txt);
static const struct snd_kcontrol_new es8388_right_adc_mux_controls =
	SOC_DAPM_ENUM("Route", right_adc_mux);	

static SOC_ENUM_SINGLE_DECL(es8388_diff_enum, ES8388_ADCCONTROL3, 7, diffsel_txt);
static const struct snd_kcontrol_new
	es8388_diffmux_controls = SOC_DAPM_ENUM("Route", es8388_diff_enum);
	
static const char *const es8388_mono_mux[] = {
	"Stereo", "Mono (Left)", "Mono (Right)" };	
static SOC_ENUM_SINGLE_DECL(es8388_mono_enum, ES8388_ADCCONTROL3, 3, es8388_mono_mux);
static const struct snd_kcontrol_new es8388_monomux_controls =
SOC_DAPM_ENUM("Route", es8388_mono_enum);

static const char *const es8388_lin_sell[] = {"Line 1L", "Line 2L", "NC", "left ADC_P", "left ADC_N"};
static SOC_ENUM_SINGLE_DECL(es8388_llin_enum, ES8388_DACCONTROL16, 3, es8388_lin_sell);	
static const struct snd_kcontrol_new es8388_left_line_controls =
SOC_DAPM_ENUM("Route", es8388_llin_enum);

static const char *const es8388_lin_selr[] = {"Line 1R", "Line 2R", "NC", "Right ADC_P", "Right ADC_N"};
static SOC_ENUM_SINGLE_DECL(es8388_rlin_enum, ES8388_DACCONTROL16, 0, es8388_lin_selr);
static const struct snd_kcontrol_new es8388_right_line_controls =
SOC_DAPM_ENUM("Route", es8388_rlin_enum);

static const DECLARE_TLV_DB_SCALE(adc_tlv, -9600, 50, 1);
static const DECLARE_TLV_DB_SCALE(dac_tlv, -9600, 50, 1);
static const DECLARE_TLV_DB_SCALE(out_tlv, -4500, 150, 0);
static const DECLARE_TLV_DB_SCALE(pgagain_tlv, 0, 300, 0);
static const DECLARE_TLV_DB_SCALE(bypass_tlv, -15, 300, 0);

static const struct snd_kcontrol_new es8388_snd_controls[] = {
	SOC_ENUM("3D Mode", stereo_3d_func),
	SOC_ENUM("DifferentialL Data", datasel),
	SOC_ENUM("DSL inputsel", dslsel),
	SOC_ENUM("DSR inputsel", dsrsel),
	SOC_ENUM("ALC Capture Function", alc_func),
	SOC_SINGLE("ALC Capture Target Volume", ES8388_ADCCONTROL11, 4, 15, 0),
	SOC_SINGLE("ALC Capture Max PGA", ES8388_ADCCONTROL10, 3, 7, 0),
	SOC_SINGLE("ALC Capture Min PGA", ES8388_ADCCONTROL10, 0, 7, 0),
	SOC_SINGLE("ALC Capture ZC Switch", ES8388_ADCCONTROL13, 6, 1, 0),
	SOC_SINGLE("ALC Capture Hold Time", ES8388_ADCCONTROL11, 0, 15, 0),
	SOC_SINGLE("ALC Capture Decay Time", ES8388_ADCCONTROL12, 4, 15, 0),
	SOC_SINGLE("ALC Capture Attack Time", ES8388_ADCCONTROL12, 0, 15, 0),
	SOC_SINGLE("ALC Capture NG Threshold", ES8388_ADCCONTROL14, 3, 31, 0),
	SOC_ENUM("ALC Capture NG Type", alc_ng_type),
	SOC_SINGLE("ALC Capture NG Switch", ES8388_ADCCONTROL14, 0, 1, 0),
	SOC_SINGLE("ZC Timeout Switch", ES8388_ADCCONTROL13, 6, 1, 0),
	SOC_DOUBLE_R_TLV("Capture Digital Volume", ES8388_ADCCONTROL8,
			 ES8388_ADCCONTROL9, 0, 192, 1, adc_tlv),
	SOC_SINGLE("Capture Mute", ES8388_ADCCONTROL7, 2, 1, 0),
	SOC_SINGLE_TLV("Left channel PGA gain", ES8388_ADCCONTROL1, 4, 8,
		       0, pgagain_tlv),
	SOC_SINGLE_TLV("Right channel PGA gain", ES8388_ADCCONTROL1, 0,
		       8, 0, pgagain_tlv),
	SOC_ENUM("Playback De-emphasis", deemph),
	SOC_ENUM("Capture Polarity", adcpol),
	SOC_DOUBLE_R_TLV("Playback Digital Volume", ES8388_DACCONTROL4, ES8388_DACCONTROL5,
			 0, 192, 1, dac_tlv),
	SOC_SINGLE_TLV("Left Mixer Left Bypass Volume", ES8388_DACCONTROL17, 3,
		       7, 1, bypass_tlv),
	SOC_SINGLE_TLV("Right Mixer Right Bypass Volume", ES8388_DACCONTROL20,
		       3, 7, 1, bypass_tlv),
	SOC_DOUBLE_R_TLV("Output 1 Playback Volume", ES8388_DACCONTROL24,
			 ES8388_DACCONTROL25, 0, 33, 0, out_tlv),
	SOC_DOUBLE_R_TLV("Output 2 Playback Volume", ES8388_DACCONTROL26,
			 ES8388_DACCONTROL27, 0, 33, 0, out_tlv),
};

/* Left Mixer */
static const struct snd_kcontrol_new es8388_left_mixer_controls[] = {
	SOC_DAPM_SINGLE("Left Playback Switch", ES8388_DACCONTROL17, 7, 1, 0),
	SOC_DAPM_SINGLE("Left Bypass Switch", ES8388_DACCONTROL17, 6, 1, 0),
};

/* Right Mixer */
static const struct snd_kcontrol_new es8388_right_mixer_controls[] = {
	SOC_DAPM_SINGLE("Right Playback Switch", ES8388_DACCONTROL20, 7, 1, 0),
	SOC_DAPM_SINGLE("Right Bypass Switch", ES8388_DACCONTROL20, 6, 1, 0),
};

static const struct snd_soc_dapm_widget es8388_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("Diff lin1-rin1"),
	SND_SOC_DAPM_INPUT("Diff lin2-rin2"),
	SND_SOC_DAPM_INPUT("LINPUT2"),
	SND_SOC_DAPM_INPUT("LINPUT1"),
	SND_SOC_DAPM_INPUT("RINPUT1"),
	SND_SOC_DAPM_INPUT("RINPUT2"),
	SND_SOC_DAPM_MUX("Left PGA Mux", SND_SOC_NOPM, 0, 0,
			 &es8388_left_adc_mux_controls),
	SND_SOC_DAPM_MUX("Right PGA Mux", SND_SOC_NOPM, 0, 0,
			 &es8388_right_adc_mux_controls),
	SND_SOC_DAPM_MICBIAS("Mic Bias", ES8388_ADCPOWER, 3, 1),

	SND_SOC_DAPM_MUX("Differential Mux", SND_SOC_NOPM, 0, 0,
			 &es8388_diffmux_controls),

	SND_SOC_DAPM_MUX("Left ADC Mux", SND_SOC_NOPM, 0, 0,
			 &es8388_monomux_controls),
	SND_SOC_DAPM_MUX("Right ADC Mux", SND_SOC_NOPM, 0, 0,
			 &es8388_monomux_controls),

	SND_SOC_DAPM_MUX("Left Line Mux", SND_SOC_NOPM, 0, 0,
			 &es8388_left_line_controls),
	SND_SOC_DAPM_MUX("Right Line Mux", SND_SOC_NOPM, 0, 0,
			 &es8388_right_line_controls),
	
	SND_SOC_DAPM_ADC("Right ADC", "Right Capture", ES8388_ADCPOWER, 4, 1),
	SND_SOC_DAPM_ADC("Left ADC", "Left Capture", ES8388_ADCPOWER, 5, 1),

	SND_SOC_DAPM_DAC("Right DAC", "Right Playback", SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_DAC("Left DAC", "Left Playback", SND_SOC_NOPM, 0, 0),

	SND_SOC_DAPM_MIXER("Left Mixer", SND_SOC_NOPM, 0, 0,
			   &es8388_left_mixer_controls[0],
			   ARRAY_SIZE(es8388_left_mixer_controls)),
	SND_SOC_DAPM_MIXER("Right Mixer", SND_SOC_NOPM, 0, 0,
			   &es8388_right_mixer_controls[0],
			   ARRAY_SIZE(es8388_right_mixer_controls)),
	SND_SOC_DAPM_PGA("Right ADC Power", ES8388_ADCPOWER, 6, 1, NULL, 0),
	SND_SOC_DAPM_PGA("Left ADC Power", ES8388_ADCPOWER, 7, 1, NULL, 0),
	SND_SOC_DAPM_PGA("Right Out 2", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Left Out 2", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Right Out 1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Left Out 1", SND_SOC_NOPM, 0, 0, NULL, 0),
	SND_SOC_DAPM_PGA("LAMP", ES8388_ADCCONTROL1, 4, 0, NULL, 0),
	SND_SOC_DAPM_PGA("RAMP", ES8388_ADCCONTROL1, 0, 0, NULL, 0),

	SND_SOC_DAPM_OUTPUT("LOUT1"),
	SND_SOC_DAPM_OUTPUT("ROUT1"),
	SND_SOC_DAPM_OUTPUT("LOUT2"),
	SND_SOC_DAPM_OUTPUT("ROUT2"),
	SND_SOC_DAPM_OUTPUT("VREF"),
};

static const struct snd_soc_dapm_route audio_map[] = {

	{"Left PGA Mux", "Line 1L", "LINPUT1"},
	{"Left PGA Mux", "Line 2L", "LINPUT2"},
	{"Left PGA Mux", "DifferentialL", "Differential Mux"},

	{"Right PGA Mux", "Line 1R", "RINPUT1"},
	{"Right PGA Mux", "Line 2R", "RINPUT2"},
	{"Right PGA Mux", "DifferentialR", "Differential Mux"},

	{"Differential Mux", "diff=lin1-rin1", "LINPUT1"},
	{"Differential Mux", "diff=lin1-rin1", "RINPUT1"},
	{"Differential Mux", "diff=lin2-rin2", "LINPUT2"},
	{"Differential Mux", "diff=lin2-rin2", "RINPUT2"},

	{"Left ADC Mux", "Stereo", "Right PGA Mux"},
	{"Left ADC Mux", "Stereo", "Left PGA Mux"},
	{"Left ADC Mux", "Mono (Left)", "Left PGA Mux"},

	{"Right ADC Mux", "Stereo", "Left PGA Mux"},
	{"Right ADC Mux", "Stereo", "Right PGA Mux"},
	{"Right ADC Mux", "Mono (Right)", "Right PGA Mux"},

	{"Left ADC Power", NULL, "Left ADC Mux"},
	{"Right ADC Power", NULL, "Right ADC Mux"},
	{"Left ADC", NULL, "Left ADC Power"},
	{"Right ADC", NULL, "Right ADC Power"},
	
	{"Left Line Mux", "Line 1L", "LINPUT1"},
	{"Left Line Mux", "Line 2L", "LINPUT2"},
	{"Left Line Mux", "left ADC_P", "Left PGA Mux"},
	{"Left Line Mux", "left ADC_N", "Left PGA Mux"},
	
	{"Right Line Mux", "Line 1R", "RINPUT1"},
	{"Right Line Mux", "Line 2R", "RINPUT2"},
	{"Right Line Mux", "Right ADC_P", "Right PGA Mux"},
	{"Right Line Mux", "Right ADC_N", "Right PGA Mux"},
	
	{"Left Mixer", "Left Playback Switch", "Left DAC"},
	{"Left Mixer", "Left Bypass Switch", "Left Line Mux"},

	{"Right Mixer", "Right Playback Switch", "Right DAC"},
	{"Right Mixer", "Right Bypass Switch", "Right Line Mux"},
	
	{"Left Out 1", NULL, "Left Mixer"},
	{"LOUT1", NULL, "Left Out 1"},
	{"Right Out 1", NULL, "Right Mixer"},
	{"ROUT1", NULL, "Right Out 1"},

	{"Left Out 2", NULL, "Left Mixer"},
	{"LOUT2", NULL, "Left Out 2"},
	{"Right Out 2", NULL, "Right Mixer"},
	{"ROUT2", NULL, "Right Out 2"},
	
};

struct _coeff_div {
	u32 mclk;
	u32 rate;
	u16 fs;
	u8 sr:4;
	u8 usb:1;
};

/* codec hifi mclk clock divider coefficients */
static const struct _coeff_div coeff_div[] = {
	/* 8k */
	{12288000, 8000, 1536, 0xa, 0x0},
	{11289600, 8000, 1408, 0x9, 0x0},
	{18432000, 8000, 2304, 0xc, 0x0},
	{16934400, 8000, 2112, 0xb, 0x0},
	{12000000, 8000, 1500, 0xb, 0x1},

	/* 11.025k */
	{11289600, 11025, 1024, 0x7, 0x0},
	{16934400, 11025, 1536, 0xa, 0x0},
	{12000000, 11025, 1088, 0x9, 0x1},

	/* 16k */
	{12288000, 16000, 768, 0x6, 0x0},
	{18432000, 16000, 1152, 0x8, 0x0},
	{12000000, 16000, 750, 0x7, 0x1},

	/* 22.05k */
	{11289600, 22050, 512, 0x4, 0x0},
	{16934400, 22050, 768, 0x6, 0x0},
	{12000000, 22050, 544, 0x6, 0x1},

	/* 32k */
	{12288000, 32000, 384, 0x3, 0x0},
	{18432000, 32000, 576, 0x5, 0x0},
	{12000000, 32000, 375, 0x4, 0x1},

	/* 44.1k */
	{11289600, 44100, 256, 0x2, 0x0},
	{16934400, 44100, 384, 0x3, 0x0},
	{12000000, 44100, 272, 0x3, 0x1},

	/* 48k */
	{12288000, 48000, 256, 0x2, 0x0},
	{18432000, 48000, 384, 0x3, 0x0},
	{12000000, 48000, 250, 0x2, 0x1},

	/* 88.2k */
	{11289600, 88200, 128, 0x0, 0x0},
	{16934400, 88200, 192, 0x1, 0x0},
	{12000000, 88200, 136, 0x1, 0x1},

	/* 96k */
	{12288000, 96000, 128, 0x0, 0x0},
	{18432000, 96000, 192, 0x1, 0x0},
	{12000000, 96000, 125, 0x0, 0x1},
};

static inline int get_coeff(int mclk, int rate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(coeff_div); i++) {
		if (coeff_div[i].rate == rate && coeff_div[i].mclk == mclk)
			return i;
	}

	return -EINVAL;
}

/*
 * Note that this should be called from init rather than from hw_params.
 */
static int es8388_set_dai_sysclk(struct snd_soc_dai *codec_dai,
				 int clk_id, unsigned int freq, int dir)
{	
//	struct snd_soc_codec *codec = codec_dai->codec;
//	struct es8388_priv *es8388 = snd_soc_codec_get_drvdata(codec);

	struct snd_soc_component *component = codec_dai->component;
	struct es8388_priv *es8388 = snd_soc_component_get_drvdata(component);

	es8388->sysclk = freq;

	return 0;
}

static int es8388_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int fmt)
{	
	struct snd_soc_component *component = codec_dai->component;
	struct es8388_priv *es8388 = snd_soc_component_get_drvdata(component);

	u8 iface = 0;
	u8 adciface = 0;
	u8 daciface = 0;

	iface=snd_soc_component_read(component, ES8388_IFACE);
	adciface=snd_soc_component_read(component, ES8388_ADC_IFACE);
	daciface=snd_soc_component_read(component, ES8388_DAC_IFACE);

//	iface = snd_soc_read(codec, ES8388_IFACE);
//	adciface = snd_soc_read(codec, ES8388_ADC_IFACE);
//	daciface = snd_soc_read(codec, ES8388_DAC_IFACE);

	/* set master/slave audio interface */
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBM_CFM:	/* MASTER MODE */
		iface |= 0x80;
		break;
	case SND_SOC_DAIFMT_CBS_CFS:	/* SLAVE MODE */
		iface &= 0x7F;
		break;
	default:
		return -EINVAL;
	}
	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		adciface &= 0xFC;
		daciface &= 0xF9;
		break;
	case SND_SOC_DAIFMT_RIGHT_J:
		adciface &= 0xFC;
		adciface |= 0x02;
		daciface &= 0xF9;
		daciface |= 0x04;
		break;
	case SND_SOC_DAIFMT_LEFT_J:
		adciface &= 0xFC;
		adciface |= 0x01;
		daciface &= 0xF9;
		daciface |= 0x02;
		break;
	case SND_SOC_DAIFMT_DSP_A:
		adciface &= 0xDC;
		adciface |= 0x03;
		daciface &= 0xB9;
		daciface |= 0x06;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		adciface &= 0xDC;
		adciface |= 0x23;
		daciface &= 0xB9;
		daciface |= 0x46;
		break;
	default:
		return -EINVAL;
	}

	/* clock inversion */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		iface &= 0xDF;
		adciface &= 0xDF;
		daciface &= 0xBF;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		iface |= 0x20;
		adciface |= 0x20;
		daciface |= 0x40;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		iface |= 0x20;
		adciface &= 0xDF;
		daciface &= 0xBF;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		iface &= 0xDF;
		adciface |= 0x20;
		daciface |= 0x40;
		break;
	default:
		return -EINVAL;
	}

//	snd_soc_write(codec, ES8388_IFACE, iface);
//	snd_soc_write(codec, ES8388_ADC_IFACE, adciface);
//	snd_soc_write(codec, ES8388_DAC_IFACE, daciface);

	snd_soc_component_write(component, ES8388_IFACE, iface);
	snd_soc_component_write(component, ES8388_ADC_IFACE, adciface);
	snd_soc_component_write(component, ES8388_DAC_IFACE, daciface);

	return 0;
}

static int es8388_pcm_startup(struct snd_pcm_substream *substream,
			      struct snd_soc_dai *dai)
{
//	struct snd_soc_codec *codec = dai->codec;
//	struct es8388_priv *es8388 = snd_soc_codec_get_drvdata(codec);
	/* The codec needs mclk/es8388->sysclk value to configure registers.
	 * remove the following code and change the code in es8388_pcm_hw_params
	 * according to the mclk configuration.
	 */
	// if (!es8388->sysclk) {
	// 	dev_err(codec->dev,
	// 	"No MCLK configured, configure es8388->sysclk in es8388_pcm_hw_params!\n");
	// 	return -EINVAL;
	// }
	return 0;
}

static int es8388_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;

	struct snd_soc_component *component = dai->component;
	struct es8388_priv *es8388 = snd_soc_component_get_drvdata(component);

//	struct snd_soc_codec *codec = rtd->codec;
//	struct es8388_priv *es8388 = snd_soc_codec_get_drvdata(codec);
//	u16 srate = snd_soc_read(codec, ES8388_IFACE) & 0x80;
//	u16 adciface = snd_soc_read(codec, ES8388_ADC_IFACE) & 0xE3;
//	u16 daciface = snd_soc_read(codec, ES8388_DAC_IFACE) & 0xC7;

	u16 srate=snd_soc_component_read(component, ES8388_IFACE) & 0x80;
	u16 adciface=snd_soc_component_read(component, ES8388_ADC_IFACE) & 0xE3;
	u16 daciface=snd_soc_component_read(component, ES8388_DAC_IFACE) & 0xC7;

	int coeff;
	static bool first = true;

	/* If the es8388->sysclk is a fixed value, for example 12.288M,
	 * set es8388->sysclk = 12288000;
	 * else if es8388->sysclk is some value times lrck, for example 128Fs,
	 * set es8388->sysclk = 128 * params_rate(params);
	 */
	coeff = get_coeff(es8388->sysclk, params_rate(params));
	if (coeff < 0) {
		coeff = get_coeff(es8388->sysclk / 2, params_rate(params));
		srate |= 0x40;
	}
	if (coeff < 0) {
		dev_err(component->dev,
			"Unable to configure sample rate %dHz with %dHz MCLK\n",
			params_rate(params), es8388->sysclk);
		return coeff;
	}

	/* bit size */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16_LE:
		adciface |= 0x000C;
		daciface |= 0x0018;
		break;
	case SNDRV_PCM_FORMAT_S20_3LE:
		adciface |= 0x0004;
		daciface |= 0x0008;
		break;
	case SNDRV_PCM_FORMAT_S24_LE:
		break;
	case SNDRV_PCM_FORMAT_S32_LE:
		adciface |= 0x0010;
		daciface |= 0x0020;
		break;
	}

	/* set iface & srate */
//	snd_soc_write(codec, ES8388_DAC_IFACE, daciface);
//	snd_soc_write(codec, ES8388_ADC_IFACE, adciface);
	snd_soc_component_write(component, ES8388_DAC_IFACE, daciface);
	snd_soc_component_write(component, ES8388_ADC_IFACE, adciface);

	if (coeff >= 0) {
//		snd_soc_write(codec, ES8388_IFACE, srate);
//		snd_soc_write(codec, ES8388_ADCCONTROL5,
//			      coeff_div[coeff].sr | (coeff_div[coeff].
//						     usb) << 4);
//		snd_soc_write(codec, ES8388_DACCONTROL2,
//			      coeff_div[coeff].sr | (coeff_div[coeff].
//						     usb) << 4);
		snd_soc_component_write(component, ES8388_IFACE, srate);
		snd_soc_component_write(component, ES8388_ADCCONTROL5,
				  coeff_div[coeff].sr | (coeff_div[coeff].
							 usb) << 4);
		snd_soc_component_write(component, ES8388_DACCONTROL2,
				  coeff_div[coeff].sr | (coeff_div[coeff].
							 usb) << 4);							 
	}

	/* 8k lrck needs special attention */
	if ((es8388->sysclk/params_rate(params) == 256) |
	    (es8388->sysclk/params_rate(params) == 512)) {
		printk("MCLK/LRCK %d\n", es8388->sysclk/params_rate(params));
		/* bypass dll and fast charge */
//		snd_soc_write(codec, 0x37, 0xd0);
//		snd_soc_write(codec, 0x39, 0xd0);
		snd_soc_component_write(component, 0x37, 0xd0);
		snd_soc_component_write(component, 0x39, 0xd0);

	} else {
		/* fast charge only */
		snd_soc_component_write(component, ES8388_ADCDSMCLOCK, 0xc0);
		snd_soc_component_write(component, ES8388_DACDSMCLOCK, 0xc0);
//		snd_soc_write(codec, ES8388_ADCDSMCLOCK, 0xc0);
//		snd_soc_write(codec, ES8388_DACDSMCLOCK, 0xc0);
	}
	if (first)
		queue_delayed_work(system_wq, &es8388->pcm_pop_work,
					msecs_to_jiffies(2000));

	first = false;
	return 0;
}

static int es8388_mute(struct snd_soc_dai *dai, int mute, int direction)
{
//	struct snd_soc_codec *codec = dai->codec;
//	struct es8388_priv *es8388 = snd_soc_codec_get_drvdata(codec);
	struct snd_soc_component *component = dai->component;
	struct es8388_priv *es8388 = snd_soc_component_get_drvdata(component);

	es8388->muted = mute;
	if (mute) {
		es8388_set_gpio(ES8388_CODEC_SET_SPK, !es8388->spk_gpio_level);
		usleep_range(18000, 20000);
//		snd_soc_write(codec, ES8388_DACCONTROL3, 0x06);
		snd_soc_component_write(component, ES8388_DACCONTROL3, 0x06);
	} else {
//		snd_soc_write(codec, ES8388_DACCONTROL3, 0x02);
		snd_soc_component_write(component, ES8388_DACCONTROL3, 0x02);
		msleep(50);
		if (!es8388->hp_inserted)
			es8388_set_gpio(ES8388_CODEC_SET_SPK, es8388->spk_gpio_level);
		usleep_range(18000, 20000);
	}
	return 0;
}

static int es8388_set_bias_level(struct snd_soc_component *component,
				 enum snd_soc_bias_level level)
{
//	struct es8388_priv *es8388 = snd_soc_codec_get_drvdata(codec);
//	struct snd_soc_component *component = dai->component;
	struct es8388_priv *es8388 = snd_soc_component_get_drvdata(component);

	int ret;

	switch (level) {
	case SND_SOC_BIAS_ON:
		dev_dbg(component->dev, "%s on\n", __func__);
		break;
	case SND_SOC_BIAS_PREPARE:
		dev_dbg(component->dev, "%s prepare\n", __func__);
		if (IS_ERR(es8388->mclk))
			break;
		if (snd_soc_component_get_bias_level(component) == SND_SOC_BIAS_ON) {
			clk_disable_unprepare(es8388->mclk);
		} else {
			ret = clk_prepare_enable(es8388->mclk);
			if (ret)
				return ret;
		}
		snd_soc_component_write(component, ES8388_ANAVOLMANAG, 0x7c);
		snd_soc_component_write(component, ES8388_CHIPLOPOW1, 0x00);
		snd_soc_component_write(component, ES8388_CHIPLOPOW2, 0x00);
		snd_soc_component_write(component, ES8388_CHIPPOWER, 0x00);
		snd_soc_component_write(component, ES8388_ADCPOWER, 0x59);
		
//		snd_soc_write(codec, ES8388_ANAVOLMANAG, 0x7C);
//		snd_soc_write(codec, ES8388_CHIPLOPOW1, 0x00);
//		snd_soc_write(codec, ES8388_CHIPLOPOW2, 0x00);
//		snd_soc_write(codec, ES8388_CHIPPOWER, 0x00);
//		snd_soc_write(codec, ES8388_ADCPOWER, 0x59);
		break;
	case SND_SOC_BIAS_STANDBY:
		dev_dbg(component->dev, "%s standby\n", __func__);
//		snd_soc_write(codec, ES8388_ANAVOLMANAG, 0x7C);
//		snd_soc_write(codec, ES8388_CHIPLOPOW1, 0x00);
//		snd_soc_write(codec, ES8388_CHIPLOPOW2, 0x00);
//		snd_soc_write(codec, ES8388_CHIPPOWER, 0x00);
//		snd_soc_write(codec, ES8388_ADCPOWER, 0x59);
		snd_soc_component_write(component, ES8388_ANAVOLMANAG, 0x7c);
		snd_soc_component_write(component, ES8388_CHIPLOPOW1, 0x00);
		snd_soc_component_write(component, ES8388_CHIPLOPOW2, 0x00);
		snd_soc_component_write(component, ES8388_CHIPPOWER, 0x00);
		snd_soc_component_write(component, ES8388_ADCPOWER, 0x59);

		break;
	case SND_SOC_BIAS_OFF:
		if (es8388->mclk)
			clk_disable_unprepare(es8388->mclk);
		dev_dbg(component->dev, "%s off\n", __func__);
//		snd_soc_write(codec, ES8388_ADCPOWER, 0xFF);
//		//snd_soc_write(codec, ES8388_DACPOWER, 0xC0);
//		snd_soc_write(codec, ES8388_CHIPLOPOW1, 0xFF);
//		snd_soc_write(codec, ES8388_CHIPLOPOW2, 0xFF);
//		snd_soc_write(codec, ES8388_CHIPPOWER, 0xFF);
//		snd_soc_write(codec, ES8388_ANAVOLMANAG, 0x7B);

		snd_soc_component_write(component, ES8388_ANAVOLMANAG, 0xFF);
		snd_soc_component_write(component, ES8388_CHIPLOPOW1, 0xFF);
		snd_soc_component_write(component, ES8388_CHIPLOPOW2, 0xFF);
		snd_soc_component_write(component, ES8388_CHIPPOWER, 0xFF);
		snd_soc_component_write(component, ES8388_ADCPOWER, 0x7B);

		break;
	}
		return 0;
}

#define es8388_RATES SNDRV_PCM_RATE_8000_96000

#define es8388_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |\
	SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FORMAT_S32_LE)

static const struct snd_soc_dai_ops es8388_ops = {
	.startup = es8388_pcm_startup,
	.hw_params = es8388_pcm_hw_params,
	.set_fmt = es8388_set_dai_fmt,
	.set_sysclk = es8388_set_dai_sysclk,
	.mute_stream = es8388_mute,
};

static struct snd_soc_dai_driver es8388_dai = {
	.name = "ES8388 HiFi",
	.playback = {
		     .stream_name = "Playback",
		     .channels_min = 1,
		     .channels_max = 2,
		     .rates = es8388_RATES,
		     .formats = es8388_FORMATS,
		     },
	.capture = {
		    .stream_name = "Capture",
		    .channels_min = 1,
		    .channels_max = 2,
		    .rates = es8388_RATES,
		    .formats = es8388_FORMATS,
		    },
	.ops = &es8388_ops,
	.symmetric_rates = 1,
};

static int es8388_suspend(struct snd_soc_component *component)
{
	snd_soc_component_write(component, ES8388_DACCONTROL3, 0x06);
	snd_soc_component_write(component, ES8388_DACCONTROL24,0x00);
	snd_soc_component_write(component, ES8388_DACCONTROL25,0x00);
	snd_soc_component_write(component, ES8388_DACCONTROL26,0x00);
	snd_soc_component_write(component, ES8388_DACCONTROL27,0x00);
	snd_soc_component_write(component, ES8388_DACCONTROL17,0x38);
	snd_soc_component_write(component, ES8388_DACCONTROL20,0x38);
	snd_soc_component_write(component, ES8388_DACPOWER, 0x00);
	snd_soc_component_write(component, ES8388_DACPOWER, 0xc0);
	snd_soc_component_write(component, ES8388_ADCPOWER, 0xFF);
	snd_soc_component_write(component, ES8388_CHIPPOWER, 0xF3);
	snd_soc_component_write(component, ES8388_DACCONTROL21, 0x9c);
	snd_soc_component_write(component, ES8388_CONTROL1, 0x06);
	snd_soc_component_write(component, ES8388_CONTROL2, 0x58);	
	return 0;
}

static int es8388_resume(struct snd_soc_component *component)
{
	snd_soc_component_write(component, ES8388_DACCONTROL21, 0x80);
	snd_soc_component_write(component, ES8388_CONTROL2, 0x50);
	snd_soc_component_write(component, ES8388_CONTROL1, 0x36);
	snd_soc_component_write(component, ES8388_CHIPPOWER, 0x00);
	snd_soc_component_write(component, ES8388_ADCPOWER, 0x09);
	usleep_range(18000, 20000);
	snd_soc_component_write(component, ES8388_DACPOWER, 0x00);
	snd_soc_component_write(component, ES8388_DACPOWER, 0x3c);
	snd_soc_component_write(component, ES8388_DACCONTROL24,0x1e);
	snd_soc_component_write(component, ES8388_DACCONTROL25,0x1e);
	snd_soc_component_write(component, ES8388_DACCONTROL26,0x1e);
	snd_soc_component_write(component, ES8388_DACCONTROL27,0x1e);
	snd_soc_component_write(component, ES8388_DACCONTROL3, 0x02);
	return 0;
}

//static struct snd_soc_codec *es8388_codec;
//static int es8388_probe(struct snd_soc_codec *codec)
static int es8388_probe(struct snd_soc_component *component)
{
#if 1

//		if (codec == NULL) {
//			dev_err(codec->dev, "Codec device not registered\n");
//			return -ENODEV;
//		}
//		es8388_codec = codec;		
		/***reset***/
		snd_soc_component_write(component,ES8388_CONTROL1,0x80);
		msleep(5);
		snd_soc_component_write(component,ES8388_CONTROL1,0x00);
		/***set power and clock***/ 
		snd_soc_component_write(component,ES8388_CONTROL2,0x60);
		snd_soc_component_write(component,ES8388_CHIPPOWER,0xF3);
		snd_soc_component_write(component,ES8388_CHIPPOWER,0xF0);
		snd_soc_component_write(component,ES8388_DACCONTROL21,0x80);
		snd_soc_component_write(component,ES8388_CONTROL1,0x36);
		snd_soc_component_write(component,ES8388_MASTERMODE,0x00);
		snd_soc_component_write(component,ES8388_DACPOWER,0x00);
		snd_soc_component_write(component,ES8388_CHIPLOPOW2,0xC3);
		snd_soc_component_write(component,ES8388_DACCONTROL3,0x02);
		snd_soc_component_write(component,ES8388_ADCCONTROL1,0x00);
		snd_soc_component_write(component,ES8388_ADCCONTROL2,0xf8);
		snd_soc_component_write(component,ES8388_ADCCONTROL3,0x9a);
		snd_soc_component_write(component,ES8388_ADCCONTROL4,0x4c);
		snd_soc_component_write(component,ES8388_ADCCONTROL5,0x02);
		snd_soc_component_write(component,ES8388_ADCCONTROL8,0x00);
		snd_soc_component_write(component,ES8388_ADCCONTROL9,0x00);
		snd_soc_component_write(component,ES8388_DACCONTROL1,0x18);
		snd_soc_component_write(component,ES8388_DACCONTROL2,0x02);
		snd_soc_component_write(component,ES8388_DACCONTROL4,0x00);
		snd_soc_component_write(component,ES8388_DACCONTROL5,0x00);
		snd_soc_component_write(component,ES8388_DACCONTROL17,0xB8);
		snd_soc_component_write(component,ES8388_DACCONTROL20,0xB8);
		snd_soc_component_write(component,ES8388_RESVER_ENABLE,0xA0);
		usleep_range(18000, 20000);
		snd_soc_component_write(component,ES8388_DACCONTROL24,0x1E);
		snd_soc_component_write(component,ES8388_DACCONTROL25,0x1E);
		snd_soc_component_write(component,ES8388_DACCONTROL26,0x1E);
		snd_soc_component_write(component,ES8388_DACCONTROL27,0x1E);
		snd_soc_component_write(component,ES8388_ADCPOWER,0x09);
		snd_soc_component_write(component,ES8388_CHIPPOWER,0x00);
		usleep_range(18000, 20000);
		/* open dac output later */
		snd_soc_component_write(component,0x04,0x3C);
#endif

#if 0
	if (codec == NULL) {
		dev_err(codec->dev, "Codec device not registered\n");
		return -ENODEV;
	}
	es8388_codec = codec;		
	/***reset***/
	snd_soc_write(codec,ES8388_CONTROL1,0x80);
	msleep(5);
	snd_soc_write(codec,ES8388_CONTROL1,0x00);
	/***set power and clock***/	
	snd_soc_write(codec,ES8388_CONTROL2,0x60);
	snd_soc_write(codec,ES8388_CHIPPOWER,0xF3);
	snd_soc_write(codec,ES8388_CHIPPOWER,0xF0);
	snd_soc_write(codec,ES8388_DACCONTROL21,0x80);
	snd_soc_write(codec,ES8388_CONTROL1,0x36);
	snd_soc_write(codec,ES8388_MASTERMODE,0x00);
	snd_soc_write(codec,ES8388_DACPOWER,0x00);
	snd_soc_write(codec,ES8388_CHIPLOPOW2,0xC3);
	snd_soc_write(codec,ES8388_DACCONTROL3,0x02);
	snd_soc_write(codec,ES8388_ADCCONTROL1,0x00);
	snd_soc_write(codec,ES8388_ADCCONTROL2,0xf8);
	snd_soc_write(codec,ES8388_ADCCONTROL3,0x9a);
	snd_soc_write(codec,ES8388_ADCCONTROL4,0x4c);
	snd_soc_write(codec,ES8388_ADCCONTROL5,0x02);
	snd_soc_write(codec,ES8388_ADCCONTROL8,0x00);
	snd_soc_write(codec,ES8388_ADCCONTROL9,0x00);
	snd_soc_write(codec,ES8388_DACCONTROL1,0x18);
	snd_soc_write(codec,ES8388_DACCONTROL2,0x02);
	snd_soc_write(codec,ES8388_DACCONTROL4,0x00);
	snd_soc_write(codec,ES8388_DACCONTROL5,0x00);
	snd_soc_write(codec,ES8388_DACCONTROL17,0xB8);
	snd_soc_write(codec,ES8388_DACCONTROL20,0xB8);
	snd_soc_write(codec,ES8388_RESVER_ENABLE,0xA0);
	usleep_range(18000, 20000);
	snd_soc_write(codec,ES8388_DACCONTROL24,0x1E);
	snd_soc_write(codec,ES8388_DACCONTROL25,0x1E);
	snd_soc_write(codec,ES8388_DACCONTROL26,0x1E);
	snd_soc_write(codec,ES8388_DACCONTROL27,0x1E);
	snd_soc_write(codec,ES8388_ADCPOWER,0x09);
	snd_soc_write(codec,ES8388_CHIPPOWER,0x00);
	usleep_range(18000, 20000);
	/* open dac output later */
	snd_soc_write(codec,0x04,0x3C);
#endif

	es8388_set_bias_level(component, SND_SOC_BIAS_STANDBY);

	return 0;
}

static const struct regmap_config es8388_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = 0x4f,

	.cache_type = REGCACHE_RBTREE,
};

//static struct snd_soc_component_driver soc_codec_dev_es8388 = {
//	.probe = es8388_probe,
//	.suspend = es8388_suspend,
//	.resume = es8388_resume,
//	.set_bias_level = es8388_set_bias_level,
//
//	.component_driver = {
//		.dapm_widgets		= es8388_dapm_widgets,
//		.num_dapm_widgets	= ARRAY_SIZE(es8388_dapm_widgets),
//		.dapm_routes		= audio_map,
//		.num_dapm_routes	= ARRAY_SIZE(audio_map),
//		.controls		= es8388_snd_controls,
//		.num_controls		= ARRAY_SIZE(es8388_snd_controls),
//	},
//	
//};

static const struct snd_soc_component_driver soc_codec_dev_es8388 = {
	.probe			= es8388_probe,
//	.remove			= es8388_remove,
	.suspend		= es8388_suspend,
	.resume			= es8388_resume,
	.set_bias_level		= es8388_set_bias_level,
	.controls		= es8388_snd_controls,
	.num_controls		= ARRAY_SIZE(es8388_snd_controls),
	.dapm_widgets		= es8388_dapm_widgets,
	.num_dapm_widgets	= ARRAY_SIZE(es8388_dapm_widgets),
//	.dapm_routes		= es8388_dapm_routes,
//	.num_dapm_routes	= ARRAY_SIZE(es8388_dapm_routes),
	.suspend_bias_off	= 1,
	.idle_bias_on		= 1,
	.use_pmdown_time	= 1,
	.endianness		= 1,
	.non_legacy_dai_naming	= 1,
};


static int es8388_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	struct es8388_priv *es8388;
	int ret = -1;
	//enum of_gpio_flags flags;
	//char reg;

	es8388 = devm_kzalloc(&i2c->dev, sizeof(struct es8388_priv), GFP_KERNEL);
	if (es8388 == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, es8388);
	es8388->i2c = i2c;

	es8388_private = es8388;
	/* Enable the following code if there is no mclk.
	 * a clock named "mclk" need to be defined in the dts (see sample dts)
	 *
	 * No need to enable the following code to get mclk if:
	 * 1. sclk/bclk is used as mclk
	 * 2. mclk is controled by soc I2S
	 */
#if 1
	es8388->mclk = devm_clk_get(&i2c->dev, "mclk");
	if (IS_ERR(es8388->mclk)) {
		dev_err(&i2c->dev, "%s mclk is missing or invalid\n", __func__);
		return PTR_ERR(es8388->mclk);
	}
	ret = clk_prepare_enable(es8388->mclk);
	if (ret)
		return ret;
#endif
	es8388->regmap = devm_regmap_init_i2c(i2c, &es8388_regmap_config);
	INIT_DELAYED_WORK(&es8388->pcm_pop_work,
				pcm_pop_work_events);
//	ret = snd_soc_register_codec(&i2c->dev,
//				     &soc_codec_dev_es8388,
//				     &es8388_dai, 1);

	ret = devm_snd_soc_register_component(&i2c->dev,
			&soc_codec_dev_es8388, &es8388_dai, 1);


	return ret;
}

static int es8388_i2c_remove(struct i2c_client *client)
{
	struct es8388_priv *es8388 = i2c_get_clientdata(client);	

	clk_disable_unprepare(es8388->mclk);

	snd_soc_unregister_component(&client->dev);
	return 0;
}

static const struct i2c_device_id es8388_i2c_id[] = {
	{"es8388", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, es8388_i2c_id);

void es8388_i2c_shutdown(struct i2c_client *client)
{
//	struct es8388_priv *es8388 = es8388_private;

//	es8388_set_gpio(ES8388_CODEC_SET_SPK, !es8388->spk_gpio_level);	
//	snd_soc_component_write(component, ES8388_CONTROL2, 0x58);
//	snd_soc_component_write(component, ES8388_CONTROL1, 0x32);
//	snd_soc_component_write(component, ES8388_CHIPPOWER, 0xf3);
//	snd_soc_component_write(component, ES8388_DACPOWER, 0xc0);
//	mdelay(50);
//	snd_soc_component_write(component, ES8388_DACCONTROL26, 0x00);
//	snd_soc_component_write(component, ES8388_DACCONTROL27, 0x00);
//	mdelay(50);
//	snd_soc_component_write(component, ES8388_CONTROL1, 0x30);
//	snd_soc_component_write(component, ES8388_CONTROL1, 0x34);
}

static const struct of_device_id es8388_of_match[] = {
	{ .compatible = "everest,es8388", },
	{ }
};
MODULE_DEVICE_TABLE(of, es8388_of_match);

static struct i2c_driver es8388_i2c_driver = {
	.driver = {
		.name = "ES8388",
		.of_match_table = of_match_ptr(es8388_of_match),
		},
//	.shutdown = es8388_i2c_shutdown,
	.probe = es8388_i2c_probe,
	.remove = es8388_i2c_remove,
	.id_table = es8388_i2c_id,
};
module_i2c_driver(es8388_i2c_driver);

MODULE_DESCRIPTION("ASoC es8388 driver");
MODULE_AUTHOR("Will <pengxiaoxin@everset-semi.com>");
MODULE_LICENSE("GPL");
