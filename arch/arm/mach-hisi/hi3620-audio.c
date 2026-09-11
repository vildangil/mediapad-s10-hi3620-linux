// SPDX-License-Identifier: GPL-2.0
/*
 * Minimal Huawei MediaPad 10 FHD / Hi3620 audio bring-up.
 *
 * This intentionally bypasses the old Android ASoC glue.  The Hi3620 ASP
 * owns its DMA engine, while HI6421 is directly reachable through the PMUSPI
 * register window.  Keep the first implementation playback-only and fixed to
 * the stock multimedia format: 48 kHz, 16-bit little-endian, stereo.
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>

#define HI3620_SCTRL_PHYS              0xfc802000
#define HI3620_PMUSPI_PHYS             0xfcc00000
#define HI3620_AUX_MAP_SIZE            0x1000

/* Hi3620 SCTRL clock/reset registers. */
#define SCTRL_CLK_EN3                  0x050
#define SCTRL_CLK_STATUS3              0x05c
#define SCTRL_RST_EN3                  0x0a4
#define SCTRL_RST_DIS3                 0x0a8
#define SCTRL_RST_STATUS3              0x0ac
#define SCTRL_DIV_REG9                 0x124

#define CLK_DDRC_SH                    BIT(8)
#define CLK_ASP                        BIT(14)
#define CLK_ASPSIO                     BIT(26)
#define RST_ASP                        BIT(14)

/* Hi3620 ASP registers. */
#define ASP_TX0                        0x0000
#define ASP_DER                        0x0014
#define ASP_DSTOP                      0x0018
#define ASP_IRSR                       0x0020
#define ASP_IER                        0x0024
#define ASP_ICR                        0x002c
#define ASP_TXNSSR                     0x0030
#define ASP_TX0RSRR                    0x0034
#define ASP_TX0ASAR                    0x00c0
#define ASP_TX0ADLR                    0x00c4
#define ASP_TX0BSAR                    0x00c8
#define ASP_TX0BDLR                    0x00cc
#define SIO0_I2S_SET                   0x101c
#define SIO0_I2S_CLR                   0x1020

#define TX0_DMA_A                      BIT(2)
#define TX0_DMA_B                      BIT(3)
#define TX0_DMAS                       (TX0_DMA_A | TX0_DMA_B)
#define ASP_BUS_ERROR                  BIT(18)
#define TX01_NEWSONG                   0x3
#define TX0_EN_BIT                     BIT(6)
#define HIGHBIT_IS_LEFT                BIT(3)
#define SIO_TX_ENABLE                  BIT(16)
#define SIO_TX_FIFO                    ((16 - 1) << 4)
#define HI3620_RATE_INDEX_48000        8

/* HI6421 registers use one byte every four bytes in the PMUSPI aperture. */
#define HI6421_LDO_AUDIO               0x36
#define HI6421_EN_CFG_1                0xb7
#define HI6421_EN_CFG_4                0xba
#define HI6421_FREQ_CONFIG             0xbe
#define HI6421_CODECENA_2              0xca
#define HI6421_CODECENA_3              0xcb
#define HI6421_CODECENA_4              0xcc
#define HI6421_MONOMIX                 0xd7
#define HI6421_CLASSDPGA               0xe2
#define HI6421_PLL_RO                  0xf6

#define HI6421_LDO_AUDIO_EN            BIT(0)
#define HI6421_LDO_AUDIO_ECO           BIT(1)
#define HI6421_AP_INTERFACE_EN         BIT(5)
#define HI6421_V200_AP_GAIN_EN         BIT(6)
#define HI6421_DACL_EN                 BIT(5)
#define HI6421_DACR_EN                 BIT(4)
#define HI6421_AHARM_EN                BIT(1)
#define HI6421_HBF1I_EN                BIT(0)
#define HI6421_PLL_PD                  BIT(3)
#define HI6421_IBIAS_PD                BIT(2)
#define HI6421_DACL_PD                 BIT(7)
#define HI6421_DACR_PD                 BIT(6)
#define HI6421_MIXERMONO_PD            BIT(0)
#define HI6421_CLASSD_PD               BIT(0)
#define HI6421_MIX_DACL                BIT(6)
#define HI6421_MIX_DACR                BIT(4)
#define HI6421_CLASSD_MUTE             BIT(1)
#define HI6421_CLASSD_DELAY_MUTE       BIT(0)

#define HI3620_AUDIO_BUFFER_MAX        (128 * 1024)
#define HI3620_AUDIO_BUFFER_MIN        4096
#define HI3620_AUDIO_PERIOD_MIN        1024
#define HI3620_AUDIO_PERIOD_MAX        (32 * 1024)

struct hi3620_audio {
	struct device *dev;
	struct snd_card *card;
	struct snd_pcm *pcm;
	void __iomem *asp;
	void __iomem *sctrl;
	void __iomem *pmu;
	int irq;
	spinlock_t lock;
	struct snd_pcm_substream *playback;
	bool running;
	size_t period_bytes;
	size_t buffer_bytes;
	size_t hw_ptr_bytes;
	unsigned int periods;
	unsigned int next_period;
};

static u64 hi3620_audio_dmamask = DMA_BIT_MASK(32);

/* Exact reset defaults from Huawei's HI6421 codec driver, 0x90..0xf2. */
static const u8 hi6421_codec_defaults[] = {
	0x40, 0x06, 0x00, 0x44, 0x24, 0x9a, 0xae, 0x51,
	0x99, 0x00, 0x24, 0x9a, 0xae, 0x51, 0x99, 0x00,
	0x50, 0x40, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x1f,
	0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x3f, 0x1f,
	0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x1f, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x60, 0x08, 0x08, 0x00, 0x7f,
	0x37, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x7f, 0xfd, 0xff, 0x7f, 0x9e, 0x9e, 0x24,
	0x64, 0x1e, 0x00, 0x00, 0x12, 0x12, 0x15, 0x00,
	0x00, 0x00, 0x00, 0x28, 0x00, 0x28, 0x01, 0x2a,
	0x2a, 0x38, 0x61, 0x00, 0x00, 0x3c, 0x3c, 0xb0,
	0x7d, 0x1f, 0x87, 0x00, 0x10, 0xda, 0x55, 0x55,
	0x2a, 0x00, 0x06,
};

static const struct snd_pcm_hardware hi3620_audio_pcm_hw = {
	.info = SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_BLOCK_TRANSFER |
		SNDRV_PCM_INFO_MMAP |
		SNDRV_PCM_INFO_MMAP_VALID,
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rates = SNDRV_PCM_RATE_48000,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = HI3620_AUDIO_BUFFER_MAX,
	.period_bytes_min = HI3620_AUDIO_PERIOD_MIN,
	.period_bytes_max = HI3620_AUDIO_PERIOD_MAX,
	.periods_min = 2,
	.periods_max = 32,
};

static inline u8 hi6421_read(struct hi3620_audio *chip, unsigned int reg)
{
	return readb(chip->pmu + (reg << 2));
}

static inline void hi6421_write(struct hi3620_audio *chip,
				unsigned int reg, u8 value)
{
	writeb(value, chip->pmu + (reg << 2));
}

static void hi6421_update_bits(struct hi3620_audio *chip, unsigned int reg,
			       u8 mask, u8 value)
{
	u8 old = hi6421_read(chip, reg);

	hi6421_write(chip, reg, (old & ~mask) | (value & mask));
	mb();
}

static void hi6421_restore_defaults(struct hi3620_audio *chip)
{
	unsigned int reg;

	BUILD_BUG_ON(ARRAY_SIZE(hi6421_codec_defaults) != (0xf2 - 0x90 + 1));

	for (reg = 0x90; reg <= 0xf2; reg++) {
		/* 0x92 and 0xc2..0xc8 are read-only/debug on HI6421V200. */
		if (reg == 0x92 || (reg >= 0xc2 && reg <= 0xc8))
			continue;
		hi6421_write(chip, reg, hi6421_codec_defaults[reg - 0x90]);
	}
	mb();
}

static void hi6421_prepare_speaker(struct hi3620_audio *chip)
{
	u8 pll_ro;

	/* Match the vendor probe: restore codec defaults first, then enable the
	 * audio LDO. Preserve its voltage selection and force normal mode.
	 */
	hi6421_restore_defaults(chip);
	hi6421_update_bits(chip, HI6421_LDO_AUDIO,
			   HI6421_LDO_AUDIO_EN | HI6421_LDO_AUDIO_ECO,
			   HI6421_LDO_AUDIO_EN);
	msleep(20);

	/* DAPM supply chain: IBIAS -> PLL -> AP interface. */
	hi6421_update_bits(chip, HI6421_CODECENA_2,
			   HI6421_IBIAS_PD | HI6421_PLL_PD, 0);
	msleep(20);
	pll_ro = hi6421_read(chip, HI6421_PLL_RO);
	if (pll_ro < 0x40) {
		/* Huawei's driver performs one PLL power-cycle when lock is weak. */
		hi6421_update_bits(chip, HI6421_CODECENA_2,
				   HI6421_PLL_PD, HI6421_PLL_PD);
		msleep(20);
		hi6421_update_bits(chip, HI6421_CODECENA_2,
				   HI6421_PLL_PD, 0);
		msleep(20);
		pll_ro = hi6421_read(chip, HI6421_PLL_RO);
	}

	/* 48 kHz AP playback digital path. */
	hi6421_update_bits(chip, HI6421_FREQ_CONFIG, BIT(3), 0);
	hi6421_update_bits(chip, HI6421_EN_CFG_4,
			   HI6421_AP_INTERFACE_EN, HI6421_AP_INTERFACE_EN);
	hi6421_update_bits(chip, HI6421_EN_CFG_1,
			   HI6421_V200_AP_GAIN_EN | HI6421_DACL_EN |
			   HI6421_DACR_EN | HI6421_AHARM_EN | HI6421_HBF1I_EN,
			   HI6421_V200_AP_GAIN_EN | HI6421_DACL_EN |
			   HI6421_DACR_EN | HI6421_AHARM_EN | HI6421_HBF1I_EN);

	/* Analog DAC L/R -> mono mixer -> Class-D speaker. */
	hi6421_update_bits(chip, HI6421_CODECENA_3,
			   HI6421_DACL_PD | HI6421_DACR_PD |
			   HI6421_MIXERMONO_PD, 0);
	hi6421_update_bits(chip, HI6421_MONOMIX,
			   HI6421_MIX_DACL | HI6421_MIX_DACR,
			   HI6421_MIX_DACL | HI6421_MIX_DACR);
	hi6421_update_bits(chip, HI6421_CODECENA_4,
			   HI6421_CLASSD_PD, 0);

	/* Stock 0xe2 is 0x61 (~0 dB). Clear both mute controls, keep gain. */
	hi6421_update_bits(chip, HI6421_CLASSDPGA,
			   HI6421_CLASSD_MUTE | HI6421_CLASSD_DELAY_MUTE, 0);
	msleep(5);

	dev_info(chip->dev,
		 "HI3620-AUDIO-CODEC: ldo=%02x b7=%02x ba=%02x ca=%02x cb=%02x cc=%02x d7=%02x e2=%02x pll_ro=%02x\n",
		 hi6421_read(chip, HI6421_LDO_AUDIO),
		 hi6421_read(chip, HI6421_EN_CFG_1),
		 hi6421_read(chip, HI6421_EN_CFG_4),
		 hi6421_read(chip, HI6421_CODECENA_2),
		 hi6421_read(chip, HI6421_CODECENA_3),
		 hi6421_read(chip, HI6421_CODECENA_4),
		 hi6421_read(chip, HI6421_MONOMIX),
		 hi6421_read(chip, HI6421_CLASSDPGA), pll_ro);
}

static void hi6421_quiesce_speaker(struct hi3620_audio *chip)
{
	/* Mute first, then power the Class-D/DAC chain down. */
	hi6421_update_bits(chip, HI6421_CLASSDPGA,
			   HI6421_CLASSD_MUTE | HI6421_CLASSD_DELAY_MUTE,
			   HI6421_CLASSD_MUTE | HI6421_CLASSD_DELAY_MUTE);
	udelay(200);
	hi6421_update_bits(chip, HI6421_CODECENA_4,
			   HI6421_CLASSD_PD, HI6421_CLASSD_PD);
	hi6421_update_bits(chip, HI6421_CODECENA_3,
			   HI6421_DACL_PD | HI6421_DACR_PD |
			   HI6421_MIXERMONO_PD,
			   HI6421_DACL_PD | HI6421_DACR_PD |
			   HI6421_MIXERMONO_PD);
	hi6421_update_bits(chip, HI6421_EN_CFG_1,
			   HI6421_V200_AP_GAIN_EN | HI6421_DACL_EN |
			   HI6421_DACR_EN | HI6421_AHARM_EN | HI6421_HBF1I_EN,
			   0);
	hi6421_update_bits(chip, HI6421_EN_CFG_4,
			   HI6421_AP_INTERFACE_EN, 0);
	hi6421_update_bits(chip, HI6421_CODECENA_2,
			   HI6421_IBIAS_PD | HI6421_PLL_PD,
			   HI6421_IBIAS_PD | HI6421_PLL_PD);
	hi6421_update_bits(chip, HI6421_LDO_AUDIO,
			   HI6421_LDO_AUDIO_ECO, HI6421_LDO_AUDIO_ECO);
}

static void hi3620_audio_enable_clocks(struct hi3620_audio *chip)
{
	u32 clk_before = readl(chip->sctrl + SCTRL_CLK_STATUS3);
	u32 rst_before = readl(chip->sctrl + SCTRL_RST_STATUS3);

	/* clk_asp's vendor friend is clk_ddrc_sh. SIO0 has its own gate. */
	writel(CLK_DDRC_SH | CLK_ASP | CLK_ASPSIO,
	       chip->sctrl + SCTRL_CLK_EN3);
	mb();
	udelay(5);

	/* Vendor clk_asp owns reset bit 14. Pulse it once before first access. */
	writel(RST_ASP, chip->sctrl + SCTRL_RST_EN3);
	mb();
	udelay(5);
	writel(RST_ASP, chip->sctrl + SCTRL_RST_DIS3);
	mb();
	udelay(20);

	dev_info(chip->dev,
		 "HI3620-AUDIO-CLK: clk3=%08x->%08x rst3=%08x->%08x div9=%08x asp=%u sio=%u ddrc_sh=%u\n",
		 clk_before, readl(chip->sctrl + SCTRL_CLK_STATUS3),
		 rst_before, readl(chip->sctrl + SCTRL_RST_STATUS3),
		 readl(chip->sctrl + SCTRL_DIV_REG9),
		 !!(readl(chip->sctrl + SCTRL_CLK_STATUS3) & CLK_ASP),
		 !!(readl(chip->sctrl + SCTRL_CLK_STATUS3) & CLK_ASPSIO),
		 !!(readl(chip->sctrl + SCTRL_CLK_STATUS3) & CLK_DDRC_SH));
}

static void hi3620_audio_asp_stop(struct hi3620_audio *chip)
{
	u32 ier;

	writel(TX0_DMAS, chip->asp + ASP_DSTOP);
	mb();
	udelay(50);
	writel(TX0_DMAS, chip->asp + ASP_ICR);

	ier = readl(chip->asp + ASP_IER);
	writel(ier & ~(TX0_DMAS | ASP_BUS_ERROR), chip->asp + ASP_IER);
	writel(0, chip->asp + ASP_TX0);
	writel(SIO_TX_ENABLE | SIO_TX_FIFO, chip->asp + SIO0_I2S_CLR);
	mb();
}

static void hi3620_audio_asp_prepare(struct hi3620_audio *chip)
{
	hi3620_audio_asp_stop(chip);

	writel(TX01_NEWSONG, chip->asp + ASP_TXNSSR);
	writel(HI3620_RATE_INDEX_48000, chip->asp + ASP_TX0RSRR);
	writel(HIGHBIT_IS_LEFT | TX0_EN_BIT, chip->asp + ASP_TX0);

	writel(SIO_TX_ENABLE | SIO_TX_FIFO, chip->asp + SIO0_I2S_CLR);
	writel(SIO_TX_ENABLE | SIO_TX_FIFO, chip->asp + SIO0_I2S_SET);

	writel(TX0_DMAS | ASP_BUS_ERROR, chip->asp + ASP_ICR);
	writel(TX0_DMAS | ASP_BUS_ERROR, chip->asp + ASP_IER);
	mb();

	dev_info(chip->dev,
		 "HI3620-AUDIO-ASP: tx0=%08x rsrr=%08x ier=%08x sio_set=%08x\n",
		 readl(chip->asp + ASP_TX0), readl(chip->asp + ASP_TX0RSRR),
		 readl(chip->asp + ASP_IER), readl(chip->asp + SIO0_I2S_SET));
}

static void hi3620_audio_program_dma(struct hi3620_audio *chip,
				     unsigned int dma, dma_addr_t addr,
				     size_t bytes)
{
	u32 addr_reg, len_reg;

	if (dma == TX0_DMA_A) {
		addr_reg = ASP_TX0ASAR;
		len_reg = ASP_TX0ADLR;
	} else {
		addr_reg = ASP_TX0BSAR;
		len_reg = ASP_TX0BDLR;
	}

	writel((u32)addr, chip->asp + addr_reg);
	writel(bytes, chip->asp + len_reg);
	writel(dma, chip->asp + ASP_DER);
}

static irqreturn_t hi3620_audio_irq(int irq, void *data)
{
	struct hi3620_audio *chip = data;
	struct snd_pcm_substream *substream = NULL;
	unsigned long flags;
	unsigned int irs, valid, elapsed = 0;

	irs = readl(chip->asp + ASP_IRSR);
	valid = irs & readl(chip->asp + ASP_IER) &
		(TX0_DMAS | ASP_BUS_ERROR);
	if (!valid)
		return IRQ_NONE;

	if (valid & ASP_BUS_ERROR) {
		writel(ASP_BUS_ERROR, chip->asp + ASP_ICR);
		dev_err_ratelimited(chip->dev,
				    "HI3620-AUDIO: ASP bus error irsr=%08x der=%08x\n",
				    irs, readl(chip->asp + ASP_DER));
	}

	spin_lock_irqsave(&chip->lock, flags);
	substream = chip->playback;

	if (valid & TX0_DMA_A) {
		writel(TX0_DMA_A, chip->asp + ASP_ICR);
		if (chip->running && substream) {
			dma_addr_t addr = substream->runtime->dma_addr +
				chip->next_period * chip->period_bytes;

			hi3620_audio_program_dma(chip, TX0_DMA_A, addr,
						 chip->period_bytes);
			chip->next_period = (chip->next_period + 1) % chip->periods;
			chip->hw_ptr_bytes =
				(chip->hw_ptr_bytes + chip->period_bytes) %
				chip->buffer_bytes;
			elapsed++;
		}
	}

	if (valid & TX0_DMA_B) {
		writel(TX0_DMA_B, chip->asp + ASP_ICR);
		if (chip->running && substream) {
			dma_addr_t addr = substream->runtime->dma_addr +
				chip->next_period * chip->period_bytes;

			hi3620_audio_program_dma(chip, TX0_DMA_B, addr,
						 chip->period_bytes);
			chip->next_period = (chip->next_period + 1) % chip->periods;
			chip->hw_ptr_bytes =
				(chip->hw_ptr_bytes + chip->period_bytes) %
				chip->buffer_bytes;
			elapsed++;
		}
	}
	spin_unlock_irqrestore(&chip->lock, flags);

	while (elapsed--)
		snd_pcm_period_elapsed(substream);

	return IRQ_HANDLED;
}

static int hi3620_audio_pcm_open(struct snd_pcm_substream *substream)
{
	struct hi3620_audio *chip = snd_pcm_substream_chip(substream);
	unsigned long flags;
	int ret;

	substream->runtime->hw = hi3620_audio_pcm_hw;
	ret = snd_pcm_hw_constraint_integer(substream->runtime,
					    SNDRV_PCM_HW_PARAM_PERIODS);
	if (ret)
		return ret;

	spin_lock_irqsave(&chip->lock, flags);
	chip->playback = substream;
	spin_unlock_irqrestore(&chip->lock, flags);
	return 0;
}

static int hi3620_audio_pcm_close(struct snd_pcm_substream *substream)
{
	struct hi3620_audio *chip = snd_pcm_substream_chip(substream);
	unsigned long flags;

	spin_lock_irqsave(&chip->lock, flags);
	chip->running = false;
	chip->playback = NULL;
	spin_unlock_irqrestore(&chip->lock, flags);
	hi3620_audio_asp_stop(chip);
	hi6421_quiesce_speaker(chip);
	return 0;
}

static int hi3620_audio_pcm_hw_params(struct snd_pcm_substream *substream,
				      struct snd_pcm_hw_params *params)
{
	struct hi3620_audio *chip = snd_pcm_substream_chip(substream);
	int ret;

	if (params_rate(params) != 48000 || params_channels(params) != 2 ||
	    params_format(params) != SNDRV_PCM_FORMAT_S16_LE)
		return -EINVAL;

	ret = snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(params));
	if (ret < 0)
		return ret;

	chip->period_bytes = params_period_bytes(params);
	chip->buffer_bytes = params_buffer_bytes(params);
	chip->periods = params_periods(params);
	return 0;
}

static int hi3620_audio_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct hi3620_audio *chip = snd_pcm_substream_chip(substream);
	unsigned long flags;

	spin_lock_irqsave(&chip->lock, flags);
	chip->running = false;
	spin_unlock_irqrestore(&chip->lock, flags);
	hi3620_audio_asp_stop(chip);
	hi6421_quiesce_speaker(chip);
	return snd_pcm_lib_free_pages(substream);
}

static int hi3620_audio_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct hi3620_audio *chip = snd_pcm_substream_chip(substream);
	unsigned long flags;

	spin_lock_irqsave(&chip->lock, flags);
	chip->running = false;
	chip->hw_ptr_bytes = 0;
	chip->next_period = 0;
	spin_unlock_irqrestore(&chip->lock, flags);

	hi6421_prepare_speaker(chip);
	hi3620_audio_asp_prepare(chip);
	return 0;
}

static int hi3620_audio_pcm_trigger(struct snd_pcm_substream *substream,
				    int cmd)
{
	struct hi3620_audio *chip = snd_pcm_substream_chip(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned long flags;
	dma_addr_t addr_a, addr_b;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		spin_lock_irqsave(&chip->lock, flags);
		chip->hw_ptr_bytes = 0;
		chip->next_period = 0;
		chip->running = true;

		addr_a = runtime->dma_addr;
		hi3620_audio_program_dma(chip, TX0_DMA_A, addr_a,
					 chip->period_bytes);
		chip->next_period = 1 % chip->periods;

		addr_b = runtime->dma_addr + chip->next_period * chip->period_bytes;
		hi3620_audio_program_dma(chip, TX0_DMA_B, addr_b,
					 chip->period_bytes);
		chip->next_period = (chip->next_period + 1) % chip->periods;
		spin_unlock_irqrestore(&chip->lock, flags);

		dev_info(chip->dev,
			 "HI3620-AUDIO-PCM: start dma=%pad period=%zu periods=%u buffer=%zu\n",
			 &runtime->dma_addr, chip->period_bytes, chip->periods,
			 chip->buffer_bytes);
		return 0;

	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		spin_lock_irqsave(&chip->lock, flags);
		chip->running = false;
		spin_unlock_irqrestore(&chip->lock, flags);
		hi3620_audio_asp_stop(chip);
		return 0;
	default:
		return -EINVAL;
	}
}

static snd_pcm_uframes_t
hi3620_audio_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct hi3620_audio *chip = snd_pcm_substream_chip(substream);
	unsigned long flags;
	size_t ptr;

	spin_lock_irqsave(&chip->lock, flags);
	ptr = chip->hw_ptr_bytes;
	spin_unlock_irqrestore(&chip->lock, flags);
	return bytes_to_frames(substream->runtime, ptr);
}

static const struct snd_pcm_ops hi3620_audio_pcm_ops = {
	.open = hi3620_audio_pcm_open,
	.close = hi3620_audio_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = hi3620_audio_pcm_hw_params,
	.hw_free = hi3620_audio_pcm_hw_free,
	.prepare = hi3620_audio_pcm_prepare,
	.trigger = hi3620_audio_pcm_trigger,
	.pointer = hi3620_audio_pcm_pointer,
};

static int hi3620_audio_probe(struct platform_device *pdev)
{
	struct hi3620_audio *chip;
	struct snd_card *card;
	struct snd_pcm *pcm;
	struct resource *res;
	int ret;

	chip = devm_kzalloc(&pdev->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	chip->dev = &pdev->dev;
	spin_lock_init(&chip->lock);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	chip->asp = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(chip->asp))
		return PTR_ERR(chip->asp);

	chip->sctrl = ioremap(HI3620_SCTRL_PHYS, HI3620_AUX_MAP_SIZE);
	chip->pmu = ioremap(HI3620_PMUSPI_PHYS, HI3620_AUX_MAP_SIZE);
	if (!chip->sctrl || !chip->pmu) {
		ret = -ENOMEM;
		goto err_unmap;
	}

	chip->irq = platform_get_irq(pdev, 0);
	if (chip->irq < 0) {
		ret = chip->irq;
		goto err_unmap;
	}

	pdev->dev.dma_mask = &hi3620_audio_dmamask;
	pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);

	hi3620_audio_enable_clocks(chip);

	ret = devm_request_irq(&pdev->dev, chip->irq, hi3620_audio_irq,
			       IRQF_SHARED, dev_name(&pdev->dev), chip);
	if (ret)
		goto err_unmap;

	ret = snd_card_new(&pdev->dev, -1, "HI3620", THIS_MODULE, 0, &card);
	if (ret)
		goto err_unmap;
	chip->card = card;

	strlcpy(card->driver, "hi3620-audio", sizeof(card->driver));
	strlcpy(card->shortname, "Huawei Hi3620 Audio", sizeof(card->shortname));
	strlcpy(card->longname, "Huawei MediaPad 10 FHD HI6421", sizeof(card->longname));

	ret = snd_pcm_new(card, "Hi3620 ASP", 0, 1, 0, &pcm);
	if (ret)
		goto err_card;
	chip->pcm = pcm;
	pcm->private_data = chip;
	strlcpy(pcm->name, "Hi3620 ASP PCM", sizeof(pcm->name));
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK, &hi3620_audio_pcm_ops);

	ret = snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV,
					     &pdev->dev,
					     HI3620_AUDIO_BUFFER_MIN,
					     HI3620_AUDIO_BUFFER_MAX);
	if (ret)
		goto err_card;

	ret = snd_card_register(card);
	if (ret)
		goto err_card;

	platform_set_drvdata(pdev, chip);
	dev_info(&pdev->dev,
		 "HI3620-AUDIO: registered card0 candidate, ASP irq=%d PMU ldo=%02x ca=%02x cb=%02x cc=%02x\n",
		 chip->irq, hi6421_read(chip, HI6421_LDO_AUDIO),
		 hi6421_read(chip, HI6421_CODECENA_2),
		 hi6421_read(chip, HI6421_CODECENA_3),
		 hi6421_read(chip, HI6421_CODECENA_4));
	return 0;

err_card:
	snd_card_free(card);
err_unmap:
	if (chip->pmu)
		iounmap(chip->pmu);
	if (chip->sctrl)
		iounmap(chip->sctrl);
	return ret;
}

static int hi3620_audio_remove(struct platform_device *pdev)
{
	struct hi3620_audio *chip = platform_get_drvdata(pdev);

	hi3620_audio_asp_stop(chip);
	hi6421_quiesce_speaker(chip);
	snd_card_free(chip->card);
	iounmap(chip->pmu);
	iounmap(chip->sctrl);
	return 0;
}

static const struct of_device_id hi3620_audio_of_match[] = {
	{ .compatible = "huawei,s10-101x-audio" },
	{ }
};
MODULE_DEVICE_TABLE(of, hi3620_audio_of_match);

static struct platform_driver hi3620_audio_driver = {
	.probe = hi3620_audio_probe,
	.remove = hi3620_audio_remove,
	.driver = {
		.name = "hi3620-s10-audio",
		.of_match_table = hi3620_audio_of_match,
	},
};
module_platform_driver(hi3620_audio_driver);

MODULE_AUTHOR("VildanG / postmarketOS Hi3620 bring-up");
MODULE_DESCRIPTION("Huawei MediaPad 10 FHD Hi3620 minimal ALSA playback");
MODULE_LICENSE("GPL");
