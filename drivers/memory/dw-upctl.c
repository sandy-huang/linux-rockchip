#include <linux/io.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/module.h>


#define DW_UPCTL_SCFG			0x000
#define DW_UPCTL_SCTL			0x004
#define DW_UPCTL_STAT			0x008
#define DW_UPCTL_INTRSTAT		0x00c
#define DW_UPCTL_MCMD			0x040
#define DW_UPCTL_POWCTL			0x044
#define DW_UPCTL_POWSTAT		0x048
#define DW_UPCTL_CMDTSTAT		0x04c
#define DW_UPCTL_CMDTSTATEN		0x050
#define DW_UPCTL_MRRCFG0		0x060
#define DW_UPCTL_MRRSTAT0		0x064
#define DW_UPCTL_MRRSTAT1		0x068
#define DW_UPCTL_MCFG			0x080
#define DW_UPCTL_PPCFG			0x084
#define DW_UPCTL_MSTAT			0x088
#define DW_UPCTL_MCFG1			0x07c /* is this correct? */
#define DW_UPCTL_DTUPDES		0x094
#define DW_UPCTL_DTUNA			0x098
#define DW_UPCTL_DTUNE			0x09c
#define DW_UPCTL_DTUPRD0		0x0a0
#define DW_UPCTL_DTUPRD1		0x0a4
#define DW_UPCTL_DTUPRD2		0x0a8
#define DW_UPCTL_DTUPRD3		0x0ac
#define DW_UPCTL_DTUAWDT		0x0b0
#define DW_UPCTL_TOGCNT1U		0x0c0
#define DW_UPCTL_TINIT			0x0c4
#define DW_UPCTL_TRSTH			0x0c8
#define DW_UPCTL_TOGCNT100N		0x0cc
#define DW_UPCTL_TREFI			0x0d0
#define DW_UPCTL_TMRD			0x0d4
#define DW_UPCTL_TRFC			0x0d8
#define DW_UPCTL_TRP			0x0dc
#define DW_UPCTL_TRTW			0x0e0
#define DW_UPCTL_TAL			0x0e4
#define DW_UPCTL_TCL			0x0e8
#define DW_UPCTL_TCWL			0x0ec
#define DW_UPCTL_TRAS			0x0f0
#define DW_UPCTL_TRC			0x0f4
#define DW_UPCTL_TRCD			0x0f8
#define DW_UPCTL_TRRD			0x0fc
#define DW_UPCTL_TRTP			0x100
#define DW_UPCTL_TWR			0x104
#define DW_UPCTL_TWTR			0x108
#define DW_UPCTL_TEXSR			0x10c
#define DW_UPCTL_TXP			0x110
#define DW_UPCTL_TXPDLL			0x114
#define DW_UPCTL_TZQCS			0x118
#define DW_UPCTL_TZQCSI			0x11c
#define DW_UPCTL_TDQS			0x120
#define DW_UPCTL_TCKSRE			0x124
#define DW_UPCTL_TCKSRX			0x128
#define DW_UPCTL_TCKE			0x12c
#define DW_UPCTL_TMOD			0x130
#define DW_UPCTL_TRSTL			0x134
#define DW_UPCTL_TZQCL			0x138
#define DW_UPCTL_TMRR			0x13c
#define DW_UPCTL_TCKESR			0x140
#define DW_UPCTL_TDPD			0x144
#define DW_UPCTL_DTUWACTL		0x200
#define DW_UPCTL_DTURACTL		0x204
#define DW_UPCTL_DTUCFG			0x208
#define DW_UPCTL_DTUECTL		0x20c
#define DW_UPCTL_DTUWD0			0x210
#define DW_UPCTL_DTUWD1			0x214
#define DW_UPCTL_DTUWD2			0x218
#define DW_UPCTL_DTUWD3			0x21c
#define DW_UPCTL_DTUWDM			0x220
#define DW_UPCTL_DTURD0			0x224
#define DW_UPCTL_DTURD1			0x228
#define DW_UPCTL_DTURD2			0x22c
#define DW_UPCTL_DTURD3			0x230
#define DW_UPCTL_DTULFSRWD		0x234
#define DW_UPCTL_DTULFSRRD		0x238
#define DW_UPCTL_DTUEAF			0x23c
#define DW_UPCTL_DFICTRLDELAY		0x240
#define DW_UPCTL_DFIODTCFG		0x244
#define DW_UPCTL_DFIODTCFG1		0x248
#define DW_UPCTL_DFIODTRANKMAP		0x24c
#define DW_UPCTL_DFITPHYWRDATA		0x250
#define DW_UPCTL_DFITPHYWRLAT		0x254
#define DW_UPCTL_DFITRDDATAEN		0x260
#define DW_UPCTL_DFITPHYRDLAT		0x264
#define DW_UPCTL_DFITPHYUPDTYPE0	0x270
#define DW_UPCTL_DFITPHYUPDTYPE1	0x274
#define DW_UPCTL_DFITPHYUPDTYPE2	0x278
#define DW_UPCTL_DFITPHYUPDTYPE3	0x27c
#define DW_UPCTL_DFITCTRLUPDMIN		0x280
#define DW_UPCTL_DFITCTRLUPDMAX		0x284
#define DW_UPCTL_DFITCTRLUPDDLY		0x288
#define DW_UPCTL_DFIUPDCFG		0x290
#define DW_UPCTL_DFITREFMSKI		0x294
#define DW_UPCTL_DFITRCFG0		0x2ac
#define DW_UPCTL_DFITRSTAT0		0x2b0
#define DW_UPCTL_DFITRWRLVLEN		0x2b4
#define DW_UPCTL_DFITRRDLVLEN		0x2b8
#define DW_UPCTL_DFITRRDLVLGATEEN	0x2bc
#define DW_UPCTL_DFISTSTAT0		0x2c0
#define DW_UPCTL_DFISTCFG0		0x2c4
#define DW_UPCTL_DFISTCFG1		0x2c8
#define DW_UPCTL_DFITDRAMCLKEN		0x2d0
#define DW_UPCTL_DFITDRAMCLKDIS		0x2d4
#define DW_UPCTL_DFISTCFG2		0x2d8
#define DW_UPCTL_DFISTPARCLK		0x2dc
#define DW_UPCTL_DFISTPARLOG		0x2e0
#define DW_UPCTL_DFILPCFG0		0x2f0
#define DW_UPCTL_DFITRWRLVLRESP0	0x300
#define DW_UPCTL_DFITRWRLVLRESP1	0x304
#define DW_UPCTL_DFITRWRLVLRESP2	0x308
#define DW_UPCTL_DFITRRDLVLRESP0	0x30c
#define DW_UPCTL_DFITRRDLVLRESP1	0x310
#define DW_UPCTL_DFITRRDLVLRESP2	0x314
#define DW_UPCTL_DFITRWRLVLDELAY0	0x318
#define DW_UPCTL_DFITRWRLVLDELAY1	0x31c
#define DW_UPCTL_DFITRWRLVLDELAY2	0x320
#define DW_UPCTL_DFITRRDLVLDELAY0	0x324
#define DW_UPCTL_DFITRRDLVLDELAY1	0x328
#define DW_UPCTL_DFITRRDLVLDELAY2	0x32c
#define DW_UPCTL_DFITRRDLVLGATEDELAY0	0x330
#define DW_UPCTL_DFITRRDLVLGATEDELAY1	0x334
#define DW_UPCTL_DFITRRDLVLGATEDELAY2	0x338
#define DW_UPCTL_DFITRCMD		0x33c
#define DW_UPCTL_IPVR			0x3f8
#define DW_UPCTL_IPTR			0x3fc
/* value should be DWC\0 */
#define DW_UPCTL_IPTR_DWC0		0x44574300

struct dw_upctl {
	struct device *dev;
	void __iomem *regs;
	int irq;

};

static void dw_upctl_dumpregs(struct dw_upctl *upctl)
{
	dev_info(upctl->dev, "reading back pctl general values\n");
	dev_info(upctl->dev, "mcfg:   0x%x\n", readl(upctl->regs + DW_UPCTL_MCFG));
	dev_info(upctl->dev, "scfg:   0x%x\n", readl(upctl->regs + DW_UPCTL_SCFG));
	dev_info(upctl->dev, "ppcfg:  0x%x\n\n", readl(upctl->regs + DW_UPCTL_PPCFG));

	dev_info(upctl->dev, "reading back dfi values\n");
	dev_info(upctl->dev, "dfistcfg0:       0x%x\n", readl(upctl->regs + DW_UPCTL_DFISTCFG0));
	dev_info(upctl->dev, "dfistcfg1:       0x%x\n", readl(upctl->regs + DW_UPCTL_DFISTCFG1));
	dev_info(upctl->dev, "dfistcfg2:       0x%x\n", readl(upctl->regs + DW_UPCTL_DFISTCFG2));
	dev_info(upctl->dev, "dfilpcfg0:       0x%x\n", readl(upctl->regs + DW_UPCTL_DFILPCFG0));
	dev_info(upctl->dev, "dfitctrldelay:   0x%x\n", readl(upctl->regs + DW_UPCTL_DFICTRLDELAY));
	dev_info(upctl->dev, "dfitphywrdata:   0x%x\n", readl(upctl->regs + DW_UPCTL_DFITPHYWRDATA));
	dev_info(upctl->dev, "dfitphyrdlat:    0x%x\n", readl(upctl->regs + DW_UPCTL_DFITPHYRDLAT));
	dev_info(upctl->dev, "dfitdramclkdis:  0x%x\n", readl(upctl->regs + DW_UPCTL_DFITDRAMCLKDIS));
	dev_info(upctl->dev, "dfitdramclken:   0x%x\n", readl(upctl->regs + DW_UPCTL_DFITDRAMCLKEN));
	dev_info(upctl->dev, "dfitphyupdtype0: 0x%x\n", readl(upctl->regs + DW_UPCTL_DFITPHYUPDTYPE0));
	dev_info(upctl->dev, "dfiodtcfg:       0x%x\n", readl(upctl->regs + DW_UPCTL_DFIODTCFG));
	dev_info(upctl->dev, "dfiodtcfg1:      0x%x\n", readl(upctl->regs + DW_UPCTL_DFIODTCFG1));
	dev_info(upctl->dev, "dfiupdcfg:       0x%x\n\n", readl(upctl->regs + DW_UPCTL_DFIUPDCFG));

	dev_info(upctl->dev, "reading back special dfi values\n");
	dev_info(upctl->dev, "dfitrddataen (expected: tcl-1): 0x%x\n", readl(upctl->regs + DW_UPCTL_DFITRDDATAEN));
	dev_info(upctl->dev, "dfitphywrlat (expected: tcwl):  0x%x\n\n", readl(upctl->regs + DW_UPCTL_DFITPHYWRLAT));

	/* sd_params -> pctl_timing */
	dev_info(upctl->dev, "reading back pctl_timing values\n");
	dev_info(upctl->dev, "togcnt1u:   0x%x\n", readl(upctl->regs + DW_UPCTL_TOGCNT1U));
	dev_info(upctl->dev, "tinit:      0x%x\n", readl(upctl->regs + DW_UPCTL_TINIT));
	dev_info(upctl->dev, "trsth:      0x%x\n", readl(upctl->regs + DW_UPCTL_TRSTH));
	dev_info(upctl->dev, "togcnt100n: 0x%x\n", readl(upctl->regs + DW_UPCTL_TOGCNT100N));
	dev_info(upctl->dev, "trefi:      0x%x\n", readl(upctl->regs + DW_UPCTL_TREFI));
	dev_info(upctl->dev, "tmrd:       0x%x\n", readl(upctl->regs + DW_UPCTL_TMRD));
	dev_info(upctl->dev, "trfc:       0x%x\n", readl(upctl->regs + DW_UPCTL_TRFC));
	dev_info(upctl->dev, "trp:        0x%x\n", readl(upctl->regs + DW_UPCTL_TRP));
	dev_info(upctl->dev, "trtw:       0x%x\n", readl(upctl->regs + DW_UPCTL_TRTW));
	dev_info(upctl->dev, "tal:        0x%x\n", readl(upctl->regs + DW_UPCTL_TAL));
	dev_info(upctl->dev, "tcl:        0x%x\n", readl(upctl->regs + DW_UPCTL_TCL));
	dev_info(upctl->dev, "tcwl:       0x%x\n", readl(upctl->regs + DW_UPCTL_TCWL));
	dev_info(upctl->dev, "tras:       0x%x\n", readl(upctl->regs + DW_UPCTL_TRAS));
	dev_info(upctl->dev, "trc:        0x%x\n", readl(upctl->regs + DW_UPCTL_TRC));
	dev_info(upctl->dev, "trcd:       0x%x\n", readl(upctl->regs + DW_UPCTL_TRCD));
	dev_info(upctl->dev, "trrd:       0x%x\n", readl(upctl->regs + DW_UPCTL_TRRD));
	dev_info(upctl->dev, "trtp:       0x%x\n", readl(upctl->regs + DW_UPCTL_TRTP));
	dev_info(upctl->dev, "twr:        0x%x\n", readl(upctl->regs + DW_UPCTL_TWR));
	dev_info(upctl->dev, "twtr:       0x%x\n", readl(upctl->regs + DW_UPCTL_TWTR));
	dev_info(upctl->dev, "texsr:      0x%x\n", readl(upctl->regs + DW_UPCTL_TEXSR));
	dev_info(upctl->dev, "txp:        0x%x\n", readl(upctl->regs + DW_UPCTL_TXP));
	dev_info(upctl->dev, "txpdll:     0x%x\n", readl(upctl->regs + DW_UPCTL_TXPDLL));
	dev_info(upctl->dev, "tzqcs:      0x%x\n", readl(upctl->regs + DW_UPCTL_TZQCS));
	dev_info(upctl->dev, "tzqcsi:     0x%x\n", readl(upctl->regs + DW_UPCTL_TZQCSI));
	dev_info(upctl->dev, "tdqs:       0x%x\n", readl(upctl->regs + DW_UPCTL_TDQS));
	dev_info(upctl->dev, "tcksre:     0x%x\n", readl(upctl->regs + DW_UPCTL_TCKSRE));
	dev_info(upctl->dev, "tcksrx:     0x%x\n", readl(upctl->regs + DW_UPCTL_TCKSRX));
	dev_info(upctl->dev, "tcke:       0x%x\n", readl(upctl->regs + DW_UPCTL_TCKE));
	dev_info(upctl->dev, "tmod:       0x%x\n", readl(upctl->regs + DW_UPCTL_TMOD));
	dev_info(upctl->dev, "trstl:      0x%x\n", readl(upctl->regs + DW_UPCTL_TRSTL));
	dev_info(upctl->dev, "tzqcl:      0x%x\n", readl(upctl->regs + DW_UPCTL_TZQCL));
	dev_info(upctl->dev, "tmrr:       0x%x\n", readl(upctl->regs + DW_UPCTL_TMRR));
	dev_info(upctl->dev, "tckesr:     0x%x\n", readl(upctl->regs + DW_UPCTL_TCKESR));
	dev_info(upctl->dev, "tdpd:       0x%x\n", readl(upctl->regs + DW_UPCTL_TDPD));
}

static int dw_upctl_probe(struct platform_device *pdev)
{
	struct dw_upctl *upctl;
	struct resource *regs;
	u32 data;

	upctl = devm_kzalloc(&pdev->dev, sizeof(*upctl), GFP_KERNEL);
	if (!upctl)
		return -ENOMEM;

	upctl->dev = &pdev->dev;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	upctl->regs = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(upctl->regs))
		return PTR_ERR(upctl->regs);

	/* try to identify the controller */
	data = readl(upctl->regs + DW_UPCTL_IPTR);
	if (data != DW_UPCTL_IPTR_DWC0) {
		dev_err(&pdev->dev, "not a DW_uPCTL, IPTR: 0x%x != 0x%x\n",
			data, DW_UPCTL_IPTR_DWC0);
	}

	data = readl(upctl->regs + DW_UPCTL_IPVR);
	/* lowest byte should be a ascii "*" */
	if ((data & 0xff) != 0x2a) {
		dev_err(&pdev->dev, "not a DW_uPCTL, IPVR is wrong: 0x%x\n",
			data);

	}

	dev_info(&pdev->dev, "found DW-uPCTL v%c.%c%c\n", (data >> 24) & 0xff,
		 (data >> 16) & 0xff, (data >> 8) & 0xff);


	/* irq may be shared between multiple pctls */
/*	upctl->irq = platform_get_irq(pdev, 0);
	if (upctl->irq < 0)
		return upctl->irq;*/

	data = readl(upctl->regs + DW_UPCTL_TOGCNT1U);
	dev_info(&pdev->dev, "freq = %dMHz\n", data);

	platform_set_drvdata(pdev, upctl);

	dw_upctl_dumpregs(upctl);

	return 0;
}

static const struct of_device_id dw_upctl_of_match[] = {
	{ .compatible = "snps,dw-upctl" },
	{},
};
MODULE_DEVICE_TABLE(of, dw_upctl_of_match);

static struct platform_driver dw_upctl_driver = {
	.probe		= dw_upctl_probe,
	.driver		= {
		.name	= "dw-upctl",
		.of_match_table = dw_upctl_of_match,
	},
};

static int __init dw_upctl_init(void)
{
	return platform_driver_register(&dw_upctl_driver);
}
module_init(dw_upctl_init);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Heiko Stuebner <heiko@sntech.de>");
MODULE_DESCRIPTION("Designware Universal DDR Protocol Controller");
