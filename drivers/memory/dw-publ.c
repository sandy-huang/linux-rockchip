#include <linux/io.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/module.h>

#define DW_PUBL_RIDR			0x000
#define DW_PUBL_RIDR_UDRID_SHIFT	24
#define DW_PUBL_RIDR_PHYMJR_SHIFT	20
#define DW_PUBL_RIDR_PHYMDR_SHIFT	16
#define DW_PUBL_RIDR_PHYMNR_SHIFT	12
#define DW_PUBL_RIDR_PUBMJR_SHIFT	8
#define DW_PUBL_RIDR_PUBMDR_SHIFT	4
#define DW_PUBL_RIDR_PUBMNR_SHIFT	0
#define DW_PUBL_RIDR_VER_MASK		0xf

#define DW_PUBL_PIR			0x004
#define DW_PUBL_PGCR			0x008
#define DW_PUBL_PGSR			0x00c
#define DW_PUBL_DLLGCR			0x010
#define DW_PUBL_ACDLLCR			0x014
#define DW_PUBL_PTR0			0x018
#define DW_PUBL_PTR1			0x01c
#define DW_PUBL_PTR2			0x020
#define DW_PUBL_ACIOCR			0x024
#define DW_PUBL_DXCCR			0x028
#define DW_PUBL_DSGCR			0x02c
#define DW_PUBL_DCR			0x030
#define DW_PUBL_DTPR0			0x034
#define DW_PUBL_DTPR1			0x038
#define DW_PUBL_DTPR2			0x03c
#define DW_PUBL_MR0			0x040
#define DW_PUBL_MR1			0x044
#define DW_PUBL_MR2			0x048
#define DW_PUBL_MR3			0x04c
#define DW_PUBL_ODTCR			0x050
#define DW_PUBL_DTAR			0x054
#define DW_PUBL_DTDR0			0x058
#define DW_PUBL_DTDR1			0x05c
#define DW_PUBL_DCUAR			0x0c0
#define DW_PUBL_DCUDR			0x0c4
#define DW_PUBL_DCURR			0x0c8
#define DW_PUBL_DCULR			0x0cc
#define DW_PUBL_DCUGCR			0x0d0
#define DW_PUBL_DCUTPR			0x0d4
#define DW_PUBL_DCUSR0			0x0d8
#define DW_PUBL_DCUSR1			0x0dc
#define DW_PUBL_BISTRR			0x100
#define DW_PUBL_BISTMSKR0		0x104
#define DW_PUBL_BISTMSKR1		0x108
#define DW_PUBL_BISTWCR			0x10c
#define DW_PUBL_BISTLSR			0x110
#define DW_PUBL_BISTAR0			0x114
#define DW_PUBL_BISTAR1			0x118
#define DW_PUBL_BISTAR2			0x11c
#define DW_PUBL_BISTUDPR		0x120
#define DW_PUBL_BISTGSR			0x124
#define DW_PUBL_BISTWER			0x128
#define DW_PUBL_BISTBER0		0x12c
#define DW_PUBL_BISTBER1		0x130
#define DW_PUBL_BISTBER2		0x134
#define DW_PUBL_BISTWCSR		0x138
#define DW_PUBL_BISTFWR0		0x13c
#define DW_PUBL_BISTFWR1		0x140
#define DW_PUBL_ZQ0CR0			0x180
#define DW_PUBL_ZQ0CR1			0x184
#define DW_PUBL_ZQ0SR0			0x188
#define DW_PUBL_ZQ0SR1			0x18c
#define DW_PUBL_ZQ1CR0			0x190
#define DW_PUBL_ZQ1CR1			0x194
#define DW_PUBL_ZQ1SR0			0x198
#define DW_PUBL_ZQ1SR1			0x19c
#define DW_PUBL_ZQ2CR0			0x1a0
#define DW_PUBL_ZQ2CR1			0x1a4
#define DW_PUBL_ZQ2SR0			0x1a8
#define DW_PUBL_ZQ2SR1			0x1ac
#define DW_PUBL_ZQ3CR0			0x1b0
#define DW_PUBL_ZQ3CR1			0x1b4
#define DW_PUBL_ZQ3SR0			0x1b8
#define DW_PUBL_ZQ3SR1			0x1bc
#define DW_PUBL_DX0GCR			0x1c0
#define DW_PUBL_DX0GSR0			0x1c4
#define DW_PUBL_DX0GSR1			0x1c8
#define DW_PUBL_DX0DLLCR		0x1cc
#define DW_PUBL_DX0DQTR			0x1d0
#define DW_PUBL_DX0DQSTR		0x1d4
#define DW_PUBL_DX1GCR			0x200
#define DW_PUBL_DX1GSR0			0x204
#define DW_PUBL_DX1GSR1			0x208
#define DW_PUBL_DX1DLLCR		0x20c
#define DW_PUBL_DX1DQTR			0x210
#define DW_PUBL_DX1DQSTR		0x214
#define DW_PUBL_DX2GCR			0x240
#define DW_PUBL_DX2GSR0			0x244
#define DW_PUBL_DX2GSR1			0x248
#define DW_PUBL_DX2DLLCR		0x24c
#define DW_PUBL_DX2DQTR			0x250
#define DW_PUBL_DX2DQSTR		0x254
#define DW_PUBL_DX3GCR			0x280
#define DW_PUBL_DX3GSR0			0x284
#define DW_PUBL_DX3GSR1			0x288
#define DW_PUBL_DX3DLLCR		0x28c
#define DW_PUBL_DX3DQTR			0x290
#define DW_PUBL_DX3DQSTR		0x294

struct dw_publ {
	struct device *dev;
	void __iomem *regs;
	int irq;

};

static void dw_publ_dumpregs(struct dw_publ *publ)
{
	/* sd_params -> phy_timing */
	dev_info(publ->dev, "reading back phy_timing values\n");
	dev_info(publ->dev, "dtpr0:     0x%x\n", readl(publ->regs + DW_PUBL_DTPR0));
	dev_info(publ->dev, "dtpr1:     0x%x\n", readl(publ->regs + DW_PUBL_DTPR1));
	dev_info(publ->dev, "dtpr2:     0x%x\n", readl(publ->regs + DW_PUBL_DTPR2));
	dev_info(publ->dev, "mr0:       0x%x\n", readl(publ->regs + DW_PUBL_MR0));
	dev_info(publ->dev, "mr1:       0x%x\n", readl(publ->regs + DW_PUBL_MR1));
	dev_info(publ->dev, "mr2:       0x%x\n", readl(publ->regs + DW_PUBL_MR2));
	dev_info(publ->dev, "mr3:       0x%x\n\n", readl(publ->regs + DW_PUBL_MR3));

	dev_info(publ->dev, "reading back general publ values\n");
	dev_info(publ->dev, "ptr0:      0x%x\n", readl(publ->regs + DW_PUBL_PTR0));
	dev_info(publ->dev, "ptr1:      0x%x\n", readl(publ->regs + DW_PUBL_PTR0));
	dev_info(publ->dev, "ptr2:      0x%x\n\n", readl(publ->regs + DW_PUBL_PTR0));

	dev_info(publ->dev, "pgcr:      0x%x\n", readl(publ->regs + DW_PUBL_PGCR));
	dev_info(publ->dev, "dcr:       0x%x\n", readl(publ->regs + DW_PUBL_DCR));
	dev_info(publ->dev, "dxccr:     0x%x\n", readl(publ->regs + DW_PUBL_DXCCR));
	dev_info(publ->dev, "dsgcr:     0x%x\n", readl(publ->regs + DW_PUBL_DSGCR));
//	dev_info(publ->dev, "dxgcr:     0x%x\n", readl(publ->regs + DW_PUBL_DXGCR));
}

static int dw_publ_probe(struct platform_device *pdev)
{
	struct dw_publ *publ;
	struct resource *regs;
	u32 data;

	publ = devm_kzalloc(&pdev->dev, sizeof(*publ), GFP_KERNEL);
	if (!publ)
		return -ENOMEM;

	publ->dev = &pdev->dev;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	publ->regs = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(publ->regs))
		return PTR_ERR(publ->regs);

	data = readl(publ->regs + DW_PUBL_RIDR);
	dev_info(&pdev->dev, "found DW-PUBL user:v%d, phy: v%d.%d.%d, pub: v%d.%d.%d\n", 
		 (data >> DW_PUBL_RIDR_UDRID_SHIFT) & DW_PUBL_RIDR_VER_MASK,
		 (data >> DW_PUBL_RIDR_PHYMJR_SHIFT) & DW_PUBL_RIDR_VER_MASK,
		 (data >> DW_PUBL_RIDR_PHYMDR_SHIFT) & DW_PUBL_RIDR_VER_MASK,
		 (data >> DW_PUBL_RIDR_PHYMNR_SHIFT) & DW_PUBL_RIDR_VER_MASK,
		 (data >> DW_PUBL_RIDR_PUBMJR_SHIFT) & DW_PUBL_RIDR_VER_MASK,
		 (data >> DW_PUBL_RIDR_PUBMDR_SHIFT) & DW_PUBL_RIDR_VER_MASK,
		 (data >> DW_PUBL_RIDR_PUBMNR_SHIFT) & DW_PUBL_RIDR_VER_MASK);

	platform_set_drvdata(pdev, publ);

	dw_publ_dumpregs(publ);

	return 0;
}

static const struct of_device_id dw_publ_of_match[] = {
	{ .compatible = "snps,dw-publ" },
	{},
};
MODULE_DEVICE_TABLE(of, dw_publ_of_match);

static struct platform_driver dw_publ_driver = {
	.probe		= dw_publ_probe,
	.driver		= {
		.name	= "dw-publ",
		.of_match_table = dw_publ_of_match,
	},
};

static int __init dw_publ_init(void)
{
	return platform_driver_register(&dw_publ_driver);
}
module_init(dw_publ_init);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Heiko Stuebner <heiko@sntech.de>");
MODULE_DESCRIPTION("Designware PHY Utility Block lite");
