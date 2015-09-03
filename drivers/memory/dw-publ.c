#include <linux/io.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/module.h>

#define DW_PUBL_RIDR			0x0
#define DW_PUBL_RIDR_UDRID_SHIFT	24
#define DW_PUBL_RIDR_PHYMJR_SHIFT	20
#define DW_PUBL_RIDR_PHYMDR_SHIFT	16
#define DW_PUBL_RIDR_PHYMNR_SHIFT	12
#define DW_PUBL_RIDR_PUBMJR_SHIFT	8
#define DW_PUBL_RIDR_PUBMDR_SHIFT	4
#define DW_PUBL_RIDR_PUBMNR_SHIFT	0
#define DW_PUBL_RIDR_VER_MASK		0xf

struct dw_publ {
	void __iomem *regs;
	int irq;

};


static int dw_publ_probe(struct platform_device *pdev)
{
	struct dw_publ *publ;
	struct resource *regs;
	u32 data;

	publ = devm_kzalloc(&pdev->dev, sizeof(*publ), GFP_KERNEL);
	if (!publ)
		return -ENOMEM;

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
