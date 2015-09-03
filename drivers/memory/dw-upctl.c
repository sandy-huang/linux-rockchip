#include <linux/io.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/module.h>

#define DW_UPCTL_TOGCNT1U	0x0c0

#define DW_UPCTL_IPVR		0x3f8
#define DW_UPCTL_IPTR		0x3fc
/* value should be DWC\0 */
#define DW_UPCTL_IPTR_EXPECTED	0x44574300

struct dw_upctl {
	void __iomem *regs;
	int irq;

};


static int dw_upctl_probe(struct platform_device *pdev)
{
	struct dw_upctl *upctl;
	struct resource *regs;
	u32 data;

	upctl = devm_kzalloc(&pdev->dev, sizeof(*upctl), GFP_KERNEL);
	if (!upctl)
		return -ENOMEM;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	upctl->regs = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(upctl->regs))
		return PTR_ERR(upctl->regs);

	/* try to identify the controller */
	data = readl(upctl->regs + DW_UPCTL_IPTR);
	if (data != DW_UPCTL_IPTR_EXPECTED) {
		dev_err(&pdev->dev, "not a DW_uPCTL, IPTR: 0x%x != 0x%x\n",
			data, DW_UPCTL_IPTR_EXPECTED);
	}

	data = readl(upctl->regs + DW_UPCTL_IPVR);
	/* lowest byte should be a ascii "*" */
	if ((data & 0xff) != 0x2a) {
		dev_err(&pdev->dev, "not a DW_uPCTL, IPTR: 0x%x != 0x%x\n",
			data, DW_UPCTL_IPTR_EXPECTED);

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
