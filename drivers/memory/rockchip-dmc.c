#include <linux/io.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>


/* memory scheduler settings */
#define RK3066_MSCH_COREID			0x00
#define RK3066_MSCH_REVISIONID			0x04

#define RK3066_MSCH_DDRCONF			0x08
#define RK3066_MSCH_DDRCONF_MASK		0xf

#define RK3066_MSCH_DDRTIMING			0x0c
#define RK3066_MSCH_DDRTIMING_BWRATIO		BIT(31)
#define RK3066_MSCH_DDRTIMING_WRTORD_SHIFT	26
#define RK3066_MSCH_DDRTIMING_WRTORD_MASK	0x1f
#define RK3066_MSCH_DDRTIMING_RDTOWR_SHIFT	21
#define RK3066_MSCH_DDRTIMING_RDTOWR_MASK	0x1f
#define RK3066_MSCH_DDRTIMING_BURSTLEN_SHIFT	18
#define RK3066_MSCH_DDRTIMING_BURSTLEN_MASK	0x7
#define RK3066_MSCH_DDRTIMING_WRTOMISS_SHIFT	12
#define RK3066_MSCH_DDRTIMING_WRTOMISS_MASK	0x3f
#define RK3066_MSCH_DDRTIMING_RDTOMISS_SHIFT	6
#define RK3066_MSCH_DDRTIMING_RDTOMISS_MASK	0x3f
#define RK3066_MSCH_DDRTIMING_ACTTOACT_SHIFT	0
#define RK3066_MSCH_DDRTIMING_ACTTOACT_MASK	0x3f

#define RK3066_MSCH_DDRMODE			0x10
#define RK3288_MSCH_DDRMODE_BWRATIOEXTENDED	BIT(1)
#define RK3066_MSCH_DDRMODE_AUTOPRECHARGE	BIT(0)

#define RK3066_MSCH_READLATENCY			0x14
#define RK3066_MSCH_READLATENCY_MASK		0x3f
#define RK3288_MSCH_READLATENCY_MASK		0xff

#define RK3288_MSCH_ACTIVATE	0x38
#define RK3288_MSCH_DEVTODEV	0x3c

/* GRF-based memory settings */
#define RK3066_GRF_SOC_CON2			0x158
#define RK3188_GRF_SOC_CON2			0x0a8
#define RK3066_GRF_SOC_CON2_MSCH_DDR3		BIT(7)
#define RK3066_GRF_SOC_CON2_BANK2_TO_RANK	BIT(2)
#define RK3066_GRF_SOC_CON2_RANK_TO_ROW15	BIT(1)
#define RK3066_GRF_SOC_CON2_UPCTL_C_ACTIVE	BIT(0)
//radxa 1000 0000

#define RK3066_GRF_DDRC_CON0			0x198
#define RK3188_GRF_DDRC_CON0			0x0ec
#define RK3188_GRF_DDRC_CON0_DDR_16BIT		BIT(15)
//#define RK3066_GRF_DDRC_CON0_DTO_LB		[12:11]
//#define RK3066_GRF_DDRC_CON0_DTO_TE		[10:9]
//#define RK3066_GRF_DDRC_CON0_DTO_PDR		[8:7]
//#define RK3066_GRF_DDRC_CON0_DTO_PDD		[6:5]
//#define RK3066_GRF_DDRC_CON0_DTO_IOM		[4:3]
//#define RK3066_GRF_DDRC_CON0_DTO_OE		[2:1]
#define RK3066_GRF_DDRC_CON0_ATO_AE		BIT(0)


#define RK3288_GRF_SOC_CON0			0x244
#define RK3288_GRF_SOC_CON0_DDR1_16BIT		BIT(9)
#define RK3288_GRF_SOC_CON0_DDR0_16BIT		BIT(8)
#define RK3288_GRF_SOC_CON0_UPCTL1_C_ACTIVE	BIT(6)
#define RK3288_GRF_SOC_CON0_UPCTL0_C_ACTIVE	BIT(5)
#define RK3288_GRF_SOC_CON0_MSCH1_DDR3		BIT(4)
#define RK3288_GRF_SOC_CON0_MSCH0_DDR3		BIT(3)
#define RK3288_GRF_SOC_CON0_MSCH1_PARTIALPOP	BIT(2)
#define RK3288_GRF_SOC_CON0_MSCH0_PARTIALPOP	BIT(1)

#define RK3288_GRF_SOC_CON2			0x24c
#define RK3288_GRF_SOC_CON2_UPCTL1_LPDDR3_ODT	BIT(13)
#define RK3288_GRF_SOC_CON2_UPCTL1_BST_DIS	BIT(12)
#define RK3288_GRF_SOC_CON2_LPDDR3_EN1		BIT(11)
#define RK3288_GRF_SOC_CON2_UPCTL0_LPDDR3_ODT	BIT(10)
#define RK3288_GRF_SOC_CON2_UPCTL0_BST_DIS	BIT(9)
#define RK3288_GRF_SOC_CON2_LPDDR3_EN0		BIT(8)


enum rockchip_dmc_type {
	RK3066_DMC,
	RK3188_DMC,
	RK3288_DMC,
};

struct rockchip_dmc_data {
	enum rockchip_dmc_type type;
	int num_channels;

	int msch_offsets[];
};

struct rockchip_dmc_data rk3188_dmc_data = {
	.type = RK3188_DMC,
	.num_channels = 1,

	.msch_offsets = {
		0x00,
	},
};

struct rockchip_dmc_data rk3288_dmc_data = {
	.type = RK3288_DMC,
	.num_channels = 2,

	.msch_offsets = {
		0x00,
		0x80,
	},
};

struct rockchip_dmc {
	struct device *dev;
	struct regmap *grf;
	struct regmap *noc;
	struct rockchip_dmc_data *soc_data;
};

static void rockchip_dmc_dumpregs(struct rockchip_dmc *dmc)
{
	u32 val;

	dev_info(dmc->dev, "reading back noc_timing value\n");
	regmap_read(dmc->noc, RK3066_MSCH_DDRCONF, &val);
	dev_info(dmc->dev, "ddrconf:       0x%x\n", val);
	regmap_read(dmc->noc, RK3066_MSCH_DDRTIMING, &val);
	dev_info(dmc->dev, "ddrtiming:     0x%x\n", val);
	regmap_read(dmc->noc, RK3066_MSCH_READLATENCY, &val);
	dev_info(dmc->dev, "readlatency:   0x%x\n", val);

	if (dmc->soc_data->type == RK3288_DMC) {
		regmap_read(dmc->noc, RK3288_MSCH_ACTIVATE, &val);
		dev_info(dmc->dev, "activate:      0x%x\n", val);
		regmap_read(dmc->noc, RK3288_MSCH_DEVTODEV, &val);
		dev_info(dmc->dev, "devtodev:      0x%x\n", val);
	}

	switch(dmc->soc_data->type) {
	case RK3066_DMC:
		regmap_read(dmc->grf, RK3066_GRF_DDRC_CON0, &val);
		dev_info(dmc->dev, "ddrc_con0:     0x%x\n", val);
		regmap_read(dmc->grf, RK3066_GRF_SOC_CON2, &val);
		dev_info(dmc->dev, "soc_con2:      0x%x\n", val);
		break;
	case RK3188_DMC:
		regmap_read(dmc->grf, RK3188_GRF_DDRC_CON0, &val);
		dev_info(dmc->dev, "ddrc_con0:     0x%x\n", val);
		regmap_read(dmc->grf, RK3188_GRF_SOC_CON2, &val);
		dev_info(dmc->dev, "soc_con2:      0x%x\n", val);
		break;
	case RK3288_DMC:
		regmap_read(dmc->grf, RK3288_GRF_SOC_CON0, &val);
		dev_info(dmc->dev, "soc_con0:     0x%x\n", val);
		regmap_read(dmc->grf, RK3288_GRF_SOC_CON2, &val);
		dev_info(dmc->dev, "soc_con2:      0x%x\n", val);
		break;
	}
}

static const struct of_device_id rockchip_dmc_of_match[] = {
	{ .compatible = "rockchip,rk3188-dmc", .data = &rk3188_dmc_data },
	{ .compatible = "rockchip,rk3288-dmc", .data = &rk3288_dmc_data },
	{},
};
MODULE_DEVICE_TABLE(of, rockchip_dmc_of_match);

static int rockchip_dmc_probe(struct platform_device *pdev)
{
	struct device_node *np = pdev->dev.of_node;
	const struct of_device_id *match;
	struct rockchip_dmc *dmc;

	dmc = devm_kzalloc(&pdev->dev, sizeof(*dmc), GFP_KERNEL);
	if (!dmc)
		return -ENOMEM;

	dmc->dev = &pdev->dev;

	match = of_match_node(rockchip_dmc_of_match, np);
	dmc->soc_data = (struct rockchip_dmc_data *)match->data;

	dmc->grf = syscon_regmap_lookup_by_phandle(np, "rockchip,grf");
	if (IS_ERR(dmc->grf)) {
		dev_err(&pdev->dev, "grf syscon not found\n");
		return PTR_ERR(dmc->grf);
	}

	dmc->noc = syscon_regmap_lookup_by_phandle(np, "rockchip,msch-niu");
	if (IS_ERR(dmc->noc)) {
		dev_err(&pdev->dev, "memory scheduler syscon not found\n");
		return PTR_ERR(dmc->noc);
	}

	platform_set_drvdata(pdev, dmc);

	rockchip_dmc_dumpregs(dmc);

	return 0;
}

static struct platform_driver rockchip_dmc_driver = {
	.probe		= rockchip_dmc_probe,
	.driver		= {
		.name	= "rockchip-dmc",
		.of_match_table = rockchip_dmc_of_match,
	},
};

static int __init rockchip_dmc_init(void)
{
	return platform_driver_register(&rockchip_dmc_driver);
}
module_init(rockchip_dmc_init);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Heiko Stuebner <heiko@sntech.de>");
MODULE_DESCRIPTION("Rockchip core dynamic memory controller");
