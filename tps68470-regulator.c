#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/mfd/tps68470.h>
#include <linux/mod_devicetable.h>

/* header maybe */

enum tps68470_regulators {
    TPS68470_CORE,
    TPS68470_ANA,
    TPS68470_VCM,
    TPS68470_VIO,
    TPS68470_VSIO,
    TPS68470_AUX1,
    TPS68470_AUX2,
};

#define TPS68470_REGULATOR(_name, _id, _ops, _n, _vr,	\
			_vm, _er, _em, _t, _lr, _nlr) \
	{						\
		.name		= _name,		\
		.id		= _id,			\
		.ops		= &_ops,		\
		.n_voltages	= _n,			\
		.type		= REGULATOR_VOLTAGE,	\
		.owner		= THIS_MODULE,		\
		.vsel_reg	= _vr,			\
		.vsel_mask	= _vm,			\
		.enable_reg	= _er,			\
		.enable_mask	= _em,			\
		.volt_table	= _t,			\
		.linear_ranges	= _lr,			\
		.n_linear_ranges = _nlr,		\
	}

/* end of header maybe */

static const struct linear_range tps68470_ldo_ranges[] = {
	REGULATOR_LINEAR_RANGE(875000, 0, 125, 17800),
};

static const struct linear_range tps68470_core_ranges[] = {
	REGULATOR_LINEAR_RANGE(900000, 0, 42, 25000),
};

static struct regulator_ops tps68470_regulator_ops = {
    .is_enabled         = regulator_is_enabled_regmap,
    .enable             = regulator_enable_regmap,
    .disable            = regulator_disable_regmap,
    .get_voltage_sel    = regulator_get_voltage_sel_regmap,
    .set_voltage_sel    = regulator_set_voltage_sel_regmap,
    .list_voltage       = regulator_list_voltage_linear_range,
    .map_voltage        = regulator_map_voltage_linear_range,
};

static const struct regulator_desc regulators[] = {
	TPS68470_REGULATOR("CORE", TPS68470_CORE,
			   tps68470_regulator_ops, 43, TPS68470_REG_VDVAL,
			   TPS68470_VDVAL_DVOLT_MASK, TPS68470_REG_VDCTL,
			   TPS68470_VDCTL_EN_MASK,
			   NULL, tps68470_core_ranges,
			   ARRAY_SIZE(tps68470_core_ranges)),
	TPS68470_REGULATOR("ANA", TPS68470_ANA,
			   tps68470_regulator_ops, 126, TPS68470_REG_VAVAL,
			   TPS68470_VAVAL_AVOLT_MASK, TPS68470_REG_VACTL,
			   TPS68470_VACTL_EN_MASK,
			   NULL, tps68470_ldo_ranges,
			   ARRAY_SIZE(tps68470_ldo_ranges)),
	TPS68470_REGULATOR("VCM", TPS68470_VCM,
			   tps68470_regulator_ops, 126, TPS68470_REG_VCMVAL,
			   TPS68470_VCMVAL_VCVOLT_MASK, TPS68470_REG_VCMCTL,
			   TPS68470_VCMCTL_EN_MASK,
			   NULL, tps68470_ldo_ranges,
			   ARRAY_SIZE(tps68470_ldo_ranges)),
	TPS68470_REGULATOR("VIO", TPS68470_VIO,
			   tps68470_regulator_ops, 126, TPS68470_REG_VIOVAL,
			   TPS68470_VIOVAL_IOVOLT_MASK, TPS68470_REG_S_I2C_CTL,
			   TPS68470_S_I2C_CTL_EN_MASK,
			   NULL, tps68470_ldo_ranges,
			   ARRAY_SIZE(tps68470_ldo_ranges)),

/*
 * (1) This register must have same setting as VIOVAL if S_IO LDO is used to
 *     power daisy chained IOs in the receive side.
 * (2) If there is no I2C daisy chain it can be set freely.
 *
 */
	TPS68470_REGULATOR("VSIO", TPS68470_VSIO,
			   tps68470_regulator_ops, 126, TPS68470_REG_VSIOVAL,
			   TPS68470_VSIOVAL_IOVOLT_MASK, TPS68470_REG_S_I2C_CTL,
			   TPS68470_S_I2C_CTL_EN_MASK,
			   NULL, tps68470_ldo_ranges,
			   ARRAY_SIZE(tps68470_ldo_ranges)),
	TPS68470_REGULATOR("AUX1", TPS68470_AUX1,
			   tps68470_regulator_ops, 126, TPS68470_REG_VAUX1VAL,
			   TPS68470_VAUX1VAL_AUX1VOLT_MASK,
			   TPS68470_REG_VAUX1CTL,
			   TPS68470_VAUX1CTL_EN_MASK,
			   NULL, tps68470_ldo_ranges,
			   ARRAY_SIZE(tps68470_ldo_ranges)),
	TPS68470_REGULATOR("AUX2", TPS68470_AUX2,
			   tps68470_regulator_ops, 126, TPS68470_REG_VAUX2VAL,
			   TPS68470_VAUX2VAL_AUX2VOLT_MASK,
			   TPS68470_REG_VAUX2CTL,
			   TPS68470_VAUX2CTL_EN_MASK,
			   NULL, tps68470_ldo_ranges,
			   ARRAY_SIZE(tps68470_ldo_ranges)),
};

static int tps68470_regulator_probe(struct platform_device *pdev)
{
    struct regulator_config config = {};
    struct regulator_dev *rdev;
    struct regmap *tps68470_regmap;
    int i;

    tps68470_regmap = dev_get_drvdata(pdev->dev.parent);

    config.dev = &pdev->dev;
    config.regmap = tps68470_regmap;

    for (i = 0; i < ARRAY_SIZE(regulators); i++) {

        rdev = devm_regulator_register(&pdev->dev, &regulators[i], &config);

        if (IS_ERR(rdev)) {
            dev_err(pdev->dev.parent, "Failed to register %s regulator.\n", pdev->name);
            return PTR_ERR(rdev);
        }
    }

    return 0;
}

static const struct platform_device_id tps68470_regulator_id_table[] = {
    { "tps68470-regulator", 0 },
    { },
};
MODULE_DEVICE_TABLE(platform, tps68470_regulator_id_table);

static struct platform_driver tps68470_regulator_driver = {
    .driver = {
        .name = "tps68470-regulator",
    },
    .probe = tps68470_regulator_probe,
    .id_table = tps68470_regulator_id_table,
};
module_platform_driver(tps68470_regulator_driver);

MODULE_AUTHOR("Daniel Scally <djrscally@gmail.com>");
MODULE_DESCRIPTION("TPS68470 Regulator Driver");
MODULE_LICENSE("GPL v2");