#include <linux/acpi.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/machine.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>


#define CIO2_GPIO_REGULATOR(_name, _id, _ops)			\
	{							\
		.name		= _name,			\
		.supply_name	= _name,			\
		.id		= _id,				\
		.ops		= _ops,				\
		.type		= REGULATOR_VOLTAGE,		\
		.owner		= THIS_MODULE,			\
	}

static int cio2_power_gpio_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct gpio_desc *gpio;
	int value;

	gpio = rdev_get_drvdata(rdev);

	value = gpiod_get_value_cansleep(gpio);

	return value;
}

static struct regulator_ops cio2_power_gpio_regulator_ops = {
	.is_enabled         = cio2_power_gpio_regulator_is_enabled,
};

static const struct regulator_desc regulators[] = {
	CIO2_GPIO_REGULATOR("avdd", 0, &cio2_power_gpio_regulator_ops),
	CIO2_GPIO_REGULATOR("dovdd", 0, &cio2_power_gpio_regulator_ops),
};

static const struct gpiod_lookup_table ov2680_gpios = {
	.dev_id = "i2c-OVTI2680:00",
	.table = {
		GPIO_LOOKUP_IDX("gpiochip0", 120, "xshutdn", 0, GPIO_ACTIVE_LOW),
		{ },
	},
};

static int cio2_power_register_regulators(struct device *pmic)
{
	struct gpio_desc *avdd_gpiod, *dovdd_gpiod;
	struct regulator_dev *rdev;
	struct regulator_config avdd_cfg = { };
	struct regulator_config dovdd_cfg = { };
	int ret;

	avdd_cfg.dev = pmic;
	dovdd_cfg.dev = pmic;

	avdd_gpiod = gpiod_get_index(pmic, NULL, 1, GPIOD_ASIS);
	if (!avdd_gpiod) {
		pr_err("Error getting gpio 1\n");
		return -ENODEV;
	}

	avdd_cfg.ena_gpiod = avdd_gpiod;
	avdd_cfg.driver_data = avdd_gpiod;

	rdev = devm_regulator_register(pmic, &regulators[0], &avdd_cfg);
	if (!rdev) {
		pr_err("Failed to register AVDD\n");
		ret = -EINVAL;
		goto err_free_avdd_gpiod;
	}

	dovdd_gpiod = gpiod_get_index(pmic, NULL, 2, GPIOD_ASIS);
	if (!dovdd_gpiod) {
		pr_err("Error getting gpio 2\n");
		ret = -ENODEV;
		goto err_free_avdd;
	}

	dovdd_cfg.ena_gpiod = dovdd_gpiod;
	dovdd_cfg.driver_data = dovdd_gpiod;

	rdev = devm_regulator_register(pmic, &regulators[1], &dovdd_cfg);
	if (!rdev) {
		pr_err("Failed to register DOVDD\n");
		ret = -EINVAL;
		goto err_free_dovdd_gpiod;
	}

	pr_info("Registered regulators successfully\n");

	return 0;

err_free_dovdd_gpiod:
	gpiod_put(dovdd_gpiod);
err_free_avdd:
	regulator_unregister(rdev);
err_free_avdd_gpiod:
	gpiod_put(avdd_gpiod);
	return ret;
}

static void cio2_power_map_sensor_gpios(void)
{
	gpiod_add_lookup_table(&ov2680_gpios);

	pr_info("Mapped xshutdn GPIO successfully\n");
}

static int cio2_power_init(void)
{
	struct acpi_device *sensor_adev;
	struct acpi_device *pmic_adev;
	struct device *sensor;
	struct device *pmic;
	int ret;

	pmic_adev = acpi_dev_get_first_match_dev("INT3472", "1", -1);
	if (!pmic_adev) {
		pr_err("No PMIC acpi_device found\n");
		return -ENODEV;
	}

	pmic = bus_find_device_by_acpi_dev(&platform_bus_type, pmic_adev);
	if (!pmic) {
		pr_err("No pmic device found\n");
		return -ENODEV;
	}

	sensor_adev = acpi_dev_get_first_match_dev("OVTI2680", NULL, -1);
	if (!sensor_adev) {
		pr_err("No Sensor acpi_device found\n");
		return -ENODEV;
	}

	sensor = bus_find_device_by_acpi_dev(&i2c_bus_type, sensor_adev);
	if (!sensor) {
		pr_err("No sensor dev found\n");
		return -ENODEV;
	}

	ret = cio2_power_register_regulators(pmic);
	if (ret) {
		pr_err("Failed to register regulators\n");
		return -EINVAL;
	}

	cio2_power_map_sensor_gpios();

	return 0;
}

static void cio2_power_exit(void)
{

}

module_init(cio2_power_init);
module_exit(cio2_power_exit);

MODULE_AUTHOR("Dan Scally <djrscally@gmail.com>");
MODULE_DESCRIPTION("Experimental module to map GPIO regulators and reset pins to ov2680 for a Lenovo Miix 510");
MODULE_LICENSE("GPL v2");