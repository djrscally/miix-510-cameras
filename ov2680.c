#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/acpi.h>
#include <linux/regulator/consumer.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>

#include "ov2680.h"

static int ov2680_read_reg(struct i2c_client *client,
			   u16 data_length, u16 reg, u16 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[6];

	if (!client->adapter) {
		dev_err(&client->dev, "%s error, no client->adapter\n",
			__func__);
		return -ENODEV;
	}

	if (data_length != OV2680_8BIT && data_length != OV2680_16BIT
					&& data_length != OV2680_32BIT) {
		dev_err(&client->dev, "%s error, invalid data length\n",
			__func__);
		return -EINVAL;
	}

	memset(msg, 0 , sizeof(msg));

    printk(KERN_CRIT "ov2680: reading from addr 0x%02x, reg 0x%04x", client->addr, reg);

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = I2C_MSG_LENGTH;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8)(reg >> 8);
	data[1] = (u8)(reg & 0xff);

	msg[1].addr = client->addr;
	msg[1].len = data_length;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;

	err = i2c_transfer(client->adapter, msg, 2);
	if (err != 2) {
		if (err >= 0)
			err = -EIO;
		dev_err(&client->dev,
			"read from offset 0x%x error %d", reg, err);
		return err;
	}

	*val = 0;
	/* high byte comes first */
	if (data_length == OV2680_8BIT)
		*val = (u8)data[0];
	else if (data_length == OV2680_16BIT)
		*val = be16_to_cpu(*(u16 *)&data[0]);
	else
		*val = be32_to_cpu(*(u32 *)&data[0]);

	return 0;
}

static int ov2680_i2c_write(struct i2c_client *client, u16 len, u8 *data)
{
	struct i2c_msg msg;
	const int num_msg = 1;
	int ret;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = data;
	ret = i2c_transfer(client->adapter, &msg, 1);

	return ret == num_msg ? 0 : -EIO;
}

static int ov2680_write_reg(struct i2c_client *client, u16 data_length,
							u16 reg, u16 val)
{
	int ret;
	unsigned char data[4] = {0};
	u16 *wreg = (u16 *)data;
	const u16 len = data_length + sizeof(u16); /* 16-bit address + data */

	if (data_length != OV2680_8BIT && data_length != OV2680_16BIT) {
		dev_err(&client->dev,
			"%s error, invalid data_length\n", __func__);
		return -EINVAL;
	}

	/* high byte goes out first */
	*wreg = cpu_to_be16(reg);

	if (data_length == OV2680_8BIT) {
		data[2] = (u8)(val);
	} else {
		/* OV2680_16BIT */
		u16 *wdata = (u16 *)&data[2];
		*wdata = cpu_to_be16(val);
	}

	ret = ov2680_i2c_write(client, len, data);
	if (ret)
		dev_err(&client->dev,
			"write error: wrote 0x%x to offset 0x%x error %d",
			val, reg, ret);

	return ret;
}

static int ov2680_write_reg_array(struct i2c_client *client,
			      const struct ov2680_reg *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].type != OV2680_TOK_TERM; i++)
		ret = ov2680_write_reg(client, regs[i].type, regs[i].reg, regs[i].val);

	return ret;
}

static int ov2680_check_sensor_id(struct i2c_client *client)
{
	struct i2c_adapter *adapter = client->adapter;
	u16 high, low;
	int ret;
	u16 id;
	u8 revision;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

	ret = ov2680_read_reg(client, OV2680_8BIT,
					OV2680_ID_REG_HIGH, &high);
	if (ret) {
		dev_err(&client->dev, "sensor_id_high = 0x%x\n", high);
		return -ENODEV;
	}
	ret = ov2680_read_reg(client, OV2680_8BIT,
					OV2680_ID_REG_LOW, &low);
	id = ((((u16) high) << 8) | (u16) low);

    printk(KERN_CRIT "ov2680: Chip ID Fetched = 0x%04x", id);

	if (id != OV2680_ID) {
		dev_err(&client->dev, "sensor ID error 0x%x\n", id);
		return -ENODEV;
	}

	ret = ov2680_read_reg(client, OV2680_8BIT,
					OV2680_SC_CMMN_SUB_ID, &high);
	revision = (u8) high & 0x0f;

	dev_dbg(&client->dev, "sensor_revision = 0x%x\n", revision);
	dev_dbg(&client->dev, "detect ov2680 success\n");

	return 0;
}

static int match_depend(struct device *dev, const void *data)
{
    return (dev && dev->fwnode == data) ? 1 : 0;
}

static int ov2680_probe(struct i2c_client *client)
{
    struct ov2680_device *sensor;

    sensor = kzalloc(sizeof(*sensor), GFP_KERNEL);
    if (!sensor) {
        dev_err(&client->dev, "out of memory\n");
        return -ENOMEM;
    }

    /* Tie i2c_client to ov2680_device, and vice versa */
    sensor->client = client;
    i2c_set_clientdata(client, sensor);

    /*
     * The driver will match the OV2680 device, but the GPIO
     * pins lie in its dependent INT3472, so we need to walk
     * up the dependencies to find that device.
    */
   struct acpi_device   *int3472_device;
   struct device        *dev;

   /* get ACPI handle of OV2680 device */
   struct acpi_handle *dev_handle = ACPI_HANDLE(&client->dev);

   /* Get dependent devices */
   struct acpi_handle_list dep_devices;
   acpi_evaluate_reference(dev_handle, "_DEP", NULL, &dep_devices);

   int i;
   for (i=0;i<dep_devices.count;i++) {
       struct acpi_device_info *devinfo;
       acpi_get_object_info(dep_devices.handles[i], &devinfo);

       if (devinfo->valid & ACPI_VALID_HID && !strcmp(devinfo->hardware_id.string, "INT3472")) {
            acpi_bus_get_device(dep_devices.handles[i], &int3472_device);
            dev = bus_find_device(&platform_bus_type, NULL, &int3472_device->fwnode, match_depend);
            int3472_device->dev = *dev;
       }
   }

   int ret;

   /* configure and enable regulators */
   for (i = 0; i < OV2680_NUM_SUPPLIES; i++) {
	   sensor->supplies[i].supply = ov2680_supply_names[i];
   }

   devm_regulator_bulk_get(&client->dev, OV2680_NUM_SUPPLIES, sensor->supplies);

   /* ret = acpi_dev_add_driver_gpios(int3472_device, int3472_acpi_gpios); */

   sensor->gpio1 = gpiod_get_index(&int3472_device->dev, NULL, 0, GPIOD_ASIS);
   sensor->gpio2 = gpiod_get_index(&int3472_device->dev, NULL, 1, GPIOD_ASIS);
   sensor->gpio3 = gpiod_get_index(&int3472_device->dev, NULL, 2, GPIOD_ASIS);
   
   /* POWER ON BABY YEAH */

   gpiod_set_value_cansleep(sensor->gpio1, 0);
   gpiod_set_value_cansleep(sensor->gpio2, 0);
   gpiod_set_value_cansleep(sensor->gpio3, 0);

   gpiod_set_value(sensor->gpio1, 0);
   usleep_range(10000, 11000);

   gpiod_set_value_cansleep(sensor->gpio1, 1);
   gpiod_set_value_cansleep(sensor->gpio2, 1);
   gpiod_set_value_cansleep(sensor->gpio3, 1);

   ov2680_check_sensor_id(client);

   return 0;
}

static int ov2680_remove(struct i2c_client *client)
{
    /*
     * Code goes here to get acpi_device, turn off all
     * the GPIO pins, remove them from the ACPI device
     * and whatnot
     */

    struct ov2680_device *sensor;

    sensor = i2c_get_clientdata(client);

    gpiod_set_value_cansleep(sensor->gpio1, 0);
    gpiod_set_value_cansleep(sensor->gpio2, 0);
    gpiod_set_value_cansleep(sensor->gpio3, 0);

    gpiod_put(sensor->gpio1);
    gpiod_put(sensor->gpio2);
    gpiod_put(sensor->gpio3);

   kzfree(sensor);

    return 0;
}

static const struct acpi_device_id ov2680_acpi_match[] = {
    {"OVTI2680", 0},
    { },
};

MODULE_DEVICE_TABLE(acpi, ov2680_acpi_match);

static struct i2c_driver ov2680_driver = {
    .driver = {
        .name = "ov2680",
        .acpi_match_table = ov2680_acpi_match,
    },
    .probe_new = ov2680_probe,
    .remove = ov2680_remove,
};

module_i2c_driver(ov2680_driver);

MODULE_AUTHOR("Dan Scally <djrscally@protonmail.com>");
MODULE_DESCRIPTION("A driver for OmniVision 2680 sensors");
MODULE_LICENSE("GPL");
