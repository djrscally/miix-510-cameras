#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/acpi.h>
#include <linux/regulator/consumer.h>
#include <linux/mfd/core.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>

#include "ov5648.h"

static int ov5648_read_reg(struct i2c_client *client,
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

	if (data_length != OV5648_8BIT && data_length != OV5648_16BIT
					&& data_length != OV5648_32BIT) {
		dev_err(&client->dev, "%s error, invalid data length\n",
			__func__);
		return -EINVAL;
	}

	memset(msg, 0 , sizeof(msg));

     printk(KERN_CRIT "ov5648: reading from addr 0x%02x, reg 0x%04x", client->addr, reg);

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
	if (data_length == OV5648_8BIT)
		*val = (u8)data[0];
	else if (data_length == OV5648_16BIT)
		*val = be16_to_cpu(*(u16 *)&data[0]);
	else
		*val = be32_to_cpu(*(u32 *)&data[0]);

	return 0;
}

static int ov5648_i2c_write(struct i2c_client *client, u16 len, u8 *data)
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

static int ov5648_write_reg(struct i2c_client *client, u16 data_length,
							u16 reg, u16 val)
{
	int ret;
	unsigned char data[4] = {0};
	u16 *wreg = (u16 *)data;
	const u16 len = data_length + sizeof(u16); /* 16-bit address + data */

	if (data_length != OV5648_8BIT && data_length != OV5648_16BIT) {
		dev_err(&client->dev,
			"%s error, invalid data_length\n", __func__);
		return -EINVAL;
	}

	/* high byte goes out first */
	*wreg = cpu_to_be16(reg);

	if (data_length == OV5648_8BIT) {
		data[2] = (u8)(val);
	} else {
		/* OV5648_16BIT */
		u16 *wdata = (u16 *)&data[2];
		*wdata = cpu_to_be16(val);
	}

	ret = ov5648_i2c_write(client, len, data);
	if (ret)
		dev_err(&client->dev,
			"write error: wrote 0x%x to offset 0x%x error %d",
			val, reg, ret);

	return ret;
}

static int ov5648_write_reg_array(struct i2c_client *client,
			        const struct ov5648_reg *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].type != OV5648_TOK_TERM; i++)
		ret = ov5648_write_reg(client, regs[i].type, regs[i].reg, regs[i].val);

	return ret;
}

static int ov5648_check_ov5648_id(struct i2c_client *client)
{
	struct i2c_adapter *adapter = client->adapter;
	u16 high, low;
	int ret;
	u16 id;
	u8 revision;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

	ret = ov5648_read_reg(client, OV5648_8BIT,
					OV5648_ID_REG_HIGH, &high);
	if (ret) {
		dev_err(&client->dev, "ov5648_id_high = 0x%x\n", high);
		return -ENODEV;
	}
	ret = ov5648_read_reg(client, OV5648_8BIT,
					OV5648_ID_REG_LOW, &low);
	id = ((((u16) high) << 8) | (u16) low);

     printk(KERN_CRIT "ov5648: Chip ID Fetched = 0x%04x", id);

	if (id != OV5648_ID) {
		dev_err(&client->dev, "ov5648 ID error 0x%x\n", id);
		return -ENODEV;
	}

	ret = ov5648_read_reg(client, OV5648_8BIT,
					OV5648_SC_CMMN_SUB_ID, &high);
	revision = (u8) high & 0x0f;

	printk(KERN_CRIT, "ov5648: ov5648 revision=0x%x\n", revision);
	printk(KERN_CRIT "detect ov5648 success\n");
	dev_dbg(&client->dev, "ov5648_revision = 0x%x\n", revision);
	dev_dbg(&client->dev, "detect ov5648 success\n");

	return 0;
}

static int match_depend(struct device *dev, const void *data)
{
     return (dev && dev->fwnode == data) ? 1 : 0;
}

static int ov5648_probe(struct i2c_client *client)
{
	struct ov5648_device 		*ov5648;
	struct i2c_board_info		*binfo;
	struct device          		*int3472_device;
	struct acpi_device			*int3472_acpi_device;
	int 						ret;
	struct i2c_client			*nclient;


     ov5648 = kzalloc(sizeof(*ov5648), GFP_KERNEL);
     if (!ov5648) {
          dev_err(&client->dev, "out of memory\n");
          return -ENOMEM;
     }

	printk(KERN_CRIT "ov5648: I2C Addr is 0x%02x\n", client->addr);
	 /* First need to get the 1st (rather than 0th) I2cSerialBus resource */
	nclient = i2c_acpi_new_device(&client->dev, 1, binfo);

	if (nclient == NULL) {
		printk(KERN_CRIT "ov5648: An error occurred loading the 1st i2cserialbus\n");
		return -ENODEV;
	}

	printk(KERN_CRIT "ov5648 New I2C Device instantiated correctly\n");

	printk(KERN_CRIT "ov5648: I2C Addr is 0x%02x\n", client->addr);

     /* Tie i2c_client to ov5648_device, and vice versa */
     ov5648->client = client;
     i2c_set_clientdata(client, ov5648);

	int3472_acpi_device = acpi_dev_get_first_match_dev("INT3472", "1", -1);

	if (int3472_acpi_device == NULL) {
		printk(KERN_CRIT "ov5648: error fetching INT3472 acpi_device.\n");
		return -EINVAL;
	}

	/* int3472_device = bus_find_device(&platform_bus_type, NULL, &int3472_acpi_device->fwnode, match_depend); */
	int3472_device = bus_find_device_by_acpi_dev(&platform_bus_type, int3472_acpi_device);

	if (int3472_device == NULL) {
		printk(KERN_CRIT "ov5648: error fetching INT3472 device.\n");
		return -EINVAL;
	}

	int3472_acpi_device->dev = *int3472_device;

    /* configure and enable regulators */
	int i;
    for (i = 0; i < OV5648_NUM_SUPPLIES; i++) {
	    ov5648->supplies[i].supply = ov5648_supply_names[i];
    }

    devm_regulator_bulk_get(&client->dev, OV5648_NUM_SUPPLIES, ov5648->supplies);

    ov5648->xshutdn = gpiod_get_index(int3472_device, NULL, 0, GPIOD_ASIS);

	if (IS_ERR(ov5648->xshutdn)) {
		printk(KERN_CRIT "ov5648: An error occurred fetching XSHUTDN.\n");
	}
    ov5648->pwdnb = gpiod_get_index(int3472_device, NULL, 1, GPIOD_ASIS);
    ov5648->led = gpiod_get_index(int3472_device, NULL, 2, GPIOD_ASIS);


    
    /* POWER ON BABY YEAH */

    gpiod_set_value_cansleep(ov5648->xshutdn, 0);
    gpiod_set_value_cansleep(ov5648->pwdnb, 0);
    gpiod_set_value_cansleep(ov5648->led, 0);

	ret = regulator_bulk_enable(OV5648_NUM_SUPPLIES, ov5648->supplies);
	if (ret) {
		printk(KERN_CRIT "ov5648: failed to enable regulators\n");
		return ret;
	}

    usleep_range(10000, 11000);

 /*   gpiod_set_value_cansleep(ov5648->xshutdn, 1); */
    gpiod_set_value_cansleep(ov5648->pwdnb, 1);
    gpiod_set_value_cansleep(ov5648->led, 1);

    usleep_range(10000, 11000);

    ov5648_check_ov5648_id(client);

    return 0;
}

static int ov5648_remove(struct i2c_client *client)
{
     /*
      * Code goes here to get acpi_device, turn off all
      * the GPIO pins, remove them from the ACPI device
      * and whatnot
      */

     struct ov5648_device *ov5648;

     ov5648 = i2c_get_clientdata(client);

	 if (ov5648 == NULL) {
		 printk(KERN_CRIT "ov5648: .remove function couldn't fetch clientdata.\n");
		 return -100;
	 }


     gpiod_set_value_cansleep(ov5648->xshutdn, 0);
     gpiod_set_value_cansleep(ov5648->pwdnb, 0);
     gpiod_set_value_cansleep(ov5648->led, 0);

     gpiod_put(ov5648->xshutdn);
     gpiod_put(ov5648->pwdnb);
     gpiod_put(ov5648->led);

	regulator_bulk_disable(OV5648_NUM_SUPPLIES, ov5648->supplies);

     return 0;
}

static const struct acpi_device_id ov5648_acpi_match[] = {
     {"OVTI5648", 0},
     { },
};

MODULE_DEVICE_TABLE(acpi, ov5648_acpi_match);

static struct i2c_driver ov5648_driver = {
     .driver = {
          .name = "ov5648",
          .acpi_match_table = ov5648_acpi_match,
     },
     .probe_new = ov5648_probe,
     .remove = ov5648_remove,
};

module_i2c_driver(ov5648_driver);

MODULE_AUTHOR("Dan Scally <djrscally@protonmail.com>");
MODULE_DESCRIPTION("A driver for OmniVision 5648 Camera");
MODULE_LICENSE("GPL");
