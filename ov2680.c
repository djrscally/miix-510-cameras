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
#include <linux/pm_runtime.h>
#include <linux/gpio/machine.h>
#include <linux/regmap.h>

#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>

#include "ov2680.h"

static struct ov2680_device *to_ov2680_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov2680_device, sd);
}


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

static int ov2680_check_ov2680_id(struct i2c_client *client)
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
		dev_err(&client->dev, "ov2680_id_high = 0x%x\n", high);
		return -ENODEV;
	}
	ret = ov2680_read_reg(client, OV2680_8BIT,
					OV2680_ID_REG_LOW, &low);
	id = ((((u16) high) << 8) | (u16) low);

     printk(KERN_CRIT "ov2680: Chip ID Fetched = 0x%04x", id);

	if (id != OV2680_ID) {
		dev_err(&client->dev, "ov2680 ID error 0x%x\n", id);
		return -ENODEV;
	}

	ret = ov2680_read_reg(client, OV2680_8BIT,
					OV2680_SC_CMMN_SUB_ID, &high);
	revision = (u8) high & 0x0f;

	printk(KERN_CRIT, "ov2680: ov2680 revision=0x%x\n", revision);
	printk(KERN_CRIT "detect ov2680 success\n");
	dev_dbg(&client->dev, "ov2680_revision = 0x%x\n", revision);
	dev_dbg(&client->dev, "detect ov2680 success\n");

	return 0;
}

static int ov2680_configure_gpios(struct ov2680_device *ov2680)
/*
 * Get the GPIO pins that are **supplied** by the PMIC and **consumed**
 * by the camera module, rather than the ones from the gpiochip0 that
 * are **consumed** by the INT3472. These are the pins that supply our
 * voltage and reset pin. I actually have no idea which pin is wired to
 * which pad on the sensor, but I'm assuming enable and idle are the 
 * voltage rails and resetn is the XSHUTDN pin, because I'd wire it 
 * that way if I was wiring it.
 */
{
	gpiod_add_lookup_table(ov2680->gpios);

	ov2680->s_enable = gpiod_get_index(&ov2680->client->dev, "s_enable", 0, GPIOD_OUT_HIGH);

	if (ov2680->s_enable == NULL) {
		dev_err(&ov2680->client->dev, "Error fetching s_enable.\n");
		return -1;
	} 

	ov2680->s_idle = gpiod_get_index(&ov2680->client->dev, "s_idle", 0, GPIOD_OUT_HIGH);

	if (ov2680->s_idle == NULL) {
		dev_err(&ov2680->client->dev, "Error fetching s_idle.\n");
		return -1;
	} 

	ov2680->s_resetn = gpiod_get_index(&ov2680->client->dev, "s_resetn", 0, GPIOD_OUT_HIGH);

	if (ov2680->s_resetn == NULL) {
		dev_err(&ov2680->client->dev, "Error fetching s_resetn.\n");
		return -1;
	} 

	/* set em all low initially */
	gpiod_set_value_cansleep(ov2680->s_enable, 0);
	gpiod_set_value_cansleep(ov2680->s_idle, 0);
	gpiod_set_value_cansleep(ov2680->s_resetn, 0);
	
	return 0;
}

static int ov2680_configure_regulators(struct ov2680_device *ov2680)
/*
 * Just get the power regulators.
*/
{
	int ret;

    /* configure and enable regulators */
	int i;
    for (i = 0; i < OV2680_NUM_SUPPLIES; i++) {
	    ov2680->supplies[i].supply = ov2680_supply_names[i];
    }

    ret = devm_regulator_bulk_get(&ov2680->client->dev, OV2680_NUM_SUPPLIES, ov2680->supplies);

	return ret;
}

static int ov2680_power_on(struct ov2680_device *ov2680)
/*
 * Powers on the camera sensor. The process for this is to bring online 
 * the regulators, then after a short pause turn on the GPIO pins to the sensor.
 * We can't actually do that, because we don't know how the PMIC is wired to it!
 * But it seems that turning on the PMIC itself does the trick, so that'll do
*/
{
	int ret;

	if (ov2680->is_enabled) {
		dev_dbg(&ov2680->client->dev, "ov2680_power_on called when chip already is_enabled.\n");
		return 0;
	}

	ret = regulator_bulk_enable(OV2680_NUM_SUPPLIES, ov2680->supplies);
	if (ret) {
		dev_err(&ov2680->client->dev, "Failed to enable regulators\n");
		return ret;
	}

    gpiod_set_value_cansleep(ov2680->s_enable, 1);
    gpiod_set_value_cansleep(ov2680->s_idle, 1);

    usleep_range(10000, 11000);

	gpiod_set_value_cansleep(ov2680->s_resetn, 1);

	/*
	 * The ov2680 datasheet specifies a short pause between turning the chip on and the
	 * first i2c message.
	*/

    usleep_range(10000, 11000);

	ov2680->is_enabled = 1;

	return 0;
}

static int ov2680_power_off(struct ov2680_device *ov2680)
{
	int ret;

	if (!ov2680->is_enabled) {
		dev_dbg(&ov2680->client->dev, "ov2680_power_off called when chip already offline.\n");
		return 0;
	}

	gpiod_set_value_cansleep(ov2680->s_resetn, 0);
	usleep_range(10000, 11000);

	gpiod_set_value_cansleep(ov2680->s_enable, 0);
	gpiod_set_value_cansleep(ov2680->s_idle, 0);

	ret = regulator_bulk_disable(OV2680_NUM_SUPPLIES, ov2680->supplies);

	if (ret) {
		dev_dbg(&ov2680->client->dev, "Error disabling the regulators.\n");
		return 1;
	}

	ov2680->is_enabled = 0;

	return 0;
}

static int ov2680_s_power(struct v4l2_subdev *sd, int on)
{
	int ret;
	struct ov2680_device *ov2680;

	ov2680 = to_ov2680_dev(sd);

	if (on) {
		ret = ov2680_power_on(ov2680);

		if (ret) {
			ov2680_power_off(ov2680);
			dev_err(&ov2680->client->dev, "Error powering on ov2680; disabling.\n");
			return 1;
		}

		return 0;
	} else {
		ret = ov2680_power_off(ov2680);
		return 0;
	}
}

static int ov2680_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ov2680_device *ov2680 = to_ov2680_dev(sd);
	int ret;

	if (ov2680->is_streaming == enable) {
		dev_dbg(&ov2680->client->dev, "Attempt to set stream=%d, but it is already in that state.\n", enable);
		return 0;
	}

	ret = ov2680_write_reg(ov2680->client, OV2680_8BIT, OV2680_REG_STREAM_CTRL, enable);
	ov2680->is_streaming = enable;

	if (ret) {
		dev_err(&ov2680->client->dev, "An error occurred setting stream enabled=%d.\n", enable);
	}

	return ret;
}

static int ov2680_register(struct v4l2_subdev *sd)
{
	printk(KERN_CRIT "Registered subdev %s\n", sd->name);
	return 0;
}

static const struct v4l2_subdev_internal_ops ov2680_internal_ops = {
	.registered		= ov2680_register,
};

static const struct v4l2_subdev_core_ops ov2680_core_ops = {
	.s_power		= ov2680_s_power,
};

static const struct v4l2_subdev_video_ops ov2680_video_ops = {
	.s_stream		= ov2680_s_stream,
};

static const struct v4l2_subdev_ops ov2680_subdev_ops = {
	.core 			= &ov2680_core_ops,
	.video			= &ov2680_video_ops,
};

static const struct media_entity_operations ov2680_subdev_entity_ops = {
	.link_validate	= v4l2_subdev_link_validate,
};

static int ov2680_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd;
	struct ov2680_device *ov2680;

	sd = i2c_get_clientdata(client);
	ov2680 = to_ov2680_dev(sd);

	if (ov2680 == NULL) {
		printk(KERN_CRIT "ov2680: .remove function couldn't fetch clientdata.\n");
		return -100;
	}

	ov2680_power_off(ov2680);

	gpiod_put(ov2680->s_enable);
	gpiod_put(ov2680->s_idle);
	gpiod_put(ov2680->s_resetn);

	gpiod_remove_lookup_table(ov2680->gpios);

	v4l2_device_unregister_subdev(sd);

	return 0;
}

static int ov2680_probe(struct i2c_client *client)
{
	struct ov2680_device 		*ov2680;
	int 						ret;
	struct clk     				*xvclk;

	ov2680 = kzalloc(sizeof(*ov2680), GFP_KERNEL);
	if (!ov2680) {
		dev_err(&client->dev, "out of memory\n");
		return -ENOMEM;
	}

	/* Sensor 'aint on, tell it so */
	ov2680->is_enabled = 0;

	/* First, tie i2c_client to ov2680_device, and vice versa */
	ov2680->client = client;

	/* Next, set the GPIO lookup table so we can find the control pins from the PMIC */

	ov2680->gpios = &ov2680_gpios;

	/* Configure the GPIO pins */
	ret = ov2680_configure_gpios(ov2680);

	if (ret < 0) {
		dev_dbg(&client->dev, "Unable to configure GPIO pins. Device initialisation failed.\n");
		goto remove_out;
	}

	/* Configure the power regulators */
	ret = ov2680_configure_regulators(ov2680);

	if (ret) {
		dev_dbg(&client->dev, "Could not configure regulators.\n");
		goto remove_out;
	}

	ret = ov2680_power_on(ov2680);

	if (ret) {
		dev_dbg(&client->dev, "Could not power on the ov2680 due to a regulator fault.\n");
		goto remove_out;
	}

	xvclk = devm_clk_get(&ov2680->client->dev, "xvclk");

	if (IS_ERR(xvclk)) {
		dev_err(&client->dev, "xvclk clock missing or invalid.\n");
	}

    ret = ov2680_check_ov2680_id(client);

	if (ret) {
		dev_dbg(&client->dev, "The sensor could not be initialised.\n");
		goto remove_out;
	}

	/* v4l2 infrastructure initialisation */
	v4l2_i2c_subdev_init(&ov2680->sd, ov2680->client, &ov2680_subdev_ops);

	/* don't really know wtf this bit does */
	ov2680->sd.internal_ops = &ov2680_internal_ops;
	ov2680->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ov2680->pad.flags = MEDIA_PAD_FL_SOURCE;
	ov2680->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ov2680->sd.entity.ops = &ov2680_subdev_entity_ops;

	ret = media_entity_pads_init(&ov2680->sd.entity, 1, &ov2680->pad);

	if (ret) {
		printk(KERN_CRIT "media entity initialisation failed.\n");
		goto remove_out;
	}

	printk(KERN_CRIT "media entity initialised as %s.\n", ov2680->sd.entity.name);

	printk(KERN_CRIT "v4l2 subdev registered as %s.\n", ov2680->sd.name);

	ret = v4l2_async_register_subdev_sensor_common(&ov2680->sd);

	if (ov2680->sd.v4l2_dev != NULL) {
		printk(KERN_CRIT "registered against the device %s.\n", ov2680->sd.v4l2_dev->name);
	} else {
		printk(KERN_CRIT "Yeah it's null mate.\n");
	}

	if (ret < 0) {
		dev_err(&ov2680->client->dev, "An error occurred registering the subdev.\n");
		printk(KERN_CRIT "oops.\n");
		goto remove_out;
	}

    return 0;

remove_out:
	dev_dbg(&client->dev, "calling ov2680_remove\n");
	ret = ov2680_remove(client);

	return ret;
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
MODULE_DESCRIPTION("A driver for OmniVision 2680 Camera");
MODULE_LICENSE("GPL");
