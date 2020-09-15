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
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/gpio/machine.h>
#include <linux/regmap.h>
#include <linux/mfd/tps68470.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>

#include "ov2680.h"

static struct ov2680_device *to_ov2680_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov2680_device, sd);
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct ov2680_device,
			     ctrls.handler)->sd;
}

static int ov2680_read_reg(struct i2c_client *client, u16 data_length, u16 reg, u32 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[6];

	dev_info (&client->dev, "%s was called.\n", __func__);

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

	dev_info(&client->dev, "    read val 0x%06x from reg 0x%04x\n", *val, reg);

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

	dev_info (&client->dev, "%s was called.\n", __func__);
	dev_info(&client->dev, "    Writing val 0x%04x to reg 0x%04x\n", val, reg);

	if (data_length != OV2680_8BIT && data_length != OV2680_16BIT && data_length != OV2680_24BIT) {
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

static int ov2680_mod_reg(struct ov2680_device *sensor, u16 reg, u8 mask, u16 val)
{
	u32 readval;
	int ret;

	dev_info (&sensor->client->dev, "%s was called.\n", __func__);

	ret = ov2680_read_reg(sensor->client, 1, reg, &readval);
	if (ret < 0)
		return ret;

	readval &= ~mask;
	val &= mask;
	val |= readval;

	return ov2680_write_reg(sensor->client, 1, reg, val);
}

static int ov2680_load_regs(struct ov2680_device *sensor, const struct ov2680_mode_info *mode)
{
	const struct reg_value *regs = mode->reg_data;
	unsigned int i;
	int ret = 0;
	u16 reg_addr;
	u16 val;

	dev_info (&sensor->client->dev, "%s was called.\n", __func__);
	dev_info(&sensor->client->dev, "    for mode %s\n", mode->name);

	for (i = 0; i < mode->reg_data_size; ++i, ++regs) {
		reg_addr = regs->reg_addr;
		val = regs->val;

		ret = ov2680_write_reg(sensor->client, 1, reg_addr, val);
		if (ret)
			break;
	}

	return ret;
}

static int ov2680_check_ov2680_id(struct i2c_client *client)
{
	struct i2c_adapter *adapter = client->adapter;
	u32 high, low;
	int ret;
	u16 id;
	u8 revision;

	dev_info (&client->dev, "%s was called.\n", __func__);

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

	if (id != OV2680_ID) {
		dev_err(&client->dev, "ov2680 ID error 0x%x\n", id);
		return -ENODEV;
	}

	ret = ov2680_read_reg(client, OV2680_8BIT,
					OV2680_SC_CMMN_SUB_ID, &high);
	revision = (u8) high & 0x0f;

	dev_info(&client->dev, "ov2680 successfully detected\n");

	return 0;
}

static int ov2680_get_pmic_dev(struct ov2680_device *ov2680)
/*
 * The camera sensor is off by default, and needs to be activated
 * by triggering the GPIO inputs to a TPS68470 power management IC
 * that sits in front of it. We need to fetch the device struct 
 * for that device by parsing ACPI
 */
{
	struct acpi_device			*int3472_acpi_device;			/* _HID of the PMIC is INT3472 */
	struct device				*int3472_device;

	/*
	 * In ACPI tables the correct device has _HID INT3472, _UID One. This is
	 * how we can identify it uniquely.
	*/
	int3472_acpi_device = acpi_dev_get_first_match_dev("INT3472", "1", -1);

	if (!int3472_acpi_device) {
		dev_dbg(&ov2680->client->dev, "An error occurred fetching the PMIC's ACPI device.\n");
		return -ENODEV;
	}

	/*
	 * Now that we have the ACPI dev, we can use that to match to the physical device.
	*/

	int3472_device = bus_find_device_by_acpi_dev(&platform_bus_type, int3472_acpi_device);

	if (!int3472_device) {
		dev_dbg(&ov2680->client->dev, "An error occurred fetching the PMIC's physical device.\n");
		return -ENODEV;
	}

	/*
	 * Store the value for use later configuring things.
	*/

	ov2680->pmic_dev = int3472_device;

	return 0;
}

static int ov2680_configure_gpios(struct ov2680_device *ov2680)
/*
 * The PMIC consumes 3 GPIO pins. Heaven only knows what the first does,
 * but the latter two turn on both the PMIC and the camera sensor, so
 * we need to be able to use them.
*/
{
	int ret;

	ret = ov2680_get_pmic_dev(ov2680);

	if (ret) {
		dev_err(&ov2680->client->dev, "Error fetching PMIC device\n");
		return ret;
	}
	ov2680->gpio0 = gpiod_get_index(ov2680->pmic_dev, NULL, 1, GPIOD_ASIS);

	if (!ov2680->gpio0) {
		dev_err(&ov2680->client->dev, "Error fetching GPIO0. Device cannot be powered on\n");
		return -EINVAL;
	}

	ov2680->gpio1 = gpiod_get_index(ov2680->pmic_dev, NULL, 2, GPIOD_ASIS);

	if (!ov2680->gpio1) {
		dev_err(&ov2680->client->dev, "Error fetching GPIO1. Device cannot be powered on\n");
		return -EINVAL;
	}

	/* pull both pins low initially */
	gpiod_set_value_cansleep(ov2680->gpio0, 0);
	gpiod_set_value_cansleep(ov2680->gpio1, 0);

	return 0;
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

	dev_info (&ov2680->client->dev, "%s was called.\n", __func__);

	if (ov2680->is_enabled)
		dev_info(&ov2680->client->dev, "ov2680_power_on called when chip already is_enabled.\n");

    gpiod_set_value_cansleep(ov2680->gpio0, 1);
    gpiod_set_value_cansleep(ov2680->gpio1, 1);

	/*
	 * The ov2680 datasheet specifies a short pause between turning the chip on and the
	 * first i2c message.
	*/

    usleep_range(10000, 11000);

	ov2680->is_enabled = 1;

	/* switch on / off */
	ov2680_write_reg(ov2680->client, OV2680_8BIT, OV2680_REG_STREAM_CTRL, 0x01);
	usleep_range(1000, 2000);
	ov2680_write_reg(ov2680->client, OV2680_8BIT, OV2680_REG_STREAM_CTRL, 0x00);

	return 0;
}

static int ov2680_power_off(struct ov2680_device *ov2680)
{
	int ret;

	dev_info (&ov2680->client->dev, "%s was called.\n", __func__);

	if (!ov2680->is_enabled) {
		dev_info(&ov2680->client->dev, "ov2680_power_off called when chip already offline.\n");
		return 0;
	}

	clk_disable_unprepare(ov2680->clk);

	gpiod_set_value_cansleep(ov2680->s_resetn, 0);
	usleep_range(10000, 11000);
	
	gpiod_set_value_cansleep(ov2680->s_enable, 0);
	gpiod_set_value_cansleep(ov2680->s_idle, 0);
	
	ret = regulator_bulk_disable(OV2680_NUM_SUPPLIES, ov2680->supplies);
	
	if (ret) {
		dev_err(&ov2680->client->dev, "Error disabling the regulators.\n");
		return 1;
	}

	ov2680->is_enabled = 0;

	return 0;
}

/* v4l2 gubbins here */

static int ov2680_bayer_order(struct ov2680_device *sensor)
{
	u32 format1, format2, hv_flip;
	int ret;

	dev_info (&sensor->client->dev, "%s was called.\n", __func__);

	ret = ov2680_read_reg(sensor->client, 1, OV2680_REG_FORMAT1, &format1);
	if (ret < 0)
		return ret;

	ret = ov2680_read_reg(sensor->client, 1, OV2680_REG_FORMAT2, &format2);
	if (ret < 0)
		return ret;

	hv_flip = (format2 & BIT(2)  << 1) | (format1 & BIT(2));

	sensor->fmt.code = ov2680_hv_flip_bayer_order[hv_flip];

	return 0;
}

static int ov2680_vflip_enable(struct ov2680_device *sensor)
{
	int ret;

	dev_info (&sensor->client->dev, "%s was called.\n", __func__);

	ret = ov2680_mod_reg(sensor, OV2680_REG_FORMAT1, BIT(2), BIT(2));
	if (ret < 0)
		return ret;

	return ov2680_bayer_order(sensor);
}

static int ov2680_vflip_disable(struct ov2680_device *sensor)
{
	int ret;

	dev_info (&sensor->client->dev, "%s was called.\n", __func__);

	ret = ov2680_mod_reg(sensor, OV2680_REG_FORMAT1, BIT(2), BIT(0));
	if (ret < 0)
		return ret;

	return ov2680_bayer_order(sensor);
}


static int ov2680_hflip_enable(struct ov2680_device *sensor)
{
	int ret;

	dev_info (&sensor->client->dev, "%s was called.\n", __func__);

	ret = ov2680_mod_reg(sensor, OV2680_REG_FORMAT2, BIT(2), BIT(2));
	if (ret < 0)
		return ret;

	return ov2680_bayer_order(sensor);
}

static int ov2680_hflip_disable(struct ov2680_device *sensor)
{
	int ret;

	dev_info (&sensor->client->dev, "%s was called.\n", __func__);

	ret = ov2680_mod_reg(sensor, OV2680_REG_FORMAT2, BIT(2), BIT(0));
	if (ret < 0)
		return ret;

	return ov2680_bayer_order(sensor);
}

static int ov2680_test_pattern_set(struct ov2680_device *sensor, int value)
{
	int ret;

	dev_info (&sensor->client->dev, "%s was called.\n", __func__);

	if (!value)
		return ov2680_mod_reg(sensor, OV2680_REG_ISP_CTRL00, BIT(7), 0);

	ret = ov2680_mod_reg(sensor, OV2680_REG_ISP_CTRL00, 0x03, value - 1);
	if (ret < 0)
		return ret;

	ret = ov2680_mod_reg(sensor, OV2680_REG_ISP_CTRL00, BIT(7), BIT(7));
	if (ret < 0)
		return ret;

	return 0;
}

static int ov2680_update_digital_gain(struct i2c_client *client, u32 d_gain)
{
	int ret;

	dev_info(&client->dev, "%s was called.\n", __func__);

	ret = ov2680_write_reg(client, OV2680_16BIT, OV2680_REG_R_DGTL_GAIN, d_gain);
	if (ret)
		return ret;

	ret = ov2680_write_reg(client, OV2680_16BIT, OV2680_REG_G_DGTL_GAIN, d_gain);
	if (ret)
		return ret;

	ret = ov2680_write_reg(client, OV2680_16BIT, OV2680_REG_B_DGTL_GAIN, d_gain);

	return ret;
}

static int ov2680_update_analog_gain(struct ov2680_device *sensor)
{
	struct ov2680_ctrls *ctrls = &sensor->ctrls;
	u16 gain;
	int ret;

	dev_info(&sensor->client->dev, "%s was called.\n", __func__);

	gain = ctrls->again->val;

	ret = ov2680_write_reg(sensor->client, OV2680_16BIT, OV2680_REG_GAIN_PK, gain);

	return 0;
}

static int ov2680_gain_get(struct ov2680_device *sensor)
{
	u32 gain;
	int ret;

	dev_info (&sensor->client->dev, "%s was called.\n", __func__);

	ret = ov2680_read_reg(sensor->client, 2, OV2680_REG_GAIN_PK, &gain);
	if (ret)
		return ret;

	return gain;
}


static int ov2680_exposure_set(struct ov2680_device *sensor)
{
	struct ov2680_ctrls *ctrls = &sensor->ctrls;
	u32 exp;
	int ret;

	dev_info (&sensor->client->dev, "%s was called.\n", __func__);

	exp = (u32)ctrls->exposure->val;
	exp <<= 4;

	return ov2680_write_reg(sensor->client, 3, OV2680_REG_EXPOSURE_PK_HIGH, exp);
}

static int ov2680_exposure_get(struct ov2680_device *sensor)
{
	int ret;
	u32 exp;

	dev_info (&sensor->client->dev, "%s was called.\n", __func__);

	ret = ov2680_read_reg(sensor->client, 3, OV2680_REG_EXPOSURE_PK_HIGH, &exp);
	if (ret)
		return ret;

	return exp >> 4;
}

static int ov2680_stream_enable(struct ov2680_device *sensor)
{

	dev_info (&sensor->client->dev, "%s was called.\n", __func__);

	return ov2680_write_reg(sensor->client, 1, OV2680_REG_STREAM_CTRL, 1);
}

static int ov2680_stream_disable(struct ov2680_device *sensor)
{

	dev_info (&sensor->client->dev, "%s was called.\n", __func__);

	return ov2680_write_reg(sensor->client, 1, OV2680_REG_STREAM_CTRL, 0);
}


static int ov2680_mode_set(struct ov2680_device *sensor)
{
	struct ov2680_ctrls *ctrls = &sensor->ctrls;
	int ret;

	dev_info (&sensor->client->dev, "%s was called.\n", __func__);

	ret = ov2680_update_analog_gain(sensor);
	if (ret < 0)
		return ret;

	ret = ov2680_exposure_set(sensor);
	if (ret < 0)
		return ret;

	ret = ov2680_load_regs(sensor, sensor->current_mode);
	if (ret < 0)
		return ret;

	sensor->mode_pending_changes = false;

	return 0;
}

static int ov2680_mode_restore(struct ov2680_device *sensor)
{
	int ret;

	dev_info (&sensor->client->dev, "%s was called.\n", __func__);

	ret = ov2680_load_regs(sensor, &ov2680_mode_data[2]);
	if (ret < 0)
		return ret;

	return ov2680_mode_set(sensor);
}

static int ov2680_s_power(struct v4l2_subdev *sd, int on)
{
	int ret;
	struct ov2680_device *ov2680;

	ov2680 = to_ov2680_dev(sd);

	dev_info (&ov2680->client->dev, "%s was called.\n", __func__);

	mutex_lock(&ov2680->lock);

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

	mutex_unlock(&ov2680->lock);

	if (on && ret == 0) {
		ret = v4l2_ctrl_handler_setup(&ov2680->ctrls.handler);
		if (ret < 0)
			return ret;

		ret = ov2680_mode_restore(ov2680);
	}

	return ret;	
}

static int ov2680_s_g_frame_interval(struct v4l2_subdev *sd, struct v4l2_subdev_frame_interval *fi)
{
	struct ov2680_device *sensor = to_ov2680_dev(sd);

	dev_info (&sensor->client->dev, "%s was called.\n", __func__);

	mutex_lock(&sensor->lock);
	fi->interval = sensor->frame_interval;
	mutex_unlock(&sensor->lock);

	return 0;
}

static int ov2680_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ov2680_device *ov2680 = to_ov2680_dev(sd);
	int ret;

	dev_info (&ov2680->client->dev, "%s was called.\n", __func__);

	if (ov2680->is_streaming == enable) {
		dev_info(&ov2680->client->dev, "Attempt to set stream=%d, but it is already in that state.\n", enable);
		return 0;
	}

	mutex_lock(&ov2680->lock);

	if (enable && ov2680->mode_pending_changes) {
		ret = ov2680_mode_set(ov2680);
		if (ret < 0) {
			goto unlock;
		}
	}

	if (enable) {
		ret = ov2680_stream_enable(ov2680);
	} else {
		ret = ov2680_stream_disable(ov2680);
	}

	ov2680->is_streaming = enable;

	if (ret) {
		dev_err(&ov2680->client->dev, "An error occurred setting stream enabled=%d.\n", enable);
	}

unlock:
	mutex_unlock(&ov2680->lock);
	return 0;
}

static int ov2680_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct ov2680_device *sensor = to_ov2680_dev(sd);

	dev_info (&sensor->client->dev, "%s was called.\n", __func__);


	if (code->pad != 0 || code->index != 0)
		return -EINVAL;

	code->code = sensor->fmt.code;

	dev_info(&sensor->client->dev, "    code set to %d\n", code->code);

	return 0;
}


static int ov2680_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct ov2680_device *sensor = to_ov2680_dev(sd);
	struct v4l2_mbus_framefmt *fmt = NULL;
	int ret = 0;

	if (format->pad != 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt = v4l2_subdev_get_try_format(&sensor->sd, cfg, format->pad);
#else
		ret = -EINVAL;
#endif
	} else {
		fmt = &sensor->fmt;
	}

	if (fmt)
		format->format = *fmt;
	
	mutex_unlock(&sensor->lock);

	return ret;
}

static int ov2680_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct ov2680_device *sensor = to_ov2680_dev(sd);
	struct v4l2_mbus_framefmt *fmt = &format->format;

	dev_info(&sensor->client->dev, "%s was called.\n", __func__);

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	struct v4l2_mbus_framefmt *try_fmt;
#endif
	const struct ov2680_mode_info *mode;
	int ret = 0;

	if (format->pad != 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	if (sensor->is_streaming) {
		dev_info(&sensor->client->dev, "Sensor is busy streaming.\n");
		ret = -EBUSY;
		goto unlock;
	}

	mode = v4l2_find_nearest_size(ov2680_mode_data,
				      ARRAY_SIZE(ov2680_mode_data), width,
				      height, fmt->width, fmt->height);
	if (!mode) {
		ret = -EINVAL;
		goto unlock;
	}

	if (format->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		try_fmt = v4l2_subdev_get_try_format(sd, cfg, 0);
		format->format = *try_fmt;
#endif
		goto unlock;
	}

	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->code = sensor->fmt.code;
	fmt->colorspace = sensor->fmt.colorspace;

	sensor->current_mode = mode;
	sensor->fmt = format->format;
	sensor->mode_pending_changes = true;

	dev_info(&sensor->client->dev, "    mode %s (%dx%d)\n", mode->name, mode->width, mode->height);

unlock:
	mutex_unlock(&sensor->lock);

	return ret;
}

static int ov2680_init_cfg(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg)
{
	struct ov2680_device *ov2680;
	ov2680 = to_ov2680_dev(sd);

	dev_info (&ov2680->client->dev, "%s was called.\n", __func__);

	struct v4l2_subdev_format fmt = {
		.which = cfg ? V4L2_SUBDEV_FORMAT_TRY
				: V4L2_SUBDEV_FORMAT_ACTIVE,
		.format = {
			.width = 1600,
			.height = 1200,
		}
	};

	return ov2680_set_fmt(sd, cfg, &fmt);
}

static int ov2680_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct ov2680_device *ov2680;
	ov2680 = to_ov2680_dev(sd);

	dev_info (&ov2680->client->dev, "%s was called.\n", __func__);

	int index = fse->index;

	dev_info(&ov2680->client->dev, "    index was %d.\n", index);

	if (index >= OV2680_MODE_MAX || index < 0)
		return -EINVAL;

	fse->min_width = ov2680_mode_data[index].width;
	fse->min_height = ov2680_mode_data[index].height;

	dev_info(&ov2680->client->dev, "    frame size: %dx%d.\n", fse->min_width, fse->min_height);

	fse->max_width = ov2680_mode_data[index].width;
	fse->max_height = ov2680_mode_data[index].height;

	return 0;
}

static int ov2680_enum_frame_interval(struct v4l2_subdev *sd,
			      struct v4l2_subdev_pad_config *cfg,
			      struct v4l2_subdev_frame_interval_enum *fie)
{
	struct v4l2_fract tpf;
	struct ov2680_device *ov2680;
	ov2680 = to_ov2680_dev(sd);

	dev_info (&ov2680->client->dev, "%s was called.\n", __func__);

	if (fie->index >= OV2680_MODE_MAX || fie->width > OV2680_WIDTH_MAX ||
	    fie->height > OV2680_HEIGHT_MAX ||
	    fie->which > V4L2_SUBDEV_FORMAT_ACTIVE)
		return -EINVAL;

	tpf.denominator = OV2680_FRAME_RATE;
	tpf.numerator = 1;

	fie->interval = tpf;

	return 0;
}

static int ov2680_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct ov2680_device *sensor = to_ov2680_dev(sd);
	struct ov2680_ctrls *ctrls = &sensor->ctrls;
	int val;

	dev_info (&sensor->client->dev, "%s was called.\n", __func__);

	if (!sensor->is_enabled)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_GAIN:
		val = ov2680_gain_get(sensor);
		if (val < 0)
			return val;
		ctrls->gain->val = val;
		break;
	case V4L2_CID_EXPOSURE:
		val = ov2680_exposure_get(sensor);
		if (val < 0)
			return val;
		ctrls->exposure->val = val;
		break;
	}

	return 0;
}

static int ov2680_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = ctrl_to_sd(ctrl);
	struct ov2680_device *sensor = to_ov2680_dev(sd);
	struct ov2680_ctrls *ctrls = &sensor->ctrls;

	dev_info (&sensor->client->dev, "%s was called.\n", __func__);

	if (!sensor->is_enabled)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_ANALOGUE_GAIN:
		return ov2680_update_analog_gain(sensor);
	case V4L2_CID_DIGITAL_GAIN:
		return ov2680_update_digital_gain(sensor->client, ctrl->val);
	case V4L2_CID_EXPOSURE:
		return ov2680_exposure_set(sensor);
	case V4L2_CID_VFLIP:
		if (sensor->is_streaming)
			return -EBUSY;
		if (ctrl->val)
			return ov2680_vflip_enable(sensor);
		else
			return ov2680_vflip_disable(sensor);
	case V4L2_CID_HFLIP:
		if (sensor->is_streaming)
			return -EBUSY;
		if (ctrl->val)
			return ov2680_hflip_enable(sensor);
		else
			return ov2680_hflip_disable(sensor);
	case V4L2_CID_TEST_PATTERN:
		return ov2680_test_pattern_set(sensor, ctrl->val);
	default:
		break;
	}

	return -EINVAL;
}

static int ov2680_register(struct v4l2_subdev *sd)
{
	struct ov2680_device *ov2680 = to_ov2680_dev(sd);
	
	dev_info(&ov2680->client->dev, "%s called and registered subdev %s.\n", __func__, sd->name);
	return 0;
}

static const struct v4l2_subdev_internal_ops ov2680_internal_ops = {
	.registered		= ov2680_register,
};

static const struct v4l2_ctrl_ops ov2680_ctrl_ops = {
	.g_volatile_ctrl = ov2680_g_volatile_ctrl,
	.s_ctrl = ov2680_s_ctrl,
};

static const struct v4l2_subdev_core_ops ov2680_core_ops = {
	.s_power = ov2680_s_power,
};

static const struct v4l2_subdev_video_ops ov2680_video_ops = {
	.g_frame_interval	= ov2680_s_g_frame_interval,
	.s_frame_interval	= ov2680_s_g_frame_interval,
	.s_stream		= ov2680_s_stream,
};

static const struct v4l2_subdev_pad_ops ov2680_pad_ops = {
	.init_cfg		= ov2680_init_cfg,
	.enum_mbus_code		= ov2680_enum_mbus_code,
	.get_fmt		= ov2680_get_fmt,
	.set_fmt		= ov2680_set_fmt,
	.enum_frame_size	= ov2680_enum_frame_size,
	.enum_frame_interval	= ov2680_enum_frame_interval,
};

static const struct v4l2_subdev_ops ov2680_subdev_ops = {
	.core	= &ov2680_core_ops,
	.video	= &ov2680_video_ops,
	.pad	= &ov2680_pad_ops,
};

static int ov2680_mode_init(struct ov2680_device *sensor)
{
	const struct ov2680_mode_info *init_mode;

	dev_info(&sensor->client->dev, "%s was called\n", __func__);

	/* set initial mode */
	sensor->fmt.code = MEDIA_BUS_FMT_SBGGR10_1X10;
	sensor->fmt.width = 1600;
	sensor->fmt.height = 1200;
	sensor->fmt.field = V4L2_FIELD_NONE;
	sensor->fmt.colorspace = V4L2_COLORSPACE_RAW; /* SRGB; */

	sensor->frame_interval.denominator = OV2680_FRAME_RATE;
	sensor->frame_interval.numerator = 1;

	/* Default to max resolution mode */
	init_mode = &ov2680_mode_data[2];

	sensor->current_mode = init_mode;

	sensor->mode_pending_changes = true;

	return 0;
}

static const struct media_entity_operations ov2680_subdev_entity_ops = {
	.link_validate	= v4l2_subdev_link_validate,
};

static int ov2680_v4l2_register(struct ov2680_device *sensor)
{
	const struct v4l2_ctrl_ops *ops = &ov2680_ctrl_ops;
	struct ov2680_ctrls *ctrls = &sensor->ctrls;
	struct v4l2_ctrl_handler *hdl = &ctrls->handler;
	int ret = 0;
	s64 vblank_max;
	s64 vblank_def;
	s64 vblank_min;

	dev_info(&sensor->client->dev, "%s was called\n", __func__);

	v4l2_i2c_subdev_init(&sensor->sd, sensor->client,
			     &ov2680_subdev_ops);

	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sensor->sd.entity.ops = &ov2680_subdev_entity_ops;
	sensor->sd.fwnode = sensor->client->dev.fwnode;
	sensor->sd.internal_ops = &ov2680_internal_ops;	

	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret < 0)
		return ret;

	v4l2_ctrl_handler_init(hdl, 7);

	hdl->lock = &sensor->lock;

	if (hdl->error) {
		dev_err(&sensor->client->dev, "Error before it all\n");
	}

	ctrls->link_freq = v4l2_ctrl_new_int_menu(hdl, ops, V4L2_CID_LINK_FREQ, 0, 0, link_freq_menu_items);
	if (ctrls->link_freq) {
		ctrls->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	}

	if (hdl->error) {
		dev_err(&sensor->client->dev, "Error after link_freq\n");
	}

	ctrls->pixel_rate = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_PIXEL_RATE, 0, link_freq_configs[0].pixel_rate, 1, link_freq_configs[0].pixel_rate);
	if (ctrls->pixel_rate) {
		ctrls->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	}
	
	if (hdl->error) {
		dev_err(&sensor->client->dev, "Error after pixel_rate\n");
	}

	vblank_max = OV2680_VTS_MAX - sensor->current_mode->height;
	vblank_def = sensor->current_mode->vts_def - sensor->current_mode->height;
	vblank_min = sensor->current_mode->vts_min - sensor->current_mode->height;
	ctrls->vblank = v4l2_ctrl_new_std(hdl, ops,
					   V4L2_CID_VBLANK, vblank_min,
					   vblank_max, 1, vblank_def);

	if (hdl->error) {
		dev_err(&sensor->client->dev, "Error after vblank\n");
	}

	ctrls->hblank = v4l2_ctrl_new_std(
				hdl, ops, V4L2_CID_HBLANK,
				2724 - sensor->current_mode->width,
				2724 - sensor->current_mode->width, 1,
				2724 - sensor->current_mode->width);
	if (ctrls->hblank)
		ctrls->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;	

	if (hdl->error) {
		dev_err(&sensor->client->dev, "Error after hblank\n");
	}


	ctrls->vflip = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_VFLIP, 0, 1, 1, 0);
	ctrls->hflip = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_HFLIP, 0, 1, 1, 0);

	ctrls->test_pattern = v4l2_ctrl_new_std_menu_items(hdl,
					&ov2680_ctrl_ops, V4L2_CID_TEST_PATTERN,
					ARRAY_SIZE(test_pattern_menu) - 1,
					0, 0, test_pattern_menu);

	ctrls->exposure = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_EXPOSURE,
					    0, 32767, 1, 0);
					
	ctrls->again = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_ANALOGUE_GAIN,
			  ANALOG_GAIN_MIN, ANALOG_GAIN_MAX, ANALOG_GAIN_STEP,
			  ANALOG_GAIN_DEFAULT);

	/* Digital gain */
	ctrls->dgain = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_DIGITAL_GAIN,
			  OV2680_DGTL_GAIN_MIN, OV2680_DGTL_GAIN_MAX,
			  OV2680_DGTL_GAIN_STEP, OV2680_DGTL_GAIN_DEFAULT);

	if (hdl->error) {
		ret = hdl->error;
		goto cleanup_entity;
	}

	ctrls->exposure->flags |= V4L2_CTRL_FLAG_VOLATILE;

	sensor->sd.ctrl_handler = hdl;

	ret = v4l2_async_register_subdev(&sensor->sd);
	if (ret < 0)
		goto cleanup_entity;

	return 0;

cleanup_entity:
	media_entity_cleanup(&sensor->sd.entity);
	v4l2_ctrl_handler_free(hdl);

	return ret;
}

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

	v4l2_device_unregister_subdev(sd);

	device_link_remove(&client->dev, &ov2680->cio2_dev->dev);

	return 0;
}

static int ov2680_probe(struct i2c_client *client)
{
	struct ov2680_device 		*ov2680;
	struct device_link			*dl;
	int 						ret;

	ov2680 = kzalloc(sizeof(*ov2680), GFP_KERNEL);
	if (!ov2680) {
		dev_err(&client->dev, "out of memory\n");
		return -ENOMEM;
	}

	/*
	 * We need to link the sensor device to that of the cio2 infrastructure
	 * as early as possible.
	 */

	ov2680->cio2_dev = pci_get_device(PCI_VENDOR_ID_INTEL, CIO2_PCI_ID, NULL);
    if (!ov2680->cio2_dev) {
        ret = -EPROBE_DEFER;
        goto remove_out;
    }

	dl = device_link_add(&client->dev, &ov2680->cio2_dev->dev, NULL);

	if (!dl)
		dev_err(&client->dev, "Failed to link to the cio2 device; this module may not function without being reloaded\n");

	/* Sensor 'aint on, tell it so */
	ov2680->is_enabled = 0;

	/* First, tie i2c_client to ov2680_device, and vice versa */
	ov2680->client = client;

	ret = ov2680_mode_init(ov2680);

	if (ret) {
		dev_err(&client->dev, "Failed to initialise mode.\n");
		goto remove_out;
	}

	ret = ov2680_configure_gpios(ov2680);

	if (ret) {
		dev_err(&client->dev, "Could not configure the GPIOs.\n");
		goto remove_out;
	} 

	ret = ov2680_power_on(ov2680);

	if (ret) {
		dev_err(&client->dev, "Could not power on the ov2680.\n");
		goto remove_out;
	} 

	/* Check that we are in fact an ov2680 device */
    ret = ov2680_check_ov2680_id(client);

	if (ret) {
		dev_dbg(&client->dev, "The sensor could not be initialised.\n");
		goto remove_out;
	}

	/* register the v4l2 device */
	ret = ov2680_v4l2_register(ov2680);

	if (ret) {
		dev_err(&client->dev, "Failed to register as a v4l2 subdevice.\n");
		goto remove_out;
	}

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

	/* Shut down till we're needed */
	/* ov2680_power_off(ov2680); */

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
