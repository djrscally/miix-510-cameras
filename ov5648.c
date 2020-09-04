#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/device.h>
#include <media/v4l2-common.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>


#include "ov5648.h"

static struct ov5648_device *to_ov5648_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov5648_device, sd);
}

static inline struct v4l2_subdev *ctrl_to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct ov5648_device,
			     ctrls.handler)->sd;
}

static int ov5648_read_reg(struct i2c_client *client, u16 data_length, u16 reg, u32 *val)
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

	if (data_length != OV5648_8BIT && data_length != OV5648_16BIT
					&& data_length != OV5648_32BIT) {
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
	if (data_length == OV5648_8BIT)
		*val = (u8)data[0];
	else if (data_length == OV5648_16BIT)
		*val = be16_to_cpu(*(u16 *)&data[0]);
	else
		*val = be32_to_cpu(*(u32 *)&data[0]);

	dev_info(&client->dev, "    read val 0x%06x from reg 0x%04x\n", *val, reg);

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

	dev_info (&client->dev, "%s was called.\n", __func__);
	dev_info(&client->dev, "    Writing val 0x%04x to reg 0x%04x\n", val, reg);

	if (data_length != OV5648_8BIT && data_length != OV5648_16BIT && data_length != OV5648_24BIT) {
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

static int ov5648_mod_reg(struct ov5648_device *sensor, u16 reg, u8 mask, u16 val)
{
	u32 readval;
	int ret;

	dev_info (&sensor->client->dev, "%s was called.\n", __func__);

	ret = ov5648_read_reg(sensor->client, 1, reg, &readval);
	if (ret < 0)
		return ret;

	readval &= ~mask;
	val &= mask;
	val |= readval;

	return ov5648_write_reg(sensor->client, 1, reg, val);
}

static int ov5648_load_regs(struct ov5648_device *sensor, const struct ov5648_mode_info *mode)
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

		ret = ov5648_write_reg(sensor->client, 1, reg_addr, val);
		if (ret)
			break;
	}

	return ret;
}

static int ov5648_check_id(struct i2c_client *client)
{
	struct i2c_adapter *adapter = client->adapter;
	u32 high, low;
	int ret;
	u16 id;

	dev_info (&client->dev, "%s was called.\n", __func__);

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

	if (id != OV5648_ID) {
		dev_err(&client->dev, "ov5648 ID error 0x%x\n", id);
		return -ENODEV;
	}

	dev_info(&client->dev, "ov5648 successfully detected\n");

	return 0;
}

static int ov5648_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_pad_config *cfg,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct ov5648_device *sensor = to_ov5648_dev(sd);

	dev_info (&sensor->client->dev, "%s was called.\n", __func__);


	if (code->pad != 0 || code->index != 0)
		return -EINVAL;

	code->code = sensor->fmt.code;

	dev_info(&sensor->client->dev, "    code set to %d\n", code->code);

	return 0;
}

static int ov5648_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct ov5648_device *ov5648;
	ov5648 = to_ov5648_dev(sd);

	dev_info (&ov5648->client->dev, "%s was called.\n", __func__);

	int index = fse->index;

	if (index >= OV5648_MODE_MAX || index < 0)
		return -EINVAL;

	fse->min_width = ov5648_mode_data[index].width;
	fse->min_height = ov5648_mode_data[index].height;

	fse->max_width = ov5648_mode_data[index].width;
	fse->max_height = ov5648_mode_data[index].height;

	return 0;
}

static int ov5648_mode_set(struct ov5648_device *sensor)
{
	struct ov5648_ctrls *ctrls = &sensor->ctrls;
	int ret;

	dev_info (&sensor->client->dev, "%s was called.\n", __func__);

	/* not implemented yet */
//	ret = ov5648_update_analog_gain(sensor);
//	if (ret < 0)
//		return ret;

//	ret = ov5648_exposure_set(sensor);
//	if (ret < 0)
//		return ret;

	ret = ov5648_load_regs(sensor, sensor->current_mode);
	if (ret < 0)
		return ret;

	sensor->mode_pending_changes = false;

	return 0;
}

static int ov5648_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct ov5648_device *sensor = to_ov5648_dev(sd);
	struct v4l2_mbus_framefmt *fmt = &format->format;

	dev_info(&sensor->client->dev, "%s was called.\n", __func__);

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	struct v4l2_mbus_framefmt *try_fmt;
#endif
	const struct ov5648_mode_info *mode;
	int ret = 0;

	if (format->pad != 0)
		return -EINVAL;

	mutex_lock(&sensor->lock);

	if (sensor->is_streaming) {
		dev_info(&sensor->client->dev, "Sensor is busy streaming.\n");
		ret = -EBUSY;
		goto unlock;
	}

	mode = v4l2_find_nearest_size(ov5648_mode_data,
				      ARRAY_SIZE(ov5648_mode_data), width,
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

static int ov5648_init_cfg(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg)
{
	struct ov5648_device *ov5648;
	ov5648 = to_ov5648_dev(sd);

	dev_info (&ov5648->client->dev, "%s was called.\n", __func__);

	struct v4l2_subdev_format fmt = {
		.which = cfg ? V4L2_SUBDEV_FORMAT_TRY
				: V4L2_SUBDEV_FORMAT_ACTIVE,
		.format = {
			.width = 2592,
			.height = 1944,
		}
	};

	return ov5648_set_fmt(sd, cfg, &fmt);
}

static int ov5648_stream_enable(struct ov5648_device *sensor)
{

	dev_info (&sensor->client->dev, "%s was called.\n", __func__);

	return ov5648_write_reg(sensor->client, 1, OV5648_REG_STREAM_CTRL, 1);
}

static int ov5648_stream_disable(struct ov5648_device *sensor)
{

	dev_info (&sensor->client->dev, "%s was called.\n", __func__);

	return ov5648_write_reg(sensor->client, 1, OV5648_REG_STREAM_CTRL, 0);
}

static int ov5648_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ov5648_device *ov5648 = to_ov5648_dev(sd);
	int ret;

	dev_info (&ov5648->client->dev, "%s was called.\n", __func__);

	if (ov5648->is_streaming == enable) {
		dev_info(&ov5648->client->dev, "Attempt to set stream=%d, but it is already in that state.\n", enable);
		return 0;
	}

	mutex_lock(&ov5648->lock);

	if (enable && ov5648->mode_pending_changes) {
		ret = ov5648_mode_set(ov5648);
		if (ret < 0) {
			goto unlock;
		}
	}

	if (enable) {
		ret = ov5648_stream_enable(ov5648);
	} else {
		ret = ov5648_stream_disable(ov5648);
	}

	ov5648->is_streaming = enable;

	if (ret) {
		dev_err(&ov5648->client->dev, "An error occurred setting stream enabled=%d.\n", enable);
	}

unlock:
	mutex_unlock(&ov5648->lock);
	return 0;
}

static int ov5648_mode_init(struct ov5648_device *sensor)
{
	const struct ov5648_mode_info *init_mode;

	dev_info(&sensor->client->dev, "%s was called\n", __func__);

	/* set initial mode */
	sensor->fmt.code = MEDIA_BUS_FMT_SBGGR10_1X10;
	sensor->fmt.width = 1600;
	sensor->fmt.height = 1200;
	sensor->fmt.field = V4L2_FIELD_NONE;
	sensor->fmt.colorspace = V4L2_COLORSPACE_RAW; /* SRGB; */

	/* Default to max resolution mode */
	init_mode = &ov5648_mode_data[OV5648_MODE_MAX-1];

	sensor->current_mode = init_mode;
	sensor->frame_interval.denominator = init_mode->framerate;
	sensor->frame_interval.numerator = 1;

	sensor->mode_pending_changes = true;

	return 0;
}

static int ov5648_s_ctrl(struct v4l2_ctrl *ctrl)
{
	return 0;
}

static const struct v4l2_ctrl_ops ov5648_ctrl_ops = {
	.s_ctrl = ov5648_s_ctrl
};

static const struct v4l2_subdev_video_ops ov5648_video_ops = {
	.s_stream		= ov5648_s_stream,
};

static const struct v4l2_subdev_pad_ops ov5648_pad_ops = {
	.init_cfg		= ov5648_init_cfg,
	.enum_mbus_code		= ov5648_enum_mbus_code,
	.set_fmt		= ov5648_set_fmt,
	.enum_frame_size	= ov5648_enum_frame_size,
};

static const struct v4l2_subdev_ops ov5648_subdev_ops = {
	.video	= &ov5648_video_ops,
	.pad	= &ov5648_pad_ops,
};

static const struct media_entity_operations ov5648_subdev_entity_ops = {
	.link_validate	= v4l2_subdev_link_validate,
};

static int ov5648_v4l2_register(struct ov5648_device *sensor)
{
	const struct v4l2_ctrl_ops *ops = &ov5648_ctrl_ops;
	struct ov5648_ctrls *ctrls = &sensor->ctrls;
	struct v4l2_ctrl_handler *hdl = &ctrls->handler;
	int ret = 0;

	v4l2_i2c_subdev_init(&sensor->sd, sensor->client,
			     &ov5648_subdev_ops);

	sensor->pad.flags = MEDIA_PAD_FL_SOURCE;
	sensor->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	sensor->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	sensor->sd.entity.ops = &ov5648_subdev_entity_ops;
	sensor->sd.fwnode = sensor->client->dev.fwnode;

	ret = media_entity_pads_init(&sensor->sd.entity, 1, &sensor->pad);
	if (ret < 0)
		return ret;

	v4l2_ctrl_handler_init(hdl, 7);

	hdl->lock = &sensor->lock;

	ctrls->link_freq = v4l2_ctrl_new_int_menu(hdl, ops, V4L2_CID_LINK_FREQ, 0, 0, link_freq_menu_items);
	if (ctrls->link_freq) {
		ctrls->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	}

	ctrls->pixel_rate = v4l2_ctrl_new_std(hdl, ops, V4L2_CID_PIXEL_RATE, 0, link_freq_configs[0].pixel_rate, 1, link_freq_configs[0].pixel_rate);
	if (ctrls->pixel_rate) {
		ctrls->pixel_rate->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	}
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

static int ov5648_probe(struct i2c_client *client)
{
	struct ov5648_device *ov5648;
	int ret;

	ov5648 = kzalloc(sizeof(*ov5648), GFP_KERNEL);
	
	if (!ov5648) {
		dev_err(&client->dev, "Couldn't allocate memory for the ov5648 device\n");
		return -ENOMEM;
	}

	ov5648->is_enabled = false;
	ov5648->client = client;

	ret = ov5648_mode_init(ov5648);

	if (ret) {
		dev_err(&client->dev, "Failed to initialise mode.\n");
		goto remove_out;
	}

	/* Check that we are in fact an ov5648 device */
    ret = ov5648_check_id(client);

	if (ret) {
		dev_dbg(&client->dev, "The sensor could not be initialised.\n");
		goto remove_out;
	}

	/* register the v4l2 device */
	ret = ov5648_v4l2_register(ov5648);

	if (ret) {
		dev_err(&client->dev, "Failed to register as a v4l2 subdevice.\n");
		goto remove_out;
	}

remove_out:
	return 0;
}

static int ov5648_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd;

	sd = i2c_get_clientdata(client);

	v4l2_device_unregister_subdev(sd);
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

MODULE_AUTHOR("Dan Scally <djrscally@gmail.com>");
MODULE_DESCRIPTION("A driver for OmniVision 5648 Camera");
MODULE_LICENSE("GPL");
