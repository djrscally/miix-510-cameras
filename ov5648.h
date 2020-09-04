#define I2C_MSG_LENGTH				0x02
#define OV5648_ID					0x5648

/* Register addresses */
#define OV5648_ID_REG_HIGH			0x300a
#define OV5648_ID_REG_LOW			0x300b
#define OV5648_REG_STREAM_CTRL		0x0100

enum ov5648_mode_id {
//	OV5648_MODE_VGA_640_480_90FPS,
//	OV5648_MODE_720P_1280_720_60FPS,
//	OV5648_MODE_Q5MP_1296_972_45FPS,
	OV5648_MODE_1080P_1920_1080_30FPS,
//	OV5648_MODE_5MP_2592_1944_15FPS,
	OV5648_MODE_MAX
};

enum ov5648_i2c_msg_len {
	OV5648_8BIT = 0x01,
	OV5648_16BIT = 0x02,
	OV5648_24BIT = 0x03,
	OV5648_32BIT = 0x04
};

struct reg_value {
	u16 reg_addr;
	u8 val;
};

struct ov5648_link_freq_config {
	u32 pixel_rate;
	const struct reg_value *regs;
	u32 nregs;
};

static const struct reg_value mipi_data_rate_840mbps[] = {
	{0x0300, 0x04},
	{0x0301, 0x00},
	{0x0302, 0x84},
	{0x0303, 0x00},
	{0x0304, 0x03},
	{0x0305, 0x01},
	{0x0306, 0x01},
	{0x030a, 0x00},
	{0x030b, 0x00},
	{0x030c, 0x00},
	{0x030d, 0x26},
	{0x030e, 0x00},
	{0x030f, 0x06},
	{0x0312, 0x01},
	{0x3031, 0x0a},
};

#define OV5648_LINK_FREQ_422MHZ			422400000
#define OV5648_LINK_FREQ_422MHZ_INDEX	0

static const struct ov5648_link_freq_config link_freq_configs[] = {
	{
		.pixel_rate = (OV5648_LINK_FREQ_422MHZ * 2 * 2) / 10,
		.regs = mipi_data_rate_840mbps,
		.nregs = ARRAY_SIZE(mipi_data_rate_840mbps),
	}
};

static const s64 link_freq_menu_items[] = {
	OV5648_LINK_FREQ_422MHZ
};

struct ov5648_mode_info {
	const char *name;
	enum ov5648_mode_id id;
	u32 width;
	u32 height;
	const struct reg_value *reg_data;
	u32 reg_data_size;	
	u32 vts_min; /* minimum vertical timing size */
	u32 vts_def; /* default vertical timing size */
	const int framerate;
};

struct ov5648_ctrls {
	struct v4l2_ctrl_handler 		handler;
	struct v4l2_ctrl				*link_freq;
	struct v4l2_ctrl				*pixel_rate;
};

struct ov5648_device {
	/* references */
    struct i2c_client       		*client;			/* client for this physical device */
	struct mutex					lock;
	struct v4l2_subdev				sd;

	/* state flags */
	bool is_enabled;
	bool is_streaming;

	/* v4l2 infrastructure */
	struct v4l2_mbus_framefmt		fmt;
	struct v4l2_fract				frame_interval;
	struct ov5648_ctrls				ctrls;
	struct media_pad				pad;

	/* video formatting */
	const struct ov5648_mode_info	*current_mode;
	bool 							mode_pending_changes;
};

static const struct reg_value ov5648_setting_1080p[] = {
	{0x3503, 0x00}, {0x3808, 0x07}, {0x3809, 0x80}, {0x380a, 0x04},
	{0x380b, 0x38}, {0x380c, 0x06}, {0x380d, 0xa4}, {0x380e, 0x05},
	{0x380f, 0x0e}, {0x3811, 0x00}, {0x3813, 0x00}, {0x3814, 0x11},
	{0x3815, 0x11}, {0x3820, 0x40}, {0x4000, 0x01}, {0x4837, 0x18},
};

static const struct ov5648_mode_info ov5648_mode_data[OV5648_MODE_MAX] = {
	{"mode_1080p_1920_1080_30fps", OV5648_MODE_1080P_1920_1080_30FPS, 1920, 1080,
	ov5648_setting_1080p, ARRAY_SIZE(ov5648_setting_1080p), 0x0808, 0x0808, 30}
};
