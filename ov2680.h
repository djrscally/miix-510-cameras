#define OV2680_XVCLK_VALUE			24000000

#define OV2680_ID_REG_HIGH          0x300a
#define OV2680_ID_REG_LOW           0x300b
#define I2C_MSG_LENGTH              0x02
#define OV2680_ID                   0x2680
#define OV2680_SC_CMMN_SUB_ID	    0x302A
#define OV2680_NUM_SUPPLIES			7

#define OV2680_REG_STREAM_CTRL		0x0100
#define OV2680_REG_SOFT_RESET		0x0103

#define OV2680_REG_CHIP_ID_HIGH		0x300a
#define OV2680_REG_CHIP_ID_LOW		0x300b

#define OV2680_REG_R_MANUAL		0x3503
#define OV2680_REG_GAIN_PK		0x350a
#define OV2680_REG_EXPOSURE_PK_HIGH	0x3500
#define OV2680_REG_TIMING_HTS		0x380c
#define OV2680_REG_TIMING_VTS		0x380e
#define OV2680_REG_FORMAT1		0x3820
#define OV2680_REG_FORMAT2		0x3821

#define OV2680_REG_ISP_CTRL00		0x5080

#define OV2680_FRAME_RATE		30

#define OV2680_REG_VALUE_8BIT		1
#define OV2680_REG_VALUE_16BIT		2
#define OV2680_REG_VALUE_24BIT		3

#define OV2680_WIDTH_MAX		1600
#define OV2680_HEIGHT_MAX		1200

static const char * const ov2680_supply_names[] = {
	"CORE",
	"ANA",
	"VCM",
	"VIO",
	"VSIO",
	"AUX1",
	"AUX2",
};

enum ov2680_tok_type {
	OV2680_8BIT  = 0x0001,
	OV2680_16BIT = 0x0002,
	OV2680_32BIT = 0x0004,
	OV2680_TOK_TERM   = 0xf000,	/* terminating token for reg list */
	OV2680_TOK_DELAY  = 0xfe00,	/* delay token for reg list */
	OV2680_TOK_MASK = 0xfff0
};

struct ov2680_reg {
	enum ov2680_tok_type type;
	u16 reg;
	u32 val;	/* @set value for read/mod/write, @mask */
};

struct ov2680_ctrls {
	struct v4l2_ctrl_handler handler;
	struct {
		struct v4l2_ctrl *auto_exp;
		struct v4l2_ctrl *exposure;
	};
	struct {
		struct v4l2_ctrl *auto_gain;
		struct v4l2_ctrl *gain;
	};

	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;
	struct v4l2_ctrl *test_pattern;
};

/* GPIO Mapping for the camera */
static struct gpiod_lookup_table ov2680_gpios = {
	.dev_id = "i2c-OVTI2680:00",
	.table = {
		GPIO_LOOKUP_IDX("tps68470-gpio", 7, "s_enable", 0, GPIO_ACTIVE_HIGH),
		GPIO_LOOKUP_IDX("tps68470-gpio", 8, "s_idle", 0, GPIO_ACTIVE_HIGH),
		GPIO_LOOKUP_IDX("tps68470-gpio", 9, "s_resetn", 0, GPIO_ACTIVE_HIGH),
		{ },
	},
};


struct ov2680_device {
	/* references */
    struct i2c_client       	*client;			/* client for this physical device */
	struct device				*pmic_dev;			/* physical device for the sensor's PMIC */
	struct v4l2_subdev			sd;
	struct clk					*xvclk;
	u32							xvclk_freq;

	/* GPIO pins to turn on the PMIC */
    struct gpio_desc        	*gpio0;
    struct gpio_desc        	*gpio1;

	/* GPIO pins to turn on the sensor */
	struct gpiod_lookup_table	*gpios;
	struct gpio_desc			*s_enable;
	struct gpio_desc			*s_idle;
	struct gpio_desc			*s_resetn;

	/* Miscellaneous gubbins */
	struct mutex				lock;
	struct regulator_bulk_data	supplies[OV2680_NUM_SUPPLIES];
	short						is_enabled;

	/* V4L2 Infrastructure */
	struct ov2680_ctrls			ctrls;
	short						is_streaming;
	struct media_pad			pad;


};

