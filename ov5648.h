#define OV5648_ID_REG_HIGH          0x300a
#define OV5648_ID_REG_LOW           0x300b
#define I2C_MSG_LENGTH              0x02
#define OV5648_ID                   0x5648
#define OV5648_SC_CMMN_SUB_ID	    0x302A
#define OV5648_NUM_SUPPLIES			3

static const char * const ov5648_supply_names[] = {
	"dovdd",		/* Digital I/O Power */
	"avdd",			/* Analog Power */
	"dvdd",			/* Digital Core Power */
};

enum ov5648_tok_type {
	OV5648_8BIT  = 0x0001,
	OV5648_16BIT = 0x0002,
	OV5648_32BIT = 0x0004,
	OV5648_TOK_TERM   = 0xf000,	/* terminating token for reg list */
	OV5648_TOK_DELAY  = 0xfe00,	/* delay token for reg list */
	OV5648_TOK_MASK = 0xfff0
};

struct ov5648_reg {
	enum ov5648_tok_type type;
	u16 reg;
	u32 val;	/* @set value for read/mod/write, @mask */
};

struct ov5648_device {
    struct i2c_client       *client;

    struct gpio_desc        *xshutdn;
    struct gpio_desc        *pwdnb;
    struct gpio_desc        *led;

	struct regulator_bulk_data supplies[3];
};

static const struct mfd_cell tps68470s[] = {
	{ .name = "tps68470-gpio"},
	{ .name = "tps68470_pmic_opregion" },
};