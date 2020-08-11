#define OV2680_ID_REG_HIGH          0x300a
#define OV2680_ID_REG_LOW           0x300b
#define I2C_MSG_LENGTH              0x02
#define OV2680_ID                   0x2680
#define OV2680_SC_CMMN_SUB_ID	    0x302A
#define OV2680_NUM_SUPPLIES			3

static const char * const ov2680_supply_names[] = {
	"dovdd",		/* Digital I/O Power */
	"avdd",			/* Analog Power */
	"dvdd",			/* Digital Core Power */
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

struct ov2680_device {
    struct i2c_client       *client;

    struct gpio_desc        *xshutdn;
    struct gpio_desc        *pwdnb;
    struct gpio_desc        *led;

	struct regulator_bulk_data supplies[3];
};