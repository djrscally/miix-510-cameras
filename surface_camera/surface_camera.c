#include <linux/acpi.h>
#include <acpi/acpi_bus.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <media/v4l2-subdev.h>

#define MAX_CONNECTED_DEVICES	5
#define SWNODE_SENSOR_HID		0
#define SWNODE_SENSOR_PORT		1
#define SWNODE_SENSOR_ENDPOINT	2
#define SWNODE_CIO2_PORT		3
#define SWNODE_CIO2_ENDPOINT	4
#define SWNODE_NULL_TERMINATOR	5

#define CIO2_HID				"INT343E"
#define CIO2_PCI_ID				0x9d32

#define NODE_HID(_HID)						\
	(struct software_node) {				\
		_HID,								\
	}

#define NODE_PORT(_PORT, _HID_NODE)			\
	(struct software_node) {				\
		_PORT,								\
		_HID_NODE,							\
	}

#define NODE_ENDPOINT(_EP, _PORT, _PROPS)	\
	(struct software_node) {				\
		_EP,								\
		_PORT,								\
		_PROPS,								\
	}

#define PROPERTY_ENTRY_NULL					\
	(struct property_entry) { }

#define SOFTWARE_NODE_NULL					\
	(struct software_node) { }

/* Extend this array with ACPI Hardware ID's of devices known to be working */

static char* supported_devices[] = {
//    "INT33BE",
    "OVTI2680",
};

struct pci_dev *cio2;

struct software_node cio2_hid_node = { CIO2_HID, };

struct sensor {
	struct i2c_client *client;
	struct software_node swnodes[6];
	struct property_entry sensor_props[6];
	struct property_entry cio2_props[3];
};

struct connected_devices {
    int n_devices;
    struct sensor sensors[MAX_CONNECTED_DEVICES];
};

struct connected_devices connected_devs = {
    .n_devices = 0,
};

/* Data representation as it is in ACPI SSDB buffer */
struct sensor_bios_data_packed {
	u8 version;
	u8 sku;
	u8 guid_csi2[16];
	u8 devfunction;
	u8 bus;
	u32 dphylinkenfuses;
	u32 clockdiv;
	u8 link;
	u8 lanes;
	u32 csiparams[10];
	u32 maxlanespeed;
	u8 sensorcalibfileidx;
	u8 sensorcalibfileidxInMBZ[3];
	u8 romtype;
	u8 vcmtype;
	u8 platforminfo;
	u8 platformsubinfo;
	u8 flash;
	u8 privacyled;
	u8 degree;
	u8 mipilinkdefined;
	u32 mclkspeed;
	u8 controllogicid;
	u8 reserved1[3];
	u8 mclkport;
	u8 reserved2[13];
} __attribute__((__packed__));

/* Fields needed by ipu4 driver */
struct sensor_bios_data {
	struct device *dev;
	u8 link;
	u8 lanes;
	u32 mclkspeed;
	u8 vcmtype;
	u8 flash;
	u8 degree;
	u8 mclkport;
	u16 xshutdown;
};

static int read_acpi_block(struct device *dev, char *id, void *data,
					 u32 size)
{
	union acpi_object *obj;
	struct acpi_buffer buffer = { ACPI_ALLOCATE_BUFFER, NULL };
	struct acpi_handle *dev_handle = ACPI_HANDLE(dev);
	int status;
	u32 buffer_length;

	status = acpi_evaluate_object(dev_handle, id, NULL, &buffer);
	if (!ACPI_SUCCESS(status))
		return -ENODEV;

	obj = (union acpi_object *)buffer.pointer;
	if (!obj || obj->type != ACPI_TYPE_BUFFER) {
		dev_err(dev, "Could't read acpi buffer\n");
		status = -ENODEV;
		goto err;
	}

	if (obj->buffer.length > size) {
		dev_err(dev, "Given buffer is too small\n");
		status = -ENODEV;
		goto err;
	}

	memcpy(data, obj->buffer.pointer, min(size, obj->buffer.length));
	buffer_length = obj->buffer.length;
	kfree(buffer.pointer);

	return buffer_length;
err:
	kfree(buffer.pointer);
	return status;
}

static int get_acpi_ssdb_sensor_data(struct device *dev,
				     struct sensor_bios_data *sensor)
{
	struct sensor_bios_data_packed sensor_data;
	int ret = read_acpi_block(dev, "SSDB", &sensor_data,
				  sizeof(sensor_data));
	if (ret < 0) {
        dev_err(dev, "Failed to fetch SSDB data\n");
		return ret;
    }

	/* Xshutdown is not part of the ssdb data */
	sensor->link = sensor_data.link;
	sensor->lanes = sensor_data.lanes;
	sensor->mclkport = sensor_data.mclkport;
	sensor->flash = sensor_data.flash;
	sensor->mclkspeed = sensor_data.mclkspeed;

	return 0;
}

static int connect_supported_devices(struct device *dev, void *data)
{
    struct acpi_device *adev;
    struct i2c_client *client;
    struct connected_devices *cdevs = data;
	struct sensor_bios_data ssdb;
	struct property_entry *sensor_props;
	struct property_entry *cio2_props;
	struct fwnode_handle *fwnode;
	struct software_node *nodes;
    const char *hid;
	u32 *data_lanes;
	char portn[7] = "port";
    int i, j, ret;

    adev = ACPI_COMPANION(dev);
    if (!adev) {
        return 0;
    }

    hid = acpi_device_hid(adev);

    for (i = 0; i < ARRAY_SIZE(supported_devices); i++) {
        if (!strcmp(hid, supported_devices[i])) { 

            dev_info(dev, "Found match: %s -> %s\n", hid,
								supported_devices[i]);

			get_acpi_ssdb_sensor_data(dev, &ssdb);

			nodes = cdevs->sensors[cdevs->n_devices].swnodes;
			sensor_props = cdevs->sensors[cdevs->n_devices].sensor_props;
			cio2_props = cdevs->sensors[cdevs->n_devices].cio2_props;
			
			/*
			 * No way to tell how many elements this array needs until 
			 * this point unfortunately, so it will have to be dynamically
			 * allocated. use devres to avoid snafu later
			 */
			data_lanes = devm_kmalloc(dev, sizeof(u32) * (int)ssdb.lanes,
								GFP_KERNEL);

			if (!data_lanes) {
				dev_err(dev, "Couldn't allocate memory for data lanes array\n");
				return -ENOMEM;
			}

			for (j = 0; j < (int)ssdb.lanes; j++) {
				data_lanes[j] = (u32)j+1;
			}

			/* Hacky af but does it need to be more complicated? */
			portn[4] = '0' + ssdb.link;
			portn[5] = '\0';

			sensor_props[0] = PROPERTY_ENTRY_U32("clock-frequency", ssdb.mclkspeed);
			sensor_props[1] = PROPERTY_ENTRY_U32("bus-type", 5);
			sensor_props[2] = PROPERTY_ENTRY_U32("clock-lanes", 0);
			sensor_props[3] = PROPERTY_ENTRY_U32_ARRAY_LEN("data-lanes", data_lanes, (int)ssdb.lanes);
			sensor_props[4] = PROPERTY_ENTRY_REF("remote-endpoint", &nodes[SWNODE_CIO2_ENDPOINT]);
			sensor_props[5] = PROPERTY_ENTRY_NULL;

			cio2_props[0] = PROPERTY_ENTRY_U32_ARRAY_LEN("data-lanes", data_lanes, (int)ssdb.lanes);
			cio2_props[1] = PROPERTY_ENTRY_REF("remote-endpoint", &nodes[SWNODE_SENSOR_ENDPOINT]);
			cio2_props[2] = PROPERTY_ENTRY_NULL;

			/* build the software nodes */

			nodes[SWNODE_SENSOR_HID] = NODE_HID(supported_devices[i]);												/* Sensor HID Node */
			nodes[SWNODE_SENSOR_PORT] = NODE_PORT("port0", &nodes[SWNODE_SENSOR_HID]);								/* Sensor Port Node */
			nodes[SWNODE_SENSOR_ENDPOINT] = NODE_ENDPOINT("endpoint0", &nodes[SWNODE_SENSOR_PORT], sensor_props);	/* Sensor Endpoint Node */
			nodes[SWNODE_CIO2_PORT] = NODE_PORT(portn, &cio2_hid_node);												/* CIO2 Port Node */
			nodes[SWNODE_CIO2_ENDPOINT] = NODE_ENDPOINT("endpoint0", &nodes[SWNODE_CIO2_PORT], cio2_props);			/* CIO2 Endpoint Node */
			nodes[SWNODE_NULL_TERMINATOR] = SOFTWARE_NODE_NULL;

			ret = software_node_register_nodes(nodes);
			if (ret) {
				dev_err(dev, "Failed to register the software nodes for %s\n", supported_devices[i]);
				return 0;
			}

			fwnode = software_node_fwnode(&nodes[SWNODE_SENSOR_HID]);
			if (!fwnode) {
				dev_err(dev, "Failed to get fwnode from software node for %s\n", supported_devices[i]);
				return 0;
			}

			fwnode->secondary = ERR_PTR(-ENODEV);
			dev->fwnode = fwnode;
			((struct v4l2_subdev *)dev->driver_data)->fwnode = fwnode;

			/* we're done */
			cdevs->sensors[cdevs->n_devices].client = client;
			cdevs->n_devices++;

			return 0;
        }
    }

    return 0;
}

static void surface_camera_remove_node(const struct software_node *n)
{
	struct fwnode_handle *fwnode = software_node_fwnode(n);
	if (fwnode)
		fwnode_remove_software_node(fwnode);
}

static int surface_camera_unregister_nodes(void)
{
	int i,j;
	struct sensor sensor;

	for (i=0; i < connected_devs.n_devices; i++) {
		sensor = connected_devs.sensors[i];

		for (j=6; j>0; j--) {
			surface_camera_remove_node(&sensor.swnodes[j]);
		}
	}

	surface_camera_remove_node(&cio2_hid_node);

	return 0;
}

static int surface_camera_init(void)
{
	struct fwnode_handle *fwnode;
    int ret;

	/* Register the CIO2 Parent node */
	ret = software_node_register(&cio2_hid_node);

	if ((ret < 0) && (ret != -EEXIST)) { /* I think -EEXIST is acceptable here */
		pr_err("Failed to register the CIO2 HID node\n");
		return -EINVAL;
	}

    /* Check for supported devices and connect them*/
    ret = i2c_for_each_dev(&connected_devs, connect_supported_devices);

    if ((ret < 0) || (connected_devs.n_devices == 0)) {
        pr_err("Failed to connect any devices\n");
        goto out;
    }

	if (connected_devs.n_devices > 0) {
		pr_info("Found %d supported devices\n", connected_devs.n_devices);
	}

	/* Find pci device and add swnode as primary */
	cio2 = pci_get_device(PCI_VENDOR_ID_INTEL, CIO2_PCI_ID, NULL);
	if (!cio2) {
		ret = -EPROBE_DEFER;
		goto out;
	}

	fwnode = software_node_fwnode(&cio2_hid_node);
	if (!fwnode) {
		pr_err("Error getting fwnode from cio2 software_node\n");
		ret = -ENODEV;
		goto out;
	}

	fwnode->secondary = ERR_PTR(-ENODEV);
	cio2->dev.fwnode = fwnode;

	ret = device_reprobe(&cio2->dev);
	if (ret) {
		dev_warn(&cio2->dev, "Reprobing error: %d\n", ret);
		goto out;
	}

	pr_info("I actually think success...\n");

	return 0;
out:
	surface_camera_unregister_nodes();
    return ret;
}

static void surface_camera_exit(void)
{
    int ret;

	cio2->dev.fwnode = ERR_PTR(-ENODEV);

	ret = surface_camera_unregister_nodes();
}

module_init(surface_camera_init);
module_exit(surface_camera_exit);

MODULE_AUTHOR("Jordan Hand");
MODULE_AUTHOR("Dan Scally <djrscally@gmail.com>");
MODULE_DESCRIPTION("A bridge driver to connect sensors to CIO2 infrastructure.");
MODULE_LICENSE("GPL v2");