// SPDX-License-Identifier: GPL-2.0
#include <linux/acpi.h>
#include <linux/device.h>
#include <linux/fwnode.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>

#include <media/v4l2-subdev.h>

static void cio2_bridge_exit(void);
static int cio2_bridge_init(void);

#define MAX_CONNECTED_DEVICES			4
#define SWNODE_SENSOR_HID			0
#define SWNODE_SENSOR_PORT			1
#define SWNODE_SENSOR_ENDPOINT			2
#define SWNODE_CIO2_PORT			3
#define SWNODE_CIO2_ENDPOINT			4
#define SWNODE_NULL_TERMINATOR			5

#define CIO2_HID				"INT343E"
#define CIO2_PCI_ID				0x9d32

#define ENDPOINT_SENSOR				0
#define ENDPOINT_CIO2				1

#define NODE_HID(_HID)				\
	((const struct software_node) {		\
		_HID,				\
	})

#define NODE_PORT(_PORT, _HID_NODE)		\
	((const struct software_node) {		\
		_PORT,				\
		_HID_NODE,			\
	})

#define NODE_ENDPOINT(_EP, _PORT, _PROPS)	\
	((const struct software_node) {		\
		_EP,				\
		_PORT,				\
		_PROPS,				\
	})

/*
 * Extend this array with ACPI Hardware ID's of devices known to be
 * working
 */

static char *supported_devices[] = {
	"INT33BE",
	"OVTI2680",
	"OVTI5648",
};

struct software_node cio2_hid_node = { CIO2_HID, };

struct sensor {
	char name[20];
	struct device *dev;
	struct software_node swnodes[6];
	struct property_entry sensor_props[6];
	struct property_entry cio2_props[3];
	struct fwnode_handle *fwnode;
};

struct cio2_bridge {
	int n_sensors;
	struct sensor sensors[MAX_CONNECTED_DEVICES];
	struct pci_dev *cio2;
	struct fwnode_handle *cio2_fwnode;
};

struct cio2_bridge bridge = { 0, };

static const struct property_entry remote_endpoints[] = {
	PROPERTY_ENTRY_REF("remote-endpoint", /* Sensor 0, Sensor Property */
			   &bridge.sensors[0].swnodes[SWNODE_CIO2_ENDPOINT]),
	PROPERTY_ENTRY_REF("remote-endpoint", /* Sensor 0, CIO2 Property */
			   &bridge.sensors[0].swnodes[SWNODE_SENSOR_ENDPOINT]),
	PROPERTY_ENTRY_REF("remote-endpoint",
			   &bridge.sensors[1].swnodes[SWNODE_CIO2_ENDPOINT]),
	PROPERTY_ENTRY_REF("remote-endpoint",
			   &bridge.sensors[1].swnodes[SWNODE_SENSOR_ENDPOINT]),
	PROPERTY_ENTRY_REF("remote-endpoint",
			   &bridge.sensors[2].swnodes[SWNODE_CIO2_ENDPOINT]),
	PROPERTY_ENTRY_REF("remote-endpoint",
			   &bridge.sensors[2].swnodes[SWNODE_SENSOR_ENDPOINT]),
	PROPERTY_ENTRY_REF("remote-endpoint",
			   &bridge.sensors[3].swnodes[SWNODE_CIO2_ENDPOINT]),
	PROPERTY_ENTRY_REF("remote-endpoint",
			   &bridge.sensors[3].swnodes[SWNODE_SENSOR_ENDPOINT]),
	{ }
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

/* Fields needed by bridge driver */
struct sensor_bios_data {
	struct device *dev;
	u8 link;
	u8 lanes;
	u32 mclkspeed;
};

static int read_acpi_block(struct device *dev, char *id, void *data, u32 size)
{
	union acpi_object *obj;
	struct acpi_buffer buffer = { ACPI_ALLOCATE_BUFFER, NULL };
	struct acpi_handle *handle = ACPI_HANDLE(dev);
	acpi_status status;
	int ret;

	status = acpi_evaluate_object(handle, id, NULL, &buffer);
	if (ACPI_FAILURE(status))
		return -ENODEV;

	obj = buffer.pointer;
	if (!obj || obj->type != ACPI_TYPE_BUFFER) {
		dev_err(dev, "Could't read acpi buffer\n");
		ret = -ENODEV;
		goto err_free_buff;
	}

	if (obj->buffer.length > size) {
		dev_err(dev, "Given buffer is too small\n");
		ret = -ENODEV;
		goto err_free_buff;
	}

	memcpy(data, obj->buffer.pointer, obj->buffer.length);
	kfree(buffer.pointer);

	return obj->buffer.length;;
err_free_buff:
	kfree(buffer.pointer);
	return ret;
}

static int get_acpi_ssdb_sensor_data(struct device *dev,
				     struct sensor_bios_data *sensor)
{
	struct sensor_bios_data_packed sensor_data;
	int ret;
	
	ret = read_acpi_block(dev, "SSDB", &sensor_data, sizeof(sensor_data));
	if (ret < 0)
		return ret;

	sensor->link = sensor_data.link;
	sensor->lanes = sensor_data.lanes;
	sensor->mclkspeed = sensor_data.mclkspeed;

	return 0;
}

static int create_endpoint_properties(struct sensor *sensor,
				      struct sensor_bios_data *ssdb)
{
	struct property_entry *sensor_props;
	struct property_entry *cio2_props;
	u32 *data_lanes;
	int i;

	data_lanes = devm_kmalloc_array(sensor->dev, ssdb->lanes, sizeof(u32),
					GFP_KERNEL);
	if (!data_lanes)
		return -ENOMEM;

	for (i = 0; i < ssdb->lanes; i++)
		data_lanes[i] = i + 1;

	sensor_props = kcalloc(6, sizeof(*sensor_props), GFP_KERNEL);
	if (!sensor_props)
		return -ENOMEM;

	sensor_props[0] = PROPERTY_ENTRY_U32("clock-frequency",
						ssdb->mclkspeed);
	sensor_props[1] = PROPERTY_ENTRY_U32("bus-type", 5);
	sensor_props[2] = PROPERTY_ENTRY_U32("clock-lanes", 0);
	sensor_props[3] = PROPERTY_ENTRY_U32_ARRAY_LEN("data-lanes",
							data_lanes,
							(int)ssdb->lanes);
	sensor_props[4] = remote_endpoints[(bridge.n_sensors * 2) + ENDPOINT_SENSOR];

	memcpy(sensor->sensor_props, sensor_props, sizeof(*sensor_props) * 6);
	kfree(sensor_props);

	cio2_props = kcalloc(6, sizeof(*cio2_props), GFP_KERNEL);
	if (!cio2_props)
		return -ENOMEM;

	cio2_props[0] = PROPERTY_ENTRY_U32_ARRAY_LEN("data-lanes",
						     data_lanes,
						     ssdb->lanes);
	cio2_props[1] = remote_endpoints[(bridge.n_sensors * 2) + ENDPOINT_CIO2];

	memcpy(sensor->cio2_props, cio2_props, sizeof(*cio2_props) * 6);
	kfree(cio2_props);

	return 0;
}

static int create_connection_swnodes(struct sensor *sensor,
				     struct sensor_bios_data *ssdb)
{
	struct software_node *nodes;
	char portn[6];
	int ret;

	ret = snprintf(portn, 6, "port%d", ssdb->link);

	if (!ret) {
		pr_info("cio2-bridge: failed to print the portname\n");
		return -EINVAL;
	}

	nodes = kcalloc(6, sizeof(*nodes) * 6, GFP_KERNEL);

	if (!nodes)
		return -ENOMEM;

	nodes[SWNODE_SENSOR_HID] = NODE_HID(sensor->name);
	nodes[SWNODE_SENSOR_PORT] = NODE_PORT("port0",
					      &sensor->swnodes[SWNODE_SENSOR_HID]);
	nodes[SWNODE_SENSOR_ENDPOINT] = NODE_ENDPOINT("endpoint0",
						      &sensor->swnodes[SWNODE_SENSOR_PORT],
						      sensor->sensor_props);
	nodes[SWNODE_CIO2_PORT] = NODE_PORT(portn,
					    &cio2_hid_node);
	nodes[SWNODE_CIO2_ENDPOINT] = NODE_ENDPOINT("endpoint0",
						    &sensor->swnodes[SWNODE_CIO2_PORT],
						    sensor->cio2_props);

	memcpy(sensor->swnodes, nodes, sizeof(*nodes) * 6);
	kfree(nodes);

	return 0;
}

static int connect_supported_devices(void)
{
	struct acpi_device *adev;
	struct device *dev;
	struct sensor_bios_data ssdb;
	struct sensor *sensor;
	struct fwnode_handle *fwnode;
	struct v4l2_subdev *sd;
	int i, j, ret;

	for (i = 0; i < ARRAY_SIZE(supported_devices); i++) {
		adev = acpi_dev_get_first_match_dev(supported_devices[i],
						    NULL, -1);

		if (!adev)
			continue;

		dev = bus_find_device_by_acpi_dev(&i2c_bus_type, adev);

		if (!dev) {
			pr_info("ACPI match for %s, but it has no i2c device\n",
				supported_devices[i]);
			continue;
		}

		if (!dev->driver_data) {
			pr_info("ACPI match for %s, but it has no driver\n",
				supported_devices[i]);
			put_device(dev);
			continue;
		}

		pr_info("Found supported device %s\n", supported_devices[i]);

		sensor = &bridge.sensors[bridge.n_sensors];
		sensor->dev = dev;

		ret = snprintf(sensor->name, 20, "%s", supported_devices[i]);
		if (!ret)
			return -EINVAL;

		/*
		 * Store sensor's existing fwnode so that it can be restored if
		 * this module is removed.
		 */
		sensor->fwnode = fwnode_handle_get(dev->fwnode);

		get_acpi_ssdb_sensor_data(dev, &ssdb);

		ret = create_endpoint_properties(sensor, &ssdb);
		if (ret)
			return ret;

		/* build the software nodes */

		ret = create_connection_swnodes(sensor, &ssdb);
		if (ret)
			return ret;

		ret = software_node_register_nodes(sensor->swnodes);
		if (ret) {
			dev_err(dev,
				"Failed to register software nodes for %s\n",
				supported_devices[i]);
			return ret;
		}

		fwnode = software_node_fwnode(&sensor->swnodes[SWNODE_SENSOR_HID]);
		if (!fwnode) {
			dev_err(dev,
				"Failed to get software node for %s\n",
				supported_devices[i]);
			ret = -ENODEV;
			goto err_unregister_nodes;
		}

		fwnode->secondary = ERR_PTR(-ENODEV);
		dev->fwnode = fwnode;

		/*
		 * The device should by this point has driver_data set to an
		 * instance of struct v4l2_subdev; set the fwnode for that too.
		 */

		sd = dev_get_drvdata(dev);
		sd->fwnode = fwnode;

		bridge.n_sensors++;
	}

	return 0;

err_unregister_nodes:

	for (j = 4; j >= 0; j--)
		software_node_unregister(&sensor->swnodes[j]);

err_put_dev:

	put_device(dev);

	return ret;
}

static int cio2_bridge_init(void)
{
	struct fwnode_handle *fwnode;
	int ret;

	ret = software_node_register(&cio2_hid_node);
	if (ret < 0) {
		pr_err("cio2-bridge: Failed to register the CIO2 HID node\n");
		return ret;
	}

	ret = connect_supported_devices();

	if ((ret < 0) || (bridge.n_sensors <= 0)) {
		pr_err("cio2_bridge: Failed to connect any devices\n");
		goto out;
	} else {
		pr_info("Found %d supported devices\n", bridge.n_sensors);
	}

	bridge.cio2 = pci_get_device(PCI_VENDOR_ID_INTEL, CIO2_PCI_ID, NULL);
	if (!bridge.cio2) {
		ret = -ENODEV;
		goto out;
	}

	fwnode = software_node_fwnode(&cio2_hid_node);
	if (!fwnode) {
		pr_err("Error getting fwnode from cio2 software_node\n");
		ret = -ENODEV;
		goto out;
	}

	/*
	 * We store the pci_dev's existing fwnode, beccause in the event we
	 * want to reload (I.E. rmmod and insmod) this module we need to give
	 * the device its original fwnode back to prevent problems down the
	 * line
	 */

	bridge.cio2_fwnode = fwnode_handle_get(bridge.cio2->dev.fwnode);

	fwnode->secondary = ERR_PTR(-ENODEV);
	bridge.cio2->dev.fwnode = fwnode;

	ret = device_reprobe(&bridge.cio2->dev);
	if (ret) {
		dev_warn(&bridge.cio2->dev, "Reprobing error: %d\n", ret);
		goto out;
	}

	return 0;
out:
	cio2_bridge_exit();
	return ret;
}

static void cio2_bridge_unregister_sensors(void)
{
	int i, j;
	struct sensor *sensor;

	for (i = 0; i < bridge.n_sensors; i++) {
		sensor = &bridge.sensors[i];

		/* give the sensor its original fwnode back */
		sensor->dev->fwnode = sensor->fwnode;
		fwnode_handle_put(sensor->fwnode);
		put_device(sensor->dev);

		for (j = 4; j >= 0; j--)
			software_node_unregister(&sensor->swnodes[j]);
	}
}

static void cio2_bridge_exit(void)
{
	/* Give the pci_dev its original fwnode back */
	if (bridge.cio2) {
		bridge.cio2->dev.fwnode = bridge.cio2_fwnode;
		fwnode_handle_put(bridge.cio2_fwnode);
		pci_dev_put(bridge.cio2);
	}

	cio2_bridge_unregister_sensors();

	software_node_unregister(&cio2_hid_node);
}

module_init(cio2_bridge_init);
module_exit(cio2_bridge_exit);

MODULE_DESCRIPTION("A bridge driver to connect sensors to CIO2 infrastructure.");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("acpi*:INT343E:*");
