// SPDX-License-Identifier: GPL-2.0
#include <linux/acpi.h>
#include <linux/cio2-bridge.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/module.h>

#include <media/v4l2-subdev.h>

/*
 * Extend this array with ACPI Hardware ID's of devices known to be
 * working
 */

static const struct ipu3_sensor supported_devices[] = {
	IPU3_SENSOR("INT33BE", "INT33BE:00"),
	IPU3_SENSOR("OVTI2680", "OVTI2680:00"),
	IPU3_SENSOR("OVTI5648", "OVTI5648:00")
};

/*
	{"INT33BE", {{"INT33BE:00", 0}, { }}},
	{"OVTI2680", {{"OVTI2680:00", 0}, { }}},
	{"OVTI5648", {{"OVTI5648:00", 0}, { }}},
*/

struct software_node cio2_hid_node = { CIO2_HID, };

struct cio2_bridge bridge = { 0, };

static const char * const port_names[] = {
	"port0", "port1", "port2", "port3"
};

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

	return obj->buffer.length;
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
	sensor->degree = sensor_data.degree;

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
	sensor_props[2] = PROPERTY_ENTRY_U32_ARRAY_LEN("data-lanes",
						       data_lanes,
						       ssdb->lanes);
	sensor_props[3] = remote_endpoints[(bridge.n_sensors * 2) + ENDPOINT_SENSOR];
	sensor_props[4] = PROPERTY_ENTRY_U8("rotation", ssdb->degree);

	memcpy(sensor->sensor_props, sensor_props, sizeof(*sensor_props) * 6);
	kfree(sensor_props);

	cio2_props = kcalloc(3, sizeof(*cio2_props), GFP_KERNEL);
	if (!cio2_props)
		return -ENOMEM;

	cio2_props[0] = PROPERTY_ENTRY_U32_ARRAY_LEN("data-lanes",
						     data_lanes,
						     ssdb->lanes);
	cio2_props[1] = remote_endpoints[(bridge.n_sensors * 2) + ENDPOINT_CIO2];

	memcpy(sensor->cio2_props, cio2_props, sizeof(*cio2_props) * 2);
	kfree(cio2_props);

	return 0;
}

static int create_connection_swnodes(struct sensor *sensor,
				     struct sensor_bios_data *ssdb)
{
	struct software_node *nodes;

	nodes = kcalloc(6, sizeof(*nodes) * 6, GFP_KERNEL);
	if (!nodes)
		return -ENOMEM;

	nodes[SWNODE_SENSOR_HID] = NODE_HID(sensor->name);
	nodes[SWNODE_SENSOR_PORT] = NODE_PORT("port0",
					      &sensor->swnodes[SWNODE_SENSOR_HID]);
	nodes[SWNODE_SENSOR_ENDPOINT] = NODE_ENDPOINT("endpoint0",
						      &sensor->swnodes[SWNODE_SENSOR_PORT],
						      sensor->sensor_props);
	nodes[SWNODE_CIO2_PORT] = NODE_PORT(port_names[ssdb->link],
					    &cio2_hid_node);
	nodes[SWNODE_CIO2_ENDPOINT] = NODE_ENDPOINT("endpoint0",
						    &sensor->swnodes[SWNODE_CIO2_PORT],
						    sensor->cio2_props);

	memcpy(sensor->swnodes, nodes, sizeof(*nodes) * 5);
	kfree(nodes);

	return 0;
}

static void cio2_bridge_unregister_sensors(void)
{
	int i, j;
	struct sensor *sensor;

	for (i = 0; i < bridge.n_sensors; i++) {
		sensor = &bridge.sensors[i];

		for (j = 4; j >= 0; j--)
			software_node_unregister(&sensor->swnodes[j]);

		/*
		 * Give the sensor its original fwnode back or the next time
		 * it's probed will fail, because ACPI matching doesn't work
		 * when your fwnode doesn't have acpi_device_fwnode_ops.
		 */
		sensor->dev->fwnode = sensor->fwnode;
		fwnode_handle_put(sensor->fwnode);

		device_release_driver(sensor->dev);
		i2c_del_driver(sensor->new_drv);
		i2c_add_driver(sensor->old_drv);
		device_reprobe(sensor->dev);
		put_device(sensor->dev);
		kfree(sensor->new_drv);
	}
}

static int cio2_bridge_reprobe_sensor(struct sensor *sensor, int dev_idx)
/*
 * We have to reprobe the sensor, but having just overwritten the ACPI fwnode
 * the usual matching won't work by default. We need to clone the existing
 * driver but add an i2c_device_id so the matching works.
 */
{
	struct i2c_client *client;
	int ret;

	client = container_of(sensor->dev, struct i2c_client, dev);

	sensor->old_drv = container_of(sensor->dev->driver, struct i2c_driver, driver);
	sensor->new_drv = kmalloc(sizeof(*sensor->new_drv), GFP_KERNEL);
	if (!sensor->new_drv)
		return -ENOMEM;
	
	sensor->new_drv->driver.name = sensor->old_drv->driver.name;
	sensor->new_drv->probe_new = sensor->old_drv->probe_new;
	sensor->new_drv->remove = sensor->old_drv->remove;
	sensor->new_drv->id_table = supported_devices[dev_idx].i2c_id;

	device_release_driver(sensor->dev);
	i2c_del_driver(sensor->old_drv);

	ret = i2c_add_driver(sensor->new_drv);
	if (ret)
		goto err_replace_old_drv;

	ret = device_reprobe(sensor->dev);
	if (ret)
		goto err_remove_new_drv;

	return 0;

err_remove_new_drv:
	i2c_del_driver(sensor->new_drv);
err_replace_old_drv:
	i2c_add_driver(sensor->old_drv);

	device_reprobe(sensor->dev);

	return ret;
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

	ret = 0;

	for (i = 0; i < ARRAY_SIZE(supported_devices); i++) {
		adev = acpi_dev_get_first_match_dev(supported_devices[i].hid,
						    NULL, -1);
		if (!adev)
			continue;

		dev = bus_find_device_by_acpi_dev(&i2c_bus_type, adev);
		if (!dev) {
			ret = -EPROBE_DEFER;
			goto err_rollback;
		}

		/*
		 * We won't be able to reprobe() the sensor after our changes
		 * here, so we rely on their probe() having completed.
		 */

		if (dev->links.status == DL_DEV_PROBING) {
			ret = -EPROBE_DEFER;
			goto err_free_dev;
		}

		pr_info("cio2-bridge: Found supported device %s\n",
			supported_devices[i].hid);

		sensor = &bridge.sensors[bridge.n_sensors];
		sensor->dev = dev;

		snprintf(sensor->name, 20, "%s", supported_devices[i].hid);

		sensor->fwnode = fwnode_handle_get(dev->fwnode);
		if (!sensor->fwnode)
			goto err_free_dev;

		ret = get_acpi_ssdb_sensor_data(dev, &ssdb);
		if (ret)
			goto err_free_fwnode;

		ret = create_endpoint_properties(sensor, &ssdb);
		if (ret)
			goto err_free_fwnode;

		ret = create_connection_swnodes(sensor, &ssdb);
		if (ret)
			goto err_free_fwnode;

		ret = software_node_register_nodes(sensor->swnodes);
		if (ret)
			goto err_free_fwnode;

		fwnode = software_node_fwnode(&sensor->swnodes[SWNODE_SENSOR_HID]);
		if (!fwnode) {
			ret = -ENODEV;
			goto err_free_swnodes;
		}

		sd = dev_get_drvdata(dev);
		if (!sd) {
			pr_info("cio2-bridge: %s has no driver\n",
				supported_devices[i].hid);
			goto err_free_swnodes;
		}

		fwnode->secondary = ERR_PTR(-ENODEV);
		dev->fwnode = fwnode;

		ret = cio2_bridge_reprobe_sensor(sensor, i);
		if (ret)
			goto err_free_swnodes;

		bridge.n_sensors++;
		continue;

err_free_swnodes:
		for (j = 4; j >= 0; j--)
			software_node_unregister(&sensor->swnodes[j]);
err_free_fwnode:
		fwnode_handle_put(sensor->fwnode);
err_free_dev:
		put_device(dev);
err_rollback:
		/*
		 * If an iteration of this loop results in -EPROBE_DEFER then
		 * we need to roll back any sensors that were successfully
		 * registered. Any other error and we'll skip that step, as
		 * it seems better to have one successfully connected sensor.
		 */

		if (ret == -EPROBE_DEFER)
			cio2_bridge_unregister_sensors();

		break;
	}

	return ret;
}

int cio2_bridge_init(struct pci_dev *cio2)
{
	struct fwnode_handle *fwnode;
	int ret;

	pci_dev_get(cio2);

	ret = software_node_register(&cio2_hid_node);
	if (ret < 0) {
		pr_err("cio2-bridge: Failed to register the CIO2 HID node\n");
		goto err_free_cio2;
	}

	ret = connect_supported_devices();
	if (ret == -EPROBE_DEFER) {
		pr_info("cio2-bridge: deferring\n");
		goto err_unregister_cio2;
	}

	if (bridge.n_sensors == 0) {
		pr_info("cio2-bridge: Failed to connect any sensors\n");
		ret = -ENODEV;
		goto err_unregister_cio2;
	}

	pr_info("cio2-bridge: Connected %d cameras\n", bridge.n_sensors);

	fwnode = software_node_fwnode(&cio2_hid_node);
	if (!fwnode) {
		pr_err("Error getting fwnode from cio2 software_node\n");
		ret = -ENODEV;
		goto err_unregister_sensors;
	}

	/*
	 * We store the pci_dev's existing fwnode, because in the event we
	 * want to reload the ipu3-cio2 driver we need to give the device its
	 * original fwnode back to prevent problems.
	 */

	bridge.cio2_fwnode = fwnode_handle_get(cio2->dev.fwnode);

	fwnode->secondary = ERR_PTR(-ENODEV);
	cio2->dev.fwnode = fwnode;

	return 0;

err_unregister_sensors:
	cio2_bridge_unregister_sensors();
err_unregister_cio2:
	software_node_unregister(&cio2_hid_node);
err_free_cio2:
	pci_dev_put(cio2);

	return ret;
}
EXPORT_SYMBOL_GPL(cio2_bridge_init);

void cio2_bridge_exit(struct pci_dev *cio2)
{
	/* Give the pci_dev its original fwnode back */
	cio2->dev.fwnode = bridge.cio2_fwnode;
	fwnode_handle_put(bridge.cio2_fwnode);
	pci_dev_put(cio2);

	cio2_bridge_unregister_sensors();

	software_node_unregister(&cio2_hid_node);

	memset(&bridge, 0, sizeof(struct cio2_bridge));
}
EXPORT_SYMBOL_GPL(cio2_bridge_exit);
