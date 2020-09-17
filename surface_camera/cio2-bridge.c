#include <linux/acpi.h>
#include <acpi/acpi_bus.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <media/v4l2-subdev.h>

#include <linux/fwnode.h>
#include <linux/kref.h>

static void cio2_bridge_exit(void);
static int cio2_bridge_init(void);

#define MAX_CONNECTED_DEVICES                4
#define SWNODE_SENSOR_HID                    0
#define SWNODE_SENSOR_PORT                   1
#define SWNODE_SENSOR_ENDPOINT               2
#define SWNODE_CIO2_PORT                     3
#define SWNODE_CIO2_ENDPOINT                 4
#define SWNODE_NULL_TERMINATOR               5

#define CIO2_HID                     "INT343E"
#define CIO2_PCI_ID                     0x9d32

#define ENDPOINT_SENSOR                      0
#define ENDPOINT_CIO2                        1

#define NODE_HID(_HID)                       \
    (const struct software_node) {           \
        _HID,                                \
    }

#define NODE_PORT(_PORT, _HID_NODE)          \
    (const struct software_node) {           \
        _PORT,                               \
        _HID_NODE,                           \
    }

#define NODE_ENDPOINT(_EP, _PORT, _PROPS)    \
    (const struct software_node) {           \
        _EP,                                 \
        _PORT,                               \
        _PROPS,                              \
    }

#define PROPERTY_ENTRY_NULL                  \
    (const struct property_entry) { }

#define SOFTWARE_NODE_NULL                   \
    (const struct software_node) { }

/*
 * Extend this array with ACPI Hardware ID's of devices known to be
 * working
 */

static char* supported_devices[] = {
    "INT33BE",
    "OVTI2680",
    "OVTI5648",
};

/*
 * software_node needs const char * names. Can't snprintf a const char *,
 * so instead we need an array of them and use the port num from SSDB as
 * an index.
 */

const char *port_names[] = {
    "port0", "port1", "port2", "port3", "port4",
    "port5", "port6", "port7", "port8", "port9"
};

struct software_node cio2_hid_node = { CIO2_HID, };

struct sensor {
    struct device *dev;
    struct software_node swnodes[5];
    struct property_entry sensor_props[7];
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
    u8 degree;
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
    int ret = read_acpi_block(dev, "SSDB", &sensor_data, sizeof(sensor_data));
    if (ret < 0) {
        dev_err(dev, "Failed to fetch SSDB data\n");
        return ret;
    }

    sensor->link = sensor_data.link;
    sensor->lanes = sensor_data.lanes;
    sensor->mclkspeed = sensor_data.mclkspeed;
    sensor->degree = sensor_data.degree;

    return 0;
}

static int create_endpoint_properties(struct device *dev,
                    struct sensor_bios_data *ssdb,
                    struct property_entry *sensor_props,
                    struct property_entry *cio2_props)
{
        u32 *data_lanes;
        int i;

        data_lanes = devm_kmalloc(dev, sizeof(u32) * (int)ssdb->lanes,
                            GFP_KERNEL);

        if (!data_lanes) {
            dev_err(dev, "Couldn't allocate memory for data lanes array\n");
            return -ENOMEM;
        }

        for (i = 0; i < (int)ssdb->lanes; i++) {
            data_lanes[i] = (u32)i+1;
        }

        sensor_props[0] = PROPERTY_ENTRY_U32("clock-frequency",
                            ssdb->mclkspeed);
        sensor_props[1] = PROPERTY_ENTRY_U32("bus-type", 5);
        sensor_props[2] = PROPERTY_ENTRY_U32("clock-lanes", 0);
        sensor_props[3] = PROPERTY_ENTRY_U32_ARRAY_LEN("data-lanes",
                            data_lanes, (int)ssdb->lanes);
        sensor_props[4] = remote_endpoints[(bridge.n_sensors * 2) + ENDPOINT_SENSOR];
	sensor_props[5] = PROPERTY_ENTRY_U8("degree", ssdb->degree);
        sensor_props[6] = PROPERTY_ENTRY_NULL;

        cio2_props[0] = PROPERTY_ENTRY_U32_ARRAY_LEN("data-lanes", data_lanes,
                            (int)ssdb->lanes);
        cio2_props[1] = remote_endpoints[(bridge.n_sensors * 2) + ENDPOINT_CIO2];
        cio2_props[2] = PROPERTY_ENTRY_NULL;

        return 0;
}

static int connect_supported_devices(void)
{
    struct acpi_device *adev;
    struct device *dev;
    struct sensor_bios_data ssdb;
    struct property_entry *sensor_props;
    struct property_entry *cio2_props;
    struct fwnode_handle *fwnode;
    struct software_node *nodes;
    struct v4l2_subdev *sd;
    int i, ret;

    for (i = 0; i < ARRAY_SIZE(supported_devices); i++) {

        adev = acpi_dev_get_first_match_dev(supported_devices[i], NULL, -1);

        if (!adev) {
            continue;
        }

        dev = bus_find_device_by_acpi_dev(&i2c_bus_type, adev);

        if (!dev) {
            pr_info("ACPI match for %s, but it has no i2c device\n",
                                supported_devices[i]);
            continue;
        }

        if (!dev->driver_data) {
            pr_info("ACPI match for %s, but it has no driver\n",
                                supported_devices[i]);
            continue;
        } else {
            pr_info("Found supported device %s\n", supported_devices[i]);
        }

        /*
         * Store sensor's existing fwnode so that it can be restored if this
         * module is removed.
         */
        bridge.sensors[bridge.n_sensors].fwnode = fwnode_handle_get(dev->fwnode);

        get_acpi_ssdb_sensor_data(dev, &ssdb);

        nodes = bridge.sensors[bridge.n_sensors].swnodes;
        sensor_props = bridge.sensors[bridge.n_sensors].sensor_props;
        cio2_props = bridge.sensors[bridge.n_sensors].cio2_props;
        fwnode = bridge.sensors[bridge.n_sensors].fwnode;
            
        ret = create_endpoint_properties(dev, &ssdb, sensor_props, cio2_props);

        if (ret)
            return ret;

        /* build the software nodes */

        nodes[SWNODE_SENSOR_HID]        = NODE_HID(supported_devices[i]);
        nodes[SWNODE_SENSOR_PORT]       = NODE_PORT(
                                            "port0",
                                            &nodes[SWNODE_SENSOR_HID]);
        nodes[SWNODE_SENSOR_ENDPOINT]   = NODE_ENDPOINT(
                                            "endpoint0",
                                            &nodes[SWNODE_SENSOR_PORT],
                                            sensor_props);
        nodes[SWNODE_CIO2_PORT]         = NODE_PORT(
                                            port_names[(int)ssdb.link],
                                            &cio2_hid_node);
        nodes[SWNODE_CIO2_ENDPOINT]     = NODE_ENDPOINT(
                                            "endpoint0",
                                            &nodes[SWNODE_CIO2_PORT],
                                            cio2_props);
        nodes[SWNODE_NULL_TERMINATOR]   = SOFTWARE_NODE_NULL;

        ret = software_node_register_nodes(nodes);
        if (ret) {
            dev_err(dev, "Failed to register the software nodes for %s\n",
                                supported_devices[i]);
            return ret;
        }

        fwnode = software_node_fwnode(&nodes[SWNODE_SENSOR_HID]);
        if (!fwnode) {
            dev_err(dev, "Failed to get fwnode from software node for %s\n",
                                supported_devices[i]);
            return ret;
        }

        fwnode->secondary = ERR_PTR(-ENODEV);
        dev->fwnode = fwnode;

        /*
         * The device should by this point has driver_data set to an instance
         * of struct v4l2_subdev; set the fwnode for that too.
         */

        sd = dev_get_drvdata(dev);
        sd->fwnode = fwnode;

        bridge.sensors[bridge.n_sensors].dev = dev;
        bridge.n_sensors++;

    }

    return 0;
}

static int cio2_bridge_init(void)
{
    struct fwnode_handle *fwnode;
    int ret;

    ret = software_node_register(&cio2_hid_node);

    if (ret < 0) {
        pr_err("Failed to register the CIO2 HID node\n");
        return -EINVAL;
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
     * We store the pci_dev's existing fwnode, beccause in the event we want 
     * to reload (I.E. rmmod and insmod) this module we need to give the device
     * its original fwnode back to prevent problems down the line
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

static int cio2_bridge_unregister_sensors(void)
{
    int i,j;
    struct sensor *sensor;

    for (i=0; i < bridge.n_sensors; i++) {
        sensor = &bridge.sensors[i];

        /* give the sensor its original fwnode back */
        sensor->dev->fwnode = sensor->fwnode;
        fwnode_handle_put(sensor->fwnode);
        put_device(sensor->dev);
        
        for (j=4; j>=0; j--) {
            software_node_unregister(&sensor->swnodes[j]);
        }
    }

    return 0;
}

static void cio2_bridge_exit(void)
{
    int ret;

    /* Give the pci_dev its original fwnode back */
    if (bridge.cio2) {
        bridge.cio2->dev.fwnode = bridge.cio2_fwnode;
        fwnode_handle_put(bridge.cio2_fwnode);
        pci_dev_put(bridge.cio2);
    }
 
    ret = cio2_bridge_unregister_sensors();

    if (ret)
        pr_err("An error occurred unregistering the sensors\n");

    software_node_unregister(&cio2_hid_node);

}

module_init(cio2_bridge_init);
module_exit(cio2_bridge_exit);

MODULE_DESCRIPTION("A bridge driver to connect sensors to CIO2 infrastructure.");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("acpi*:INT343E:*");
