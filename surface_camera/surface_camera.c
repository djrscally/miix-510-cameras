#include <linux/acpi.h>
#include <acpi/acpi_bus.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <media/v4l2-subdev.h>

#include <linux/fwnode.h>

#include "surface_camera.h"

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

struct pci_dev *cio2;

struct software_node cio2_hid_node = { CIO2_HID, };

struct sensor {
    struct device *dev;
    struct software_node swnodes[5];
    struct property_entry sensor_props[6];
    struct property_entry cio2_props[3];
    struct fwnode_handle *fwnode;
};

struct connected_devices {
    int n_devices;
    struct sensor sensors[MAX_CONNECTED_DEVICES];
};

struct connected_devices cdevs = {
    .n_devices = 0,
};

static const struct property_entry remote_endpoints[] = {
    PROPERTY_ENTRY_REF("remote-endpoint", /* Sensor 0, Sensor Property */
                        &cdevs.sensors[0].swnodes[SWNODE_CIO2_ENDPOINT]),
    PROPERTY_ENTRY_REF("remote-endpoint", /* Sensor 0, CIO2 Property */
                        &cdevs.sensors[0].swnodes[SWNODE_SENSOR_ENDPOINT]),    
    PROPERTY_ENTRY_REF("remote-endpoint",
                        &cdevs.sensors[1].swnodes[SWNODE_CIO2_ENDPOINT]),    
    PROPERTY_ENTRY_REF("remote-endpoint",
                        &cdevs.sensors[1].swnodes[SWNODE_SENSOR_ENDPOINT]),
    PROPERTY_ENTRY_REF("remote-endpoint",
                        &cdevs.sensors[2].swnodes[SWNODE_CIO2_ENDPOINT]),
    PROPERTY_ENTRY_REF("remote-endpoint",
                        &cdevs.sensors[2].swnodes[SWNODE_SENSOR_ENDPOINT]),
    PROPERTY_ENTRY_REF("remote-endpoint",
                        &cdevs.sensors[3].swnodes[SWNODE_CIO2_ENDPOINT]),
    PROPERTY_ENTRY_REF("remote-endpoint",
                        &cdevs.sensors[3].swnodes[SWNODE_SENSOR_ENDPOINT]),
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
    int ret = read_acpi_block(dev, "SSDB", &sensor_data, sizeof(sensor_data));
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

static int connect_supported_devices(void)
{
    struct acpi_device *adev;
    struct device *dev;
    struct sensor_bios_data ssdb;
    struct property_entry *sensor_props;
    struct property_entry *cio2_props;
    struct fwnode_handle *fwnode;
    struct software_node *nodes;
    u32 *data_lanes;
    int i, j, ret;

    for (i = 0; i < ARRAY_SIZE(supported_devices); i++) {

        adev = acpi_dev_get_first_match_dev(supported_devices[i], NULL, -1);

        if (!adev) {
            continue;
        }

        dev = bus_find_device_by_acpi_dev(&i2c_bus_type, adev);

        if (!dev) {
            continue;
        }

        if (!dev->driver_data) {
            pr_info("Found supported device %s, but it has no driver\n",
                                supported_devices[i]);
            continue;
        } else {
            pr_info("Found supported device %s\n", supported_devices[i]);
        }

        get_acpi_ssdb_sensor_data(dev, &ssdb);

        nodes = cdevs.sensors[cdevs.n_devices].swnodes;
        sensor_props = cdevs.sensors[cdevs.n_devices].sensor_props;
        cio2_props = cdevs.sensors[cdevs.n_devices].cio2_props;
        fwnode = cdevs.sensors[cdevs.n_devices].fwnode;
            
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

        sensor_props[0] = PROPERTY_ENTRY_U32("clock-frequency",
                            ssdb.mclkspeed);
        sensor_props[1] = PROPERTY_ENTRY_U32("bus-type", 5);
        sensor_props[2] = PROPERTY_ENTRY_U32("clock-lanes", 0);
        sensor_props[3] = PROPERTY_ENTRY_U32_ARRAY_LEN("data-lanes",
                            data_lanes, (int)ssdb.lanes);
        sensor_props[4] = remote_endpoints[(cdevs.n_devices * 2) + ENDPOINT_SENSOR];
        sensor_props[5] = PROPERTY_ENTRY_NULL;

        cio2_props[0] = PROPERTY_ENTRY_U32_ARRAY_LEN("data-lanes", data_lanes,
                            (int)ssdb.lanes);
        cio2_props[1] = remote_endpoints[(cdevs.n_devices * 2) + ENDPOINT_CIO2];
        cio2_props[2] = PROPERTY_ENTRY_NULL;

        /* build the software nodes */

        nodes[SWNODE_SENSOR_HID] = NODE_HID(supported_devices[i]);                                                /* Sensor HID Node */
        nodes[SWNODE_SENSOR_PORT] = NODE_PORT("port0",&nodes[SWNODE_SENSOR_HID]);                                /* Sensor Port Node */
        nodes[SWNODE_SENSOR_ENDPOINT] = NODE_ENDPOINT("endpoint0", &nodes[SWNODE_SENSOR_PORT], sensor_props);     /* Sensor Endpoint Node */
        nodes[SWNODE_CIO2_PORT] = NODE_PORT(port_names[(int)ssdb.link], &cio2_hid_node);                         /* CIO2 Port Node */
        nodes[SWNODE_CIO2_ENDPOINT] = NODE_ENDPOINT("endpoint0", &nodes[SWNODE_CIO2_PORT], cio2_props);           /* CIO2 Endpoint Node */
        nodes[SWNODE_NULL_TERMINATOR] = SOFTWARE_NODE_NULL;

        ret = software_node_register_nodes(nodes);
        if (ret) {
            dev_err(dev, "Failed to register the software nodes for %s\n",
                                supported_devices[i]);
            return 0;
        }

        fwnode = software_node_fwnode(&nodes[SWNODE_SENSOR_HID]);
        if (!fwnode) {
            dev_err(dev, "Failed to get fwnode from software node for %s\n",
                                supported_devices[i]);
            return 0;
        }

        fwnode->secondary = ERR_PTR(-ENODEV);
        set_primary_fwnode(dev, fwnode);
        ((struct v4l2_subdev *)dev->driver_data)->fwnode = fwnode;

        /* we're done */
        cdevs.sensors[cdevs.n_devices].dev = dev;
        cdevs.n_devices++;

    }

    return 0;
}

static int surface_camera_init(void)
{
    struct fwnode_handle *fwnode;
    int ret;

    /* Register the CIO2 Parent node */
    ret = software_node_register(&cio2_hid_node);

    if (ret < 0) {
        pr_err("Failed to register the CIO2 HID node\n");
        return -EINVAL;
    }

    /* Check for supported devices and connect them*/
    ret = connect_supported_devices();

    if ((ret < 0) || (cdevs.n_devices == 0)) {
        pr_err("Failed to connect any devices\n");
        goto out;
    }

    if (cdevs.n_devices > 0) {
        pr_info("Found %d supported devices\n", cdevs.n_devices);
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
    set_primary_fwnode(&cio2->dev, fwnode);

    ret = device_reprobe(&cio2->dev);
    if (ret) {
        dev_warn(&cio2->dev, "Reprobing error: %d\n", ret);
        goto out;
    }

    return 0;
out:
    surface_camera_exit();
    return ret;
}

static int surface_camera_unregister_sensors(void)
{
    int i,j;
    struct sensor sensor;
    struct fwnode_handle *fwnode;

    for (i=0; i < cdevs.n_devices; i++) {

        sensor = cdevs.sensors[i];

        for (j=4; j>=0; j--) {
            
            software_node_unregister(&sensor.swnodes[j]);

            fwnode = software_node_fwnode(&sensor.swnodes[j]);

            if (!IS_ERR_OR_NULL(fwnode)) {

                fwnode->secondary = NULL;

                fwnode_handle_put(fwnode);
            }

        }

        put_device(sensor.dev);
    }

    return 0;
}

static void surface_camera_exit(void)
{
    int ret;
    struct fwnode_handle *fwnode;

    if (cio2) {
        cio2->dev.fwnode->secondary = NULL;
        fwnode_handle_put(cio2->dev.fwnode);
        cio2->dev.fwnode = NULL;
    }

    pci_dev_put(cio2);
 
    ret = surface_camera_unregister_sensors();

    if (ret) {
        pr_err("An error occurred unregistering the sensors\n");
    }

    software_node_unregister(&cio2_hid_node);

    fwnode = software_node_fwnode(&cio2_hid_node);

    if (fwnode) {
        fwnode->secondary = NULL;
        fwnode_handle_put(fwnode);
    }  

}

module_init(surface_camera_init);
module_exit(surface_camera_exit);

MODULE_DESCRIPTION("A bridge driver to connect sensors to CIO2 infrastructure.");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("acpi*:INT343E:*");
