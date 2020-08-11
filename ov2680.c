#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/acpi.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>

static const struct acpi_gpio_params gpio1 = {0, 0, false};
static const struct acpi_gpio_params gpio2 = {1, 0, false};
static const struct acpi_gpio_params gpio3 = {2, 0, false};

static const struct acpi_gpio_mapping int3472_acpi_gpios[] = {
    {"gpio1", &gpio1, 1},
    {"gpio2", &gpio2, 1},
    {"gpio3", &gpio3, 1},
    {},
};

static int match_depend(struct device *dev, const void *data)
{
    return (dev && dev->fwnode == data) ? 1 : 0;
}

static int ov2680_probe(struct i2c_client *client)
{
    
    /*
     * The driver will match the OV2680 device, but the GPIO
     * pins lie in its dependent INT3472, so we need to walk
     * up the dependencies to find that device.
    */
   struct acpi_device   *int3472_device;
   struct device        *dev;

   /* get ACPI handle of OV2680 device */
   struct acpi_handle *dev_handle = ACPI_HANDLE(&client->dev);

   /* Get dependent devices */
   struct acpi_handle_list dep_devices;
   acpi_evaluate_reference(dev_handle, "_DEP", NULL, &dep_devices);

   int i;
   for (i=0;i<dep_devices.count;i++) {
       struct acpi_device_info *devinfo;
       acpi_get_object_info(dep_devices.handles[i], &devinfo);

       if (devinfo->valid & ACPI_VALID_HID && !strcmp(devinfo->hardware_id.string, "INT3472")) {
            acpi_bus_get_device(dep_devices.handles[i], &int3472_device);
            dev = bus_find_device(&platform_bus_type, NULL, &int3472_device->fwnode, match_depend);
            int3472_device->dev = *dev;
       }
   }

   int ret;

   /* ret = acpi_dev_add_driver_gpios(int3472_device, int3472_acpi_gpios); */

   struct gpio_desc *gpiod1, *gpiod2, *gpiod3;

   gpiod1 = gpiod_get_index(&int3472_device->dev, NULL, 0, GPIOD_ASIS);
   gpiod2 = gpiod_get_index(&int3472_device->dev, NULL, 1, GPIOD_ASIS);
   gpiod3 = gpiod_get_index(&int3472_device->dev, NULL, 2, GPIOD_ASIS);
   
   gpiod_set_value_cansleep(gpiod1, 1);
   gpiod_set_value_cansleep(gpiod2, 1);
   gpiod_set_value_cansleep(gpiod3, 1);

   return 0;
}

static int ov2680_remove(struct i2c_client *client)
{
    /*
     * Code goes here to get acpi_device, turn off all
     * the GPIO pins, remove them from the ACPI device
     * and whatnot
     */

    return 0;
}

static const struct acpi_device_id ov2680_acpi_match[] = {
    {"OVTI2680", 0},
    { },
};

MODULE_DEVICE_TABLE(acpi, ov2680_acpi_match);

static struct i2c_driver ov2680_driver = {
    .driver = {
        .name = "ov2680-3",
        .acpi_match_table = ov2680_acpi_match,
    },
    .probe_new = ov2680_probe,
    .remove = ov2680_remove,
};

module_i2c_driver(ov2680_driver);

MODULE_AUTHOR("Dan Scally <djrscally@protonmail.com>");
MODULE_DESCRIPTION("A driver for OmniVision 2680 sensors");
MODULE_LICENSE("GPL");
