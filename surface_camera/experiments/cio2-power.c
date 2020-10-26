#include <linux/acpi.h>
#include <linux/kernel.h>
#include <linux/module.h>

#define CRS_END_TAG_DESCRIPTOR		0x79
#define CRS_RESOURCE_SIZE_BITMASK	0x80
#define CRS_L_RESOURCE_TYPE_BITMASK	0x7f

#define CRS_GPIO_CONN_TYPE_BYTE		0x04
#define CRS_GPIO_CONSUMER_FLAG_BYTE	0x05
#define CRS_GPIO_INT_IO_FLAGS_BYTE	0x07

#define CRS_GPIO_PIN_TBL_OFFSET_LOW	0x0e
#define CRS_GPIO_PIN_TBL_OFFSET_HIGH	0x0f

#define CRS_GPIO_VEND_DATA_OFFSET_LOW	0x13
#define CRS_GPIO_VEND_DATA_OFFSET_HIGH	0x14

#define CRS_GPIO_RES_NAME_OFFSET_LOW	0x11
#define CRS_GPIO_RES_NAME_OFFSET_HIGH	0x12

static int cio2_power_parse_gpio(u8 * buffer, u16 len)
{
	u16 vendor_data_offset;
	u16 resource_source_name_offset;
	u16 resource_source_name_length;
	u16 i;
	char *resource_source_name;

	// GPIO Connection Type
	if (*(buffer + CRS_GPIO_CONN_TYPE_BYTE))
		pr_info("    GPIO Is an IO Connection\n");
	else
		pr_info("    GPIO is an Interrupt Connection\n");

	if (*(buffer + CRS_GPIO_CONSUMER_FLAG_BYTE))
		pr_info("    Device consumes this GPIO pin\n");
	else
		pr_info("    Device produces this GPIO pin\n");

	vendor_data_offset = *(buffer + CRS_GPIO_VEND_DATA_OFFSET_LOW);
	resource_source_name_offset = *(buffer + CRS_GPIO_RES_NAME_OFFSET_LOW);

	resource_source_name_length = (vendor_data_offset - resource_source_name_offset);

	pr_info("    resource source name length: %d\n", resource_source_name_length);

	resource_source_name = kmalloc(resource_source_name_length, GFP_KERNEL);
	if (!resource_source_name)
		return -ENOMEM;

	printk(KERN_INFO "Exact name: ");
	for (i = 0; i < resource_source_name_length; i++) {
		printk(KERN_INFO "%c", *(buffer + resource_source_name_offset + i));
		resource_source_name[i] = *(buffer + resource_source_name_offset + i);
	}
	printk(KERN_INFO "\n");

	pr_info("    resource source name: %s\n", resource_source_name);
	pr_info("    resource source name: %s\n", "\\_SB.PCI0.GPI0");

	acpi_handle handle;
	acpi_status status;

	char testvar[] = "\\_SB.PCI0.GPI0";

	if (!strcmp(resource_source_name, "\\_SB.PCI0.GPI0"))
		pr_info("RSN matches manual\n");

	if (!strcmp(testvar, "\\_SB.PCI0.GPI0"))
		pr_info("testvar matches manual\n");

	if (!strcmp(resource_source_name, testvar))
		pr_info("Both variables match too\n");

	for (i = 0; i < 15; i++)
		pr_info("%c %s %c at pos %d\n", resource_source_name[i],
			resource_source_name[i] == testvar[i] ? "==" : "!=",
			testvar[i], i);

	pr_info("    resource source name: %s\n", resource_source_name);
	pr_info("    resource source name: %s\n", testvar);

	status = acpi_get_handle(NULL, resource_source_name, &handle);
	if (ACPI_FAILURE(status)) {
		pr_info("    Failed to evaluate Resource Source path. Error Code: %d\n", status);
	} else {
		pr_info("    Successfully evaluated Resource Source Path.\n");

		struct acpi_device *gpio_adev;
		int ret;

		ret = acpi_bus_get_device(handle, &gpio_adev);

		if (!ret)
			pr_info("    Found ACPI Device: %s\n", dev_name(&gpio_adev->dev));
	}

	kfree(resource_source_name);
	return 0;
	
}

static int cio2_power_parse_crs(union acpi_object *obj)
{
	u8 resource_type = 0x00;
	u8 tag_byte = 0x00;
	u16 len;
	u64 offset = 0;
	
	do {
		tag_byte = *(obj->buffer.pointer + offset);

		// get size:
		if (tag_byte & CRS_RESOURCE_SIZE_BITMASK) {
			pr_info("Resource at offset 0x%02x is a Large Resource Data Type\n",
				offset);

			len = *(obj->buffer.pointer + offset + 1);

			pr_info("Resource is %d bytes long\n", len);

			resource_type = tag_byte & CRS_L_RESOURCE_TYPE_BITMASK;

			switch(resource_type) {
			case 0x00:
				pr_info("Resource type: reserved\n");
				break;
			case 0x01:
				pr_info("Resource type: 24 bit thing\n");
				break;
			case 0x02:
				pr_info("Resource type: generic Connection Descriptor\n");
				break;
			case 0x0c:
				pr_info("Resource type: GPIO Description\n");
				cio2_power_parse_gpio(obj->buffer.pointer + offset, len);
				break;
			default:
				pr_info("Fuck only knows\n");
			}

		}

		offset = offset + len + 0x03;
		// get type:

		// get length:
	} while (tag_byte != CRS_END_TAG_DESCRIPTOR);

	return 0;
}

static int cio2_power_init(void)
{
	struct acpi_buffer buffer = { ACPI_ALLOCATE_BUFFER, NULL };
	struct acpi_device *adev;
	union acpi_object *obj;
	acpi_handle handle;
	acpi_status status;
	int ret;

	adev = acpi_dev_get_first_match_dev("INT3472", "0", -1);
	if (!adev) {
		pr_err("No PMIC acpi_device found\n");
		return -ENODEV;
	}

	handle = adev->handle;

	status = acpi_evaluate_object(handle, "_CRS", NULL, &buffer);
	if (ACPI_FAILURE(status)) {
		ret = -ENODEV;
		goto err_free_pmic_adev;
	}

	obj = buffer.pointer;

	if (!obj) {
		ret = -ENODEV;
		goto err_free_pmic_adev;
	}

	if (obj->type != ACPI_TYPE_BUFFER) {
		ret = -EINVAL;
		goto err_free_pmic_adev;
	}

	pr_info("_CRS buffer length: %d\n", obj->buffer.length);
	print_hex_dump(KERN_INFO, "", DUMP_PREFIX_OFFSET, 16, 1, obj->buffer.pointer, obj->buffer.length, true);

	ret = cio2_power_parse_crs(obj);

	kfree(buffer.pointer);
	acpi_dev_put(adev);

	return 0;

err_free_pmic_adev:
	acpi_dev_put(adev);

	return ret;
}

static void cio2_power_exit(void) { }

module_init(cio2_power_init);
module_exit(cio2_power_exit);

MODULE_AUTHOR("Dan Scally <djrscally@gmail.com>");
MODULE_DESCRIPTION("Experimental module to map GPIO regulators and reset pins to ov2680 for a Lenovo Miix 510");
MODULE_LICENSE("GPL v2");