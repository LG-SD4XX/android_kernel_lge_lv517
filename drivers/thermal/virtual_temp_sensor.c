#define pr_fmt(fmt) "%s: " fmt, __func__

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/list.h>
#include <linux/thermal.h>
#include <linux/qpnp/qpnp-adc.h>

#define VTS_NAME	"vts"

struct composite_sensor {
	const char			*name;
	int				weight;
	enum qpnp_vadc_channels		channel;
	struct list_head		list;
};

typedef struct virtual_temp_sensor {
	struct device			*dev;
	struct thermal_zone_device	*tz_vts;
	struct qpnp_vadc_chip		*vadc_dev;
	struct composite_sensor		*sensors;
	u32				scaling_factor;
	int				constant;
} VTS;

LIST_HEAD(composite_sensors_head);

static int vts_tz_get_temp(struct thermal_zone_device *thermal,
				unsigned long *temp)
{
	VTS *vts = thermal->devdata;
	struct composite_sensor *sensor;
	long val = 0, xo_val = 0, quiet_val = 0;
	char *xo_therm = "xo_therm";
	char *quiet_therm = "quiet_therm";
	list_for_each_entry(sensor, &composite_sensors_head, list) {
		struct qpnp_vadc_result results;
		int ret;
		ret = qpnp_vadc_read(vts->vadc_dev, sensor->channel, &results);
		if (ret) {
			pr_err("Fail to get adc(%d)\n", sensor->channel);
			return ret;
		}
		if(!strcmp(sensor->name, xo_therm))
			xo_val = sensor->weight * results.physical;

		if(!strcmp(sensor->name, quiet_therm))
			quiet_val = sensor->weight * results.physical;
	
		val = xo_val + quiet_val;
	}
	val += vts->constant;
	val *= vts->scaling_factor;
	val /= 1000L;

	if (val < 0 ) {
		*temp = 0;
	} else {
		*temp = (unsigned long)val;
	}

	return 0;
}

static struct thermal_zone_device_ops vts_thermal_zone_ops = {
	.get_temp = vts_tz_get_temp,
};

static int vts_probe(struct platform_device *pdev)
{
	struct device_node *of_node = pdev->dev.of_node;
	struct device_node *child;
	VTS *vts = kmalloc(sizeof(VTS), GFP_KERNEL);
	int ret = 0;
	int count = 0;
	struct composite_sensor *_sensor, *temp;

	/* Alloc chipset data on memory */
	pr_info("vts_probe start\n");
	if (!vts) {
		pr_err("Fail to get *vts.\n");
		return -ENOMEM;
	}

	/* Get devices */
	vts->dev = &pdev->dev;
	vts->vadc_dev = qpnp_get_vadc(vts->dev, "vts");
	if (IS_ERR(vts->vadc_dev)) {
		ret = PTR_ERR(vts->vadc_dev);
		if (ret != -EPROBE_DEFER)
			pr_err("Fail to get vadc.\n");
		goto fail;
	}
	vts->tz_vts = thermal_zone_device_register(
				VTS_NAME, 0, 0,	vts,
				&vts_thermal_zone_ops, NULL, 0, 0);
	if (IS_ERR(vts->tz_vts)) {
		ret = PTR_ERR(vts->tz_vts);
		if (ret != -EPROBE_DEFER)
			pr_err("Fail to get thermal_zone_device.\n");
		goto fail;
	}

	/* Get infos from device tree */
	if (of_property_read_u32(of_node, "lge,scaling-factor", &vts->scaling_factor))
		vts->scaling_factor = 1;
	if (of_property_read_u32(of_node, "lge,constant", &vts->constant))
		vts->constant = 0;

	for_each_child_of_node(of_node, child) {
		struct composite_sensor *sensor = kmalloc(
				sizeof(struct composite_sensor), GFP_KERNEL);
		if (!sensor) {
			ret = PTR_ERR(sensor);
			pr_err("Fail to malloc sensor.\n");
			goto fail;
		}

		if (of_property_read_string(child, "label", &sensor->name)) {
			kfree(sensor);
			continue;
		}
		if (of_property_read_u32(child, "channel", &sensor->channel)) {
			kfree(sensor);
			continue;
		}
		if (of_property_read_u32(child, "weight", &sensor->weight)) {
			kfree(sensor);
			continue;
		}

		if (of_property_read_bool(child,"weight-negative")) {
			sensor->weight *= -1;
		}

		pr_info("%s is registered. chan=%d, weight=%d\n",
			sensor->name, sensor->channel, sensor->weight);
		INIT_LIST_HEAD(&sensor->list);
		list_add_tail(&sensor->list, &composite_sensors_head);
		count++;
	}
	pr_info("Add %d sensors for virtual temp sensor\n", count);

	platform_set_drvdata(pdev, vts);
	pr_info("probe done\n");
	return 0;
fail:
	pr_info("Fail to register vts\n");
	list_for_each_entry_safe(_sensor, temp, &composite_sensors_head, list) {
		list_del(&_sensor->list);
		kfree(_sensor);
	}
	vts->vadc_dev = NULL;
	thermal_zone_device_unregister(vts->tz_vts);
	platform_set_drvdata(pdev, NULL);
	kfree(vts);
	return ret;
}

static int vts_remove(struct platform_device *pdev)
{
	VTS *vts = platform_get_drvdata(pdev);
	struct composite_sensor *sensor, *temp;
	thermal_zone_device_unregister(vts->tz_vts);
	list_for_each_entry_safe(sensor, temp, &composite_sensors_head, list) {
		list_del(&sensor->list);
		kfree(sensor);
	}
	platform_set_drvdata(pdev, NULL);
	kfree(vts);
	return 0;
}

static const struct of_device_id vts_match[] = {
	{ .compatible = "lge,vts", },
	{}
};

static struct platform_driver vts_driver = {
	.probe = vts_probe,
	.remove = vts_remove,
	.driver = {
		.name = "vts",
		.owner = THIS_MODULE,
		.of_match_table = vts_match,
	},
};

static int __init vts_init_driver(void)
{
	return platform_driver_register(&vts_driver);
}
late_initcall(vts_init_driver);

static void __exit vts_exit_driver(void)
{
	return platform_driver_unregister(&vts_driver);
}
module_exit(vts_exit_driver);

MODULE_DESCRIPTION("Virtual temperature sensor driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yongkwan Kim <yongk.kim@lge.com>");
