/*
 * PS3 RTC Driver
 *
 * Copyright 2009 Sony Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/rtc.h>

#include <asm/lv1call.h>
#include <asm/ps3.h>


static u64 read_rtc(void)
{
	int result;
	u64 rtc_val;
	u64 tb_val;

	result = lv1_get_rtc(&rtc_val, &tb_val);
	BUG_ON(result);

	return rtc_val;
}

static int ps3_get_time(struct device *dev, struct rtc_time *tm)
{
	to_tm(read_rtc() + ps3_os_area_get_rtc_diff(), tm);
	tm->tm_year -= 1900;
	tm->tm_mon -= 1;
	return 0;
}

static int ps3_set_time(struct device *dev, struct rtc_time *tm)
{
	u64 now = mktime(tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday,
			 tm->tm_hour, tm->tm_min, tm->tm_sec);
	ps3_os_area_set_rtc_diff(now - read_rtc());
	return 0;
}

static const struct rtc_class_ops ps3_rtc_ops = {
	.read_time = ps3_get_time,
	.set_time = ps3_set_time,
};

static int __devinit ps3_rtc_probe(struct platform_device *dev)
{
	struct rtc_device *rtc;

	rtc = rtc_device_register("rtc-ps3", &dev->dev, &ps3_rtc_ops,
				     THIS_MODULE);
	if (IS_ERR(rtc))
		return PTR_ERR(rtc);

	platform_set_drvdata(dev, rtc);
	return 0;
}

static int __devexit ps3_rtc_remove(struct platform_device *dev)
{
	rtc_device_unregister(platform_get_drvdata(dev));
	return 0;
}

static struct platform_driver ps3_rtc_driver = {
	.driver = {
		.name = "rtc-ps3",
		.owner = THIS_MODULE,
	},
	.probe = ps3_rtc_probe,
	.remove = __devexit_p(ps3_rtc_remove),
};

static int __init ps3_rtc_init(void)
{
	return platform_driver_register(&ps3_rtc_driver);
}

static void __exit ps3_rtc_fini(void)
{
	platform_driver_unregister(&ps3_rtc_driver);
}

module_init(ps3_rtc_init);
module_exit(ps3_rtc_fini);

MODULE_AUTHOR("Sony Corporation");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ps3 RTC driver");
MODULE_ALIAS("platform:rtc-ps3");
