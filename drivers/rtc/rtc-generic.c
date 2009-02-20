/* rtc-generic: RTC driver using the generic RTC abstraction
 *
 * Copyright (C) 2008 Kyle McMartin <kyle@mcmartin.ca>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/time.h>
#include <linux/platform_device.h>

#include <asm/rtc.h>

/* as simple as can be, and no simpler. */
struct generic_rtc {
	struct rtc_device *rtc;
	spinlock_t lock;
};

static int generic_get_time(struct device *dev, struct rtc_time *tm)
{
	struct generic_rtc *p = dev_get_drvdata(dev);
	unsigned long flags, ret;

	spin_lock_irqsave(&p->lock, flags);
	ret = get_rtc_time(tm);
	spin_unlock_irqrestore(&p->lock, flags);

	if (ret & RTC_BATT_BAD)
		return -EOPNOTSUPP;

	return 0;
}

static int generic_set_time(struct device *dev, struct rtc_time *tm)
{
	struct generic_rtc *p = dev_get_drvdata(dev);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&p->lock, flags);
	ret = set_rtc_time(tm);
	spin_unlock_irqrestore(&p->lock, flags);

	if (ret < 0)
		return -EOPNOTSUPP;

	return 0;
}

static const struct rtc_class_ops generic_rtc_ops = {
	.read_time = generic_get_time,
	.set_time = generic_set_time,
};

static int __devinit generic_rtc_probe(struct platform_device *dev)
{
	struct generic_rtc *p;

	p = kzalloc(sizeof (*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;

	spin_lock_init(&p->lock);

	p->rtc = rtc_device_register("rtc-generic", &dev->dev,
				     &generic_rtc_ops, THIS_MODULE);
	if (IS_ERR(p->rtc)) {
		int err = PTR_ERR(p->rtc);
		kfree(p);
		return err;
	}

	platform_set_drvdata(dev, p);

	return 0;
}

static int __devexit generic_rtc_remove(struct platform_device *dev)
{
	struct generic_rtc *p = platform_get_drvdata(dev);

	rtc_device_unregister(p->rtc);
	kfree(p);

	return 0;
}

static struct platform_driver generic_rtc_driver = {
	.driver = {
		.name = "rtc-generic",
		.owner = THIS_MODULE,
	},
	.probe = generic_rtc_probe,
	.remove = __devexit_p(generic_rtc_remove),
};

static int __init generic_rtc_init(void)
{
	return platform_driver_register(&generic_rtc_driver);
}

static void __exit generic_rtc_fini(void)
{
	platform_driver_unregister(&generic_rtc_driver);
}

module_init(generic_rtc_init);
module_exit(generic_rtc_fini);

MODULE_AUTHOR("Kyle McMartin <kyle@mcmartin.ca>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Generic RTC driver");
MODULE_ALIAS("platform:rtc-generic");
