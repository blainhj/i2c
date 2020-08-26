#include <dm.h>
	
void board_init()
{
	struct udevice *i2c = NULL;
	int err;

	/*active gpio i2c */
	err = uclass_get_device_by_name(UCLASS_I2C,
			"i2c-gpio-0", &i2c);
	if (err) {
		pr_err("Can't find i2c-gpio-0 (%d)\n", err);
		return err;
	}
}
