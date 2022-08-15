#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "bmp581.h"


LOG_MODULE_DECLARE(BMP581, CONFIG_SENSOR_LOG_LEVEL);

int bmp581_trigger_set(const struct device *dev,
		       const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler)
{
    int ret = -ENOTSUP;
    struct bmp581_data* drv = (struct bmp581_data *) dev->data;
    uint8_t int_source = 0;

    switch(trig->type)
    {
        case SENSOR_TRIG_DATA_READY:
            int_source = BMP5_SET_BITS_POS_0(int_source, BMP5_INT_DRDY_EN, BMP5_ENABLE);
            ret = reg_write(BMP5_REG_INT_SOURCE, &int_source, 1, drv);
            drv->drdy_handler = handler;
            break;
        default:
            break;
    }

	return ret;
}


static void bmp581_handle_int(struct bmp581_data* drv)
{
    int ret = 0;
    uint8_t int_status = 0;

    ret = reg_read(BMP5_REG_INT_STATUS, &int_status, 1, drv);
    if (ret != BMP5_OK)
    {
        LOG_DBG("Failed to read interrupt status.");
        return;
    }

    if ( (int_status & BMP5_INT_ASSERTED_DRDY) && drv->drdy_handler)
    {
        struct sensor_trigger drdy_trig = {
            .type = SENSOR_TRIG_DATA_READY,
            .chan = SENSOR_CHAN_PRESS,
        };
        drv->drdy_handler(drv->dev, &drdy_trig);
    }
}

static void bmp581_work_cb(struct k_work *work)
{
	struct bmp581_data *drv = CONTAINER_OF(work, struct bmp581_data, work);
	bmp581_handle_int(drv);
}

static void bmg160_gpio_callback(const struct device *port,
				 struct gpio_callback *cb,
				 uint32_t pin)
{
	struct bmp581_data *drv = CONTAINER_OF(cb, struct bmp581_data, gpio_cb);

	ARG_UNUSED(port);
	ARG_UNUSED(pin);

	k_work_submit(&drv->work);
}

static inline int setup_int(const struct device *dev,
			      bool enable)
{
	struct bmp581_data *data = dev->data;
	const struct bmp581_config *const cfg = dev->config;

	return gpio_pin_interrupt_configure(data->gpio,
					    cfg->input.pin,
					    enable
					    ? GPIO_INT_EDGE_TO_ACTIVE
					    : GPIO_INT_DISABLE);
}

int bmp581_trigger_init(const struct device *dev)
{
    const struct bmp581_config* cfg = dev->config;
    struct bmp581_data* drv = dev->data;
    int ret = BMP5_OK;

    uint8_t int_source = 0;
    uint8_t int_status = 0;
    uint8_t int_config = 0;

    ret = reg_read(BMP5_REG_INT_CONFIG, &int_config, 1, drv);
    if (ret == BMP5_OK)
    {
        /* Any change between latched/pulsed mode has to be applied while interrupt is disabled */
        /* Step 1 : Turn off all INT sources (INT_SOURCE -> 0x00) */
        ret = reg_write(BMP5_REG_INT_SOURCE, &int_source, 1, drv);
        if (ret == BMP5_OK)
        {
            /* Step 2 : Read the INT_STATUS register to clear the status */
            ret = reg_read(BMP5_REG_INT_STATUS, &int_status, 1, drv);

            if (ret == BMP5_OK)
            {
                /* Step 3 : Set the desired mode in INT_CONFIG.int_mode */
                int_config = BMP5_SET_BITS_POS_0(int_config, BMP5_INT_MODE, BMP5_INT_MODE_LATCHED);
                int_config = BMP5_SET_BITSLICE(int_config, BMP5_INT_POL, BMP5_INT_POL_ACTIVE_HIGH);
                int_config = BMP5_SET_BITSLICE(int_config, BMP5_INT_OD, BMP5_INT_OD_PUSHPULL);
                int_config = BMP5_SET_BITSLICE(int_config, BMP5_INT_EN, BMP5_ENABLE);

                /* Finally transfer the interrupt configurations */
                ret = reg_write(BMP5_REG_INT_CONFIG, &int_config, 1, drv);

            }
        }
    }

    if (ret != BMP5_OK)
    {
        LOG_DBG("Unable to configure interrupts for BMP5xx.");
		return ret;
    }

    drv->gpio = device_get_binding((char *)cfg->input.port);
	if (!drv->gpio) {
		LOG_DBG("Gpio controller %s not found", cfg->input.port);
		return -EINVAL;
	}

    drv->dev = dev;
    drv->work.handler = bmp581_work_cb;

    ret = gpio_pin_configure(drv->gpio, cfg->input.pin, cfg->input.dt_flags | GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		return ret;
	}

    gpio_init_callback(&drv->gpio_cb, bmg160_gpio_callback, BIT(cfg->input.pin));

    ret = gpio_add_callback(drv->gpio, &drv->gpio_cb);
	if (ret < 0) {
		return ret;
	}

    return setup_int(dev, true);
}
