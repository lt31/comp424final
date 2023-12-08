#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/kernel.h>
#include <linux/gpio/consumer.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/ktime.h>
#include <linux/timekeeping.h>
#include <linux/fs.h>
#include <linux/uaccess.h>



/* Declaration of variables */
static struct gpio_desc *led_gpio, *button_gpio;
static int button_irq;
static int elapsed_ms = 0;
ktime_t start_time, old_time, elapsed;
module_param(elapsed_ms, int, S_IRUGO);

void onButtonPress(void) {
    ktime_t new_time = ktime_get();
    elapsed = ktime_sub(new_time, old_time);
    old_time = new_time;
    elapsed_ms = ktime_to_ns(elapsed);

    printk("Elapsed: %i\n", elapsed_ms);
}

/* Interrupt Service Routine */
static irq_handler_t button_isr(unsigned int irq, void *dev_id, struct pt_regs *regs)
{
        // Get button GPIO and swap
        int led_val = gpiod_get_value(led_gpio);
        // print to log
        //printk(KERN_INFO "Button pressed! LED toggled to %s.\n", led_val);
        onButtonPress();
        return (irq_handler_t)IRQ_HANDLED;
}
static int led_probe(struct platform_device *pdev)
{
        int retval;
        // Get the LED GPIO
        led_gpio = devm_gpiod_get(&pdev->dev, "led", GPIOD_OUT_LOW);
        // Get the Button GPIO
        button_gpio = devm_gpiod_get(&pdev->dev, "button", GPIOD_IN);
        // Set debounce for the button
        gpiod_set_debounce(button_gpio, 200000); // Debounce of 20ms
        // Request interrupt for button
        button_irq = gpiod_to_irq(button_gpio);
        retval = request_irq(button_irq, (irq_handler_t)button_isr,
                             IRQF_TRIGGER_FALLING, "button_irq_handler", NULL);
        printk(KERN_INFO "LED driver probe successful.\n");
        return 0;
}
// remove function
static int led_remove(struct platform_device *pdev)
{
        // free irq, print, then return
        free_irq(button_irq, NULL);
        printk(KERN_INFO "LED driver removed.\n");
        return 0;
}
static struct of_device_id matchy_match[] = {
    {.compatible = "leddriv"},
    {},
};
// platform driver object
static struct platform_driver adam_driver = {
    // set these equal to what we defined earlier
    .probe = led_probe,
    .remove = led_remove,
    .driver = {
        .name = "The Rock: this name doesn't even matter",
        .owner = THIS_MODULE,
        .of_match_table = matchy_match,
    },
};
module_platform_driver(adam_driver);
MODULE_DESCRIPTION("424's finest");
MODULE_AUTHOR("GOAT");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:adam_driver");
