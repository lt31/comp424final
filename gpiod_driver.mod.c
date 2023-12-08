#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/export-internal.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif


static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0xb1ad28e0, "__gnu_mcount_nc" },
	{ 0xca5e70df, "__platform_driver_register" },
	{ 0xefd6cf06, "__aeabi_unwind_cpp_pr0" },
	{ 0xc1514a3b, "free_irq" },
	{ 0x92997ed8, "_printk" },
	{ 0x7d4468f1, "devm_gpiod_get" },
	{ 0xf3698ea7, "gpiod_set_debounce" },
	{ 0x5ecd9ee5, "gpiod_to_irq" },
	{ 0x92d5838e, "request_threaded_irq" },
	{ 0x65d23d93, "platform_driver_unregister" },
	{ 0xb388353c, "gpiod_get_value" },
	{ 0xb43f9365, "ktime_get" },
	{ 0x11e7998f, "param_ops_int" },
	{ 0x78a319e7, "module_layout" },
};

MODULE_INFO(depends, "");


MODULE_INFO(srcversion, "7366DCDB011696C38E91A0C");
