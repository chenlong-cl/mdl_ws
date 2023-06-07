#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;

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
	{ 0xd31aec96, "module_layout" },
	{ 0x8e17b3ae, "idr_destroy" },
	{ 0xbf2d5410, "usb_deregister" },
	{ 0x70e992dd, "put_tty_driver" },
	{ 0x53dd569e, "tty_unregister_driver" },
	{ 0x571fd711, "usb_register_driver" },
	{ 0x445d3fc5, "tty_register_driver" },
	{ 0x66198db2, "tty_set_operations" },
	{ 0x67b27ec1, "tty_std_termios" },
	{ 0x727347c6, "__tty_alloc_driver" },
	{ 0x79aa04a2, "get_random_bytes" },
	{ 0x409873e3, "tty_termios_baud_rate" },
	{ 0x6c257ac0, "tty_termios_hw_change" },
	{ 0x6cbbfc54, "__arch_copy_to_user" },
	{ 0x88db9f48, "__check_object_size" },
	{ 0x8b0e420d, "tty_port_close" },
	{ 0xdcb764ad, "memset" },
	{ 0x24873f5a, "usb_register_dev" },
	{ 0x7d21d7f0, "tty_port_register_device" },
	{ 0xb84d5469, "usb_get_intf" },
	{ 0xb2ec816, "usb_driver_claim_interface" },
	{ 0xec3f4348, "usb_control_msg" },
	{ 0xa5afb83e, "usb_autopm_get_interface" },
	{ 0xad9480d6, "tty_port_init" },
	{ 0x655fc02e, "usb_alloc_urb" },
	{ 0xa8423b46, "usb_alloc_coherent" },
	{ 0x977f511b, "__mutex_init" },
	{ 0xd9a5ea54, "__init_waitqueue_head" },
	{ 0xb8f11603, "idr_alloc" },
	{ 0xb8b9f817, "kmalloc_order_trace" },
	{ 0xcfd01ed8, "usb_ifnum_to_if" },
	{ 0xe47ae5ff, "tty_standard_install" },
	{ 0x296695f, "refcount_warn_saturate" },
	{ 0x20978fb9, "idr_find" },
	{ 0x4b0a3f52, "gic_nonsecure_priorities" },
	{ 0x49f2fa43, "cpu_hwcaps" },
	{ 0xf424b0fe, "cpu_hwcap_keys" },
	{ 0x14b89635, "arm64_const_caps_ready" },
	{ 0xeb233a45, "__kmalloc" },
	{ 0x41915aed, "tty_port_open" },
	{ 0x92540fbf, "finish_wait" },
	{ 0x8ddd8aad, "schedule_timeout" },
	{ 0x8c26d495, "prepare_to_wait_event" },
	{ 0xfe487975, "init_wait_entry" },
	{ 0x826f1e2d, "usb_anchor_urb" },
	{ 0xb3dafda0, "usb_autopm_get_interface_async" },
	{ 0xd395aa26, "tty_port_hangup" },
	{ 0xf657826e, "usb_put_intf" },
	{ 0x7665a95b, "idr_remove" },
	{ 0x7cf9d2b5, "tty_port_tty_wakeup" },
	{ 0xcbd4898c, "fortify_panic" },
	{ 0x56c80a72, "tty_flip_buffer_push" },
	{ 0x73c29162, "tty_insert_flip_string_fixed_flag" },
	{ 0x4829a47e, "memcpy" },
	{ 0xc5b6f236, "queue_work_on" },
	{ 0x2d3385d3, "system_wq" },
	{ 0x58556d52, "tty_port_tty_hangup" },
	{ 0xb5b54b34, "_raw_spin_unlock" },
	{ 0xba8fbd64, "_raw_spin_lock" },
	{ 0x6ebe366f, "ktime_get_mono_fast_ns" },
	{ 0xc5850110, "printk" },
	{ 0x59af06cf, "usb_find_interface" },
	{ 0xae0dc4e9, "_dev_info" },
	{ 0xb2c1ec19, "usb_driver_release_interface" },
	{ 0xcb855872, "usb_free_urb" },
	{ 0x1725508d, "tty_unregister_device" },
	{ 0xf93851fd, "tty_kref_put" },
	{ 0xad4c7916, "tty_vhangup" },
	{ 0xe4810414, "tty_port_tty_get" },
	{ 0x409bcb62, "mutex_unlock" },
	{ 0x3eeb2322, "__wake_up" },
	{ 0x2ab7989d, "mutex_lock" },
	{ 0x365356c1, "usb_deregister_dev" },
	{ 0x3bb4710, "tty_port_put" },
	{ 0xb6cf5754, "usb_free_coherent" },
	{ 0x4b750f53, "_raw_spin_unlock_irq" },
	{ 0x8427cc7b, "_raw_spin_lock_irq" },
	{ 0x3c12dfe, "cancel_work_sync" },
	{ 0x759ea8c3, "usb_kill_urb" },
	{ 0x85742949, "usb_get_from_anchor" },
	{ 0x4b15198, "usb_autopm_put_interface" },
	{ 0xd35550fe, "usb_autopm_get_interface_no_resume" },
	{ 0x3127bd06, "usb_autopm_put_interface_async" },
	{ 0x1ca7fb0d, "usb_submit_urb" },
	{ 0x37a0cba, "kfree" },
	{ 0x9ac23f86, "kmem_cache_alloc_trace" },
	{ 0x52d1fb52, "kmalloc_caches" },
	{ 0x59982873, "_dev_err" },
	{ 0x86332725, "__stack_chk_fail" },
	{ 0xc6d17a7c, "usb_bulk_msg" },
	{ 0xd35cce70, "_raw_spin_unlock_irqrestore" },
	{ 0x34db050b, "_raw_spin_lock_irqsave" },
	{ 0x1fdc7df2, "_mcount" },
};

MODULE_INFO(depends, "");

MODULE_ALIAS("usb:v1A86pE018d*dc*dsc*dp*ic*isc*ip*in*");
MODULE_ALIAS("usb:v1A86p55D9d*dc*dsc*dp*ic*isc*ip*in*");
