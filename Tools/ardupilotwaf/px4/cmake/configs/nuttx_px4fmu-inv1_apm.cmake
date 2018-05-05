include(configs/nuttx_px4fmu-common_apm)

list(APPEND config_module_list
    drivers/boards/px4fmu-inv1
    drivers/pwm_input
    lib/rc
)
