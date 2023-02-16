# in development

To open the PWM Jetson Nano Ports, use the following commands in the terminal every time the Nano boots up:
    $ sudo busybox devmem 0x700031fc 32 0x45
    $ sudo busybox devmem 0x6000d504 32 0x2
    $ echo 0 > /sys/class/pwm/pwmchip0/export
    $ echo 50000 > /sys/class/pwm/pwmchip0/pwm0/period
    $ echo 25000 > /sys/class/pwm/pwmchip0/pwm0/duty_cycle
    $ echo 1 > /sys/class/pwm/pwmchip0/pwm0/enable
    $ sudo busybox devmem 0x700031fc 32 0x45
    $ sudo busybox devmem 0x6000d504 32 0x2
    $ sudo busybox devmem 0x70003248 32 0x46
    $ sudo busybox devmem 0x6000d100 32 0x00
