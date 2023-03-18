# Autonomous Parallel Parking Drift Car

The purpose of this project was to create a drifting car that powerslides into a parallel parking spot.

https://user-images.githubusercontent.com/87098227/226077785-7c41633c-0b24-44a4-a2bb-61c67c5146b3.mp4


For my system I created a feedforward PI controller in a multi-threaded C++ program to read feedback from steer and throttle encoders. This script is in the main_script folder. The controller follows multiple different reference trajectories to slide into the parking spot. To compile and run the script, use these commands:

`$ g++ -o pid_control pid_control.cpp -lJetsonGPIO -lpthread -lserial`

`$ sudo ./pid_control`

Note that this script uses the C++ library JetsonGPIO, so it needs to be installed in order to use this.
    
I have also included other test scripts in the test_scripts folder that is used to test individual components of this car. There is also a servo_arduino_code folder for the arduino io code to command the servo.

The vehicle I used is a Traxxas Ford Fiesta, which is a 4WD RC car. Since 4WD cars are normally are more difficult to make drift than RWD cars, I put pvc pipe on the two front wheels. This helps the car slide into the spot when it turns.

![IMG-0392](https://user-images.githubusercontent.com/87098227/226077342-5ff8e98f-e626-40f4-a14c-fd39bacd15a4.jpg)

Additionally, to upgrade the RC car, I created an entirely new circuit diagram, as seen below.

![circuit_diagram](https://user-images.githubusercontent.com/87098227/226075116-6485a229-984b-4538-802c-5a034389a6a1.png)

For this system, the Jetson Nano is my main module that communicates with other modules to control the car. For the throttle, I opened the PWM ports on the Jetson Nano using direct memory access to send the signal to the VESC to control the brushless DC motor. Then, for the steer, I connected an Arduino to the Jetson Nano and used serial communication to send commands to the Arduino to control the servo for steering. Next, there are two PIC32's that are used as quadrature converters to read both the throttle and steer encoders. I also use serial communication to read the encoder data.



To open the PWM Jetson Nano Ports, use the following commands in the terminal after the Nano boots up:

    `$ sudo busybox devmem 0x700031fc 32 0x45`

    `$ sudo busybox devmem 0x6000d504 32 0x2`

    `$ echo 0 > /sys/class/pwm/pwmchip0/export`

    `$ echo 50000 > /sys/class/pwm/pwmchip0/pwm0/period`

    `$ echo 25000 > /sys/class/pwm/pwmchip0/pwm0/duty_cycle`

    `$ echo 1 > /sys/class/pwm/pwmchip0/pwm0/enable`

    `$ sudo busybox devmem 0x700031fc 32 0x45`

    `$ sudo busybox devmem 0x6000d504 32 0x2`

    `$ sudo busybox devmem 0x70003248 32 0x46`

    `$ sudo busybox devmem 0x6000d100 32 0x00`
