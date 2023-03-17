import serial

with serial.Serial('/dev/ttyACM0', 9600, timeout=10) as ser:
    while True:
        led_on = input("Send a servo value: ")
        # pos = ""
        # for x in servo_pos:
        #     pos += x
        # print(servo_pos)
        # ser.write(bytes(servo_pos, 'utf-8'))
        if led_on in 'yY':
            ser.write(bytes('YES\n', 'utf-8'))
        if led_on in 'nN':
            ser.write(bytes('NO\n', 'utf-8'))