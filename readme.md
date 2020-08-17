# Linux Drivers for Lenovo Miix 510 Webcams

This repo will, once complete, hold drivers for the webcams for the Lenovo Miix 510. The device has two cameras; an OmniVision 2680 as the front camera, and an OmniVision 5648 as the back camera. The drivers plug in to the v4l2 subsystem.

### Ooh! Do they work?

No, this is very much a work in progress. The 5648 driver isn't in a state worth committing yet. The 2680 driver will at least turn on the camera and read the sensor id on the chip to confirm it's working correctly, but you can't get an image yet.

### I want them precious. How do I get them?

If you really really want these ugly, barely functional work-in-progress drivers, you can do this:

```
$ git clone https://github.com/djrscally/miix-510-cameras  
$ cd miix-510-cameras  
$ make
$ sudo insmod ./ov2680.ko
```

And that should be it.

### Why was this so difficult?

There's a few problems:

1. The cameras are powered off by default behind a PMIC called a TPS68470, identified as INT3472 in the ACPI tables. The ACPI tables don't define an I2cSerialBus2 section for the PMIC, so its driver never realises that it's present.
2. The TPS68470 driver doesn't actually turn it on; there are 3 GPIO lines **into** the PMIC defined in the ACPI tables, but they're off by default and the PMIC driver doesn't toggle them. They're also not _named_ in the ACPI tables so it's not a simple matter to determine which ones are the power lines and which the reset pin. In my case, 1 and 2 were power and 0 was reset, so the chip can be turned on like so:

```
$ sudo gpioset gpiochip0 122=1 143=1
```

Before that step, neither the PMIC nor the camera's i2c interface is even enabled, meaning they don't show anywhere at all. Once the device is on, you should be able to see it using i2cdetect:

```
$ sudo i2cdetect -r -y 7
```

this will return something like:

```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- 0c -- -- -- 
10: 10 -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
40: -- -- -- -- -- -- -- -- 48 -- -- -- -- -- -- -- 
50: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
60: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
70: -- -- -- -- -- -- -- -- 
```

The OV2680 is at 0x10. The TPS68470 is the device at 0x48. I have no idea what is at 0x0c; that's one of the two possible addresses defined for the oc5648, but powering that chip's PMIC on reveals it to be at 0x36. Who cares, really. Anyway, now that the camera is turned on you can verify that this isn't bullshit by talking to the sensor and asking for its id. The OV2680 datasheet tells us that its id sits at register 0x300a:

```
$ sudo i2ctransfer 7 w2@0x10 0x30 0x0a r2
WARNING! This program can confuse your I2C bus, cause data loss and worse!
I will send the following messages to device file /dev/i2c-7:
msg 0: addr 0x10, write, len 2, buf 0x30 0x0a
msg 1: addr 0x10, read, len 2
Continue? [y/N] y
0x26 0x80
```

That command basically says "write two bytes (being 0x30 and 0x0a) to bus 7 addr 0x10, then read two bytes". The two bytes we write are the address of the register we're interested in, and we learn that the value stored there is 0x26 and 0x80, I.E. 2680! We detected the chip, hooray. Putting that into the drivers is a bit more difficult. It doesn't really make sense to turn the GPIO pins for the PMIC on in this driver; that ought really to go into that modules driver.