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

