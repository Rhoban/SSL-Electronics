# Maple Bootloader for Robotis boards

This is an adaptation of the maple bootloader in order to make it work with the
Robotis CM900 and OpenCM9.04 boards:

* [Robotis CM900](http://support.robotis.com/en/product/auxdevice/controller/cm-900_manual.htm)
* [Robotis OpenCM9.04](http://support.robotis.com/en/product/auxdevice/controller/opencm9.04.htm)

## How to install it?

### The easy way

#### OpenCM9.04

The easy way is to use a simple sketch with the robotis OpenCM Framework to do it. Simply run
OpenCM IDE and open the `opencm904_maple_loader` sketch that is in the `sketch/` directory of
this repository.

**If you load that sketch, it will rewrite your bootloader to the OpenCM9.04 bootloader;
OpenCM IDE won't work anymore!**

Wait until your board led blinks, and your bootloader should be changed. You can press the reset
button and you should see the "bootloader blink".

### The standard way

#### Step 1. Build or get the right bootloader

You can either build it or get it (from the `snapshot` directory). See below.

#### Step 2. Wire a serial line to your board

This step may require some soldering or electronic hacking, you'll have to wire up a serial 3.3V 
adapter. You can for instance use a [3.3V FTDI cable](http://www.ftdichip.com/Products/Cables/USBTTLSerial.htm)
or a [3.3V FTDI Breakout](https://www.sparkfun.com/products/retired/8772). 5V logic should also
work with this.

Then, wire your serial adapter to the Serial1 (TX1 and RX1) pins of your board.

#### Step 3. Put your board in hardware bootloader mode

Press the RESET button of your board, and then put the BOOT0 signal to 3.3V (this can be done
with an extra wire), release the RESET button, and release the BOOT0.

#### Step 4. Flash the bootloader!

You can now flash the bootloader:

```
# You may have to adapt the serial port, this is an example:
python stm32loader.py -p /dev/ttyUSB -evw bootloader.bin
```

Tada! It should then be OK.

### What's next?

Next, you can program your board using `dfu-util`. This is done by default in the libmaple
Makefile.

You can now use `cm900` and `opencm904` variants in the LibMaple, or use the RobotsWar 
framework
https://github.com/RobotsWar/RobotsWar

## Boards

### CM900

![CM900](imgs/cm900.jpg)

The `master` branch of this repository is supporting CM900 board, you can find a
snapshot of the bootloader in `snapshot/maple-boot-cm900.bin`.

### OpenCM9.04

![OpenCM9.04](imgs/opencm9.04.jpg)

Ths `opencm904` branch of this repository is supporting the OpenCM9.04 board. You can
find a snapshot of this bootloader in `snapshot/maple-boot-opencm904.bin`.
