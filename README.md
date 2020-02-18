# SSL Boards

## Compiling firmware

Be sure you follow the README.md from https://github.com/Rhoban/Maple/ to have the prerequisite.

You can clone the `Maple` repository, for instance in your home:

    git clone git@github.com:Rhoban/Maple.git

Then, go in the firmware directory (for instance in brushless or in mainboard) and make a symlink to it:

    ln -s $HOME/Maple Maple

You can then compile and install:

for master board:
    export COMPILE_MASTER=yes
    export BOOTLOADER_PORT=/dev/ttyACM0
    make clean
    make install

for robot board:
    export COMPILE_MASTER=no
    export BOOTLOADER_PORT=/dev/ttyACM0
    make clean
    make install


-------------------------------------------------

## Components

You can find most of the component source in the `FARNELL` attribute of each component.

### Main board

### Connectors

* JST-ZH 2 pin connector + cables
* JST-XH 7 pins connector for kicker board
* XT-60 connector (same as wire connector)

#### Maple mini

Board containing micro-controller (mainboard controller)

#### GY-85

9DOF IMU

#### nRF24L01 modules

2.4Ghz communication modules with short antenna (Farnell: 2143316)

### Screws

* M3 holes are on the board, on a 126x82mm rectangle.
* M3x10 spacers can be used to hold the drivers on the board

-------------------------------------------------

## Brushless driver

### Connectors

* JST-ZH 5 pin connector (for hall sensor)
* JST-ZH 6 pin connector (for encoder)
* JST-XH 3 pin connector (for motor phases)
  
-------------------------------------------------

## Kicker board

### Power capacitor

Power capacitors, current target is 2200uF / 200V (2102441)

### Flash brushless driver
first, add a udev rules in /etc/udev/rules.d/10-maple.rules 

ATTRS{idProduct}=="0003", ATTRS{idVendor}=="1eaf", MODE="0666", GROUP="plugdev" SYMLINK+="maple"

in brushless/firmware directory
   make
Then, with robot off and usb cable connected to driver (connector under the robot), enter the following command
just after plug-in:

dfu-util -a1 -d 1eaf:0003 -D build/maple.bin  -d /dev/maple

This is because driver remains in dfu only a short time after power up. If it fails, try again


### Connectors

* JST-XH 7 pins connector

#### Voltage booster

Can be easily found with keywords "390V booster"

### Screws

M3 spacers to fix the board

-------------------------------------------------

## Misc

#### Fuses

F447 fuses on the wire are used to protect
