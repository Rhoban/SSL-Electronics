# SSL Boards

## Compiling firmware

Be sure you follow the README.md from https://github.com/Rhoban/Maple/ to have the prerequisite.

You can clone the `Maple` repository, for instance in your home:

    git clone git@github.com:Rhoban/Maple.git

Then, go in the firmware directory (for instance in brushless or in mainboard) and make a symlink to it:

    ln -s $HOME/Maple Maple

You can then compile and install:

    make
    make install

-------------------------------------------------

## Components

### Main board

### Connectors

* JST-ZH 2 pin connector + cables
* JST-XH 6 pins connector for kicker board
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

### Connectors

* JST-XH 6 pins connector

#### Voltage booster

Can be easily found with keywords "390V booster"

### Screws

M3 spacers to fix the board

-------------------------------------------------

## Misc

#### Fuses

F447 fuses on the wire are used to protect
