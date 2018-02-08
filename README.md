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


## Components

### Maple mini

Board containing micro-controller (mainboard controller)

### GY-85

9DOF IMU

### nRF24L01 modules

2.4Ghz communication modules

### Voltage booster

Can be easily found with keywords "390V booster"

### Fuses

F447
