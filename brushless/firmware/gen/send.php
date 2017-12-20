<?php

/**
 * This can be used to generate leds_sends.h
 */

$leds = ['led1', 'led2', 'led3'];

foreach ($leds as $led) {
    for ($k=23; $k>=0; $k--) {
        echo "send_bit((led1>>$k)&1);\n";
    }
    echo "\n";
    for ($k=23; $k>=0; $k--) {
        echo "send_bit((led1b>>$k)&1);\n";
    }
    echo "\n";
}
