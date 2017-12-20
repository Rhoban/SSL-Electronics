<?php

/**
 * This can be use to generate (and maybe tune) the leds_nops.h file
 */

$nops = array(
    400 => 20,
    450 => 0,
    800 => 28,
    850 => 50
);

foreach ($nops as $nop => $n) {
    echo "#define NOPS_$nop \\\n";
    echo "  asm volatile(\"";
    for ($k=0; $k<$n; $k++) echo "nop;";
    echo "\");\n\n";
}
