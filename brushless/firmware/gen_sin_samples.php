<?php

$samples = [];
$max_value = 16384;

for ($k = 0; $k < 2048; $k++) {
    $x = ($k * pi()) / (2 * 2048.0);
    $samples[] = round((sin($x)+1)/2.0 * $max_value);
}

?>
#define MAX_VALUE <?php echo $max_value; ?>

static uint16_t samples[2048] = {
    <?php echo implode(', ', $samples); ?>
};
