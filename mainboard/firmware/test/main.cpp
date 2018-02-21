#include <stdio.h>
#include "../kinematic.h"

int main()
{
    float fl, fr, bl, br;
    kinematic_compute(1, 1, 1, &fl, &fr, &bl, &br);

    printf("Front left: %f\n", fl);
    printf("Front right: %f\n", fr);
    printf("Back left: %f\n", bl);
    printf("Back right: %f\n", br);
}
