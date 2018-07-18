#include <stdio.h>
#include <stdlib.h>
#include "sin_lut.h"

int main()
{
    for (int k=0; k<8095; k++) {
        printf("%d\n", sin_lut(k));
    }
}