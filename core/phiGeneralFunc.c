#include "..\include\phiSystemDynamicSetting.h"

void delay(int milliseconds)
{
    long waits;
    clock_t now, after;

    waits = milliseconds * (CLOCKS_PER_SEC / 1000);
    now = after = clock();
    while ((now - after) < waits)
        now = clock();
}