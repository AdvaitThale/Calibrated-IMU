#include <stdio.h>

int main()
{
    float sqroot(float num)
    {
        long i;
        float x, y;
        const float r = 1.5F;
        x = num * 0.5F;
        y = num;
        i = *(long *)&y;
        i = 0x5f3659df - (i >> 1);
        y = *(float *)&i;
        y = y * (r - (x * y * y)); // Newton Iteration(1st)
        printf("%f", y);
        return y;
    }
}