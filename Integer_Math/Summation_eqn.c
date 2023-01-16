/*100 sum x=0 (15x + 30 sin(10xy) + cuberoot(xy) + y + 1)

 */

#include <stdio.h>
#include <math.h>

int x;
const int y = 63;
double sum;

int main()
{
    for (x = 0; x <= 100; x++)
    {
        sum = sum + (15 * x + 30 * sin(10 * x * y) + cbrt(x * y) + y + 1);
        // sum = sum + (x + 63);
    }
    printf("%f", sum);
    return 0;
}