#include "mlinterp.hpp"
#include <cmath>
#include <iostream>
using namespace std;

int main()
{
    using namespace mlinterp;
    double x[2] = {-1.0, 1.0};
    double y[2] = {-1.0, 1.0};
    double z[2] = {-1.0, 1.0};
    int dim[] = {2, 2, 2};
    double value[27];
    for (int ix = 0; ix < 2; ix++)
    {
        for (int iy = 0; iy < 2; iy++)
        {
            for (int iz = 0; iz < 2; iz++)
            {
                int n = iz + 2 * iy + 2 * 2 * ix;
                value[n] = x[ix] + 2.0 * y[iy] + 3.0 * z[iz];
            }
        }
    }
    double x_interp[1] = {0.1};
    double y_interp[1] = {0.2};
    double z_interp[1] = {0.3};
    double value_interp[1];
    interp(
        dim, 1,                               // Number of points
        value, value_interp,                  // Output axis (z)
        x, x_interp, y, y_interp, z, z_interp // Input axes (x and y)
    );
    std::cout << value_interp[0] << std::endl;
    std::cout << floor(-1.2) << std::endl;
    std::cout << floor(1.2) << std::endl;
}