#ifndef MATH_FUNCTIONS_H
#define MATH_FUNCTIONS_H
#include <cmath>
#include <vector>
#include <array>

class MathFunctions {
public:
    static double joystickCurve(double x, double a);
    static double negativeSqrt(double x);
    static std::vector<double> getPointed(std::vector<std::vector<double>> );
    static double getAngle(std::vector<double> origin, std::vector<double> angle, std::vector<double> target);
    static std::array<double, 3> hueToRGB(double hue);

};

#endif // MATH_FUNCTIONS_H