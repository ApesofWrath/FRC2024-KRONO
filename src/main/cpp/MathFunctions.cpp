#include "MathFunctions.h"

double MathFunctions::joystickCurve(double x, double a) {
    bool b = std::signbit(x);
    double sign = b ? -1.0 : 1.0;
    double abs_x = std::abs(x);
    return (abs_x / (1 + a * (1 - abs_x)))*sign;
}

double MathFunctions::negativeSqrt(double x) {
    bool b = std::signbit(x);
    double sign = b ? -1.0 : 1.0;
    double abs_x = std::abs(x);
    return (sqrt(abs_x))*sign;
}