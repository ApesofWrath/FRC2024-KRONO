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

// double MathFunctions::getPointed(std::vector<std::vector<double>>){
//     minVal = 
// }

// double MathFunctions::POVDistance(std::vector<double> origin, std::vector<double> angle, std::vector<double> target){
//     centered = 
// }