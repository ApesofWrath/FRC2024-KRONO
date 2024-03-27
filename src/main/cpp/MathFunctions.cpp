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

// Return the RGB values (range 0-1) of given hue (range 0-360, values outside will be normalized) 
std::array<double, 3> MathFunctions::hueToRGB(double hue){
    double normalizedHue = std::fmod(hue, 360);
    double chroma = 1;
    double hue_prime = normalizedHue/60;
    double X = chroma * (1.0 - std::abs(std::fmod(hue_prime, 2.0) - 1.0));
    std::array<double, 3> result;
    if (hue_prime < 1){
        result = {chroma, X, 0};
    
    }else if (hue_prime < 2){
        result = {X, chroma, 0};
    }else if (hue_prime < 3){
        result = {0, chroma, X};
    }else if (hue_prime < 4){
        result = {0, X, chroma};
    }else if (hue_prime < 5){
        result = {X, 0, chroma};
    }else{
        result = {chroma, 0, X};
    }
    return result;
    
}

// double MathFunctions::getPointed(std::vector<std::vector<double>>){
//     minVal = 
// }

// double MathFunctions::POVDistance(std::vector<double> origin, std::vector<double> angle, std::vector<double> target){
//     centered = 
// }