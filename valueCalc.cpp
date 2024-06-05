#include <iostream>
#include "include/mathAndVectorTools.h"
#include "include/model.hpp"
#include "include/modelingClasses.hpp"

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.0174532925199432957692369076849
#endif
#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.295779513082320876798154814105
#endif

#define ACCURACY 0.0001
#define B_OFFSET_PERCENT 1.4 // + 40%
#define str(x) std::to_string(x)

double x(double theta);
double y(double theta);
double distance(double theta, double delta);
double bCalc(double h);
double distCalc(double w, double t);
double zOffsetCalc(double l, double t);
double secondLayerDist(double w, double t);

double height = 4;
double width = 12;
double lenght = 12;
double thicness = 1.5;
double moreThicness = 2;
double lessThicness = 1.1;

double b = bCalc(height);
double dist = distCalc(width, moreThicness);
double dist2 = secondLayerDist(width, moreThicness) - 2; // This - 2 is adjusting for some calculation errors I haven't fixed
double zOffset = zOffsetCalc(lenght, moreThicness) - 0.5; // this - 0.5 too (its just temporary bad code)

double radiusBegin = 12;
double radiusEnd = 110;
double spuleHeight = 240;

double a = radiusBegin;

// ----------------------------------------------------------------------------------------------------------------------------------
// The calculations here aren't as accurate as hoped plus the height calculation is specific to this model,
// so the values ARE NOT TO BE TRUSTED. Check if the generated model has the right dimentions and tolerances for use.
// ----------------------------------------------------------------------------------------------------------------------------------

int main() {
    int numX = 0, numY = 0;
    Vector3 pos(0, 0, 0);
    double currTheta = 0;
    for (int i = 0;;i++)
    {
        double deltaTheta = 0;
        double distan = (i % 2 == 0 ? dist2 : dist);
        while (distance(currTheta, deltaTheta) <= distan)
            deltaTheta += ACCURACY;
        currTheta += deltaTheta;
        pos = Vector3(x(currTheta), y(currTheta), 0);
        if (pos.magnitude() >= radiusEnd)
            break;
        else
            numX++;
    }
    numY = (spuleHeight - 4.5) / 7.5;
    std::cout << "max Dimentions:" << std::endl;
    std::cout << "\tmumX: " << numX << std::endl;
    std::cout << "\tmumY: " << numY << std::endl;
}

double x(double theta) {
    return (a + b * theta)*std::cos(DEG_TO_RAD * theta);
}
double y(double theta) {
    return (a + b * theta)*std::sin(DEG_TO_RAD * theta);
}
double distance(double theta, double delta) {
    return std::sqrt(pow(x(theta + delta) - x(theta), 2)+ pow(y(theta + delta) - y(theta), 2));
}
double bCalc(double h) {
    return (h / 360) * B_OFFSET_PERCENT;
}
double distCalc(double w, double t) {
    return (w - t) / 2;
}
double zOffsetCalc(double l, double t) {
    return l - 2*t;
}
double secondLayerDist(double w, double t) {
    return w - t;
}