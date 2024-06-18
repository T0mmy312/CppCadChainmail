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

#define ACCURACY 0.001
#define B_OFFSET_PERCENT 1.4 // + 40%
#define str(var) std::to_string(var)

double x(double theta);
double y(double theta);
double xP(double theta); // P is prime (the derivative)
double yP(double theta); // P is prime (the derivative)
double distance(double theta, double delta);
double f(double delta, double theta, double dist);
double fP(double delta, double theta, double dist); // P is prime (the derivative)
double bCalc(double h);
double distCalc(double w, double t);
double zOffsetCalc(double l, double t);

double height = 4;
double width = 12;
double lenght = 12;
double thickness = 1.5;
double mainThickness = 2;
double lessThickness = 1.1;

double a = 20;
double b = bCalc(height);
int numX = 100;
int numY = 5;
double dist = distCalc(width, mainThickness); // don't know why but 7 seems to look better for this and zOffset
double zOffset = zOffsetCalc(lenght, mainThickness);

// ----------------------------------------------------------------------------------------------------------------------------------
// The generated model has NOT YET BEEN TESTED, so it only works in theory and in fusion (by spliting it by separate).
// ----------------------------------------------------------------------------------------------------------------------------------

int main() {
    model chainLink = loadModelAscii("models/ThickChainlink.stl", "unused", "chainlink");
    chainLink.avrPosToZero();
    model chainmail("models/chainmailOutput.stl", "chainmail");

    std::vector<Vector3> points;
    double currTheta = 0;
    Vector3 point(x(currTheta), y(currTheta), 0);
    points.push_back(point);
    double lastRightDeltaTheta = 1;
    for (int i = 0; i < numX * 2; i++) {
        double deltaTheta = lastRightDeltaTheta;
        double lastDeltaTheta = -100;
        while (absolute(deltaTheta - lastDeltaTheta) > ACCURACY) {
            lastDeltaTheta = deltaTheta;
            deltaTheta -= f(deltaTheta, currTheta, dist) / fP(deltaTheta, currTheta, dist);
        }
        lastRightDeltaTheta = deltaTheta;
        currTheta += deltaTheta;

        point = Vector3(x(currTheta), y(currTheta), 0);
        points.push_back(point);
        Vector3 pos = (point + points[i]) / 2;
        if (i % 2 == 0)
            pos.z = zOffset;
        
        model chainLinkDuplicate = duplicate(&chainLink);
        double angle = -RAD_TO_DEG * std::atan2(point.y - points[i].y, point.x - points[i].x);
        chainLinkDuplicate.moveTo(pos);
        chainLinkDuplicate.rotate(Vector3(0, 0, 1), angle);

        int yIteration = std::ceil(numY / 2.0);
        for (int j = 1; j < yIteration; j++) {
            if (i % 2 == 0 && numY % 2 == 1 && j == yIteration - 1)
                continue; // skips the last y layer on an odd numY
            model chainLinkDupeDupe = duplicate(&chainLinkDuplicate);
            chainLinkDupeDupe.move(Vector3(0, 0, 2*zOffset) * j);
            chainmail.addPolygons(&chainLinkDupeDupe.polygons);
        }
        chainmail.addPolygons(&chainLinkDuplicate.polygons);
    }
    chainmail.writeFile();
}

double x(double theta) {
    return (a + b * theta)*std::cos(DEG_TO_RAD * theta);
}
double y(double theta) {
    return (a + b * theta)*std::sin(DEG_TO_RAD * theta);
}
double xP(double theta) {
    return b * std::cos(DEG_TO_RAD * theta) - (a + b * theta) * std::sin(DEG_TO_RAD * theta);
}
double yP(double theta) {
    return b * std::sin(DEG_TO_RAD * theta) + (a + b * theta) * std::cos(DEG_TO_RAD * theta);
}
double distance(double theta, double delta) {
    return std::sqrt(pow(x(theta + delta) - x(theta), 2)+ pow(y(theta + delta) - y(theta), 2));
}
double f(double delta, double theta, double dist) {
    return std::sqrt(pow(x(theta + delta) - x(theta), 2)+ pow(y(theta + delta) - y(theta), 2)) - dist;
}
double fP(double delta, double theta, double dist) {
    return (xP(theta + delta)*(x(theta + delta) - x(theta)) + yP(theta + delta)*(y(theta + delta) - y(theta)))/(std::sqrt(pow(x(theta + delta) - x(theta), 2)+ pow(y(theta + delta) - y(theta), 2)));
}
double bCalc(double h) {
    return (h / 360) * B_OFFSET_PERCENT;
}
double distCalc(double w, double t) {
    return (3.0/4.0) * (w - t);
}
double zOffsetCalc(double l, double t) {
    return (3.0/4.0) * (l - t);
}