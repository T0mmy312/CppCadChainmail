#pragma once

#ifndef _MATHANDVECTORTOOLS_H_
#define _MATHANDVECTORTOOLS_H_

#include <vector>
#include <iostream>
#include <math.h>

//? ----------------------------------------------------------------------------------------------------------------------------------
//? Math and Vector3 tools
//? ----------------------------------------------------------------------------------------------------------------------------------

#define PI 3.14159265359f
#define DEG_TO_RAD 0.0174532925199f
#define RAD_TO_DEG 57.2957795131f
#define FLOAT_COMP_DIF 0.000001f // the difference needed for to floats to be the same in a comparison
#define MAX_FLOAT (float)0x7FFFFFFF

double max(double a, double b) {
    if (a > b)
        return a;
    return b;
}
double min(double a, double b) {
    if (a > b)
        return b;
    return a;
}

typedef std::vector<std::vector<float>> Matrix;

class Vector3
{
public:
    Vector3(float _x = 0, float _y = 0, float _z = 0)
    {
        x = _x;
        y = _y;
        z = _z;
    }
    ~Vector3() {}

    float x;
    float y;
    float z;

    void print()
    {
        std::cout << "(x:" << x << ", y:" << y << ", z:" << z << ")";
    }
    float magnitude() // returns the lenght of the vector
    {
        return sqrt(x*x + y*y + z*z);
    }
    Vector3 normalized() // returns the normalized vector of this vector (magnitude = 1)
    {
        float mag = sqrt(x*x + y*y + z*z);
        return Vector3(x / mag, y / mag, z / mag);
    }

    Vector3 operator+(Vector3 const& obj) {
        return Vector3(x + obj.x, y + obj.y, z + obj.z);
    }
    void operator+=(Vector3 const& obj) {
        *this = *this + obj;
    }
    Vector3 operator-(Vector3 const& obj) {
        return Vector3(x - obj.x, y - obj.y, z - obj.z);
    }
    void operator-=(Vector3 const& obj) {
        *this = *this - obj;
    }
    Vector3 operator*(float const& obj) {
        return Vector3(x * obj, y * obj, z * obj);
    }
    void operator*=(float const& obj) {
        *this = *this * obj;
    }
    Vector3 operator/(float const& obj) {
        return Vector3(x / obj, y / obj, z / obj);
    }
    void operator/=(float const& obj) {
        *this = *this / obj;
    }

private:
};

double absolute(double x) { // returns the absolute of x (|x|)
    if (x < 0)
        return -x;
    return x;
}

class Quaternion
{
public:
    Quaternion(float _r = 1, float _i = 0, float _j = 0, float _k = 0) {
        r = _r;
        i = _i;
        j = _j;
        k = _k;
    }
    Quaternion(Vector3 axis, float roationDEG) {
        setAxisRotationDeg(axis, roationDEG);
    }
    ~Quaternion() {}

    float r; // real component
    float i; // first imaginary component
    float j; // second imaginary component
    float k; // third imaginary component

    void print() {
        std::cout << "q = "<<r<<" + "<<i<<"i + "<<j<<"j + "<<k<<"k";
    }

    float magnitude() { // returns the lenght of the Quaternion
        return sqrt(r*r + i*i + j*j + k*k);
    }

    Quaternion inverse() { // returns the inverse of the Quaternion
        return Quaternion(r, -i, -j, -k);
    }

    Vector3 eulerAnglesRad() { // angles roll, pitch and yaw in radians
        Vector3 eulerAngles;
        eulerAngles.y = asin(2 * (r * j - i * k));
        if (absolute(eulerAngles.y - PI/2) <= FLOAT_COMP_DIF)
            return Vector3(0, eulerAngles.y, -2*atan2(i, r));
        else if (abs(eulerAngles.y + PI/2) <= FLOAT_COMP_DIF)
            return Vector3(0, eulerAngles.y, 2*atan2(i, r));
        eulerAngles.x = atan2(2 * (r * i + j * k), r*r - i*i - j*j - k*k);
        eulerAngles.z = atan2(2 * (r * k + i * j), r*r + i*i + j*j + k*k);
        return eulerAngles;
    }
    Vector3 eulerAnglesDeg() { // angles roll, pitch and yaw in degrees
        return eulerAnglesRad() * RAD_TO_DEG;
    }

    void setEulerAnglesRad(Vector3 eulerInRad) { // x roll, y pitch, z yaw
        float cx = cos(eulerInRad.x / 2);
        float cy = cos(eulerInRad.y / 2);
        float cz = cos(eulerInRad.z / 2);
        float sx = sin(eulerInRad.x / 2);
        float sy = sin(eulerInRad.y / 2);
        float sz = sin(eulerInRad.z / 2);
        r = cx * cy * cz + sx * sy * sz;
        i = sx * cy * cz + cx * sy * sz;
        j = cx * sy * cz + sx * cy * sz;
        k = cx * cy * sz + sx * sy * cz;
    }
    void setEulerAnglesDeg(Vector3 eulerInDeg) { // x roll, y pitch, z yaw
        setEulerAnglesRad(eulerInDeg * DEG_TO_RAD);
    }

    void setAxisRotationRad(Vector3 axis, float rotationInRad) { // rotaion around a axis to Quaternion
        float hr = rotationInRad / 2;
        r = cos(hr);
        i = axis.x * sin(hr);
        j = axis.y * sin(hr);
        k = axis.z * sin(hr);
    }
    void setAxisRotationDeg(Vector3 axis, float rotationInDeg) { // rotaion around a axis to Quaternion
        setAxisRotationRad(axis, rotationInDeg * DEG_TO_RAD);
    }

    void operator=(Vector3 eulerInRad) { // x roll, y pitch, z yaw
       setEulerAnglesDeg(eulerInRad);
    }

    Quaternion operator*(Quaternion other) { // Carefull order of multiplication is important
        Quaternion result;
        result.r = (r*other.r - i*other.i - j*other.j - k*other.k);
        result.i = (r*other.i + i*other.r + j*other.k - k*other.j);
        result.j = (r*other.j - i*other.k + j*other.r + k*other.i);
        result.k = (r*other.k + i*other.j - j*other.i + k*other.r);
        return result;
    }
};

Vector3 rotateActive(Vector3 point, Quaternion rotation) { // Point is rotated with respect to the coordinate system
    Quaternion p(0, point.x, point.y, point.z); // Quaternion representation of the Point
    Quaternion pRes = rotation.inverse() * p * rotation; // pRes.i, pRes.j and pRes.k are the x, y and z of the rotated point
    return Vector3(pRes.i, pRes.j, pRes.k);
}
Vector3 rotatePassive(Vector3 point, Quaternion rotation) { // The coordinate system is rotated with respect to the Point
    Quaternion p(0, point.x, point.y, point.z); // Quaternion representation of the Point
    Quaternion pRes = rotation * p * rotation.inverse(); // pRes.i, pRes.j and pRes.k are the x, y and z of the rotated point
    return Vector3(pRes.i, pRes.j, pRes.k);
}

double sqr(double x) { // just returns x^2
    return x * x;
}

double scalarProd(Vector3 a, Vector3 b) { // returns the scalar produkt of two vectors
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vector3 crossProd(Vector3 a, Vector3 b) { // returns the normal vector of two vectors
    return Vector3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, b.y * a.x - a.y * b.x);
}

double angleInRads(Vector3 a, Vector3 b) { // returns the angle between two vectors
    return acos(scalarProd(a, b) / (a.magnitude() * b.magnitude()));
}

double det3x3(Matrix a) { // returns the determinat of a 3x3 Matrix
    return a[0][0]*a[1][1]*a[2][2] + a[0][1]*a[1][2]*a[2][0] + a[0][2]*a[1][0]*a[2][1] - a[2][0]*a[1][1]*a[0][2] - a[2][1]*a[1][2]*a[0][0] - a[2][2]*a[1][0]*a[0][1];
}

double clamp(double minC, double x, double maxC) { // clamps x between min and max
    if (x < minC)
        return minC;
    else if ( x > maxC)
        return maxC;
    return x;
}

#endif