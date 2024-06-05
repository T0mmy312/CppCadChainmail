#pragma once

#ifndef _MODELING_CLASSES_HPP_
#define _MODELING_CLASSES_HPP_

#include <vector>
#include "model.hpp"
#include "mathAndVectorTools.h"

void setPolyPointer(poly* p, Vector3 a, Vector3 b, Vector3 c) {
    p->p1 = a;
    p->p2 = b;
    p->p3 = c;
}

class cube
{
public:
    cube(Vector3 position, Vector3 size);
    ~cube() {}

    Vector3 pos; // read only!
    Vector3 scale; // read only!

    std::vector<poly*> polys;

    void transform(Vector3 position, Vector3 size);
    void rescale(Vector3 s);
    void reposition(Vector3 p);
};

cube::cube(Vector3 position, Vector3 size) {
    for (int i = 0; i < 12; i++) {
        polys.push_back((poly*)malloc(sizeof(poly)));
    }
    transform(position, size);
}

void cube::transform(Vector3 position, Vector3 size) {
    pos = position;
    scale = size;

    Vector3 hSize = size / 2;

    Vector3 pxpypz = position + hSize;
    Vector3 nxpypz = position + Vector3(-hSize.x, hSize.y, hSize.z);
    Vector3 pxpynz = position + Vector3(hSize.x, hSize.y, -hSize.z);
    Vector3 nxpynz = position + Vector3(-hSize.x, hSize.y, -hSize.z);
    Vector3 nxnypz = position + Vector3(-hSize.x, -hSize.y, hSize.z);
    Vector3 pxnypz = position + Vector3(hSize.x, -hSize.y, hSize.z);
    Vector3 nxnynz = position + hSize * -1;
    Vector3 pxnynz = position + Vector3(hSize.x, -hSize.y, -hSize.z);

    setPolyPointer(polys[0], pxpypz, nxpypz, nxpynz); // top face triangle 1
    setPolyPointer(polys[1], pxpypz, nxpynz, pxpynz); // top face triangle 2
    setPolyPointer(polys[2], pxnypz, nxnypz, nxnynz); // bottom face triangle 1
    setPolyPointer(polys[3], pxnypz, nxnynz, pxnynz); // bottom face triangle 2
    setPolyPointer(polys[4], pxpypz, pxpynz, pxnynz); // right face triangle 1
    setPolyPointer(polys[5], pxpypz, pxnynz, pxnypz); // right face triangle 2
    setPolyPointer(polys[6], nxpypz, nxpynz, nxnynz); // left face triangle 1
    setPolyPointer(polys[7], nxpypz, nxnynz, nxnypz); // left face triangle 2
    setPolyPointer(polys[8], pxpypz, nxpypz, nxnypz); // front face triangle 1
    setPolyPointer(polys[9], pxpypz, nxnypz, pxnypz); // front face triangle 2
    setPolyPointer(polys[10], pxpynz, nxpynz, nxnynz); // back face triangle 1
    setPolyPointer(polys[11], pxpynz, nxnynz, pxnynz); // back face triangle 2
}
void cube::rescale(Vector3 s) {
    transform(pos, s);
}
void cube::reposition(Vector3 p) {
    transform(p, scale);
}

#endif