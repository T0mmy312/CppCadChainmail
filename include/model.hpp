#pragma once

#ifndef _MODEL_HPP_
#define _MODEL_HPP_

#include <stdint.h>
#include <string>
#include <iostream>
#include <vector>
#include <fstream>
#include <functional>
#include "mathAndVectorTools.h"

#define STL_80BYTE_HEADER "Model created by CppCad                                                        "
#define STL_TRIANGLE_NUM_BYTES 4
#define STL_FLOAT_NUM_BYTES 4
#define STL_ATTRIBUTE_BYTE_COUNT_NUM_BYTES 2
#define STL_ATTRIBUTE_NUM_BYTES_VALUE 0 //! keep this 0 moast readers don't know what to do with this

//? ----------------------------------------------------------------------------------------------------------------------------------
//? poly class
//? ----------------------------------------------------------------------------------------------------------------------------------

class poly
{
public:
    poly(Vector3 _p1 = Vector3(), Vector3 _p2 = Vector3(), Vector3 _p3 = Vector3());
    ~poly() {}

    Vector3 p1;
    Vector3 p2;
    Vector3 p3;

    Vector3 normal();

    void operator=(poly other) {
        p1 = other.p1;
        p2 = other.p2;
        p3 = other.p3;
    }
    Vector3* operator[](int index) {
        switch (index) {
        case 0:
            return &p1;
        case 1:
            return &p2;
        case 2:
            return &p3;
        }
        throw std::invalid_argument("Invalid index (can only be 0, 1, 2)");
        return &p1;
    }
};

poly::poly(Vector3 _p1, Vector3 _p2, Vector3 _p3) {
    p1 = _p1;
    p2 = _p2;
    p3 = _p3;
}

Vector3 poly::normal() {
    return crossProd(p2 - p1, p3 - p1).normalized();
}

//? ----------------------------------------------------------------------------------------------------------------------------------
//? model class
//? ----------------------------------------------------------------------------------------------------------------------------------

void writePolygon(std::ofstream* file, poly* polygon);

class model
{
public:
    model(std::string _filename);
    model(std::string _filename, std::string _objName);
    model(std::string _filename, std::vector<poly*> _polygons);
    model(std::string _filename, std::vector<poly*> _polygons, std::string _objName);
    ~model() {}

    std::string filename; // the filepath to save the model to
    std::string objName; // the name of the model (kind of useless only used in the ascii file format anyway) 
    std::vector<poly*> polygons; // pointers to all polygons of this model (pointers so you can change the induvidual polygons without needing to save an index)

    Vector3 position; // READ ONLY!

    void addPolygon(poly* polygon); // adds a polygon
    void addPolygons(std::vector<poly*> polys); // adds multiple polygons
    void addPolygons(std::vector<poly>* polys); // adds multiple polygons
    void addPolygons(std::vector<poly*>* polys); // adds multiple polygons
    void writeFile(); // writes the stl data to the given filepath
    Vector3 avrPos(); // returns the mid point of the model
    void forEach(std::function<void(model*, int, poly*, void*)> funk, void* args = nullptr); // executes a funktion for each polygon.
    /*  
     1. arg is the current model
     2. arg is the current polygon index
     3. arg is a pointer to the current polygon
     4. arg is a void pointer to pass in any needed data for your funktion (passes in the args argument)
    */
    void avrPosToZero(); // offsets all polygons so that the avr pos is (0, 0, 0)
    void move(Vector3 movement); // offsets all polygons by movement
    void moveTo(Vector3 pos); // offsets all polygons so that the avr pos is pos
    void rotate(Vector3 axis, float angle); // rotates the model around the avr pos with this axis and angle using Quaternions
};

// ----------------------------------------------------------------------------------------------------------------------------------
// constructors
// ----------------------------------------------------------------------------------------------------------------------------------

model::model(std::string _filename) {
    filename = _filename;
    objName = "unnamed";
    position = Vector3();
}
model::model(std::string _filename, std::string _objName) {
    filename = _filename;
    objName = _objName;
    position = Vector3();
}
model::model(std::string _filename, std::vector<poly*> _polygons) {
    filename = _filename;
    objName = "unnamed";
    polygons = _polygons;
    position = Vector3();
}
model::model(std::string _filename, std::vector<poly*> _polygons, std::string _objName) {
    filename = _filename;
    objName = _objName;
    polygons = _polygons;
    position = Vector3();
}

// ----------------------------------------------------------------------------------------------------------------------------------
// methods
// ----------------------------------------------------------------------------------------------------------------------------------

void model::addPolygon(poly* polygon) {
    polygons.push_back(polygon);
}

void model::addPolygons(std::vector<poly*> polys) {
    for (int i = 0; i < polys.size(); i++)
        addPolygon(polys[i]);
}
void model::addPolygons(std::vector<poly>* polys) {
    for (int i = 0; i < polys->size(); i++)
        addPolygon(&(*polys)[i]);
}
void model::addPolygons(std::vector<poly*>* polys) {
    for (int i = 0; i < polys->size(); i++)
        addPolygon((*polys)[i]);
}

void model::writeFile() {
    std::ofstream file(filename);
    file << "solid " << objName << std::endl;

    for (int i = 0; i < polygons.size(); i++) 
    {
        writePolygon(&file, polygons[i]);
    }

    file << "endsolid " << objName << std::endl;
    file.close();
}

Vector3 model::avrPos() {
    float maxX = -MAX_FLOAT;
    float minX = MAX_FLOAT;
    float maxY = -MAX_FLOAT;
    float minY = MAX_FLOAT;
    float maxZ = -MAX_FLOAT;
    float minZ = MAX_FLOAT;
    for (int i = 0; i < polygons.size() * 3; i++) {
        maxX = max((*polygons[i / 3])[i % 3]->x, maxX);
        minX = min((*polygons[i / 3])[i % 3]->x, minX);
        maxY = max((*polygons[i / 3])[i % 3]->y, maxY);
        minY = min((*polygons[i / 3])[i % 3]->y, minY);
        maxZ = max((*polygons[i / 3])[i % 3]->z, maxZ);
        minZ = min((*polygons[i / 3])[i % 3]->z, minZ);
    }
    return Vector3((minX + maxX) / 2.0, (minY + maxY) / 2.0, (minZ + maxZ) / 2.0);
}

void model::forEach(std::function<void(model*, int, poly*, void*)> funk, void* args) {
    for (int i = 0; i < polygons.size(); i++)
        funk(this, i, polygons[i], args);
}

void model::avrPosToZero() {
    Vector3 sub = avrPos();
    forEach([=](model* m, int i, poly* p, void* args){
        p->p1 -= sub;
        p->p2 -= sub;
        p->p3 -= sub;
    });
    position = Vector3();
}

void model::move(Vector3 movement) {
    forEach([=](model* m, int i, poly* p, void* args){
        p->p1 += movement;
        p->p2 += movement;
        p->p3 += movement;
    });
}

void model::moveTo(Vector3 pos) {
    avrPosToZero();
    move(pos);
}

void model::rotate(Vector3 axis, float angle) {
    Quaternion rotation(axis, angle);
    Vector3 origin = avrPos();
    forEach([=](model* m, int i, poly* p, void* args){
        Vector3 difP1 = p->p1 - origin;
        p->p1 = rotateActive(difP1, rotation) + origin;
        Vector3 difP2 = p->p2 - origin;
        p->p2 = rotateActive(difP2, rotation) + origin;
        Vector3 difP3 = p->p3 - origin;
        p->p3 = rotateActive(difP3, rotation) + origin;
    });
}

// ----------------------------------------------------------------------------------------------------------------------------------
// funktions
// ----------------------------------------------------------------------------------------------------------------------------------

void writePolygon(std::ofstream* file, poly* polygon) {
    Vector3 normal = polygon->normal();
    (*file) << "facet normal " << normal.x << " " << normal.y << " " << normal.z << std::endl;
    (*file) << "    outer loop" << std::endl;
    (*file) << "        vertex " << polygon->p1.x << " " << polygon->p1.y << " " << polygon->p1.z << std::endl;
    (*file) << "        vertex " << polygon->p2.x << " " << polygon->p2.y << " " << polygon->p2.z << std::endl;
    (*file) << "        vertex " << polygon->p3.x << " " << polygon->p3.y << " " << polygon->p3.z << std::endl;
    (*file) << "    endloop" << std::endl;
    (*file) << "endfacet" << std::endl;
}

model duplicate(model* toDupe) { // duplicates a model without having shared polygon memory
    model newModel(toDupe->filename, toDupe->objName);
    for (int i = 0; i < toDupe->polygons.size(); i++) {
        poly* p = (poly*)malloc(sizeof(poly));
        (*p) = (*toDupe->polygons[i]);
        newModel.addPolygon(p);
    }
    return newModel;
}

//? ----------------------------------------------------------------------------------------------------------------------------------
//? load Model
//? ----------------------------------------------------------------------------------------------------------------------------------

unsigned long readNum(std::ifstream* file, int numBytes);
std::vector<std::string> split(std::string str, char splitChr);

/* Currently not working
model loadModelBinary(std::string filepath, std::string filepathModel, std::string name = "unnamed") {
    std::ifstream file(filepath);
    char* buffer = (char*)malloc(80);
    file.read(buffer, 80);
    free(buffer);
    unsigned int numTriangles = readNum(&file, STL_TRIANGLE_NUM_BYTES);
    char normalbuffer[STL_FLOAT_NUM_BYTES * 3];
    char attributesSizeBuffer[STL_ATTRIBUTE_BYTE_COUNT_NUM_BYTES];
    model solid(filepathModel, name);
    for (int i = 0; i < numTriangles; i++) {
        poly p;
        file.read(normalbuffer, STL_FLOAT_NUM_BYTES * 3);
        p.p1 = Vector3((float)readNum(&file, STL_FLOAT_NUM_BYTES), (float)readNum(&file, STL_FLOAT_NUM_BYTES), (float)readNum(&file, STL_FLOAT_NUM_BYTES));
        p.p2 = Vector3((float)readNum(&file, STL_FLOAT_NUM_BYTES), (float)readNum(&file, STL_FLOAT_NUM_BYTES), (float)readNum(&file, STL_FLOAT_NUM_BYTES));
        p.p3 = Vector3((float)readNum(&file, STL_FLOAT_NUM_BYTES), (float)readNum(&file, STL_FLOAT_NUM_BYTES), (float)readNum(&file, STL_FLOAT_NUM_BYTES));
        file.read(attributesSizeBuffer, STL_ATTRIBUTE_BYTE_COUNT_NUM_BYTES);
        solid.addPolygon(&p);
    }
    file.close();
    return solid;
}
*/

model loadModelAscii(std::string filepath, std::string filepathModel, std::string name = "unnamed") {
    std::ifstream file(filepath);
    if (!file.good())
        return model("invalid", "invalid");
    std::string line;
    model loadedModel(filepathModel, name);
    int vertex = 0;
    poly* p = (poly*)malloc(sizeof(poly));
    while (!file.eof()) {
        std::getline(file, line);
        if (line.find("facet normal") != std::string::npos) {
            vertex = 0;
            p = (poly*)malloc(sizeof(poly));
        }
        else if (line.find("vertex") != std::string::npos) {
            std::vector<std::string> splitVec = split(line, ' ');
            if (splitVec.size() < 4) //      vertex   -2.900000e+00 6.000000e+00 7.000000e+00
                continue;
            Vector3 v(std::stof(splitVec[1]), std::stof(splitVec[2]), std::stof(splitVec[3]));
            switch (vertex) {
            case 0:
                p->p1 = v;
                break;
            case 1:
                p->p2 = v;
                break;
            case 2:
                p->p3 = v;
                break;
            }
            vertex++;
        }
        else if (line.find("endfacet") != std::string::npos && vertex >= 3) {
            loadedModel.addPolygon(p);
        }
    }
    return loadedModel;
}

std::vector<std::string> split(std::string str, char splitChr) {
    std::vector<std::string> ret;
    std::string partString = "";
    str += splitChr;
    for (int i = 0; i < str.length(); i++) {
        if (str[i] != splitChr)
            partString += str[i];
        else if (!partString.empty()) {
            ret.push_back(partString);
            partString.clear();
        }
    }
    return ret;
}

unsigned long readNum(std::ifstream* file, int numBytes) {
    unsigned long num = 0;
    unsigned char part;
    for (int i = 0; i < numBytes; i++) {
        if (!file->read(reinterpret_cast<char*>(&part), 1)) {
            std::cerr << "Error reading from file" << std::endl;
            return num;
        }
        num |= static_cast<unsigned long>(part) << (8 * i);
    }
    return num;
}

#endif