#ifndef DATA_STRUCTURE_H
#define	DATA_STRUCTURE_H

#include <iostream>

namespace pose_server{

struct Point3D {
    float x;
    float y;
    float z;
};

struct Point2D{
    int x;
    int y;

    bool operator==(const Point2D& other) const{
        return x == other.x && y == other.y;
    }
};

struct Triangle{
    Point2D p1, p2, p3;
};

struct Line{
    Point2D p1, p2;
};

struct Angles{
    float a, b, c;
};

struct Rot3D {
    float rx;
    float ry;
    float rz;
};

struct Box{
    std::string cls;
    float pb;
    int xmin;
    int ymin;
    int xmax;
    int ymax;
    bool near_side;
    float height;
};

struct BoxCv{
    std::string cls;
    int cx;
    int cy;
    float angle;

};
/*
struct Transform{
    std::string name;
    float x;
    float y;
    float z;
    float rx;
    float rz;
};
*/
//struct Points{
//    Point2D pt[2];
//};

}

namespace depth_server{
struct Point3D {
    float x;
    float y;
    float z;
};
}

#endif /* DATA_STRUCTURE_H */