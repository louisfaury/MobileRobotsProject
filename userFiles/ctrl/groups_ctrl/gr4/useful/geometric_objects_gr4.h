#ifndef GEOMETRIC_OBJECTS_GR4_H
#define GEOMETRIC_OBJECTS_GR4_H

#include "namespace_ctrl.h"
#include "CtrlStruct_gr4.h"
#include "useful_gr4.h"

NAMESPACE_INIT(ctrlGr4);

class GeometricObject
{
public:
    GeometricObject();

    virtual void getDescription(char*) = 0; // fills a char array with descriptor

    static constexpr int DESCRIPTOR_SIZE = 1024; // descriptor size
protected:
};

class Point : public GeometricObject
{
public:
    Point();
    Point(double x, double y);

    double x(){ return m_x; }
    double y(){ return m_y; }
    double computeDistance(Point p);
    virtual void getDescription(char* descriptor);

protected:
    double m_x;
    double m_y;
};

class Segment : public GeometricObject
{
public:
    Segment();
    Segment(Point p1, Point p2);

    Point p1(){ return m_p1; }
    Point p2(){ return m_p2;}
    bool computeIntersection(Point p);
    bool computeIntersection(Segment s);
    virtual void getDescription(char* descriptor);

protected:
    Point m_p1;
    Point m_p2;
};

class Rectangle : public GeometricObject
{
public:
    Rectangle(Point center, double length, double width);

    bool isInside(Point p);
    bool isInside(Segment s);
    bool computeIntersection(Rectangle rect);
    bool computeIntersection(Segment seg);
    bool computeIntersection(Point p);
    Point center(){ return m_center; }
    Segment* edges(){ return m_edges; }
    double length(){ return m_length; }
    double width(){ return m_width; }
    virtual void getDescription(char* descriptor);

protected:
    Segment m_edges[4];
    Point m_center;
    double m_length;
    double m_width;
};

NAMESPACE_CLOSE();

#endif // GEOMETRIC_OBJECTS_GR4_H
