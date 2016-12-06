/*!
 * @file geometrics_object_gr4.h
 * @author Louis Faury
 * @date 14/11
 */

#ifndef GEOMETRIC_OBJECTS_GR4_H
#define GEOMETRIC_OBJECTS_GR4_H

#include "namespace_ctrl.h"
#include "CtrlStruct_gr4.h"
#include "useful_gr4.h"
#include <string>

NAMESPACE_INIT(ctrlGr4);

class GeometricObject
{
public:
    GeometricObject();

    virtual bool computeIntersection(GeometricObject* obj) = 0;
    virtual void getDescription(char*) = 0; // fills a char array with descriptor, not use till now, will maybe be used for printing nice things on matlab
                                            // for milestone presentation
    virtual std::string tag(){ return m_tag; }

    static constexpr int DESCRIPTOR_SIZE = 1024; // descriptor size
protected:
    std::string m_tag;
};

class Point : public GeometricObject
{
public:
    Point();
    Point(double x, double y);

    double x(){ return m_x; }
    double y(){ return m_y; }
    void setCoord(double, double);
    double computeDistance(Point p);
    virtual bool computeIntersection(GeometricObject *obj){ return false; }// general inherited function
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
    virtual bool computeIntersection(GeometricObject* obj); // general inherited function
    virtual void getDescription(char* descriptor);

protected:
    Point m_p1;
    Point m_p2;
};

class Circle : public GeometricObject
{
public:
    Circle();
    Circle(Point center, double radius);
    Point center(){ return m_center; }
    double radius(){ return m_radius; }
    bool isInside(Point p);
    void setCenter(Point center){m_center = center;}
    void setRadius(double radius){m_radius = radius;}
    virtual bool computeIntersection(GeometricObject *obj);
    virtual void getDescription(char* descriptor);

protected:
    Point m_center;
    double m_radius;
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
    bool computeIntersection(Circle c);
    Point center(){ return m_center; }
    Segment* edges(){ return m_edges; }
    double length(){ return m_length; }
    double width(){ return m_width; }
    virtual bool computeIntersection(GeometricObject* obj);// general inherited function
    virtual void getDescription(char* descriptor);

protected:
    Segment m_edges[4];
    Point m_center;
    double m_length;
    double m_width;
};

NAMESPACE_CLOSE();

#endif // GEOMETRIC_OBJECTS_GR4_H
