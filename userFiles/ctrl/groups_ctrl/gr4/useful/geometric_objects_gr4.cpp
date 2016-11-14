#include "geometric_objects_gr4.h"
#include "algorithm"

NAMESPACE_INIT(ctrlGr4);

Point::Point() : GeometricObject(), m_x(0.), m_y(0.)
{
}

Point::Point(double x, double y) : GeometricObject(), m_x(x), m_y(y)
{

}

void Point::getDescription(char *descriptor)
{
    memcpy(descriptor, this, sizeof(Point));
}

double Point::computeDistance(Point p)
{
    double squareDist = (m_x-p.x())*(m_x-p.x()) + (m_y-p.y())*(m_y-p.y());
    return sqrt(squareDist);
}

Segment::Segment()
{
}

void Segment::getDescription(char *descriptor)
{
    memcpy(descriptor, this, sizeof(Segment));
}

Segment::Segment(Point p1, Point p2) : GeometricObject(), m_p1(p1), m_p2(p2)
{
}

bool Segment::computeIntersection(Point p)
{//tests if p is in the segment
    bool res = false;
    double length = m_p1.computeDistance(m_p2);

    // test if in line
    bool inLine = ( std::fabs( (m_p2.y() - m_p1.y())*(p.x()-m_p1.x()) - (m_p2.x()-m_p1.x())*(p.y()-m_p1.y()) ) < EPSILON );
    if (inLine)
    {//check distances
        if ( m_p1.computeDistance(p)<length)
            if (m_p2.computeDistance(p) < length)
                res = true; // p is in segment
    }

    return res;
}

bool Segment::computeIntersection(Segment s)
{
    bool res = false;

    double length = m_p1.computeDistance(m_p2);
    double lengthS = s.p1().computeDistance(s.p2());

    double dX( m_p2.x()-m_p1.x() );
    double dY( m_p2.y()-m_p1.y() );
    double dXs( s.p2().x()-s.p1().x() );
    double dYs( s.p2().y()-s.p1().y());

    double pall = dXs*dY - dX*dYs;
    bool parallel = ( std::fabs( pall ) < EPSILON ); //segments are parallels
    if (parallel)
    {
        res = ( computeIntersection(s.p1()) || computeIntersection(s.p2() ));
    }
    else
    {
        double intersectionX = (dXs*(m_p1.x()*dY - m_p1.y()*dX)-dX*(s.p1().x()*dYs - s.p1().y()*dXs))/pall;
        double intersectionY = (dYs*(p1().x()*dY - p1().y()*dX)-dY*(s.p1().x()*dYs - s.p1().y()*dXs))/pall;
        Point inter(intersectionX, intersectionY);

        if (    (s.p1().computeDistance(inter) < lengthS + EPSILON)
             && (s.p2().computeDistance(inter) < lengthS + EPSILON)
             && (m_p1.computeDistance(inter) < length + EPSILON)
             && (m_p2.computeDistance(inter) < length + EPSILON) )
            res = true;
    }

    return res;
}

Rectangle::Rectangle(Point center, double length, double width) : GeometricObject(), m_center(center), m_length(length), m_width(width)
{
    Point x1, x2;
    for (int i=0; i<4; i++)
    {
        switch(i)
        {
        case 0 :
            x1 = Point(center.x() - length/2, center.y() - width/2);
            x2 = Point(center.x() + length/2, center.y() - width/2);
            break;
        case 1 :
            x1 = Point(center.x() + length/2, center.y() - width/2);
            x2 = Point(center.x() + length/2, center.y() + width/2);
            break;
        case 2 :
            x1 = Point(center.x() + length/2, center.y() + width/2);
            x2 = Point(center.x() - length/2, center.y() + width/2);
            break;
        case 3 :
            x1 = Point(center.x() - length/2, center.y() + width/2);
            x2 = Point(center.x() - length/2, center.y() - width/2);
            break;
        default :
            break;
        }
        m_edges[i] = Segment(x1,x2);
    }
}

void Rectangle::getDescription(char *descriptor)
{
    memcpy(descriptor, this, sizeof(Rectangle));
}


bool Rectangle::isInside(Point p)
{
    bool res = false;

    if ( ( std::fabs( m_center.x() - p.x()) < m_length + EPSILON ) && ( std::fabs( m_center.y() - p.y()) < m_width + EPSILON ) )
        res = true;

    return res;
}

bool Rectangle::isInside(Segment s)
{
   bool res = false;
   res = isInside(s.p1()) && isInside(s.p2());

   return res;
}

bool Rectangle::computeIntersection(Rectangle rect)
{
    bool res;

    // is he inside ?
    if ( isInside(rect.center()) )
        res = true;
    // am I inside ?
    if ( rect.isInside(m_center) )
        res = true;
    // test every segment :
    Segment* rectEdges = rect.edges();
    for (int i=0; i<4; i++)
    {
        for (int j=0; j<4; j++)
        {
            if ( m_edges[i].computeIntersection(rectEdges[j]) )
            {
                res = true;
                break;
            }
        }
        if (res = true)
            break;
    }

    return res;
}

bool Rectangle::computeIntersection(Segment seg)
{
    bool res = false;
    // test intersection between seg and rect ;
    for (int i=0; i<4; i++)
    {
        if (m_edges[i].computeIntersection(seg))
        {
            res = true;
            break;
        }
    }

    if (res==false)
    {// test if both point are inside the rectangle
        res = isInside(seg);
    }

    return res;
}


NAMESPACE_CLOSE();
