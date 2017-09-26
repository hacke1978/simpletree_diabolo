#include "cylinder.h"

const float Cylinder::_MAX_DISTANCE = 0.0001;

Cylinder::Cylinder(pcl::ModelCoefficients coeff, QObject *parent) : QObject(parent)
{
    assert(coeff.values.size () == 7);
    this->values = coeff.values;

}

float Cylinder::get_radius() const
{
    return this->values.at(6);
}

void Cylinder::set_radius(const float radius)
{
    this->values.at(6) = radius;
}

float Cylinder::get_length() const
{
    float squared_length = values.at(3)*values.at(3) + values.at(4)*values.at(4) + values.at(5)*values.at(5);
    float length = std::sqrt(squared_length);
    return length;
}

float Cylinder::get_half_length() const
{
    float length = get_length();
    return (length/2);
}

float Cylinder::get_half_size() const
{
    float radius = get_radius();
    float half_length = get_half_length();
    float half_size = std::sqrt(radius*radius + half_length*half_length);
    return half_size;
}

float Cylinder::get_volume() const
{
    const float pi = boost::math::constants::pi<float> ();
    float volume = get_length () * get_radius () * get_radius () * pi;
    return volume;
}

const QSharedPointer<PointI> Cylinder::get_center() const
{
    QSharedPointer<PointI> center (new PointI);
    center->x = values.at(0) + (values.at(3)/2);
    center->y = values.at(1) + (values.at(4)/2);
    center->z = values.at(2) + (values.at(5)/2);
    return center;
}

std::vector<float> Cylinder::get_values()
{
    return this->values;
}

void Cylinder::set_segment(QWeakPointer<Segment> segment)
{
    this->segment = segment;
}

QSharedPointer<Segment> Cylinder::get_segment()
{
    return this->segment.lock();
}

bool Cylinder::operator ==(const QSharedPointer<Cylinder> cylinder2)
{
    return (   (values[0] == cylinder2->values[0])
            && (values[1] == cylinder2->values[1])
            && (values[2] == cylinder2->values[2])
            && (values[3] == cylinder2->values[3])
            && (values[4] == cylinder2->values[4])
            && (values[5] == cylinder2->values[5])
            && (values[6] == cylinder2->values[6]));
}

bool Cylinder::is_parent_of(const QSharedPointer<Cylinder> child) const
{
    QSharedPointer<PointI> end = get_end();
    QSharedPointer<PointI> start = child->get_start();
    float x = end->x - start->x;
    float y = end->y - start->y;
    float z = end->z - start->z;
    float dist = std::sqrt(x*x + y*y + z*z);
    if(dist < _MAX_DISTANCE)
    {
        return true;
    }
    return false;
}

bool Cylinder::is_child_of(const QSharedPointer<Cylinder> parent) const
{
    QSharedPointer<PointI> end = parent->get_end();

    QSharedPointer<PointI> start = get_start();

    float x = end->x - start->x;
    float y = end->y - start->y;
    float z = end->z - start->z;
    float dist = std::sqrt(x*x + y*y + z*z);
    if(dist < _MAX_DISTANCE)
    {
        return true;
    }
    return false;
}

//const QVector<QSharedPointer<Cylinder> >
//Cylinder::get_children(QVector<QSharedPointer<Cylinder> > & cylinders) const
//{
//    QVector<QSharedPointer<Cylinder> > children;
//    QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);
//    while(it.hasNext())
//    {
//        QSharedPointer<Cylinder> cylinder = it.next();
//        if(is_parent_of(cylinder))
//        {
//            children.push_back(cylinder);
//        }
//    }
//    return children;
//}

QSharedPointer<PointI> Cylinder::projection_on_axis(const QSharedPointer<PointI> point) const
{
    /**
     * http://gamedev.stackexchange.com/questions/72528/how-can-i-project-a-3d-point-onto-a-3d-line
     */

    QSharedPointer<PointI> start = get_start();
    QSharedPointer<PointI> end   = get_end();


    float a = (-start->x+point->x)*(-start->x+end->x) + (-start->y+point->y)*(-start->y+end->y) + (-start->z+point->z)*(-start->z+end->z);
    float b = (-start->x+end->x)*(-start->x+end->x) + (-start->y+end->y)*(-start->y+end->y) + (-start->z+end->z)*(-start->z+end->z);
    float x = (a/b)*(end->x-start->x);
    float y = (a/b)*(end->y-start->y);
    float z = (a/b)*(end->z-start->z);

    float x2 = start->x + x;
    float y2 = start->y + y;
    float z2 = start->z + z;

    QSharedPointer<PointI> projection (new PointI);
    projection->x = x2;
    projection->y = y2;
    projection->z = z2;
    return projection;
}

float Cylinder::dist_on_axis(const QSharedPointer<PointI> point) const
{
    QSharedPointer<PointI> projection = projection_on_axis(point);
    QSharedPointer<PointI> start = get_start();
    QSharedPointer<PointI> end   = get_end();
    float distance = 0;
    float dist_to_start = std::sqrt((projection->x-start->x)*(projection->x-start->x)
                                  + (projection->y-start->y)*(projection->y-start->y)
                                  + (projection->z-start->z)*(projection->z-start->z));
    float dist_to_end = std::sqrt((projection->x-end->x)*(projection->x-end->x)
                                + (projection->y-end->y)*(projection->y-end->y)
                                + (projection->z-end->z)*(projection->z-end->z));
    float length = get_length();
    if((dist_to_start<=length)&&(dist_to_end<=length))
    {
        return distance;
    } else
    {
        return std::min(dist_to_end,dist_to_start);
    }

}

ImprovementType Cylinder::getImprovement() const
{
    return improvement;
}

void Cylinder::setImprovement(ImprovementType value)
{
    improvement = value;
}

float Cylinder::dist_to_point(const QSharedPointer<PointI> point) const
{
    QSharedPointer<PointI> projection = projection_on_axis(point);
    float dist_on_ax = dist_on_axis(point);
    float dist_to_axis = std::sqrt((projection->x-point->x)*(projection->x-point->x)
                                 + (projection->y-point->y)*(projection->y-point->y)
                                 + (projection->z-point->z)*(projection->z-point->z));
    float multiplier = -1;
    if(dist_to_axis >= get_radius())
    {
        multiplier = 1;
    }
    float dist_to_hull = dist_to_axis - get_radius();
    float distance = multiplier * std::sqrt((dist_on_ax*dist_on_ax)+(dist_to_hull*dist_to_hull));
    return distance;
}

QSharedPointer<PointI> Cylinder::get_start() const
{
    QSharedPointer<PointI> start (new PointI);
    start->x = values.at(0);
    start->y = values.at(1);
    start->z = values.at(2);
    return start;
}

QSharedPointer<PointI> Cylinder::get_end() const
{
    QSharedPointer<PointI> end (new PointI);
    end->x = values.at(0)+values.at(3);
    end->y = values.at(1)+values.at(4);
    end->z = values.at(2)+values.at(5);
    return end;
}

QString Cylinder::to_string() const
{
    QString str;
    QSharedPointer<PointI> start = get_start ();
    QSharedPointer<PointI> end = get_end ();
    str.append(QString::number(start->x)).append(QString(",")).append(QString::number(start->y)).append(QString(",")).append(QString::number(start->z)).append(QString(","));
    str.append(QString::number(end->x)).append(QString(",")).append(QString::number(end->y)).append(QString(",")).append(QString::number(end->z)).append(QString(","));
    str.append(QString::number(get_volume())).append(QString(",")).append(QString::number(get_length())).append(QString(",")).append(QString::number(get_radius()));
    return str;
}




