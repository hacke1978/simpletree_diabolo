#include "cylinder.h"

const float Cylinder::_MAX_DISTANCE = 0.0001;

Cylinder::Cylinder(pcl::ModelCoefficients coeff, QObject *parent) : QObject(parent)
{
    assert(coeff.values.size () == 7);
    this->values = coeff.values;

}

Cylinder::Cylinder(const Cylinder &cylinder)
{
    values = cylinder.values;
    detection = cylinder.detection;
    improvement = cylinder.improvement;

}

Eigen::Vector3f Cylinder::get_axis()
{
    Eigen::Vector3f axis (values[3],values[4],values[5]);
    return (axis);
}


float Cylinder::get_radius() const
{
    return this->values[6];
}

void Cylinder::set_radius(const float radius)
{
    this->values[6] = radius;
}

float Cylinder::get_length() const
{
    float x = values[3];
    float y = values[4];
    float z = values[5];
    float squared_length = x*x+y*y+z*z;
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



const PointS Cylinder::get_center() const
{
    PointS center (values.at(0) + (values.at(3)/2),values.at(1) + (values.at(4)/2),values.at(2) + (values.at(5)/2));


    //    PointS center;
    //    center.x = values.at(0) + (values.at(3)/2);
    //    center.y = values.at(1) + (values.at(4)/2);
    //    center.z = values.at(2) + (values.at(5)/2);
    return center;
}

const PointS Cylinder::get_start() const
{
    PointS start (values[0],values[1],values[2]);
    //    start.x = values.at(0);
    //    start.y = values.at(1);
    //    start.z = values.at(2);
    return start;
}

const PointS Cylinder::get_end() const
{
    PointS end (values.at(0) + values.at(3),values.at(1) + values.at(4),values.at(2) + values.at(5));
    //    end.x = values.at(0) + values.at(3);
    //    end.y = values.at(1) + values.at(4);
    //    end.z = values.at(2) + values.at(5);
    return end;
}

pcl::ModelCoefficients Cylinder::get_coeff() const
{
    pcl::ModelCoefficients coeff;
    coeff.values.push_back(this->values[0]);
    coeff.values.push_back(this->values[1]);
    coeff.values.push_back(this->values[2]);
    coeff.values.push_back(this->values[3]);
    coeff.values.push_back(this->values[4]);
    coeff.values.push_back(this->values[5]);
    coeff.values.push_back(this->values[6]);
    return coeff;
}

void Cylinder::set_coeff(pcl::ModelCoefficients coeff)
{
    this->values == coeff.values;
}

const QSharedPointer<PointS> Cylinder::get_center_ptr() const
{
    QSharedPointer<PointS> center (new PointS(values.at(0) + (values.at(3)/2),values.at(1) + (values.at(4)/2),values.at(2) + (values.at(5)/2)));
    //    center->x = values.at(0) + (values.at(3)/2);
    //    center->y = values.at(1) + (values.at(4)/2);
    //    center->z = values.at(2) + (values.at(5)/2);
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
    QSharedPointer<PointS> end = get_end_ptr();
    QSharedPointer<PointS> start = child->get_start_ptr();
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
    QSharedPointer<PointS> end = parent->get_end_ptr();

    QSharedPointer<PointS> start = get_start_ptr();

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

void Cylinder::set_start_end(const QSharedPointer<PointS> start, const QSharedPointer<PointS> end)
{
    float x = start->x;
    float y = start->y;
    float z = start->z;
    float x2 = end->x - start->x;
    float y2 = end->y - start->y;
    float z2 = end->z - start->z;

    values[0] =  x ;
    values[1] =  y ;
    values[2] =  z ;
    values[3] =  x2 ;
    values[4] =  y2 ;
    values[5] =  z2 ;
}

PointS Cylinder::projection_on_axis(const PointS point) const
{
    /**
     * http://gamedev.stackexchange.com/questions/72528/how-can-i-project-a-3d-point-onto-a-3d-line
     */

    PointS start = get_start();
    PointS end   = get_end();


    float a = (-start.x+point.x)*(-start.x+end.x) + (-start.y+point.y)*(-start.y+end.y) + (-start.z+point.z)*(-start.z+end.z);
    float b = (-start.x+end.x)*(-start.x+end.x) + (-start.y+end.y)*(-start.y+end.y) + (-start.z+end.z)*(-start.z+end.z);
    float x = (a/b)*(end.x-start.x);
    float y = (a/b)*(end.y-start.y);
    float z = (a/b)*(end.z-start.z);

    float x2 = start.x + x;
    float y2 = start.y + y;
    float z2 = start.z + z;

    PointS projection (x2,y2,z2);
    //    projection.x = x2;
    //    projection.y = y2;
    //    projection.z = z2;
    return projection;
}


QSharedPointer<PointS> Cylinder::projection_on_axis_ptr(const QSharedPointer<PointS> point) const
{
    /**
     * http://gamedev.stackexchange.com/questions/72528/how-can-i-project-a-3d-point-onto-a-3d-line
     */

    QSharedPointer<PointS> start = get_start_ptr();
    QSharedPointer<PointS> end   = get_end_ptr();


    float a = (-start->x+point->x)*(-start->x+end->x) + (-start->y+point->y)*(-start->y+end->y) + (-start->z+point->z)*(-start->z+end->z);
    float b = (-start->x+end->x)*(-start->x+end->x) + (-start->y+end->y)*(-start->y+end->y) + (-start->z+end->z)*(-start->z+end->z);
    float x = (a/b)*(end->x-start->x);
    float y = (a/b)*(end->y-start->y);
    float z = (a/b)*(end->z-start->z);

    float x2 = start->x + x;
    float y2 = start->y + y;
    float z2 = start->z + z;

    QSharedPointer<PointS> projection (new PointS(x2,y2,z2));
    //    projection->x = x2;
    //    projection->y = y2;
    //    projection->z = z2;
    return projection;
}

float Cylinder::dist_to_axis(const QSharedPointer<PointS> point) const
{
    QSharedPointer<PointS> projection = projection_on_axis_ptr(point);
    float x = projection->x-point->x;
    float y = projection->y-point->y;
    float z = projection->z-point->z;
    float dist_to_axis = std::sqrt(x*x+y*y+z*z);
    return dist_to_axis;
}

float Cylinder::dist_to_point(const PointS point) const
{
    PointS projection = projection_on_axis(point);
    float dist_on_ax = dist_on_axis(point);
    float x = projection.x-point.x;
    float y = projection.y-point.y;
    float z = projection.z-point.z;
    float dist_to_axis = std::sqrt(x*x+y*y+z*z);
    float multiplier = -1;
    if(dist_to_axis >= get_radius())
    {
        multiplier = 1;
    }
    float dist_to_hull = dist_to_axis - get_radius();
    float distance = multiplier * std::sqrt((dist_on_ax*dist_on_ax)+(dist_to_hull*dist_to_hull));
    return distance;
}

DetectionType Cylinder::get_detection() const
{
    return detection;
}

void Cylinder::set_detection(const DetectionType &value)
{
    detection = value;
}

bool Cylinder::getWell_fitted() const
{
    return well_fitted;
}

void Cylinder::setWell_fitted(bool value)
{
    well_fitted = value;
}

float Cylinder::dist_on_axis(const PointS point) const
{
    PointS projection = projection_on_axis(point);
    PointS start = get_start();
    PointS end   = get_end();
    float distance = 0;
    float dist_to_start = std::sqrt((projection.x-start.x)*(projection.x-start.x)
                                    + (projection.y-start.y)*(projection.y-start.y)
                                    + (projection.z-start.z)*(projection.z-start.z));
    float dist_to_end = std::sqrt((projection.x-end.x)*(projection.x-end.x)
                                  + (projection.y-end.y)*(projection.y-end.y)
                                  + (projection.z-end.z)*(projection.z-end.z));
    float length = get_length();
    if((dist_to_start<=length)&&(dist_to_end<=length))
    {
        return distance;
    } else
    {
        return std::min(dist_to_end,dist_to_start);
    }
}

float Cylinder::dist_on_axis_ptr(const QSharedPointer<PointS> point) const
{
    QSharedPointer<PointS> projection = projection_on_axis_ptr(point);
    QSharedPointer<PointS> start = get_start_ptr();
    QSharedPointer<PointS> end   = get_end_ptr();
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

int Cylinder::getParent_ID() const
{
    return parent_ID;
}

void Cylinder::setParent_ID(int value)
{
    parent_ID = value;
}

int Cylinder::getID() const
{
    return ID;
}

void Cylinder::setID(int value)
{
    ID = value;
}

int Cylinder::getNumber_pts() const
{
    return number_pts;
}

void Cylinder::setNumber_pts(int value)
{
    number_pts = value;
}

float Cylinder::getMean_distance_sqrd_angle() const
{
    return mean_distance_sqrd_angle;
}

void Cylinder::setMean_distance_sqrd_angle(float value)
{
    mean_distance_sqrd_angle = value;
}

float Cylinder::getMean_distance_sqrd() const
{
    return mean_distance_sqrd;
}

void Cylinder::setMean_distance_sqrd(float value)
{
    mean_distance_sqrd = value;
}

float Cylinder::getMean_distance() const
{
    return mean_distance;
}

void Cylinder::setMean_distance(float value)
{
    mean_distance = value;
}

QSharedPointer<Cylinder>
Cylinder::clone()
{
    pcl::ModelCoefficients coeff;
    coeff.values.push_back(this->values[0]);
    coeff.values.push_back(this->values[1]);
    coeff.values.push_back(this->values[2]);
    coeff.values.push_back(this->values[3]);
    coeff.values.push_back(this->values[4]);
    coeff.values.push_back(this->values[5]);
    coeff.values.push_back(this->values[6]);
    QSharedPointer<Cylinder> cylinder (new Cylinder(coeff));
    cylinder->set_improvement(get_improvement());
    cylinder->set_detection(get_detection());
    cylinder->set_allometry_improvement((get_allometry_improvement()));
    cylinder->set_segment(get_segment());
    cylinder->setID(getID());
    cylinder->setParent_ID(getParent_ID());
    cylinder->setWell_fitted(getWell_fitted());
    cylinder->setMean_distance(getMean_distance());
    cylinder->setMean_distance_sqrd(getMean_distance_sqrd());
    cylinder->setMean_distance_sqrd_angle(getMean_distance_sqrd_angle());
    cylinder->setNumber_pts(getNumber_pts());
    return cylinder;
}



AllometryImproved Cylinder::get_allometry_improvement() const
{
    return allom;
}

void Cylinder::set_allometry_improvement(AllometryImproved value)
{
    allom = value;
}


ImprovementType Cylinder::get_improvement() const
{
    return improvement;
}

void Cylinder::set_improvement(ImprovementType value)
{
    improvement = value;
}

float Cylinder::dist_to_point_ptr(const QSharedPointer<PointS> point) const
{
    QSharedPointer<PointS> projection = projection_on_axis_ptr(point);
    float dist_on_ax = dist_on_axis_ptr(point);
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

QSharedPointer<PointS> Cylinder::get_start_ptr() const
{
    QSharedPointer<PointS> start (new PointS (values.at(0),values.at(1),values.at(2)));
    //    start->x = values.at(0);
    //    start->y = values.at(1);
    //    start->z = values.at(2);
    return start;
}

QSharedPointer<PointS> Cylinder::get_end_ptr() const
{
    QSharedPointer<PointS> end (new PointS(values.at(0)+values.at(3),values.at(1)+values.at(4),values.at(2)+values.at(5)) );
    //    end->x = values.at(0)+values.at(3);
    //    end->y = values.at(1)+values.at(4);
    //    end->z = values.at(2)+values.at(5);
    return end;
}

QString Cylinder::to_string() const
{
    QString str;
    QSharedPointer<PointS> start = get_start_ptr ();
    QSharedPointer<PointS> end = get_end_ptr ();
    str.append(QString::number(start->x)).append(QString(",")).append(QString::number(start->y)).append(QString(",")).append(QString::number(start->z)).append(QString(","));
    str.append(QString::number(end->x)).append(QString(",")).append(QString::number(end->y)).append(QString(",")).append(QString::number(end->z)).append(QString(","));
    str.append(QString::number(get_volume())).append(QString(",")).append(QString::number(get_length())).append(QString(",")).append(QString::number(get_radius()));
    return str;
}




