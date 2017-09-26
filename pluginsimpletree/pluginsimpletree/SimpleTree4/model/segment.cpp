#include "segment.h"




QVector<QSharedPointer<Cylinder> > Segment::get_cylinders()
{
    return _cylinders;
}

void Segment::set_cylinders(const QVector<QSharedPointer<Cylinder> > cylinders)
{
    QVectorIterator<QSharedPointer<Cylinder> > it (cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        cylinder->set_segment(sharedFromThis());
    }
    _cylinders = cylinders;
}



QVector<float> Segment::get_radii() const
{
    QVector<float> radii;
    QVectorIterator<QSharedPointer<Cylinder> > it(_cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        radii.push_back(cylinder->get_radius());
    }
    return radii;
}

float Segment::getReverse_pipe_order() const
{
    return _reverse_pipe_order;
}

void Segment::setReverse_pipe_order(float reverse_pipe_order)
{
    _reverse_pipe_order = reverse_pipe_order;
}

QSharedPointer<Segment> Segment::clone()
{
    QVector<QSharedPointer<Cylinder> > cylinders = get_cylinders();
    QVector<QSharedPointer<Cylinder> > cylinders_copy;
    QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        QSharedPointer<Cylinder> cylinder_copy = cylinder->clone();
        cylinders_copy.push_back(cylinder_copy);
    }



    QSharedPointer<Segment> segment_copy (new Segment);
    QVectorIterator<QSharedPointer<Cylinder> > hit(cylinders_copy);
    while(hit.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = hit.next();
        cylinder->set_segment(segment_copy);
        segment_copy->add_cylinder(cylinder);
    }

    QVector<QSharedPointer<Segment> > children = get_child_segments();
    QVectorIterator<QSharedPointer<Segment> > git(children);
    while(git.hasNext())
    {
        QSharedPointer<Segment> child = git.next();
        QSharedPointer<Segment> child_copy = child->clone();
        segment_copy->add_child_segment(child_copy);
        child_copy->set_parent_segment(segment_copy);
    }

    segment_copy->set_branch_id(_branch_id);
    segment_copy->set_branch_order(_branch_order);
    segment_copy->set_id(_id);
    segment_copy->setReverse_pipe_order(_reverse_pipe_order);
    return segment_copy;

}

bool Segment::remove_cylinder(const QSharedPointer<Cylinder> cylinder)
{
    int i = _cylinders.removeAll(cylinder);
    if(_cylinders.size()==0)
    {
        remove();
    }

    if(i == 1)
        return true;
    return false;
}

int Segment::get_id() const
{
    return _id;
}

void Segment::set_id(int id)
{
    _id = id;
}

QVector<QSharedPointer<Segment> > Segment::get_child_segments_recursively()
{
    QVector<QSharedPointer<Segment> > segment_list;
    segment_list.push_back(sharedFromThis());
    QVectorIterator<QSharedPointer<Segment> > it (_children);
    while(it.hasNext())
    {
        QSharedPointer<Segment> child = it.next();
        segment_list.append(child->get_child_segments_recursively());
    }
    return segment_list;
}

int Segment::get_branch_id() const
{
    return _branch_id;
}

void Segment::set_branch_id(int branch_id)
{
    _branch_id = branch_id;
}

int Segment::get_branch_order() const
{
    return _branch_order;
}

void Segment::set_branch_order(int branch_order)
{
    _branch_order = branch_order;
}

Segment::Segment(QObject *parent) : QObject(parent)
{

}

bool Segment::add_cylinder(const QSharedPointer<Cylinder> cylinder)
{
    bool is_contained  = _cylinders.contains(cylinder);
    if(!is_contained)
    {
        cylinder->set_segment(sharedFromThis());
        _cylinders.push_back(cylinder);

    }
    return (!is_contained);
}

float Segment::get_median_radius() const
{
    QVector<float> radii = get_radii();
    float median = SimpleMath<float>::get_median(radii);
    return median;
}

float Segment::get_mean_radius() const
{
    QVector<float> radii = get_radii();
    float mean = SimpleMath<float>::get_mean(radii);
    return mean;
}

float Segment::get_mean_length() const
{
    QVector<float> lengths;

    QVectorIterator<QSharedPointer<Cylinder> > it(_cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        lengths.push_back(cylinder->get_length());
    }
    float mean = SimpleMath<float>::get_mean(lengths);
    return mean;
}

bool Segment::remove_child(QSharedPointer<Segment> child)
{
    int number = _children.removeAll(child);
    if(number>0)
    {
        return true;
    }
    return false;
}

bool Segment::remove()
{
    QSharedPointer<Segment> parent = get_parent_segment();
    if(parent)
    {
        return(parent->remove_child(sharedFromThis()));
    }
    return false;
}

bool Segment::add_child_segment(QSharedPointer<Segment> child, bool connect_parent)
{
    bool is_contained = _children.contains(child);
    if(!is_contained)
    {
        if(connect_parent)
        {
            child->set_parent_segment(sharedFromThis(),false);
        }
        _children.push_back(child);
    }
    return (!is_contained);
}

QSharedPointer<Segment> Segment::get_parent_segment() const
{
    return _parent.lock();
}

void Segment::set_parent_segment(QSharedPointer<Segment> parent, bool connect_child)
{
    if(connect_child)
    {
        parent->add_child_segment(sharedFromThis(), false);
    }
    _parent = parent;
}



QVector<QSharedPointer<Segment> > Segment::get_child_segments() const
{
    return _children;
}

void Segment::set_child_segments(const QVector<QSharedPointer<Segment> > new_children)
{
    QVector<QSharedPointer<Segment> > children =  get_child_segments();
    QVectorIterator<QSharedPointer<Segment> > it(children);
    while(it.hasNext())
    {
        QSharedPointer<Segment> child = it.next();
        child->remove();
    }

    QVectorIterator<QSharedPointer<Segment> > it2(new_children);
    while(it2.hasNext())
    {
        QSharedPointer<Segment> child = it2.next();
        add_child_segment(child);
    }
}

QSharedPointer<PointS> Segment::get_start() const
{
    assert(!_cylinders.isEmpty());
    QSharedPointer<Cylinder> cylinder = _cylinders.first();
    return cylinder->get_start_ptr();
}

QSharedPointer<PointS> Segment::get_end() const
{
    assert(!_cylinders.isEmpty());
    QSharedPointer<Cylinder> cylinder = _cylinders.last();
    return cylinder->get_end_ptr();
}

float Segment::get_volume() const
{
    float volume = 0;
    QVectorIterator<QSharedPointer<Cylinder> > it(_cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        volume += cylinder->get_volume();
    }
    return volume;
}

float Segment::get_length() const
{
    float length = 0;
    QVectorIterator<QSharedPointer<Cylinder> > it(_cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        length += cylinder->get_length();
    }
    return length;
}

bool Segment::is_root() const
{
    return (!get_parent_segment());
}

bool Segment::is_leave() const
{
    return(_children.isEmpty());
}

QString Segment::to_string() const
{
    QString str;
    QSharedPointer<PointS> start = get_start ();
    QSharedPointer<PointS> end = get_end ();
    str.append(QString::number(start->x)).append(QString(",")).append(QString::number(start->y)).append(QString(",")).append(QString::number(start->z)).append(QString(","));
    str.append(QString::number(end->x)).append(QString(",")).append(QString::number(end->y)).append(QString(",")).append(QString::number(end->z)).append(QString(","));
    str.append(QString::number(get_volume())).append(QString(",")).append(QString::number(get_length())).append(QString(",")).append(QString::number(get_median_radius()));
    str.append(QString::number(_id)).append(QString(",")).append(QString::number(_branch_id)).append(QString(",")).append(QString::number(_branch_order));
    return str;
}

void Segment::merge()
{
    QVectorIterator<QSharedPointer<Cylinder> > it(_cylinders);
    QVector<QSharedPointer<Cylinder> > new_cylinders;
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder_first = it.next();
        if(it.hasNext())
        {
            QSharedPointer<Cylinder> cylinder_second = it.next();
            float radius = (cylinder_first->get_radius()+cylinder_second->get_radius())/2;
            float x1 = cylinder_first->values.at(0);
            float y1 = cylinder_first->values.at(1);
            float z1 = cylinder_first->values.at(2);
            float x2 = cylinder_second->values.at(0)+cylinder_second->values.at(3);
            float y2 = cylinder_second->values.at(1)+cylinder_second->values.at(4);
            float z2 = cylinder_second->values.at(2)+cylinder_second->values.at(5);

            pcl::ModelCoefficients coeff;
            coeff.values.resize(7);
            coeff.values.at(0) = x1;
            coeff.values.at(1) = y1;
            coeff.values.at(2) = z1;

            coeff.values.at(3) = x2-x1;
            coeff.values.at(4) = y2-y1;
            coeff.values.at(5) = z2-z1;

            coeff.values.at(6) = radius;
            QSharedPointer<Cylinder> cylinder_new ( new Cylinder (coeff));
            new_cylinders.push_back(cylinder_new);
        } else {
            new_cylinders.push_back(cylinder_first);
        }
    }
    set_cylinders(new_cylinders);
}
