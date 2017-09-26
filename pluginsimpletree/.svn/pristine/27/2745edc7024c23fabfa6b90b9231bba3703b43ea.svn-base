#include "tree.h"


const QString Tree::getTreeID() const
{
    return _treeID;
}

void Tree::setTreeID(const QString &value)
{
    _treeID = value;
}

QSharedPointer<Segment> Tree::get_root_segment() const
{
    return _root_segment;
}

const float Tree::get_min_height() const
{
    QVector<QSharedPointer<Cylinder> > cylinders = get_all_cylinders();
    float min_height = std::numeric_limits<float>::max();
    QVectorIterator<QSharedPointer<Cylinder> > it (cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        QSharedPointer<PointI> start = cylinder->get_start();
        QSharedPointer<PointI> end = cylinder->get_end();
        if(start->z < min_height)
        {
            min_height = start->z;
        }
        if(end->z < min_height)
        {
            min_height = end->z;
        }

    }
    return min_height;
}

const float Tree::get_max_height() const
{
    QVector<QSharedPointer<Cylinder> > cylinders = get_all_cylinders();
    float max_height = std::numeric_limits<float>::lowest();
    QVectorIterator<QSharedPointer<Cylinder> > it (cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        QSharedPointer<PointI> start = cylinder->get_start();
        QSharedPointer<PointI> end = cylinder->get_end();
        if(start->z > max_height)
        {
            max_height = start->z;
        }
        if(end->z > max_height)
        {
            max_height = end->z;
        }

    }
    return max_height;
}

const QVector<QSharedPointer<Cylinder> > Tree::get_stem_cylinders() const
{
    QVector<QSharedPointer<Cylinder> > cylinders = get_all_cylinders();
    QVector<QSharedPointer<Cylinder> > stem_cylinders;
    QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        if(cylinder->get_segment()->get_branch_order()==0)
        {
            stem_cylinders.push_back(cylinder);
        }
    }
    return stem_cylinders;
}

const float Tree::get_volume_to_base(const QSharedPointer<Segment> segment, const QSharedPointer<Segment> root) const
{
    float volume = segment->get_volume();
    if(segment!= root)
    {
        if(segment->get_parent_segment())
        {
            volume +=  get_volume_to_base(segment->get_parent_segment(),root);
        }
    }
    return volume;
}

const QVector<QSharedPointer<Segment> > Tree::get_leave_segments(const QSharedPointer<Segment> base) const
{
    QVector<QSharedPointer<Segment> > segments = get_child_segments(base);
    QVector<QSharedPointer<Segment> > leave_segments;
    QVectorIterator<QSharedPointer<Segment> > it(segments);
    while(it.hasNext())
    {
        QSharedPointer<Segment> segment = it.next();
        if(segment->is_leave())
        {
            leave_segments.push_back(segment);
        }
    }
    return leave_segments;
}

const float Tree::get_height() const
{
    return (get_max_height()-get_min_height()+_height_above_ground);
}

const float Tree::get_length() const
{
    float length = 0;
    QVector<QSharedPointer<Cylinder> > stem_cylinders = get_stem_cylinders();
    QVectorIterator<QSharedPointer<Cylinder> > it (stem_cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        length += cylinder->get_length();
    }
    length += _height_above_ground;
    return length;
}

const float Tree::get_volume() const
{
    QVector<QSharedPointer<Cylinder> > cylinders = get_all_cylinders();
    float volume = 0;
    QVectorIterator<QSharedPointer<Cylinder> > it (cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        volume += cylinder->get_volume();
    }
    return volume;
}

const float Tree::get_growth_volume(const QSharedPointer<Cylinder> cylinder) const
{
    QVector<QSharedPointer<Cylinder> >  cylinders = get_child_cylinders(cylinder);
    float growth_volume = 0;
    QVectorIterator<QSharedPointer<Cylinder> > it (cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        growth_volume += cylinder->get_volume();
    }
    return growth_volume;
}

const QVector<QSharedPointer<Cylinder> > Tree::get_child_cylinders(const QSharedPointer<Cylinder> base_cylinder) const
{
    QSharedPointer<Segment> base_segment = base_cylinder->get_segment();
    QVector<QSharedPointer<Segment> > segments = get_child_segments(base_segment);
    segments.removeAll(base_segment);

    QVector<QSharedPointer<Cylinder> > cylinders;
    QVector<QSharedPointer<Cylinder> > base_cylinders = base_segment->get_cylinders();
    int index = base_cylinders.indexOf(base_cylinder);
    if(index>=0)
    {
        for(index; index<base_cylinders.size(); index++)
        {
            cylinders.push_back(base_cylinders.at(index));
        }
    }

    QVectorIterator<QSharedPointer<Segment> > it (segments);
    while(it.hasNext())
    {
        QSharedPointer<Segment> segment = it.next();
        cylinders.append(segment->get_cylinders());
    }
    return cylinders;
}


const QVector<QSharedPointer<Segment> > Tree::get_child_segments(const QSharedPointer<Segment> base_segment) const
{
    QVector<QSharedPointer<Segment> > segments;
    segments.push_back(base_segment);
    QVector<QSharedPointer<Segment> > children = base_segment->get_child_segments();
    QVectorIterator<QSharedPointer<Segment> > it (children);
    while(it.hasNext())
    {
        QSharedPointer<Segment> child = it.next();
        QVector<QSharedPointer<Segment> > children_next_generation = get_child_segments(child);
        segments.append(children_next_generation);
    }
    return segments;
}

const QVector<QSharedPointer<Cylinder> > Tree::get_all_cylinders() const
{
    QVector<QSharedPointer<Segment> > segments = get_all_segments();
    QVector<QSharedPointer<Cylinder> > cylinders;
    QVectorIterator<QSharedPointer<Segment> > it (segments);
    while(it.hasNext())
    {
        QSharedPointer<Segment> segment = it.next();
        cylinders.append(segment->get_cylinders());
    }
    return cylinders;
}

const QVector<QSharedPointer<Segment> > Tree::get_all_segments() const
{
    return get_child_segments(_root_segment);
}

Tree::Tree(QSharedPointer<Segment> root_segment,  QString treeID, float height_above_ground )
{
    _root_segment = root_segment;
    _treeID = treeID;
    _height_above_ground = height_above_ground;
}
