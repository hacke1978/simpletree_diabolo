#include "removefalsecylinders.h"


RemoveFalseCylinders::RemoveFalseCylinders(QSharedPointer<Tree> tree)
{
    _tree = tree;
    remove();
    merge();
}

void RemoveFalseCylinders::remove()
{
    QVector<QSharedPointer<Segment> > segments = _tree->get_leave_segments(_tree->get_root_segment());
    QVectorIterator<QSharedPointer<Segment> > it (segments);
    while(it.hasNext())
    {
        QSharedPointer<Segment> segment = it.next();
         QVector<QSharedPointer<Cylinder> > cylinders = segment->get_cylinders();
        if(!(cylinders.size()>1))
        {
            segment->remove();
        }
    }
}

void RemoveFalseCylinders::merge()
{
    QVector<QSharedPointer<Segment> > segments = _tree->get_all_segments();
    QVectorIterator<QSharedPointer<Segment> > it (segments);
    while(it.hasNext())
    {
        QSharedPointer<Segment> segment = it.next();
        if(segment->get_child_segments().size()==1)
        {
            QSharedPointer<Segment> child = segment->get_child_segments().at(0);
            QVector<QSharedPointer<Segment> > grand_children = child->get_child_segments();
            child->remove();
            QVector<QSharedPointer<Cylinder> > cylinders = child->get_cylinders();
            QVectorIterator<QSharedPointer<Cylinder> > it2(cylinders);
            while(it2.hasNext())
            {
                QSharedPointer<Cylinder> cylinder = it2.next();
                segment->add_cylinder(cylinder);
            }
            QVectorIterator<QSharedPointer<Segment> > it3(grand_children);
            while(it3.hasNext())
            {
                QSharedPointer<Segment> grand_child = it3.next();
                segment->add_child_segment(grand_child);
            }
        }
    }
}
