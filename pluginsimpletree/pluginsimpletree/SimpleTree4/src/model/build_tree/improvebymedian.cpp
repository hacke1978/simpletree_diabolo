#include "improvebymedian.h"

ImproveByMedian::ImproveByMedian(QSharedPointer<Tree> tree)
{
    _tree = tree;
    QVector<QSharedPointer<Segment> > segments = tree->get_all_segments();
    QVectorIterator<QSharedPointer<Segment> > it (segments);
    while(it.hasNext())
    {
        QSharedPointer<Segment> segment = it.next();
        median_smooth(segment);
    }
}

const float ImproveByMedian::_MAX_PERCENTAGE = 1.2f;
const float ImproveByMedian::_MIN_PERCENTAGE = 0.8f;

void ImproveByMedian::median_smooth(QSharedPointer<Segment> segment)
{
    float median = segment->get_median_radius();
    QVector<QSharedPointer<Cylinder> > cylinders = segment->get_cylinders();
    QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        float radius  = cylinder->get_radius();
        if(radius>median*_MAX_PERCENTAGE)
        {
            cylinder->set_radius(median);

        }
        if(radius<median*_MIN_PERCENTAGE)
        {
            cylinder->set_radius(median);

        }
    }
}

void ImproveByMedian::mean_smooth(QSharedPointer<Segment> segment)
{
    float mean = segment->get_mean_radius();
    QVector<QSharedPointer<Cylinder> > cylinders = segment->get_cylinders();
    QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylinder = it.next();
        float radius  = cylinder->get_radius();
        if(radius>mean*_MAX_PERCENTAGE)
        {
            cylinder->set_radius(mean);

        }
        if(radius<mean*_MIN_PERCENTAGE)
        {
            cylinder->set_radius(mean);
        }
    }
}
