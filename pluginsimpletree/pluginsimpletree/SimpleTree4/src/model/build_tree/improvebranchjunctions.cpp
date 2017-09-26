#include "improvebranchjunctions.h"

ImproveBranchJunctions::ImproveBranchJunctions(QSharedPointer<Tree> tree)
{
    _tree = tree;
    improve();
}

void ImproveBranchJunctions::improve()
{
    QVector<QSharedPointer<Segment> > segments = _tree->get_all_segments();
    QVectorIterator<QSharedPointer<Segment> > it(segments);
    while(it.hasNext())
    {
        QSharedPointer<Segment> segment = it.next();
        if (segment->get_cylinders().size()>1)
        {
          QSharedPointer<Cylinder> first = segment->get_cylinders().at (0);
          QSharedPointer<Cylinder> second = segment->get_cylinders().at (1);
          first->values.clear ();
          first->values.push_back (second->values[0] - second->values[3]);
          first->values.push_back (second->values[1] - second->values[4]);
          first->values.push_back (second->values[2] - second->values[5]);
          first->values.push_back (second->values[3]);
          first->values.push_back (second->values[4]);
          first->values.push_back (second->values[5]);
          first->values.push_back (second->values[6]);
        }
    }
}
