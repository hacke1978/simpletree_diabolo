#include "reordertree.h"

float ReorderTree::get_maximum_volume_to_leave(QSharedPointer<Segment> seg)
{
    QSharedPointer<Tree> tree (new Tree(seg,"tree"));
    QVector<float> volumina;
    QVector<QSharedPointer<Segment> > segments = tree->get_leave_segments(seg);
    QVectorIterator<QSharedPointer<Segment> > it (segments);
    while(it.hasNext())
    {
        QSharedPointer<Segment>  leaf = it.next();
        float volume = tree->get_volume_to_base(leaf, seg);
        volumina.push_back(volume);
    }
    float result = *(std::max_element(volumina.begin(), volumina.end()));
    return result;
}

bool ReorderTree::compareSegments(QSharedPointer<Segment> seg1, QSharedPointer<Segment> seg2)
{
    float volume1 = get_maximum_volume_to_leave(seg1);
    float volume2 = get_maximum_volume_to_leave(seg2);
    return(volume2<volume1);
}

void ReorderTree::reorder()
{
    QVector<QSharedPointer<Segment> > segments = _tree->get_all_segments();
    QVectorIterator<QSharedPointer<Segment> > it (segments);
    while(it.hasNext())
    {
        QSharedPointer<Segment> segment = it.next();
        if(segment->get_child_segments().size()>1)
        {
            QVector<QSharedPointer<Segment> > children = segment->get_child_segments();
            std::sort(children.begin(), children.end(), compareSegments);
            segment->set_child_segments(children);
        }
    }
}





ReorderTree::ReorderTree(QSharedPointer<Tree> tree)
{
    _tree = tree;
    reorder();
}
