#ifndef IMPROVEBYMEDIAN_H
#define IMPROVEBYMEDIAN_H

#include "src/model/tree.h"

class ImproveByMedian
{
private:
    const static float _MAX_PERCENTAGE;

    const static float _MIN_PERCENTAGE;

    QSharedPointer<Tree> _tree;

    /**
     * @brief median_smooth Within the segments outlier radii are detected via median check;
     */
    void median_smooth(QSharedPointer<Segment> segment);

    /**
     * @brief median_smooth Within the segments outlier radii are detected via mean check;
     */
    void mean_smooth(QSharedPointer<Segment> segment);

public:
    ImproveByMedian(QSharedPointer<Tree> tree);
};

#endif // IMPROVEBYMEDIAN_H
