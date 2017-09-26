#ifndef REMOVEFALSECYLINDERS_H
#define REMOVEFALSECYLINDERS_H

#include "src/model/tree.h"
class RemoveFalseCylinders
{
    QSharedPointer<Tree> _tree;

    /**
     * @brief improve Removes all leaf segments which contain only 1 cylinder as those are artefacts
     */
    void
    remove();
    /**
     * @brief merge If after the removal of the segments two segments can be merged, they will be merged
     */
    void
    merge();
public:
    RemoveFalseCylinders(QSharedPointer<Tree> tree);
};

#endif // REMOVEFALSECYLINDERS_H
