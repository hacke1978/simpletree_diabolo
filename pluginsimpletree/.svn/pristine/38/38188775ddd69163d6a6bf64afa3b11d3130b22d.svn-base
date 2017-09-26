#ifndef HELPERFORMT_H
#define HELPERFORMT_H

#include <step/workerstepmt.h>
#include <step/st_stepabstractmodellingmt.h>
#include <QVector>
#include <QThreadPool>
struct SimpleTreeMT;
class ST_StepAbstractModellingMT;
class HelperForMT: public QThreadPool
{
    QVector<SimpleTreeMT> _st_mt_vec;
    ST_StepAbstractModellingMT * _step;
public:
    HelperForMT(QVector<SimpleTreeMT> st_mt_vec, ST_StepAbstractModellingMT * step);
    void start_mt();
};

#endif // HELPERFORMT_H
