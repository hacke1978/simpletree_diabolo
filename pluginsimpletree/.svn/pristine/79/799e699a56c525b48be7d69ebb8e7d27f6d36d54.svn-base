/****************************************************************************

 Copyright (C) 2016-2017 INRA (Institut National de la Recherche Agronomique, France) and IGN (Institut National de l'information Géographique et forestière, France)
 All rights reserved.

 Contact : jan.hackenberg@posteo.de

 Developers : Jan Hackenberg

 This file is part of Simpletree plugin Version 4 for Computree.

 Simpletree plugin is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 Simpletree plugin is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with Simpletree plugin.  If not, see <http://www.gnu.org/licenses/>.

*****************************************************************************/
#include "modellingthreadpool2.h"

ModellingThreadPool2::ModellingThreadPool2(QVector<StepParameter2> params): _params(params)
{
    int thread_number = QThread::idealThreadCount()-1;
   // thread_number = std::max(11,thread_number);
//        int thread_number = 1;
    QThreadPool::globalInstance()->setMaxThreadCount(thread_number);
    this->setMaxThreadCount(thread_number);
    QString str("convert;parameter_find;single_model;iterative_model;attractor;build_tree;pype;improve;rest;nmbrpts;id \n");
    timings.push_back(str);


}

void ModellingThreadPool2::start_computation()
{
    QVectorIterator<StepParameter2> it (_params);
    while(it.hasNext())
    {
        StepParameter2 param = it.next();
        WorkerModelling2 * worker (new WorkerModelling2(param, sharedFromThis()));
        start(worker);
    }
    waitForDone();
    emit_timinglist(timings);
    emit_number_cylinders(_number_cylinders_total);
}

void ModellingThreadPool2::count_cylinders(int number_per_tree)
{
    _number_cylinders_total += number_per_tree;
}

void ModellingThreadPool2::sent_timings(QString str)
{
    timings.push_back(str);
}

void ModellingThreadPool2::sent_qstring_tp(QString str)
{
    emit emit_qstring_tp(str);
}

void ModellingThreadPool2::sent_finished_tp()
{
    emit emit_finished_tp();
}
