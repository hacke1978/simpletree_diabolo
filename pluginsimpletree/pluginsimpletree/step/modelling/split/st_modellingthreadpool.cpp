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
#include "st_modellingthreadpool.h"

ST_ModellingThreadPool::ST_ModellingThreadPool(QVector<ST_StepParameter> params): _params(params)
{
    int thread_number = QThread::idealThreadCount()-1;
    this->setMaxThreadCount(thread_number);


}

void ST_ModellingThreadPool::start_computation()
{
    QVectorIterator<ST_StepParameter> it (_params);
    while(it.hasNext())
    {
        ST_StepParameter param = it.next();
        ST_Workemodelling * worker (new ST_Workemodelling(param, sharedFromThis()));
        QObject::connect(worker, SIGNAL(emit_finished_worker())      , this, SLOT(sent_finished_tp()) );
        QObject::connect(worker, SIGNAL(emit_qstring_worker(QString)), this, SLOT(sent_qstring_tp(QString)) );
        start(worker);
    }
    waitForDone();
}

void ST_ModellingThreadPool::sent_qstring_tp(QString str)
{
    emit emit_qstring_tp(str);
}

void ST_ModellingThreadPool::sent_finished_tp()
{
    emit emit_finished_tp();
}
