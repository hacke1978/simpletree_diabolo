/****************************************************************************

 Copyright (C) 2016-2017 Jan Hackenberg
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

#ifndef THREADPOOL_STD_OUT_MULTITHREAD_H
#define THREADPOOL_STD_OUT_MULTITHREAD_H



#include <QThreadPool>
#include <QStringList>
#include <QEnableSharedFromThis>

#include <step/filtering/std_out_multithread/std_mult_param.h>
#include <step/filtering/std_out_multithread/worker_thread_std_out_multithread.h>

class ThreadPoool_std_out_multithread:  public QThreadPool, public QEnableSharedFromThis<ThreadPoool_std_out_multithread>
{
    Q_OBJECT

    QVector<St_step_std_out_param> _params;


    int counter = 0;

public:

    ThreadPoool_std_out_multithread(QVector<St_step_std_out_param> params);

    void compute();



public slots:

    void worker_finish_tp();

signals:

    void emit_finished_worker(int counter);

};

#endif // THREADPOOL_STD_OUT_MULTITHREAD_H
