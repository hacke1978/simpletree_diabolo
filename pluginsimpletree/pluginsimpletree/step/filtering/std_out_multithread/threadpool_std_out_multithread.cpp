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

#include "threadpool_std_out_multithread.h"

#include <QTime>


void ThreadPoool_std_out_multithread::compute()
{
    QTime timer;
    timer.start();

    pcl::console::TicToc tt;
    tt.tic ();
    size_t size = _params.size();
    setExpiryTimeout(2000);
    tt.tic();
    int millisec = 0;
    millisec =  tt.toc ();
    for(size_t i = 0; i < size; i++)
    {
        St_step_std_out_param param = _params.at(i);
        Worker_Std_out_multithread  *worker (new  Worker_Std_out_multithread(param, sharedFromThis() ) );
        start(worker);
    }
    waitForDone();
}

void ThreadPoool_std_out_multithread::worker_finish_tp()
{
    emit emit_finished_worker(++counter);
}

ThreadPoool_std_out_multithread::ThreadPoool_std_out_multithread(QVector<St_step_std_out_param> params)
{
_params = params;
int thread_number = QThread::idealThreadCount()-1;
QThreadPool::globalInstance()->setMaxThreadCount(thread_number);
}

