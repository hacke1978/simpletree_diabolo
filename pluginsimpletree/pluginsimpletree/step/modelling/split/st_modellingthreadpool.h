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

#ifndef ST_MODELLINGTHREADPOOL_H
#define ST_MODELLINGTHREADPOOL_H

#include <QThreadPool>
#include <QEnableSharedFromThis>
#include "step/modelling/split/st_workermodelling.h"
#include "step/modelling/split/st_stepparameter.h"

class ST_ModellingThreadPool:  public QThreadPool, public QEnableSharedFromThis<ST_ModellingThreadPool>
{
    Q_OBJECT
    QVector<ST_StepParameter> _params;
public:
    ST_ModellingThreadPool(QVector<ST_StepParameter> params);

    void start_computation();

public slots:

    void sent_qstring_tp(QString str);

    void sent_finished_tp();

signals:


    void emit_finished_tp();

    void emit_qstring_tp(QString str);
};

#endif // ST_MODELLINGTHREADPOOL_H
