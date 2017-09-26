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


#ifndef ST_ACTIONSTEMPOINTDETECTION_H
#define ST_ACTIONSTEMPOINTDETECTION_H


#include "views/actions/st_actionstempointdetectionoptions.h"
#include "ct_actions/abstract/ct_abstractactionforgraphicsview.h"
#include "ct_itemdrawable/ct_scene.h"

#include <QRect>

class ST_ActionStemPointDetection_dataContainer
{
public:
    ST_ActionStemPointDetection_dataContainer();
    double            _eigen1_min;
    double            _eigen1_max;
    double            _eigen2_min;
    double            _eigen2_max;
    double            _eigen3_min;
    double            _eigen3_max;
};

class ST_ActionStemPointDetection : public CT_AbstractActionForGraphicsView
{
    Q_OBJECT
public:

    ST_ActionStemPointDetection(QList<CT_Scene*>* sceneList,  ST_ActionStemPointDetection_dataContainer* dataContainer);

    ~ST_ActionStemPointDetection();

    QString uniqueName() const;
    QString title() const;
    QString description() const;
    QIcon icon() const;
    QString type() const;

    void init();

    bool mousePressEvent(QMouseEvent *e);
    bool mouseMoveEvent(QMouseEvent *e);
    bool mouseReleaseEvent(QMouseEvent *e);
    bool wheelEvent(QWheelEvent *e);

    bool keyPressEvent(QKeyEvent *e);
    bool keyReleaseEvent(QKeyEvent *e);

    void draw(GraphicsViewInterface &view, PainterInterface &painter);
    void drawOverlay(GraphicsViewInterface &view, QPainter &painter);

    CT_AbstractAction* copy() const;

public slots:

    void update();

private slots:
    void redrawOverlay();
    void redrawOverlayAnd3D();


private:
    int         _value;
    QList<CT_Scene*>* _sceneList;



    ST_ActionStemPointDetection_dataContainer* _dataContainer;

};


#endif // ST_ACTIONSTEMPOINTDETECTION_H
