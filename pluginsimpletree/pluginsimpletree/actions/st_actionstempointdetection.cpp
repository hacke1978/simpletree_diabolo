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

#include "actions/st_actionstempointdetection.h"
#include "ct_global/ct_context.h"
#include <QMouseEvent>
#include <QKeyEvent>
#include <QIcon>
#include <QPainter>

ST_ActionStemPointDetection_dataContainer::ST_ActionStemPointDetection_dataContainer()
{
    _eigen1_min = 0;
    _eigen1_max = 0;
    _eigen2_min = 0;
    _eigen2_max = 0;
    _eigen3_min = 0;
    _eigen3_max = 0;
}


ST_ActionStemPointDetection::ST_ActionStemPointDetection(QList<CT_Scene *> *sceneList, ST_ActionStemPointDetection_dataContainer *dataContainer) : CT_AbstractActionForGraphicsView()
{
    _sceneList = sceneList;
    _dataContainer = dataContainer;
    _value = 0;
}

ST_ActionStemPointDetection::~ST_ActionStemPointDetection()
{
}

QString ST_ActionStemPointDetection::uniqueName() const
{
    return "ST_ActionStemPointDetection";
}

QString ST_ActionStemPointDetection::title() const
{
    return "Stempointdetection";
}

QString ST_ActionStemPointDetection::description() const
{
    return "Detects the stem and major branch points.";
}

QIcon ST_ActionStemPointDetection::icon() const
{
    return QIcon(":/icons/select_rectangular.png");
}

QString ST_ActionStemPointDetection::type() const
{
    return CT_AbstractAction::TYPE_MODIFICATION;
}

void ST_ActionStemPointDetection::init()
{


    CT_AbstractActionForGraphicsView::init();

    if(nOptions() == 0)
    {
        // create the option widget if it was not already created
        ST_ActionStemPointDetectionOptions *option = new ST_ActionStemPointDetectionOptions(this);

        // add the options to the graphics view
        graphicsView()->addActionOptions(option);

        option->set_eigen_1_min(_dataContainer->_eigen1_min);
        option->set_eigen_1_max(_dataContainer->_eigen1_max);
        option->set_eigen_2_min(_dataContainer->_eigen2_min);
        option->set_eigen_2_max(_dataContainer->_eigen2_max);
        option->set_eigen_3_min(_dataContainer->_eigen3_min);
        option->set_eigen_3_max(_dataContainer->_eigen3_max);



        connect(option, SIGNAL(parametersChanged()), this, SLOT(update()));

        // register the option to the superclass, so the hideOptions and showOptions
        // is managed automatically
        registerOption(option);
        for (int i = 0 ; i < _sceneList->size() ; i++)
        {
            document()->addItemDrawable(*(_sceneList->at(i)));
        }
        document()->redrawGraphics(DocumentInterface::RO_WaitForConversionCompleted);
        dynamic_cast<GraphicsViewInterface*>(document()->views().first())->camera()->fitCameraToVisibleItems();
    }
}



void ST_ActionStemPointDetection::update()
{
    ST_ActionStemPointDetectionOptions *option = (ST_ActionStemPointDetectionOptions*)optionAt(0);

    _dataContainer->_eigen1_min = option->get_eigen_1_min();
    _dataContainer->_eigen1_min = option->get_eigen_1_min();
    _dataContainer->_eigen2_min = option->get_eigen_2_min();
    _dataContainer->_eigen2_min = option->get_eigen_2_min();
    _dataContainer->_eigen3_min = option->get_eigen_3_min();
    _dataContainer->_eigen3_min = option->get_eigen_3_min();

    redrawOverlayAnd3D();
}

void ST_ActionStemPointDetection::redrawOverlay()
{
    document()->redrawGraphics();
}

void ST_ActionStemPointDetection::redrawOverlayAnd3D()
{
    setDrawing3DChanged();
    document()->redrawGraphics();
}

bool ST_ActionStemPointDetection::mousePressEvent(QMouseEvent *e)
{
    Q_UNUSED(e);
    return false;
}

bool ST_ActionStemPointDetection::mouseMoveEvent(QMouseEvent *e)
{
    Q_UNUSED(e);
    return false;
}

bool ST_ActionStemPointDetection::mouseReleaseEvent(QMouseEvent *e)
{
    Q_UNUSED(e);
    return false;
}

bool ST_ActionStemPointDetection::wheelEvent(QWheelEvent *e)
{
    Q_UNUSED(e);
    return false;
}

bool ST_ActionStemPointDetection::keyPressEvent(QKeyEvent *e)
{
    Q_UNUSED(e);
    return false;
}

bool ST_ActionStemPointDetection::keyReleaseEvent(QKeyEvent *e)
{
    Q_UNUSED(e);
    return false;
}

void ST_ActionStemPointDetection::draw(GraphicsViewInterface &view, PainterInterface &painter)
{
    Q_UNUSED(view);
    Q_UNUSED(painter);
}

void ST_ActionStemPointDetection::drawOverlay(GraphicsViewInterface &view, QPainter &painter)
{
    Q_UNUSED(view);
    Q_UNUSED(painter);

}

CT_AbstractAction* ST_ActionStemPointDetection::copy() const
{
    return new ST_ActionStemPointDetection(_sceneList, _dataContainer);
}
