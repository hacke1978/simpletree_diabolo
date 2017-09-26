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

#ifndef ST_ASCIDEXPORTER_H
#define ST_ASCIDEXPORTER_H

#include "ct_exporter/abstract/ct_abstractexporterpointattributesselection.h"
#include "ct_view/itemdrawable/ct_itemdrawablehierarchycollectionwidget.h"
#include "ct_tools/attributes/ct_attributestocloudworkert.h"
#include "ct_tools/itemdrawable/ct_itemdrawablecollectionbuildert.h"
#include "ct_itemdrawable/abstract/ct_abstractpointsattributes.h"

class ST_ASCIDExporter : public CT_AbstractExporterPointAttributesSelection
{
    Q_OBJECT
public:
    ST_ASCIDExporter();
    ~ST_ASCIDExporter();

    virtual QString getExporterCustomName() const;

    CT_StepsMenu::LevelPredefined getExporterSubMenuName() const;

    void init();

    bool setItemDrawableToExport(const QList<CT_AbstractItemDrawable*> &list);

    virtual CT_AbstractExporter* copy() const;

protected:

    bool protectedExportToFile();
};

#endif // ST_ASCIDEXPORTER_H
