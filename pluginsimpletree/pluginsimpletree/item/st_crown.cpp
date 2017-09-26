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

#include "st_crown.h"


CT_DEFAULT_IA_INIT(ST_Crown)

ST_Crown::ST_Crown() : CT_AbstractItemDrawableWithoutPointCloud()
{
}

ST_Crown::ST_Crown(const CT_OutAbstractSingularItemModel *model,
                 const CT_AbstractResult *result,
                 QSharedPointer<Crown> crown) : CT_AbstractItemDrawableWithoutPointCloud(model, result)
{
    _crown = crown;
}

ST_Crown::ST_Crown(const QString &modelName,
                 const CT_AbstractResult *result,
                 QSharedPointer<Crown> crown) : CT_AbstractItemDrawableWithoutPointCloud(modelName, result)
{
   _crown = crown;
}


CT_AbstractItemDrawable* ST_Crown::copy(const CT_OutAbstractItemModel *model, const CT_AbstractResult *result, CT_ResultCopyModeList copyModeList)
{
    ST_Crown *ref = new ST_Crown((const CT_OutAbstractSingularItemModel *)model, result, _crown);
    ref->setAlternativeDrawManager(getAlternativeDrawManager());

    return ref;
}

CT_AbstractItemDrawable *ST_Crown::copy(const QString &modelName, const CT_AbstractResult *result, CT_ResultCopyModeList copyModeList)
{
    ST_Crown *ref = new ST_Crown(modelName, result, _crown);
    ref->setAlternativeDrawManager(getAlternativeDrawManager());
    return ref;
}

QString ST_Crown::getCrownID() const
{
    return _id.completeName();
}


