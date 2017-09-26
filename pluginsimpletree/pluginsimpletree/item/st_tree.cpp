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
#include "item/st_tree.h"

CT_DEFAULT_IA_INIT(ST_Tree)

ST_Tree::ST_Tree() : CT_AbstractItemDrawableWithoutPointCloud()
{
}

ST_Tree::ST_Tree(const CT_OutAbstractSingularItemModel *model,
                 const CT_AbstractResult *result,
                 QSharedPointer<Tree> tree) : CT_AbstractItemDrawableWithoutPointCloud(model, result)
{
    _tree = tree;
}

ST_Tree::ST_Tree(const QString &modelName,
                 const CT_AbstractResult *result,
                 QSharedPointer<Tree> tree) : CT_AbstractItemDrawableWithoutPointCloud(modelName, result)
{
    _tree = tree;
}

//QString ST_Tree::staticGetType()
//{
//    return "ST_Tree";

//}

CT_AbstractItemDrawable* ST_Tree::copy(const CT_OutAbstractItemModel *model, const CT_AbstractResult *result, CT_ResultCopyModeList copyModeList)
{
    ST_Tree *ref = new ST_Tree((const CT_OutAbstractSingularItemModel *)model, result, _tree);
    ref->setAlternativeDrawManager(getAlternativeDrawManager());

    return ref;
}

CT_AbstractItemDrawable *ST_Tree::copy(const QString &modelName, const CT_AbstractResult *result, CT_ResultCopyModeList copyModeList)
{
    ST_Tree *ref = new ST_Tree(modelName, result, _tree);
    ref->setAlternativeDrawManager(getAlternativeDrawManager());
    return ref;
}

QString ST_Tree::getTreeID() const
{
    return _id.completeName();
}


