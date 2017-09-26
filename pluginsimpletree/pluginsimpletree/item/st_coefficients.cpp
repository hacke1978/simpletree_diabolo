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


#include "item/st_coefficients.h"

CT_DEFAULT_IA_INIT(ST_Coefficients)

ST_Coefficients::ST_Coefficients() : CT_AbstractItemDrawableWithoutPointCloud()
{
}

ST_Coefficients::ST_Coefficients(const CT_OutAbstractSingularItemModel *model,
                 const CT_AbstractResult *result, MethodCoefficients coeff) : CT_AbstractItemDrawableWithoutPointCloud(model, result)
{
    _coeff = coeff;
}

//QString ST_Coefficients::staticGetType()
//{
//    return "ST_Coefficients";

//}



ST_Coefficients::ST_Coefficients(const QString &modelName,
                 const CT_AbstractResult *result, MethodCoefficients coeff) : CT_AbstractItemDrawableWithoutPointCloud(modelName, result)
{
    _coeff = coeff;
}

CT_AbstractItemDrawable* ST_Coefficients::copy(const CT_OutAbstractItemModel *model, const CT_AbstractResult *result, CT_ResultCopyModeList copyModeList)
{
    ST_Coefficients *ref = new ST_Coefficients((const CT_OutAbstractSingularItemModel *)model, result, _coeff);
    ref->setAlternativeDrawManager(getAlternativeDrawManager());
    return ref;
}

CT_AbstractItemDrawable *ST_Coefficients::copy(const QString &modelName, const CT_AbstractResult *result, CT_ResultCopyModeList copyModeList)
{
    ST_Coefficients *ref = new ST_Coefficients(modelName, result,_coeff);
    ref->setAlternativeDrawManager(getAlternativeDrawManager());
    return ref;
}

QString ST_Coefficients::getcoeffID() const
{
    return _id.completeName();
}

