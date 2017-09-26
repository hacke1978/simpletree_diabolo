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

#ifndef ST_COEFFICIENTS_H
#define ST_COEFFICIENTS_H

#include "ct_itemdrawable/abstract/ct_abstractitemdrawablewithoutpointcloud.h"
#include "ct_itemdrawable/tools/drawmanager/ct_standardreferencepointdrawmanager.h"

#include "SimpleTree4/model/tree.h"
#include "SimpleTree4/method/method_coefficients.h"

#include "ct_tools/model/ct_autorenamemodels.h"
#include <QSharedPointer>


class ST_Coefficients : public CT_AbstractItemDrawableWithoutPointCloud
{
    Q_OBJECT
    CT_TYPE_IMPL_MACRO(ST_Coefficients, CT_AbstractItemDrawableWithoutPointCloud,SimpleTreeCoefficients)

public:
    ST_Coefficients();

    /**
      * \brief Contructeur
      */
    ST_Coefficients(const CT_OutAbstractSingularItemModel *model,
            const CT_AbstractResult *result,
            MethodCoefficients coeff);

    ST_Coefficients(const QString &modelName,
            const CT_AbstractResult *result,
            MethodCoefficients coeff);

    virtual CT_AbstractItemDrawable* copy(const CT_OutAbstractItemModel *model, const CT_AbstractResult *result, CT_ResultCopyModeList copyModeList);

    virtual CT_AbstractItemDrawable* copy(const QString &modelName, const CT_AbstractResult *result, CT_ResultCopyModeList copyModeList);


    MethodCoefficients get_coeff() const {return _coeff;}

        QString getcoeffID() const;

private:


    CT_AutoRenameModels _id;

    MethodCoefficients _coeff;
    CT_DEFAULT_IA_BEGIN(ST_Coefficients)
    CT_DEFAULT_IA_V3(ST_Coefficients, CT_AbstractCategory::staticInitDataId(), &ST_Coefficients::getcoeffID, QObject::tr("CoefficientsID"), "cfid")
    CT_DEFAULT_IA_V3(ST_Coefficients, CT_AbstractCategory::staticInitDataId(), &ST_Coefficients::getcoeffID, QObject::tr("CoefficientsID"), "cfid")
    CT_DEFAULT_IA_END(ST_Coefficients)


};

#endif // ST_COEFFICIENTS_H
