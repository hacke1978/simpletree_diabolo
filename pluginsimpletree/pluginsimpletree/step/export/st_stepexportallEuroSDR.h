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

#ifndef ST_STEPEXPORTALL_EURO_SDR
#define ST_STEPEXPORTALL_EURO_SDR

#include "ct_step/abstract/ct_abstractstep.h"
#include "ct_result/model/inModel/ct_inresultmodelgrouptocopy.h"
#include "ct_result/model/outModel/ct_outresultmodelgroupcopy.h"
#include "ct_itemdrawable/abstract/ct_abstractitemdrawablewithpointcloud.h"
#include "ct_view/ct_stepconfigurabledialog.h"
#include "ct_result/model/outModel/tools/ct_outresultmodelgrouptocopypossibilities.h"
#include "ct_itemdrawable/ct_pointsattributesscalartemplated.h"
#include "ct_itemdrawable/ct_pointsattributesnormal.h"
#include "ct_itemdrawable/ct_ttreegroup.h"
#include "ct_itemdrawable/ct_cylinder.h"
#include "ct_colorcloud/abstract/ct_abstractcolorcloud.h"
#include <pcl/console/print.h>
#include <step/modelling/modellingthreadpool.h>
#include "SimpleTree4/export/export.h"
#include <step/simpletreestep.h>
#include "ct_itemdrawable/ct_transformationmatrix.h"
#include "ct_itemdrawable/tools/iterator/ct_groupiterator.h"
#include "ct_itemdrawable/ct_image2d.h"
// Alias for indexing models
#define DEFin_res "res"
#define DEFin_grp "grp"
#define DEFin_grp_header "grp_hd"
#define DEFin_grp_DTM "grp_dtm"
#define DEFin_header_in "header_in"
#define DEFin_DTM_in "dtm_in"

#define DEFin_model_in "model_in"
#define DEFin_coeff_in "coeff_in"
#define DEFin_header "header"
#define DEFin_res2 "res2"
#define DEFin_grp2 "grp2"
#define DEFin_trMat "transMat"
#define DEFin_cloud "cloud_in"

class ST_StepExportAllEuroSDR: public CT_AbstractStep, public SimpleTreeStep
{
    Q_OBJECT

public:

    /*! \brief Step constructor
     *
     * Create a new instance of the step
     *
     * \param dataInit Step parameters object
     */
    ST_StepExportAllEuroSDR(CT_StepInitializeData &dataInit);

    ~ST_StepExportAllEuroSDR();

    /*! \brief Step description
     *
     * Return a description of the step function
     */
    virtual QString getStepDescription() const;

    /*! \brief Step detailled description
     *
     * Return a detailled description of the step function
     */
    virtual QString getStepDetailledDescription() const;

    /*! \brief Step URL
     *
     * Return a URL of a wiki for this step
     */
    virtual QString getStepURL() const;

    /*! \brief Step copy
     *
     * Step copy, used when a step is added by step contextual menu
     */
    virtual ST_StepExportAllEuroSDR* createNewInstance(CT_StepInitializeData &dataInit);



protected:

    /*! \brief Input results specification
     *
     * Specification of input results models needed by the step (IN)
     */
    void createInResultModelListProtected();


    /*! \brief Output results specification
     *
     * Specification of output results models created by the step (OUT)
     */
    void createOutResultModelListProtected();

    /*! \brief Parameters DialogBox
     *
     * DialogBox asking for step parameters
     */
    virtual void createPostConfigurationDialog();



    /*! \brief Algorithm of the step
     *
     * Step computation, using input results, and creating output results
     */
    virtual void compute();







private:

    void write_plot(CT_Image2D<float> *dtm, QVector<PointCloudS::Ptr> clouds, QVector<QSharedPointer<Tree> > qsms,
                    QVector<MethodCoefficients> cfs, QString path);
    void write_taper(CT_Image2D<float> *dtm, QVector<PointCloudS::Ptr> clouds, QVector<QSharedPointer<Tree> > qsms,
                    QVector<MethodCoefficients> cfs, QString path);
    void write_dtm(CT_Image2D<float>* dtm, QString path, float res = 0.2f);
    void split_file_name (QString& file_name, QString& plot_id, QString& scan_design);
    void exportPoints(QDataStream &stream,
                      const CT_AbstractPointCloudIndex *constPCIndex,
                      const CT_AbstractColorCloud *cc);

    QStringList _file_name_list;

    bool _save_st_tree = true;
    bool _save_armap_tree = true;
    bool _save_coefficients = true;
    bool _save_all = true;
    bool _save_ply = true;


public:

CT_AutoRenameModels _cloud_in;
};

#endif // ST_STEPEXPORTALL_EURO_SDR
