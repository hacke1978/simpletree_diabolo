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


#ifndef ST_STEPCOMPUTEDTM
#define ST_STEPCOMPUTEDTM

#ifdef USE_OPENCV
#include "ct_step/abstract/ct_abstractstep.h"
#include "ct_tools/model/ct_autorenamemodels.h"
#include "ct_itemdrawable/ct_image2d.h"
#include "step/simpletreestep.h"

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <SimpleTree4/math/simplemath.h>


class ST_StepComputeDTM : public CT_AbstractStep, public SimpleTreeStep
{
    Q_OBJECT

public:

    /*! \brief Step constructor
     *
     * Create a new instance of the step
     *
     * \param dataInit Step parameters object
     */
    ST_StepComputeDTM(CT_StepInitializeData &dataInit);

    /*! \brief Step description
     *
     * Return a description of the step function
     */
    QString getStepDescription() const;

    /*! \brief Step detailled description
     *
     * Return a detailled description of the step function
     */
    QString getStepDetailledDescription() const;

    /*! \brief Step copy
     *
     * Step copy, used when a step is added by step contextual menu
     */
    CT_VirtualAbstractStep* createNewInstance(CT_StepInitializeData &dataInit);

protected:

    float get_percentage_under_plane(pcl::ModelCoefficients::Ptr coeff, PointCloudS::Ptr cloud);

    /*! \brief Input results specification
     *
     * Specification of input results models needed by the step (IN)
     */
    void createInResultModelListProtected();

    /*! \brief Parameters DialogBox
     *
     * DialogBox asking for step parameters
     */
    void createPostConfigurationDialog();

    /*! \brief Output results specification
     *
     * Specification of output results models created by the step (OUT)
     */
    void createOutResultModelListProtected();

    /*! \brief Algorithm of the step
     *
     * Step computation, using input results, and creating output results
     */
    void compute();



private:
    double _grid_size200;
    double _grid_size100;
    double _grid_size50;

    double _dif200 = 1;
    double _dif100 = 1;
    double _dif50 = 1;

    int _minPts = 15;
    float _min_perc_under = 0.33f;
    float _min_dist_below = 0.1;

    void interpolate_DTM(CT_Image2D<float>*  dtm200, double res);
    void median_filter(CT_Image2D<float>*  dtm200, float res);
    void cross_check_dtm (CT_Image2D<float>*  dtm200, CT_Image2D<float>*  dtm);
    void fill_DTM_a(QVector<PointCloudS::Ptr> clouds200, CT_Image2D<float>*  dtm200, CT_Image2D<float>*  dtm100, CT_Image2D<float>*  dtm50);
    void fill_DTM_b(QVector<PointCloudS::Ptr> clouds100,  CT_Image2D<float>*  dtm100, CT_Image2D<float>*  dtm50);
    void fill_DTM_c(QVector<PointCloudS::Ptr> clouds50,  CT_Image2D<float>*  dtm50);

    QVector<PointCloudS::Ptr> clouds_rasterized(PointCloudS::Ptr cloud, CT_Image2D<float>* DTM);

    double  _gridsize = 0.5;
    int     _nCells;
    double _max_distance = 1;
    float _max_angle_plane_normal = 75; //25

    double _clip_min = -10;
    double _clip_max = 10;

    double _max_deviation = 0.8; //0.4

    bool _clip_by_bbox = true;

    bool _als = false;



    CT_AutoRenameModels     _outDTMModelName200;
    CT_AutoRenameModels     _outDTMModelName100;
    CT_AutoRenameModels     _outDTMModelName50;

};
#endif
#endif // ST_STEPCOMPUTEDTM
