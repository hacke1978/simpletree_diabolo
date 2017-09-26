#ifndef ST_STEPCROWN_H
#define ST_STEPCROWN_H


#include <pcl/io/obj_io.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>

#include <QStandardPaths>

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
#include "ct_itemdrawable/ct_meshmodel.h"

#include <pcl/console/print.h>
#include <step/modelling/modellingthreadpool.h>
#include "SimpleTree4/export/export.h"
#include "item/st_crown.h"
#include "item/st_tree.h"
#include "item/st_coefficients.h"

#include <iostream>

#include "ctlibio/readers/ct_reader_obj.h"

// Alias for indexing models
#define DEFin_res "res"
#define DEFin_grp "grp"
#define DEFin_model_in "model_in"
#define DEFout_crown "crown_out"
#define DEFin_header "header"
#define DEFin_coeff "coeff"


class ST_StepComputeCrown: public CT_AbstractStep
{
    Q_OBJECT

public:

    /*! \brief Step constructor
     *
     * Create a new instance of the step
     *
     * \param dataInit Step parameters object
     */
    ST_StepComputeCrown(CT_StepInitializeData &dataInit);

    ~ST_StepComputeCrown();

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
    virtual ST_StepComputeCrown* createNewInstance(CT_StepInitializeData &dataInit);

    virtual QString getStepName() const;



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
    CT_AutoRenameModels _coeff_out;
    CT_AutoRenameModels _tree_out;
    CT_AutoRenameModels _mesh_3d;
    CT_AutoRenameModels _mesh_2d;

};


#endif // ST_STEPCROWN_H
