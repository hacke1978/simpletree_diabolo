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
#include "st_stepmodelmovingaverage.h"






ST_StepModelMa::ST_StepModelMa(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{

    // pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
}

ST_StepModelMa::~ST_StepModelMa()
{
}


// Step description (tooltip of contextual menu)
QString ST_StepModelMa::getStepDescription() const
{
    return tr("QSM spherefollowing method - moving average smoothing.");
}

// Step detailled description
QString ST_StepModelMa::getStepDetailledDescription() const
{
    return tr("This step peforms a moving average smoothing on a QSM model.");
}

// Step URL
QString ST_StepModelMa::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
    //return CT_AbstractStep::getStepURL(); //by default URL of the plugin
}

// Step copy method
ST_StepModelMa* ST_StepModelMa::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepModelMa(dataInit);
}





void ST_StepModelMa::createPostConfigurationDialog()
{
    CT_StepConfigurableDialog *configDialog = newStandardPostConfigurationDialog();

    configDialog->addTitle(tr("You will find help and description about the paramters in the following publication."));
    configDialog->addTitle(tr("This is also the correct citation you should give for scientific publications."));
    configDialog->addEmpty();
    configDialog->addTitle(tr("Hackenberg, J.; Spiecker, H.; Calders, K.; Disney, M.; Raumonen, P."));
    configDialog->addTitle(tr("<em>SimpleTree —An Efficient Open Source Tool to Build Tree Models from TLS Clouds.</em>"));
    configDialog->addTitle(tr("Forests <b>2015</b>, 6, 4245-4294. "));
    configDialog->addEmpty();
    configDialog->addEmpty();
    configDialog->addText(tr("This step calculates a model with pre found parameters without any post processing routines."));
    //        configDialog->addDouble(tr("Cut height :"), "m",0 , 10, 2, _cut_height);

}


// Creation and affiliation of IN models
void ST_StepModelMa::createInResultModelListProtected()
{
    CT_InResultModelGroupToCopy *resIn_res = createNewInResultModelForCopy(DEFin_res, tr("cloud_in"));
    resIn_res->setZeroOrMoreRootGroup();
    resIn_res->addGroupModel("", DEFin_grp, CT_AbstractItemGroup::staticGetType(), tr("grp_in"));
    resIn_res->addItemModel(DEFin_grp, DEFin_cloud_in, CT_AbstractItemDrawableWithPointCloud::staticGetType(), tr("Isolated Tree cloud"));
    resIn_res->addItemModel(DEFin_grp, DEFin_coeff_in, ST_Coefficients::staticGetType(), tr ("parameter set"));
    //  resIn_res->addItemModel(DEFin_grp, DEFin_header, CT_FileHeader::staticGetType(), tr("File Header"));
    resIn_res->addItemModel(DEFin_grp, DEFin_tree_in, ST_Tree::staticGetType(), tr("tree model"));


}

// Creation and affiliation of OUT models
void ST_StepModelMa::createOutResultModelListProtected()
{
    CT_OutResultModelGroupToCopyPossibilities *resCpy_res = createNewOutResultModelToCopy(DEFin_res);

    if(resCpy_res!=NULL)
    {
        resCpy_res->addItemModel(DEFin_grp, _tree_out, new ST_Tree(), tr("tree - improved by smoothing average filter"));
        resCpy_res->addItemModel(DEFin_grp, _coeff_out, new ST_Coefficients(), tr("coefficients of tree - modelled with allometry"));
        resCpy_res->addGroupModel(DEFin_grp, _out_cylinder_grp, new CT_StandardItemGroup(), tr("Cylinder group - smoothing average filter"));
        resCpy_res->addItemModel(_out_cylinder_grp, _attractor_stem, new CT_Cylinder(), tr("attractor stem cylinders - smoothing average filter"));
        resCpy_res->addItemModel(_out_cylinder_grp, _spherefollowing_stem, new CT_Cylinder(), tr("spherefollowing stem cylinders - smoothing average filter"));

        resCpy_res->addItemModel(_out_cylinder_grp, _attractor_branch, new CT_Cylinder(), tr("attractor branch cylinders - smoothing average filter"));
        resCpy_res->addItemModel(_out_cylinder_grp, _spherefollowing_branch, new CT_Cylinder(), tr("spherefollowing branch  cylinders - smoothing average filter"));
    }
}



void ST_StepModelMa::add_cylinder_data(QSharedPointer<Tree> tree, CT_ResultGroup *resCpy_res, CT_StandardItemGroup *grpCpy_grp)
{
    QVector<QSharedPointer<Cylinder> > cylinders = tree->get_all_cylinders();
    QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);
    while(it.hasNext()) {
        QSharedPointer<Cylinder> cylin = it.next();
        int branchID = cylin->get_segment()->get_branch_id();
        int branchOrder = cylin->get_segment()->get_branch_order();
        int segmentID = cylin->get_segment()->get_id();
        int parentSegmentID = -1;
        if (!(cylin->get_segment()->is_root()))
        {
            parentSegmentID = cylin->get_segment()->get_parent_segment()->get_id();
        }
        float growthVolume = tree->get_growth_volume(cylin);

        int detection = cylin->get_detection();

        int improvement = cylin->get_improvement();


        CT_StandardItemGroup* cylinderGroup =  new CT_StandardItemGroup(_out_cylinder_grp.completeName(), resCpy_res);
        grpCpy_grp->addGroup(cylinderGroup);


        QSharedPointer<PointS> center = cylin->get_center_ptr();
        QSharedPointer<PointS> start = cylin->get_start_ptr();
        QSharedPointer<PointS> stop = cylin->get_end_ptr();


        CT_CylinderData *data = new CT_CylinderData(Eigen::Vector3d(center->x , center->y , center->z),
                                                    Eigen::Vector3d(stop->x-start->x, stop->y-start->y, stop->z-start->z),
                                                    cylin->get_radius(),
                                                    cylin->get_length());


        if(cylin->get_detection()==DetectionType::SPHEREFOLLOWING)
        {
            if(cylin->get_segment()->get_branch_order()==0)
            {
                CT_Cylinder* cylinder = new CT_Cylinder(_spherefollowing_stem.completeName(), resCpy_res, data);
                cylinderGroup->addItemDrawable(cylinder);
            } else {
                CT_Cylinder* cylinder = new CT_Cylinder(_spherefollowing_branch.completeName(), resCpy_res, data);
                cylinderGroup->addItemDrawable(cylinder);
            }
        } else {
            if(cylin->get_segment()->get_branch_order()==0)
            {
                CT_Cylinder* cylinder = new CT_Cylinder(_attractor_stem.completeName(), resCpy_res, data);
                cylinderGroup->addItemDrawable(cylinder);
            } else {
                CT_Cylinder* cylinder = new CT_Cylinder(_attractor_branch.completeName(), resCpy_res, data);
                cylinderGroup->addItemDrawable(cylinder);
            }

        }
    }
}




void ST_StepModelMa::compute()
{
    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resCpy_res = outResultList.at(0);
    CT_ResultGroupIterator itCpy_grp_temp(resCpy_res, this, DEFin_grp);
    float number_clouds = 0;
    float processed_clouds = 0;
    while (itCpy_grp_temp.hasNext() && !isStopped())
    {
        itCpy_grp_temp.next();
        number_clouds++;
    }


    CT_ResultGroupIterator itCpy_grp(resCpy_res, this, DEFin_grp);

    while (itCpy_grp.hasNext() && !isStopped())
    {
        processed_clouds++;
        CT_StandardItemGroup* grpCpy_grp = (CT_StandardItemGroup*) itCpy_grp.next();
        ST_Coefficients* coeff_in = (ST_Coefficients*) grpCpy_grp->firstItemByINModelName(this,DEFin_coeff_in );
        ST_Tree * tree_in = (ST_Tree*) grpCpy_grp->firstItemByINModelName(this, DEFin_tree_in);
        MethodCoefficients coeff = coeff_in->get_coeff();
        if(tree_in !=0)
        {
            QSharedPointer<Tree> tree_old = tree_in->getTree();
            QSharedPointer<Tree> tree = tree_old->clone();

            QVector<QSharedPointer<Cylinder> > cylinders = tree->get_all_cylinders();
            QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);

            while(it.hasNext())
            {
                QSharedPointer<Cylinder> cyl = it.next();
                QVector<QSharedPointer<Cylinder> > neighbors = tree->get_neighbor_cylinders(cyl);
                QVector<float> radii;
                QVectorIterator<QSharedPointer<Cylinder> > git (neighbors);
                while(git.hasNext())
                {
                    QSharedPointer<Cylinder> n_cyl = git.next();
                    radii.push_back(n_cyl->get_radius());
                }
                float average = SimpleMath<float>::get_median(radii);
                float radius = cyl->get_radius();
                if(radius > 1.1f*average || radius < 0.9*average)
                {
                    cyl->set_radius(average);
                }
            }

            add_cylinder_data(tree, resCpy_res, grpCpy_grp);
            QSharedPointer<Tree> tree_clone = tree->clone();
            ST_Tree * st_tree = new ST_Tree(_tree_out.completeName(),resCpy_res,tree_clone);
            grpCpy_grp->addItemDrawable(st_tree);
            ST_Coefficients *  st_coefficients = new ST_Coefficients( _coeff_out.completeName(), resCpy_res,coeff);
            grpCpy_grp->addItemDrawable(st_coefficients);


        }
        int percentage = (((float)processed_clouds)/((float)number_clouds)*100.0f);
        setProgress(percentage);
    }


}
