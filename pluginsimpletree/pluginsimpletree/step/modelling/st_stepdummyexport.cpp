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
#include "st_stepdummyexport.h"


#define DEF_SearchOutResult "result"
#define DEFout_topologyGroup "topologyGroup"
#define DEFout_nodeGroup "nodeGroup"
#define DEFout_nodeCylinders "nodeCylinders"
#define DEFout_nodeMesh "nodeMesh"


ST_StepDummyExport::ST_StepDummyExport(CT_StepInitializeData &dataInit) : CT_AbstractStep(dataInit)
{
    // pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
}

ST_StepDummyExport::~ST_StepDummyExport()
{
}


// Step description (tooltip of contextual menu)
QString ST_StepDummyExport::getStepDescription() const
{
    return tr("QSM spherefollowing method.");
}

// Step detailled description
QString ST_StepDummyExport::getStepDetailledDescription() const
{
    return tr("See SimpleTree homepage." );
}

// Step URL
QString ST_StepDummyExport::getStepURL() const
{
    return tr("http://www.simpletree.uni-freiburg.de/");
    //return CT_AbstractStep::getStepURL(); //by default URL of the plugin
}

// Step copy method
ST_StepDummyExport* ST_StepDummyExport::createNewInstance(CT_StepInitializeData &dataInit)
{
    return new ST_StepDummyExport(dataInit);
}





void ST_StepDummyExport::createPostConfigurationDialog()
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


}


// Creation and affiliation of IN models
void ST_StepDummyExport::createInResultModelListProtected()
{
    setNotNeedInputResult();
}

// Creation and affiliation of OUT models
void ST_StepDummyExport::createOutResultModelListProtected()
{
    CT_OutResultModelGroup *resOut = createNewOutResultModel(DEF_SearchOutResult);

    resOut->setRootGroup(DEFout_grp);
    resOut->addGroupModel(DEFout_grp, DEFout_topologyGroup, new CT_TTreeGroup(), tr("Topology"));
    resOut->addGroupModel(DEFout_topologyGroup, DEFout_nodeGroup, new CT_TNodeGroup(), tr("Amap compatible model"));
    resOut->addItemModel(DEFout_nodeGroup, DEFout_nodeCylinders, new CT_Cylinder(), tr("Cylinder"));
    resOut->addItemModel(DEFout_nodeGroup, DEFout_nodeMesh, new CT_MeshModel(), tr("Mesh"));
}

void ST_StepDummyExport::compute()
{

    QList<CT_ResultGroup*> outResultList = getOutResultList();
    CT_ResultGroup* resOut = outResultList.at(0);

    QSharedPointer<Tree> dummy = create_dummy();
    CT_StandardItemGroup* grp = new CT_StandardItemGroup(DEFout_grp, resOut);
    resOut->addGroup(grp);

    CT_TTreeGroup *ctTree2 = constructTopology(resOut,dummy);
    if(ctTree2 != NULL)
        grp->addGroup(ctTree2);


    qDebug() << "dummy size (should be 9)" << dummy->get_all_cylinders().size();
    qDebug() << " dummy number leaf segmentes (2)" << dummy->get_leave_segments(dummy->get_root_segment());
    qDebug() << "volume (9 times pi (between 27,9 and 28))" << dummy->get_volume();

}

CT_TTreeGroup *ST_StepDummyExport::constructTopology(const CT_AbstractResult *res_r, QSharedPointer<Tree> tree)
{
    QSharedPointer<Segment> segment = tree->get_root_segment();
    CT_TTreeGroup *topology = new CT_TTreeGroup(DEFout_topologyGroup, res_r);

        CT_TNodeGroup *root = new CT_TNodeGroup(DEFout_nodeGroup, res_r);
        topology->setRootNode(root);
        setCylinders(res_r, root, segment,tree);
        constructTopologyRecursively(res_r,root, segment, tree);

    return topology;

}

void ST_StepDummyExport::constructTopologyRecursively(const CT_AbstractResult *res_r, CT_TNodeGroup *parent, QSharedPointer<Segment> segment, QSharedPointer<Tree> tree)
{
    QVector<QSharedPointer<Segment> > children = segment->get_child_segments();
    QVectorIterator<QSharedPointer<Segment> > it (children);
    while(it.hasNext())
    {
        QSharedPointer<Segment> segment_child = it.next();

            CT_TNodeGroup* branchGroup = new CT_TNodeGroup(DEFout_nodeGroup, res_r);
            parent->addBranch(branchGroup);
            setCylinders(res_r, branchGroup, segment_child, tree);
            constructTopologyRecursively(res_r,branchGroup, segment_child,tree);

    }
}

void ST_StepDummyExport::setCylinders(const CT_AbstractResult *res_r, CT_TNodeGroup *root, QSharedPointer<Segment> segment, QSharedPointer<Tree> tree)
{

    QVector<QSharedPointer<Cylinder> >cylinders = segment->get_cylinders();
    QVectorIterator<QSharedPointer<Cylinder> > it(cylinders);
    while(it.hasNext())
    {
        QSharedPointer<Cylinder> cylin = it.next();


            CT_TNodeGroup* cylinderGroup = new CT_TNodeGroup(DEFout_nodeGroup, res_r);
            root->addComponent(cylinderGroup);
            QSharedPointer<PointS> center = cylin->get_center_ptr();
            QSharedPointer<PointS> start = cylin->get_start_ptr();
            QSharedPointer<PointS> stop = cylin->get_end_ptr();
            CT_CylinderData *data = new CT_CylinderData(Eigen::Vector3d(center->x , center->y , center->z),
                                                        Eigen::Vector3d(stop->x-start->x, stop->y-start->y, stop->z-start->z),
                                                        cylin->get_radius(),
                                                        cylin->get_length());
            CT_Cylinder* cylinder = new CT_Cylinder(DEFout_nodeCylinders, res_r, data);
            cylinderGroup->addItemDrawable(cylinder);

            CT_Mesh* mesh = new CT_Mesh();

            Eigen::Vector3d centerCyl = data->getCenter();

            //mesh->createCylinder(data->getRadius(), data->getHeight(), centerCyl(0), centerCyl(1), centerCyl(2));
            // Add the transformation matrix

            CT_MeshModel* meshModel = new CT_MeshModel(DEFout_nodeMesh, res_r, mesh);
            //cylinderGroup->addItemDrawable(meshModel);

    }
}

QSharedPointer<Tree> ST_StepDummyExport::create_dummy()
{

    QSharedPointer<Segment> root_seg (new Segment);

    {
        pcl::ModelCoefficients coeff;
        coeff.values.push_back(0);
        coeff.values.push_back(0);
        coeff.values.push_back(0);

        coeff.values.push_back(0);
        coeff.values.push_back(0);
        coeff.values.push_back(1);

        coeff.values.push_back(1);
        QSharedPointer<Cylinder> cylinder (new Cylinder(coeff));
        root_seg->add_cylinder(cylinder);

    }

    {
        pcl::ModelCoefficients coeff;
        coeff.values.push_back(0);
        coeff.values.push_back(0);
        coeff.values.push_back(1);

        coeff.values.push_back(0);
        coeff.values.push_back(0);
        coeff.values.push_back(1);

        coeff.values.push_back(1);
        QSharedPointer<Cylinder> cylinder (new Cylinder(coeff));
        root_seg->add_cylinder(cylinder);

    }

    {
        pcl::ModelCoefficients coeff;
        coeff.values.push_back(0);
        coeff.values.push_back(0);
        coeff.values.push_back(2);

        coeff.values.push_back(0);
        coeff.values.push_back(0);
        coeff.values.push_back(1);

        coeff.values.push_back(1);
        QSharedPointer<Cylinder> cylinder (new Cylinder(coeff));
        root_seg->add_cylinder(cylinder);

    }

    {
        QSharedPointer<Segment> child1 (new Segment);

        {
            pcl::ModelCoefficients coeff;
            coeff.values.push_back(0);
            coeff.values.push_back(0);
            coeff.values.push_back(3);

            coeff.values.push_back(0);
            coeff.values.push_back(1);
            coeff.values.push_back(0);

            coeff.values.push_back(1);
            QSharedPointer<Cylinder> cylinder (new Cylinder(coeff));
            child1->add_cylinder(cylinder);

        }

        {
            pcl::ModelCoefficients coeff;
            coeff.values.push_back(0);
            coeff.values.push_back(1);
            coeff.values.push_back(3);

            coeff.values.push_back(0);
            coeff.values.push_back(1);
            coeff.values.push_back(0);

            coeff.values.push_back(1);
            QSharedPointer<Cylinder> cylinder (new Cylinder(coeff));
            child1->add_cylinder(cylinder);

        }

        {
            pcl::ModelCoefficients coeff;
            coeff.values.push_back(0);
            coeff.values.push_back(2);
            coeff.values.push_back(3);

            coeff.values.push_back(0);
            coeff.values.push_back(1);
            coeff.values.push_back(0);

            coeff.values.push_back(1);
            QSharedPointer<Cylinder> cylinder (new Cylinder(coeff));
            child1->add_cylinder(cylinder);

        }
        root_seg->add_child_segment(child1);
    }

    {
        QSharedPointer<Segment> child2 (new Segment);

        {
            pcl::ModelCoefficients coeff;
            coeff.values.push_back(0);
            coeff.values.push_back(0);
            coeff.values.push_back(3);

            coeff.values.push_back(0);
            coeff.values.push_back(-1);
            coeff.values.push_back(0);

            coeff.values.push_back(1);
            QSharedPointer<Cylinder> cylinder (new Cylinder(coeff));
            child2->add_cylinder(cylinder);

        }

        {
            pcl::ModelCoefficients coeff;
            coeff.values.push_back(0);
            coeff.values.push_back(-1);
            coeff.values.push_back(3);

            coeff.values.push_back(0);
            coeff.values.push_back(-1);
            coeff.values.push_back(0);

            coeff.values.push_back(1);
            QSharedPointer<Cylinder> cylinder (new Cylinder(coeff));
            child2->add_cylinder(cylinder);

        }

        {
            pcl::ModelCoefficients coeff;
            coeff.values.push_back(0);
            coeff.values.push_back(-2);
            coeff.values.push_back(3);

            coeff.values.push_back(0);
            coeff.values.push_back(-1);
            coeff.values.push_back(0);

            coeff.values.push_back(1);
            QSharedPointer<Cylinder> cylinder (new Cylinder(coeff));
            child2->add_cylinder(cylinder);

        }
        root_seg->add_child_segment(child2);
    }
    QSharedPointer<Tree> tree (new Tree(root_seg,"Dummy"));
    return tree;
}





