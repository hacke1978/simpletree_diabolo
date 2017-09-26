#include "st_stephelper.h"






ST_StepHelper::ST_StepHelper()
{

}

ST_StepHelper::~ST_StepHelper()
{
}






void ST_StepHelper::add_cylinder_data(QSharedPointer<Tree> tree, CT_ResultGroup *resCpy_res, CT_StandardItemGroup *grpCpy_grp
                                      ,QString cylinder_name, QString cylinder_grp_name)
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


        CT_StandardItemGroup* cylinderGroup = new CT_StandardItemGroup(_cylinder_grp.completeName(), resCpy_res);
        grpCpy_grp->addGroup(cylinderGroup);


        QSharedPointer<PointS> center = cylin->get_center_ptr();
        QSharedPointer<PointS> start = cylin->get_start_ptr();
        QSharedPointer<PointS> stop = cylin->get_end_ptr();


        CT_CylinderData *data = new CT_CylinderData(Eigen::Vector3d(center->x , center->y , center->z),
                                                    Eigen::Vector3d(stop->x-start->x, stop->y-start->y, stop->z-start->z),
                                                    cylin->get_radius(),
                                                    cylin->get_length());



        CT_Cylinder* cylinder = new CT_Cylinder(_cylinders.completeName(), resCpy_res, data);
        cylinderGroup->addItemDrawable(cylinder);

        cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_branchIDModelName.completeName(), CT_AbstractCategory::DATA_ID, resCpy_res, branchID));
        cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_branchOrderModelName.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, branchOrder));
        cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_segmentIDModelName.completeName(), CT_AbstractCategory::DATA_ID, resCpy_res, segmentID));
        cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_parentSegmentIDModelName.completeName(), CT_AbstractCategory::DATA_ID, resCpy_res, parentSegmentID));
        cylinder->addItemAttribute(new CT_StdItemAttributeT<float>(_growthVolumeModelName.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, growthVolume));
        cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_detection_type.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, detection));
        cylinder->addItemAttribute(new CT_StdItemAttributeT<int>(_improvement_type.completeName(), CT_AbstractCategory::DATA_NUMBER, resCpy_res, improvement));

    }
}


