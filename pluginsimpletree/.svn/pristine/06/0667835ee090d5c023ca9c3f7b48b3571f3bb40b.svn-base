CT_PREFIX = ../../computreev5

exists(../../computreev5) {
    CT_PREFIX = ../../computreev5
    DEFINES += COMPUTREE_V5
}

MUST_USE_PCL = 1
MUST_USE_OPENCV = 1

COMPUTREE += ctlibpcl

include($${CT_PREFIX}/shared.pri)
include($${PLUGIN_SHARED_DIR}/include.pri)

include($${CT_PREFIX}/include_ct_library.pri)
TARGET = plug_simpletree

HEADERS += $${PLUGIN_SHARED_INTERFACE_DIR}/interfaces.h \
st_pluginentry.h \
st_pluginmanager.h \
step/export/st_stepexportall.h \
step/segmentation/st_stepsegmentation.h\
step/segmentation/st_stepsegmentation2.h\
step/segmentation/st_stepsegmentation3.h\
step/segmentation/st_stepextractsliceabovedtm.h\
step/temp/st_stepfitmultipleDBH.h\
SimpleTree4/model/pointsimpletree.h \
SimpleTree4/model/segment.h \
SimpleTree4/model/cylinder.h \
SimpleTree4/model/tree.h \
SimpleTree4/model/build_tree/buildtree.h \
SimpleTree4/model/build_tree/improvebranchjunctions.h \
SimpleTree4/model/build_tree/improvebymedian.h \
SimpleTree4/model/build_tree/improvefit.h \
SimpleTree4/model/build_tree/removefalsecylinders.h \
SimpleTree4/model/build_tree/reordertree.h \
SimpleTree4/math/simplemath.h \
SimpleTree4/math/simplemath.hpp \
SimpleTree4/method/point_cloud_operations/stempointdetection.h \
SimpleTree4/method/point_cloud_operations/voxelgridfilter.h \
SimpleTree4/method/point_cloud_operations/enrichcloud.h \
step/st_stepenrichcloudwithcurvature.h \
step/st_stepdetectstem.h\
step/st_stepextractstem.h\
step/clustering/st_stepextractlargestcluster.h\
step/clustering/st_stepeuclideanclustering.h\
step/filtering/st_stepradiusoutlierremoval.h\
step/filtering/st_stepstatisticaloutlierremoval.h\
step/clustering/st_stepmergeclouds.h\
step/temp/st_stepfitsinglecylinder.h\
#step/st_stepabstractmodellingmt.h \
step/upanddownscaling/st_stepupscalecloud.h\
#step/st_stepmodellingfolderfromabstractmt.h \
    actions/st_actionstempointdetection.h \
    views/actions/st_actionstempointdetectionoptions.h \
    SimpleTree4/method/point_cloud_operations/clustercloud.h \
    SimpleTree4/method/geometrical_operations/circlefit.h \
    SimpleTree4/method/geometrical_operations/cylinderfit.h \
    SimpleTree4/method/point_cloud_operations/spheresurfaceextraction.h \
    SimpleTree4/method/point_cloud_operations/subdividestemandbranchpoints.h \
    SimpleTree4/method/method_coefficients.h \
    SimpleTree4/method/computedistancecylinderscloud.h \
    SimpleTree4/method/optimizationspherefollowing.h \
    SimpleTree4/method/workerspherefollowing.h \
    SimpleTree4/method/spherefollowingrecursive.h \
    SimpleTree4/model/build_tree/improvebymerge.h \
    SimpleTree4/model/build_tree/improvebyallometry.h \
    step/st_templatestep.h \
    SimpleTree4/model/build_tree/improvebypipemodel.h \
    SimpleTree4/method/point_cloud_operations/computemeanandstandarddeviation.h \
    SimpleTree4/model/build_tree/improvedbyadvancedmedian.h \
    SimpleTree4/method/computeallometry.h \
    SimpleTree4/method/workerfit.h \
    SimpleTree4/method/optimizationfit.h \
step/filtering/st_stepghostpointremoval.h\
step/upanddownscaling/st_stepvoxelgridfilter.h\
    SimpleTree4/method/optimizationdownhillsimplex.h \
    SimpleTree4/method/geometrical_operations/extractfittedpoints.h \
    SimpleTree4/method/workerdownhillsimplex.h \
    SimpleTree4/method/geometrical_operations/nlscylinderfit.h \
    SimpleTree4/method/spherefollowing2.h \
    SimpleTree4/import/readcsv.h \
    SimpleTree4/export/exporttree.h \
   # step/workerstepmt.h \
 #   step/helperformt.h \
 #   step/st_steploadsimpletreemodel.h \
    SimpleTree4/import/readtree.h \
    SimpleTree4/method/optimizationgap.h \
    SimpleTree4/method/geometrical_operations/allocatepoints.h \
    SimpleTree4/model/build_tree/improvebyattractor.h \
    SimpleTree4/method/point_cloud_operations/predictstablevolume.h \
    step/modelling/modellingthreadpool.h \
    step/modelling/structstepprameter.h \
    step/modelling/st_stepcompletefoldermodelling.h \
step/modelling/dijkstra/st_stepdjikstra.h \
    step/modelling/workermodelling.h \
    SimpleTree4/method/point_cloud_operations/convertcttost.h \
    step/modelling/predictstartparameters.h \
    Skeletonization/Dijkstra/dijkstra.h \
Skeletonization/Dijkstra/fiboheap.h \
Skeletonization/Dijkstra/fiboqueue.h \
    Skeletonization/Dijkstra/dijkstra_coefficients.h \
    SimpleTree4/method/point_cloud_operations/extractlowestclusters.h \
    SimpleTree4/method/point_cloud_operations/bincloud.h \
    Skeletonization/Dijkstra/generateskeletoncloud.h \
    Skeletonization/Dijkstra/buildtopology.h \
    item/st_tree.h \
item/st_coefficients.h \
step/export/st_stepexportmultipleclouds.h\
step/export/st_stepexportcoefficients.h\
    Skeletonization/Dijkstra/clusterandsort.h \
    SimpleTree4/export/export.h \
SimpleTree4/import/readcoeff.h \
step/import/st_importcoeff.h \
step/modelling/split/st_stepmodelwithparam1.h \
step/modelling/split/st_stepmodelwithparam2.h \
step/modelling/split/st_modellingthreadpool.h \
step/modelling/split/st_stepparameter.h \
step/modelling/split/st_workermodelling.h \
step/modelling/split/st_stepmodelmovingaverage.h \
step/gmm/st_step_eigen_ml.h \
    SimpleTree4/model/crown.h \
    item/st_crown.h \
  #  step/modelling/st_stepcomputecrown.h \
    step/clustering/st_stepbuffering.h \
step/clustering/st_stepsplitbyheight.h \
step/clustering/st_stepfilterclusters.h \
step/clustering/st_stepfilterclustersbydistance.h \
step/modelling/new/workermodelling2.h \
step/modelling/new/modellingthreadpool2.h \
step/modelling/new/predictstartparameters2.h \
step/modelling/new/st_stepcompletefoldermodelling2.h \
step/clustering/st_stepgmm.h \
step/modelling/new/structstepprameter2.h \
step/filtering/st_stepfiltergroundpoints.h\
step/filtering/st_stepclearsky.h\
step/modelling/st_stepdummyexport.h\
step/segmentation/st_stepsegmentationall.h\
    step/simpletreestep.h \
    step/segmentation/st_stepsegmentedchm.h \
    step/extract/st_extractmajorbranches.h \
    step/filtering/st_stepfilterstemseeds.h \
    step/mesh/st_step_poisson.h \
    step/modelling/new/st_stepdetectmisfits.h \
    SimpleTree4/method/geometrical_operations/detectfalsecylinders.h \
    step/extract/st_extractmajorbranchesadvancedfeature.h \
    SimpleTree4/method/geometrical_operations/stem_taper.h \
    SimpleTree4/method/computeallometry_length.h \
    step/modelling/split/st_stepmodelling_growth_length.h \
    SimpleTree4/model/build_tree/improvebyallometry_growth_length.h \
    SimpleTree4/method/computeallometry_length_RC.h \
    SimpleTree4/model/build_tree/improvebyallometry_growth_lengthRC.h \
    step/modelling/split/st_stepmodelling_growth_lengthRC.h \
    step/dtm/st_stepcomputedtm.h \
    Skeletonization/Dijkstra/dijkstrafast.h \
    step/filtering/st_stepfilterstempoints.h \
    step/export/st_stepexportallEuroSDR.h \
    step/filtering/st_stepfilterstempointscircle.h \
    step/export/st_stepexportdtm.h \
    exporter/st_ascidexporter.h \
    SimpleTree4/model/build_tree/removesidewardconnection.h \
    step/modelling/split/st_stepmodelwithpype.h \
    step/modelling/st_step_adjust_branch_order.h \
  #  step/abstract/st_stephelper.h \
  #  step/modelling/st_stepcomputecrown.h \
    step/pub2/st_stepcropbranchorder.h \
    step/pub2/st_stepincreasemindiameter.h \
    #step/st_stepabstractmodelling.h \
   # step/st_stepcompletemodelling.h \
   # step/st_stepcompletemodellingfolder.h \
  #  step/st_stepcompletemodellingfromabstract.h \
 #   step/st_stepdetecttree.h \
  #  step/st_stepmodellingfolderfromabstract.h \
    step/modelling/split/st_stepcropQSM.h \
    step/modelling/split/st_stepallometrycorrected_qsm.h \
    step/modelling/split/st_stepallometrycorrected_qsm_len.h \
    SimpleTree4/model/pointdjikstra.h \
    Skeletonization/Dijkstra/dijkstra_attractor.h \
    step/modelling/improve/st_step_detect_wrong_fits_in_qsm.h \
    SimpleTree4/method/computeallometrySave.h \
    step/diabolo_eric/st_stepcompletefoldermodelling_eric.h \
   step/diabolo_eric/workermodelling_eric.h \
    step/diabolo_eric/modellingthreadpool_eric.h \
    step/diabolo_eric/structstepprameter_eric.h \
    step/diabolo_eric/predictstartparameters_eric.h \
    step/filtering/std_out_multithread/threadpool_std_out_multithread.h \
    step/filtering/std_out_multithread/worker_thread_std_out_multithread.h \
    step/filtering/std_out_multithread/st_step_std_out_multithread.h \
    step/filtering/std_out_multithread/std_mult_param.h \
    SimpleTree4/method/point_cloud_operations/extractcloudnearmodel.h

SOURCES += \
    st_pluginentry.cpp \
    st_pluginmanager.cpp \
step/filtering/st_stepfiltergroundpoints.cpp\
step/filtering/st_stepclearsky.cpp\
step/export/st_stepexportall.cpp \
step/export/st_stepexportcoefficients.cpp\
step/temp/st_stepfitmultipleDBH.cpp\
step/segmentation/st_stepsegmentation.cpp\
step/segmentation/st_stepsegmentation2.cpp\
step/segmentation/st_stepsegmentation3.cpp\
step/segmentation/st_stepextractsliceabovedtm.cpp\
SimpleTree4/model/segment.cpp \
SimpleTree4/model/tree.cpp \
step/gmm/st_step_eigen_ml.cpp \
SimpleTree4/model/cylinder.cpp \
SimpleTree4/model/build_tree/buildtree.cpp \
SimpleTree4/model/build_tree/improvebranchjunctions.cpp \
SimpleTree4/model/build_tree/improvebymedian.cpp \
SimpleTree4/model/build_tree/improvefit.cpp \
SimpleTree4/model/build_tree/removefalsecylinders.cpp \
SimpleTree4/model/build_tree/reordertree.cpp \
step/clustering/st_stepeuclideanclustering.cpp\
SimpleTree4/math/math.cpp \
SimpleTree4/math/simplemath.cpp \
step/clustering/st_stepmergeclouds.cpp\
   SimpleTree4/method/point_cloud_operations/stempointdetection.cpp \
   SimpleTree4/method/point_cloud_operations/voxelgridfilter.cpp \
   SimpleTree4/method/point_cloud_operations/enrichcloud.cpp \
    step/st_stepenrichcloudwithcurvature.cpp \
step/export/st_stepexportmultipleclouds.cpp\
step/temp/st_stepfitsinglecylinder.cpp\
step/st_stepdetectstem.cpp\
step/st_stepextractstem.cpp\
step/modelling/dijkstra/st_stepdjikstra.cpp \
step/upanddownscaling/st_stepvoxelgridfilter.cpp\
step/clustering/st_stepgmm.cpp \
step/upanddownscaling/st_stepupscalecloud.cpp\
#step/st_stepmodellingfolderfromabstractmt.cpp \
#step/st_stepabstractmodellingmt.cpp \
step/clustering/st_stepextractlargestcluster.cpp\
step/filtering/st_stepradiusoutlierremoval.cpp\
step/filtering/st_stepghostpointremoval.cpp\
step/filtering/st_stepstatisticaloutlierremoval.cpp\
    actions/st_actionstempointdetection.cpp \
    views/actions/st_actionstempointdetectionoptions.cpp \
    SimpleTree4/method/point_cloud_operations/clustercloud.cpp \
    SimpleTree4/method/geometrical_operations/circlefit.cpp \
    SimpleTree4/method/geometrical_operations/cylinderfit.cpp \
    SimpleTree4/method/point_cloud_operations/spheresurfaceextraction.cpp \
    SimpleTree4/method/point_cloud_operations/subdividestemandbranchpoints.cpp \
    SimpleTree4/method/computedistancecylinderscloud.cpp \
    SimpleTree4/method/optimizationspherefollowing.cpp \
    SimpleTree4/method/workerspherefollowing.cpp \
    SimpleTree4/method/spherefollowingrecursive.cpp \
    SimpleTree4/model/build_tree/improvebymerge.cpp \
    SimpleTree4/model/build_tree/improvebyallometry.cpp \
    step/st_templatestep.cpp \
    SimpleTree4/model/build_tree/improvebypipemodel.cpp \
    SimpleTree4/method/point_cloud_operations/computemeanandstandarddeviation.cpp \
    SimpleTree4/model/build_tree/improvedbyadvancedmedian.cpp \
    SimpleTree4/method/computeallometry.cpp \
    SimpleTree4/method/workerfit.cpp \
    SimpleTree4/method/optimizationfit.cpp \
    SimpleTree4/method/optimizationdownhillsimplex.cpp \
    SimpleTree4/method/geometrical_operations/extractfittedpoints.cpp \
    SimpleTree4/method/workerdownhillsimplex.cpp \
    SimpleTree4/method/geometrical_operations/nlscylinderfit.cpp \
    SimpleTree4/method/spherefollowing2.cpp \
    SimpleTree4/import/readcsv.cpp \
    SimpleTree4/export/exporttree.cpp \
   # step/workerstepmt.cpp \
  #  step/helperformt.cpp \
   # step/st_steploadsimpletreemodel.cpp \
step/modelling/split/st_stepmodelmovingaverage.cpp \
    SimpleTree4/import/readtree.cpp \
    SimpleTree4/method/optimizationgap.cpp \
    SimpleTree4/method/geometrical_operations/allocatepoints.cpp \
    SimpleTree4/model/build_tree/improvebyattractor.cpp \
    SimpleTree4/method/point_cloud_operations/predictstablevolume.cpp \
    step/modelling/modellingthreadpool.cpp \
    step/modelling/st_stepcompletefoldermodelling.cpp \
    step/modelling/workermodelling.cpp \
    SimpleTree4/method/point_cloud_operations/convertcttost.cpp \
    step/modelling/predictstartparameters.cpp \
    Skeletonization/Dijkstra/dijkstra.cpp \
    SimpleTree4/method/point_cloud_operations/extractlowestclusters.cpp \
    SimpleTree4/method/point_cloud_operations/bincloud.cpp \
    Skeletonization/Dijkstra/generateskeletoncloud.cpp \
    Skeletonization/Dijkstra/buildtopology.cpp \
    item/st_tree.cpp \
item/st_coefficients.cpp \
    Skeletonization/Dijkstra/clusterandsort.cpp \
    SimpleTree4/export/export.cpp \
SimpleTree4/import/readcoeff.cpp \
step/import/st_importcoeff.cpp \
step/modelling/split/st_stepmodelwithparam1.cpp \
step/modelling/split/st_stepmodelwithparam2.cpp \
step/modelling/split/st_modellingthreadpool.cpp \
step/modelling/split/st_workermodelling.cpp \
    SimpleTree4/model/crown.cpp \
    item/st_crown.cpp \
   # step/modelling/st_stepcomputecrown.cpp \
    step/clustering/st_stepbuffering.cpp \
step/clustering/st_stepsplitbyheight.cpp \
step/clustering/st_stepfilterclusters.cpp \
step/clustering/st_stepfilterclustersbydistance.cpp \
step/modelling/new/workermodelling2.cpp \
step/modelling/new/modellingthreadpool2.cpp \
step/modelling/new/predictstartparameters2.cpp \
step/modelling/new/st_stepcompletefoldermodelling2.cpp \
step/segmentation/st_stepsegmentationall.cpp\
step/modelling/st_stepdummyexport.cpp\
    step/simpletreestep.cpp \
    step/segmentation/st_stepsegmentedchm.cpp \
    step/extract/st_extractmajorbranches.cpp \
    step/filtering/st_stepfilterstemseeds.cpp \
    step/mesh/st_step_poisson.cpp \
    step/modelling/new/st_stepdetectmisfits.cpp \
    SimpleTree4/method/geometrical_operations/detectfalsecylinders.cpp \
    step/extract/st_extractmajorbranchesadvancedfeature.cpp \
    SimpleTree4/method/geometrical_operations/stem_taper.cpp \
    SimpleTree4/method/computeallometry_length.cpp \
    step/modelling/split/st_stepmodelling_growth_length.cpp \
    SimpleTree4/model/build_tree/improvebyallometry_growth_length.cpp \
    SimpleTree4/method/computeallometry_length_RC.cpp \
    SimpleTree4/model/build_tree/improvebyallometry_growth_lengthRC.cpp \
    step/modelling/split/st_stepmodelling_growth_lengthRC.cpp \
    step/dtm/st_stepcomputedtm.cpp \
    Skeletonization/Dijkstra/dijkstrafast.cpp \
    step/filtering/st_stepfilterstempoints.cpp \
    step/export/st_stepexportallEuroSDR.cpp \
    step/filtering/st_stepfilterstempointscircle.cpp \
    step/export/st_stepexportdtm.cpp \
    exporter/st_ascidexporter.cpp \
    SimpleTree4/model/build_tree/removesidewardconnection.cpp \
    step/modelling/split/st_stepmodelwithpype.cpp \
    step/modelling/st_step_adjust_branch_order.cpp \
  #  step/abstract/st_stephelper.cpp \
  #  step/modelling/st_stepcomputecrown.cpp \
    step/pub2/st_stepcropbranchorder.cpp \
    step/pub2/st_stepincreasemindiameter.cpp \
   # step/st_stepabstractmodelling.cpp \
 #   step/st_stepcompletemodelling.cpp \
 #   step/st_stepcompletemodellingfolder.cpp \
  #  step/st_stepcompletemodellingfromabstract.cpp \
#    step/st_stepdetecttree.cpp \
  #  step/st_stepmodellingfolderfromabstract.cpp \
    step/modelling/split/st_stepcropQSM.cpp \
    step/modelling/split/st_stepallometrycorrected_qsm.cpp \
    step/modelling/split/st_stepallometrycorrected_qsm_len.cpp \
    Skeletonization/Dijkstra/dijkstra_attractor.cpp \
    step/modelling/improve/st_step_detect_wrong_fits_in_qsm.cpp \
    SimpleTree4/method/computeallometrySave.cpp \
    step/diabolo_eric/st_stepcompletefoldermodelling_eric.cpp \
    step/diabolo_eric/workermodelling_eric.cpp \
    step/diabolo_eric/modellingthreadpool_eric.cpp \
    step/diabolo_eric/predictstartparameters_eric.cpp \
    step/filtering/std_out_multithread/worker_thread_std_out_multithread.cpp \
    step/filtering/std_out_multithread/threadpool_std_out_multithread.cpp \
    step/filtering/std_out_multithread/st_step_std_out_multithread.cpp \
    SimpleTree4/method/point_cloud_operations/extractcloudnearmodel.cpp


TRANSLATIONS += languages/pluginsimpletree_en.ts \
                languages/pluginsimpletree_fr.ts

INCLUDEPATH += .
INCLUDEPATH += ./item

FORMS += \
    views/actions/st_actionstempointdetectionoptions.ui

#win32:CONFIG(release, debug|release): LIBS += -L$$PWD/'../../Computree_dependencies/PCL 1.7.2/3rdParty/Qhull/lib/' -lqhullcpp
#else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/'../../Computree_dependencies/PCL 1.7.2/3rdParty/Qhull/lib/' -lqhullcppd
#else:unix: LIBS += -L$$PWD/'../../Computree_dependencies/PCL 1.7.2/3rdParty/Qhull/lib/' -lqhullcpp

#INCLUDEPATH += $$PWD/'../../Computree_dependencies/PCL 1.7.2/3rdParty/Qhull/include'
#DEPENDPATH += $$PWD/'../../Computree_dependencies/PCL 1.7.2/3rdParty/Qhull/include'

#win32:CONFIG(release, debug|release): LIBS += -L$$PWD/'../../Computree_dependencies/PCL 1.7.2/3rdParty/Qhull/lib/' -lqhull
#else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/'../../Computree_dependencies/PCL 1.7.2/3rdParty/Qhull/lib/' -lqhulld
#else:unix: LIBS += -L$$PWD/'../../Computree_dependencies/PCL 1.7.2/3rdParty/Qhull/lib/' -lqhull

#INCLUDEPATH += $$PWD/'../../Computree_dependencies/PCL 1.7.2/3rdParty/Qhull/include'
#DEPENDPATH += $$PWD/'../../Computree_dependencies/PCL 1.7.2/3rdParty/Qhull/include'

#win32:CONFIG(release, debug|release): LIBS += -L$$PWD/'../../Computree_dependencies/PCL 1.7.2/3rdParty/Qhull/lib/' -lqhull_p
#else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/'../../Computree_dependencies/PCL 1.7.2/3rdParty/Qhull/lib/' -lqhull_pd
#else:unix: LIBS += -L$$PWD/'../../Computree_dependencies/PCL 1.7.2/3rdParty/Qhull/lib/' -lqhull_p

#INCLUDEPATH += $$PWD/'../../Computree_dependencies/PCL 1.7.2/3rdParty/Qhull/include'
#DEPENDPATH += $$PWD/'../../Computree_dependencies/PCL 1.7.2/3rdParty/Qhull/include'




#win32:CONFIG(release, debug|release): LIBS += -L'C:/Program Files/PCL 1.7.2/3rdParty/Qhull/lib/' -lqhullstatic
#else:win32:CONFIG(debug, debug|release): LIBS += -L'C:/Program Files/PCL 1.7.2/3rdParty/Qhull/lib/' -lqhullstaticd
#else:unix: LIBS += -L'C:/Program Files/PCL 1.7.2/3rdParty/Qhull/lib/' -lqhullstatic

#INCLUDEPATH += 'C:/Program Files/PCL 1.7.2/3rdParty/Qhull/include'
#DEPENDPATH += 'C:/Program Files/PCL 1.7.2/3rdParty/Qhull/include'

#win32:CONFIG(release, debug|release): LIBS += -L'C:/Program Files/PCL 1.7.2/3rdParty/Qhull/lib/' -lqhullstatic_p
#else:win32:CONFIG(debug, debug|release): LIBS += -L'C:/Program Files/PCL 1.7.2/3rdParty/Qhull/lib/' -lqhullstatic_pd
#else:unix: LIBS += -L'C:/Program Files/PCL 1.7.2/3rdParty/Qhull/lib/' -lqhullstatic_p

#INCLUDEPATH += 'C:/Program Files/PCL 1.7.2/3rdParty/Qhull/include'
#DEPENDPATH += 'C:/Program Files/PCL 1.7.2/3rdParty/Qhull/include'

DISTFILES += \
    ../../../ComputreeST_5/log_onf.log
