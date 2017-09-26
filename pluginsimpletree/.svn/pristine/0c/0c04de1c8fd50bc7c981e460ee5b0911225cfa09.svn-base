#ifndef ST_StepCompleteModellingFROMABSTRACT_H
#define ST_StepCompleteModellingFROMABSTRACT_H

#include "step/st_stepabstractmodelling.h"
#include <QStandardPaths>
/*!
 * \class ST_StepStemPointDetection
 * \ingroup Steps_ST
 * \brief <b>detects stem points.</b>
 *
 * No detailled description for this step
 *
 * \param _param1
 *
 */

class ST_StepCompleteModellingFromAbstract: public ST_StepAbstractModelling
{
    Q_OBJECT

public:

    /*! \brief Step constructor
     *
     * Create a new instance of the step
     *
     * \param dataInit Step parameters object
     */
    ST_StepCompleteModellingFromAbstract(CT_StepInitializeData &dataInit);

    ~ST_StepCompleteModellingFromAbstract();

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

    /*! \brief Step URL
     *
     * Return a URL of a wiki for this step
     */
    QString getStepURL() const;

    /*! \brief Step copy
     *
     * Step copy, used when a step is added by step contextual menu
     */
    CT_VirtualAbstractStep* createNewInstance(CT_StepInitializeData &dataInit);

protected:

    bool _leave_removed;
    bool _is_high_quality;

    /*! \brief Parameters DialogBox
     *
     * DialogBox asking for step parameters
     */
    void createPostConfigurationDialog();


    /*! \brief Algorithm of the step
     *
     * Step computation, using input results, and creating output results
     */
    void compute();












};

#endif // ST_StepCompleteModellingFROMABSTRACT_H

