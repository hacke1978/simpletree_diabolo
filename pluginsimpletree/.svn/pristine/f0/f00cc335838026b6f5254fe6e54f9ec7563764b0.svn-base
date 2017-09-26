#ifndef ST_STEPMODELLINGFOLDERFROMABSTRACT_H
#define ST_STEPMODELLINGFOLDERFROMABSTRACT_H

#include "st_stepabstractmodelling.h"

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

class ST_StepModellingFolderFromAbstract: public ST_StepAbstractModelling
{
    Q_OBJECT

public:

    /*! \brief Step constructor
     *
     * Create a new instance of the step
     *
     * \param dataInit Step parameters object
     */
    ST_StepModellingFolderFromAbstract(CT_StepInitializeData &dataInit);

    ~ST_StepModellingFolderFromAbstract();

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
    virtual CT_VirtualAbstractStep* createNewInstance(CT_StepInitializeData &dataInit);

protected:

//    /*! \brief Input results specification
//     *
//     * Specification of input results models needed by the step (IN)
//     */
//    void createInResultModelListProtected();

    /*! \brief Parameters DialogBox
     *
     * DialogBox asking for step parameters
     */
    virtual void createPostConfigurationDialog();

//    /*! \brief Output results specification
//     *
//     * Specification of output results models created by the step (OUT)
//     */
//    void createOutResultModelListProtected();

    /*! \brief Algorithm of the step
     *
     * Step computation, using input results, and creating output results
     */
    virtual void compute();





private:






};

#endif // ST_STEPMODELLINGFOLDERFROMABSTRACT_H

