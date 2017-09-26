#ifndef ST_STEPMODELLINGFOLDERFROMABSTRACTMT_H
#define ST_STEPMODELLINGFOLDERFROMABSTRACTMT_H

#include "st_stepabstractmodellingmt.h"

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




class ST_StepModellingFolderFromAbstractMT: public ST_StepAbstractModellingMT
{
    Q_OBJECT

public:

    /*! \brief Step constructor
     *
     * Create a new instance of the step
     *
     * \param dataInit Step parameters object
     */
    ST_StepModellingFolderFromAbstractMT(CT_StepInitializeData &dataInit);

    ~ST_StepModellingFolderFromAbstractMT();

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






};

#endif // ST_STEPMODELLINGFOLDERFROMABSTRACTMT_H

