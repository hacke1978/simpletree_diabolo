#include "st_actionstempointdetectionoptions.h"
#include "ui_st_actionstempointdetectionoptions.h"

#include "actions/st_actionstempointdetection.h"

#include <QColorDialog>

ST_ActionStemPointDetectionOptions::ST_ActionStemPointDetectionOptions(const ST_ActionStemPointDetection *action) :
    CT_GAbstractActionOptions(action),
    ui(new Ui::ST_ActionStemPointDetectionOptions())
{
    ui->setupUi(this);

        connect(ui->eigen_1_min, SIGNAL(valueChanged(double)), this, SIGNAL(parametersChanged()));
        connect(ui->eigen_1_max, SIGNAL(valueChanged(double)), this, SIGNAL(parametersChanged()));
        connect(ui->eigen_2_min, SIGNAL(valueChanged(double)), this, SIGNAL(parametersChanged()));
        connect(ui->eigen_2_max, SIGNAL(valueChanged(double)), this, SIGNAL(parametersChanged()));
        connect(ui->eigen_3_min, SIGNAL(valueChanged(double)), this, SIGNAL(parametersChanged()));
        connect(ui->eigen_3_max, SIGNAL(valueChanged(double)), this, SIGNAL(parametersChanged()));

        ui->eigen_1_min->setToolTip(tr ("min Value for Eigenvalue 1") );
        ui->eigen_1_min->setToolTip(tr ("max Value for Eigenvalue 1") );
        ui->eigen_1_min->setToolTip(tr ("min Value for Eigenvalue 2") );
        ui->eigen_1_min->setToolTip(tr ("max Value for Eigenvalue 2") );
        ui->eigen_1_min->setToolTip(tr ("min Value for Eigenvalue 3") );
        ui->eigen_1_min->setToolTip(tr ("max Value for Eigenvalue 3") );

}

ST_ActionStemPointDetectionOptions::~ST_ActionStemPointDetectionOptions()
{
    delete ui;
}

double ST_ActionStemPointDetectionOptions::get_eigen_1_min() const
{
    return ui->eigen_1_min->value();
}

double ST_ActionStemPointDetectionOptions::get_eigen_1_max() const
{
    return ui->eigen_1_max->value();
}

double ST_ActionStemPointDetectionOptions::get_eigen_2_min() const
{
    return ui->eigen_2_min->value();
}

double ST_ActionStemPointDetectionOptions::get_eigen_2_max() const
{
    return ui->eigen_2_max->value();
}

double ST_ActionStemPointDetectionOptions::get_eigen_3_min() const
{
    return ui->eigen_3_min->value();
}

double ST_ActionStemPointDetectionOptions::get_eigen_3_max() const
{
    return ui->eigen_3_max->value();
}

void ST_ActionStemPointDetectionOptions::set_eigen_1_min(double x)
{
    ui->eigen_1_min->setValue(x);
}

void ST_ActionStemPointDetectionOptions::set_eigen_1_max(double x)
{
    ui->eigen_1_max->setValue(x);
}

void ST_ActionStemPointDetectionOptions::set_eigen_2_min(double x)
{
    ui->eigen_2_min->setValue(x);
}

void ST_ActionStemPointDetectionOptions::set_eigen_2_max(double x)
{
    ui->eigen_2_max->setValue(x);
}

void ST_ActionStemPointDetectionOptions::set_eigen_3_min(double x)
{
    ui->eigen_3_min->setValue(x);
}

void ST_ActionStemPointDetectionOptions::set_eigen_3_max(double x)
{
    ui->eigen_3_max->setValue(x);
}
