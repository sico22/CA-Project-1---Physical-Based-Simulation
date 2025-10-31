#include "widgetfluid.h"
#include "ui_widgetfluid.h"

WidgetFluid::WidgetFluid(QWidget *parent)
    : QWidget(parent),
    ui(new Ui::WidgetFluid)
{
    ui->setupUi(this);

    // Connect Update button to slot
    connect(ui->btnUpdate, &QPushButton::clicked,
            this, &WidgetFluid::onUpdateButtonClicked);
}

WidgetFluid::~WidgetFluid()
{
    delete ui;
}

// ---- Slot ----
void WidgetFluid::onUpdateButtonClicked()
{
    emit updatedParameters();  // notify scene that parameters changed
}

// ---- Getters ----
double WidgetFluid::getGravity() const {
    return ui->doubleSpinBoxGravity->value();
}

double WidgetFluid::getRestDensity() const {
    return ui->doubleSpinBoxDensity->value();
}

double WidgetFluid::getStiffness() const {
    return ui->doubleSpinBoxStiffness->value();
}

double WidgetFluid::getViscosity() const {
    return ui->doubleSpinBoxViscosity->value();
}

double WidgetFluid::getSmoothingRadius() const {
    return ui->doubleSpinBoxRadius->value();
}

double WidgetFluid::getParticleMass() const {
    return ui->doubleSpinBoxMass->value();
}
