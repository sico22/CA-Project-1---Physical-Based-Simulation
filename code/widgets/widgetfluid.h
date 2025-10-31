#ifndef WIDGETFLUID_H
#define WIDGETFLUID_H

#include <QWidget>

namespace Ui {
class WidgetFluid;
}

class WidgetFluid : public QWidget
{
    Q_OBJECT

public:
    explicit WidgetFluid(QWidget *parent = nullptr);
    ~WidgetFluid();

    // ---- Getters for SPH parameters ----
    double getGravity() const;
    double getRestDensity() const;
    double getStiffness() const;
    double getViscosity() const;
    double getSmoothingRadius() const;
    double getParticleMass() const;

signals:
    void updatedParameters();

private slots:
    void onUpdateButtonClicked();

private:
    Ui::WidgetFluid *ui;
};

#endif // WIDGETFLUID_H
