#ifndef TOULOUSECONTROLMODE_HPP
#define TOULOUSECONTROLMODE_HPP

#include "FishModelBase.hpp"

class ToulouseControlMode : public FishModelBase  {
    Q_OBJECT
public:
    //! Constructor.
    explicit ToulouseControlMode(FishBot* robot);

    //! The step of the control mode.
    virtual ControlTargetPtr step() override;

    //! Informs on what kind of control targets this control mode generates.
    virtual QList<ControlTargetType> supportedTargets() override;

private slots:
    //! Sets the model parameters from the settings.
    virtual void updateModelParameters() override;

protected:
    //! Initializes the model based on the setup map and parameters.
    virtual void resetModel() override;
};

#endif // SOCIALFISHCONTROLMODE_HPP
