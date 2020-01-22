#ifndef TOULOUSECONTROLMODE_HPP
#define TOULOUSECONTROLMODE_HPP

#include "FishModelBase.hpp"

class ToulouseControlMode : public FishModelBase  {
    Q_OBJECT
public:
    //! Constructor.
    explicit ToulouseControlMode(FishBot* robot, QList<FishBot*> robots = QList<FishBot*>());

    //! Called when the control mode is activated. Used to reset mode's parameters.
    virtual void start() override;
    //! The step of the control mode.
    virtual ControlTargetPtr step() override;

    //! Informs on what kind of control targets this control mode generates.
    virtual QList<ControlTargetType> supportedTargets() override;

    //! Returns a pointer to the simulator of this control mode.
    Fishmodel::Simulation* simulation() { return m_sim; }

private slots:
    //! Sets the model parameters from the settings.
    virtual void updateModelParameters() override;

protected:
    //! Initializes the model based on the setup map and parameters.
    virtual void resetModel() override;

private:
    //! List of preceding connected robots.
    QList<FishBot*> m_robots;
};

#endif // SOCIALFISHCONTROLMODE_HPP
