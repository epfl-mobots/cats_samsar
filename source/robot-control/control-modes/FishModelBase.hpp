#ifndef CATS2_FISH_MODEL_BASE_HPP
#define CATS2_FISH_MODEL_BASE_HPP

#include "ControlMode.hpp"
#include "SetupMap.hpp"
#include "ModelParameters.hpp"
#include "navigation/GridBasedMethod.hpp"
#include <model/model.hpp>

#include <AgentState.hpp>
#include <Timer.hpp>

#include <memory>

/*!
 * Common parent for controllers following the fish models.
 */
class FishModelBase : public ControlMode, public GridBasedMethod
{
    Q_OBJECT
public:
    //! Constructor.
    explicit FishModelBase(FishBot* robot, ControlModeType::Enum type);
    //! Destructor.
    virtual ~FishModelBase() override;

    //! Called when the control mode is activated. Used to reset mode's parameters.
    virtual void start() override;
    //! The step of the control mode.
    virtual ControlTargetPtr step() override;

    //! Informs on what kind of control targets this control mode generates.
    virtual QList<ControlTargetType> supportedTargets() override;

    //! Sets the model's parameters. Every time the parameters are changed, the
    //! model is reset.
    void setParameters(ModelParameters parameters);

protected slots:
    //! Sets the model parameters from the settings.
    virtual void updateModelParameters() = 0;

protected:
    //! Initializes the model based on the setup map and parameters.
    virtual void resetModel() = 0;

    //! Computes the target position from the model.
    PositionMeters computeTargetPosition();

protected:
    //! The model related data.
    std::unique_ptr<Fishmodel::Arena> m_arena;
    std::unique_ptr<Fishmodel::Simulation> m_sim;

private:
    //! The resolution of the setup matrix.
    static constexpr double ModelResolutionM = 0.005; // i.e. 5 mm

    //! The parameters of the model.
    ModelParameters m_parameters;

    //! The robot's target as generated by the model.
    PositionMeters m_targetPosition;
    //! The target update timer. The model is updated at it's own frequency
    //! defined by FishModelSettings::dt.
    Timer m_targetUpdateTimer;
};

#endif // CATS2_GENERIC_FISH_MODEL_HPP
