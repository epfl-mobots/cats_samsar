#ifndef DENSESTPOINT_HPP
#define DENSESTPOINT_HPP

#include "GenericFishModel.hpp"

class DensestPoint : public GenericFishModel {
    Q_OBJECT
public:
    //! Constructor.
    explicit DensestPoint(FishBot* robot);

private slots:
    //! Sets the model parameters from the settings.
    virtual void updateModelParameters() override;

protected:
    //! Initializes the model based on the setup map and parameters.
    virtual void resetModel() override;
};

#endif // DENSESTPOINT_HPP
