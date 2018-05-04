#ifndef SOCIALFISHCONTROLMODE_HPP
#define SOCIALFISHCONTROLMODE_HPP

#include "GenericFishModel.hpp"

class SocialFishControlMode : public GenericFishModel {
    Q_OBJECT
public:
    //! Constructor.
    explicit SocialFishControlMode(FishBot* robot);

private slots:
    //! Sets the model parameters from the settings.
    virtual void updateModelParameters() override;

protected:
    //! Initializes the model based on the setup map and parameters.
    virtual void resetModel() override;
};

#endif // SOCIALFISHCONTROLMODE_HPP
