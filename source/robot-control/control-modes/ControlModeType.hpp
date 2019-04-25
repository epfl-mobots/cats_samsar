#ifndef CATS2_CONTROL_MODE_TYPE_HPP
#define CATS2_CONTROL_MODE_TYPE_HPP

#include <QtCore/QString>

/*!
 * Provides an enum for control mode type.
 */
class ControlModeType {
public:
    enum Enum {
        IDLE,
        MANUAL,
        GO_TO_POSITION,
        GO_STRAIGHT,
        FISH_MODEL,
        FISH_MODEL_WITH_WALLS,
        ZONE_BASED_FISH_MODEL,
        TRAJECTORY,
        WHEEL_VELOCITIES,
        POSITION_LISTENER,
        TOULOUSE_MODE,
        FOLLOW_GROUP,
        SOCIAL_FISH_MODEL,
        DENSEST_POINT,
        UNDEFINED
    };

    //! Gets the type of the control mode from the settings' string.
    static Enum fromSettingsString(QString modeName)
    {
        if (modeName.toLower() == "idle")
            return IDLE;
        else if (modeName.toLower() == "manual")
            return MANUAL;
        else if (modeName.toLower() == "gotoposition")
            return GO_TO_POSITION;
        else if (modeName.toLower() == "gostraight")
            return GO_STRAIGHT;
        else if (modeName.toLower() == "fishmodel")
            return FISH_MODEL;
        else if (modeName.toLower() == "fishmodelwithwalls")
            return FISH_MODEL_WITH_WALLS;
        else if (modeName.toLower() == "zonebasedfishmodel")
            return ZONE_BASED_FISH_MODEL;
        else if (modeName.toLower() == "trajectory")
            return TRAJECTORY;
        else if (modeName.toLower() == "wheelvelocities")
            return WHEEL_VELOCITIES;
        else if (modeName.toLower() == "postionslistener")
            return POSITION_LISTENER;
        else if (modeName.toLower() == "followgroup")
            return FOLLOW_GROUP;
        else if (modeName.toLower() == "socialfishmodel")
            return SOCIAL_FISH_MODEL;
        else if (modeName.toLower() == "toulousemodel")
            return TOULOUSE_MODE;
        else if (modeName.toLower() == "densestpoint")
            return DENSEST_POINT;
        else
            return UNDEFINED;
    }

    //! Returns that control mode type string in human friendly format.
    static QString toString(Enum controlModeType)
    {
        QString string;

        switch (controlModeType) {
        case MANUAL:
            string = "Manual";
            break;
        case GO_TO_POSITION:
            string = "Go to position";
            break;
        case GO_STRAIGHT:
            string = "Go straight";
            break;
        case FISH_MODEL:
            string = "Fish model";
            break;
        case FISH_MODEL_WITH_WALLS:
            string = "Fish model with walls";
            break;
        case ZONE_BASED_FISH_MODEL:
            string = "Zone based fish model";
            break;
        case TRAJECTORY:
            string = "Trajectory";
            break;
        case WHEEL_VELOCITIES:
            string = "Wheel Velocities";
            break;
        case POSITION_LISTENER:
            string = "Position Listener";
            break;
        case FOLLOW_GROUP:
            string = "Follow group";
            break;
        case SOCIAL_FISH_MODEL:
            string = "Social Fish Model";
            break;
        case TOULOUSE_MODE:
            string = "Toulouse Model";
            break;
        case DENSEST_POINT:
            string = "Densest Point";
            break;
        case IDLE:
        default:
            string = "Idle";
            break;
        }
        return string;
    }
};

#endif // CATS2_CONTROL_MODE_TYPE_HPP
