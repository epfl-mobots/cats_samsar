#ifndef CATS2_TRACKING_ROUTINE_TYPE_HPP
#define CATS2_TRACKING_ROUTINE_TYPE_HPP

#include <QtCore/QString>

/*!
 * \brief All types of tracking routines that can be used.
 */
class TrackingRoutineType
{
public:
    enum Enum {
        BLOB_DETECTOR,
        // to extend
        UNDEFINED
    };

    //! Gets the type of the tracking routine from the settings' string.
    static TrackingRoutineType::Enum fromSettingsString(QString routineName)
    {
        if (routineName.toLower() == "blobdetector")
            return TrackingRoutineType::BLOB_DETECTOR;
        else
            return TrackingRoutineType::UNDEFINED;
    }

    //! Returns the tracking routine type string in human friendly format.
    static QString toString(Enum setupType) {
        QString string = "Undefined";

        switch (setupType) {
        case BLOB_DETECTOR:
            string = "Basic blob detection";
            break;
        default:
            break;
        }
        return string;
    }
};

#endif // CATS2_TRACKING_ROUTINE_TYPE_HPP
