#ifndef CATS2_TRACKING_SETUP_HPP
#define CATS2_TRACKING_SETUP_HPP

#include "TrackingHandler.hpp"

#include <CommonTypes.hpp>
#include <VideoGrabber.hpp>

#include <QtCore/QSharedPointer>
#include <QtCore/QMap>

/*!
 * \brief The TrackingSetup class corresponds to a arena tracking procedures, it brings together
 * a video grabber attached to a tracker and sends out the detected agents' positions converted to
 * world coordinates by using a corresponding coordinates transformation.
 */
class TrackingSetup
{
public:
    //! Constructor.
    TrackingSetup(SetupType type);

public:
    //! Provides the configuration file section name that corresponds to given setup type.
    static QString setupSettingsNameByType(SetupType type)
    {
        return m_setupTypeSettingsName.value(type);  // if the type is not in the map then an
                                                    // empty string is returned
    }

private:
    //! Links the setup type with the corresponding section name in the configuration file.
    static QMap<SetupType, QString> m_setupTypeSettingsName;

private:
    //! The type of setup, for instance, main camera or the camera below.
    SetupType m_type;

    //! The grabber.
    QSharedPointer<VideoGrabber> m_grabber;
    //! The tracker.
    QSharedPointer<TrackingHandler> m_tracking;


    // TODO : add Grabber / Viewer / Tracker / CoordinateTransformation instance here, the are to be initialized from the settings based on the given setup type
};

#endif // CATS2_TRACKING_SETUP_HPP

