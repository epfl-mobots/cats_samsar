#include "GridBasedMethod.hpp"

#include "settings/RobotControlSettings.hpp"

#include <opencv2/video.hpp>
#include <opencv2/video/background_segm.hpp>

/*!
 * Constructor.
 */
GridBasedMethod::GridBasedMethod(double gridSizeMeters) :
    m_setupMap(RobotControlSettings::get().setupMap()),
    m_gridSizeMeters(gridSizeMeters)
{

}

/*!
 * Computes the grid node point from the world position.
 */
QPoint GridBasedMethod::positionToGridNode(PositionMeters position) const
{
    QPoint point;
    point.setX(floor((position.x() - m_setupMap.minX()) / m_gridSizeMeters + 0.5));
    point.setY(floor((position.y() - m_setupMap.minY()) / m_gridSizeMeters + 0.5));
    return point;
}

/*!
 * Computes the world position corresponding to the grid node point.
 */
PositionMeters GridBasedMethod::gridNodeToPosition(QPoint gridNode) const
{
     PositionMeters position;
     position.setX(gridNode.x() * m_gridSizeMeters + m_setupMap.minX());
     position.setY(gridNode.y() * m_gridSizeMeters + m_setupMap.minY());
     return position;
}

/*!
 * Returns the grid matrix corresponding to the setup map.
 */
cv::Mat GridBasedMethod::generateGrid()
{
    if (m_setupMap.isValid()) {
        // build the matrix
        int cols = floor((m_setupMap.maxX() - m_setupMap.minX()) / m_gridSizeMeters + 0.5);
        int rows = floor((m_setupMap.maxY() - m_setupMap.minY()) / m_gridSizeMeters + 0.5);
        if ((cols > 0) && (rows > 0)) {
            // a matrix representing the setup
            cv::Mat arenaMatrix(rows, cols, CV_8U);
            // fill the matrix
            double y = m_setupMap.minY();
            for (int row = 0; row < rows; ++row) { // rows go from min_y up to max_y
                double x = m_setupMap.minX();
                for (int col = 0; col < cols; ++col) { // cols go from min_x right to max_x
                    if (m_setupMap.containsPoint(PositionMeters(x, y)))
                        arenaMatrix.at<uchar>(row, col) = static_cast<int>(FREE);
                    else
                        arenaMatrix.at<uchar>(row, col) = static_cast<int>(OCCUPIED);
                    x += m_gridSizeMeters;
                }
                y += m_gridSizeMeters;
            }
            return arenaMatrix;
        }
    }
    return cv::Mat();
}
