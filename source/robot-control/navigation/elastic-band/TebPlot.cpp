/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph Rösmann
 * Adaptation: Mickaël Salamin, Fall 2019
 * Implementation: ROS package teb_local_planner
 * Source: https://github.com/rst-tu-dortmund/teb_local_planner/
 *********************************************************************/

#include <elastic-band/TebPlot.hpp>

#include <QtCharts/QChart>
#include <QtCharts/QChartView>
#include <QtCharts/QValueAxis>
#include <QtCharts/QLegendMarker>
#include <QtCharts/QScatterSeries>
#include <QtCharts/QLineSeries>
#include <QtCharts/QXYSeries>

namespace elastic_band
{

QT_CHARTS_USE_NAMESPACE

QMainWindow* TebPlot::closeWindow(QMainWindow* window)
{
    if (window != nullptr) {
        window->setAttribute(Qt::WA_DeleteOnClose);
        window->close();
        window = nullptr;
    }
    return window;
}

QMainWindow* TebPlot::plotPath(const Trajectory* trajectory, QMainWindow* window, const QString title,
                               const QPair<double, double> range, const QPair<int, int> size)
{
    if (trajectory == nullptr) {
        return closeWindow(window);
    }

    QValueAxis* axisX = new QValueAxis();
    axisX->setTitleText("X [m]");
    axisX->setLabelFormat("%g");
    axisX->setTickCount(6);

    QValueAxis* axisY = new QValueAxis();
    axisY->setTitleText("Y [m]");
    axisY->setLabelFormat("%g");
    axisY->setTickCount(6);

    QList<QString> legends;
    legends.append(QString("Start"));
    legends.append(QString("Goal"));
    legends.append(QString("Position"));

    QList<QXYSeries*> data;
    for (size_t i = 0; i < static_cast<size_t>(legends.size()); i++) {
        QXYSeries* series;
        if (i == 0) {
            series = new QScatterSeries();
            series->append(trajectory->trajectory().front()->pose().x(), trajectory->trajectory().front()->pose().y());
        } else if (i == 1) {
            series = new QScatterSeries();
            series->append(trajectory->trajectory().back()->pose().x(), trajectory->trajectory().back()->pose().y());
        } else {
            series = new QLineSeries();
            for (size_t j = 0; j < static_cast<size_t>(trajectory->trajectory().size()); j++) {
                series->append(trajectory->trajectory().at(j)->pose().x(), trajectory->trajectory().at(j)->pose().y());
            }
        }
        series->setName(legends.at(i));
        data.append(series);
    }

    QChart* chart = new QChart();
    for (size_t i = 0; i < static_cast<size_t>(data.size()); i++) {
        chart->addSeries(data.at(i));
    }
    chart->setAxisX(axisX);
    chart->setAxisY(axisY);
    chart->axisX()->setRange(range.first, range.second);
    chart->axisY()->setRange(range.first, range.second);
    for (size_t i = 0; i < static_cast<size_t>(data.size()); i++) {
        data.at(i)->attachAxis(chart->axisX());
        data.at(i)->attachAxis(chart->axisY());
    }
    chart->legend()->setVisible(true);
    chart->legend()->setAlignment(Qt::AlignBottom);
    //chart->legend()->setMarkerShape(QLegend::MarkerShapeFromSeries); // Introduced in Qt 5.9
    chart->setTitle(title);
    if (!title.isEmpty()) {
        QFont font = chart->titleFont();
        int size = font.pointSize();
        font.setPointSize(++size);
        font.setBold(true);
        chart->setTitleFont(font);
    }

    QChartView* view = new QChartView(chart);
    view->setRenderHint(QPainter::Antialiasing);
    view->setMinimumSize(size.first, size.second);
    view->setMaximumSize(size.first, size.second);

    if (window == nullptr) {
        window = new QMainWindow();
    }
    window->setCentralWidget(view);
    window->show();

    return window;
}

QMainWindow* TebPlot::plotProfile(const Trajectory* trajectory, QMainWindow* window, const QString title,
                                  const QList<QVector<double>> data, const QList<QString> legends,
                                  const QList<bool> limited, const QList<double> limits, const QList<size_t> links,
                                  const QString axisY, const QString axisX, const QPair<int, int> size)
{
    if (trajectory == nullptr || data.isEmpty() || legends.isEmpty()) {
        return closeWindow(window);
    }

    QList<QLineSeries*> lines;
    for (size_t idx = 0, i = 0; i < static_cast<size_t>(legends.size()); i++) {
        QLineSeries* series = new QLineSeries();
        if (static_cast<size_t>(limited.size()) > i && limited.at(i)) {
            const size_t idxMin = 0;
            const size_t idxMax = data.size() - 1;
            if (trajectory->trajectory().size() > idxMax && static_cast<size_t>(limits.size()) > i) {
                series->append(std::chrono::duration_cast<std::chrono::milliseconds>(trajectory->trajectory().at(idxMin)->timestamp()).count(), limits.at(i));
                series->append(std::chrono::duration_cast<std::chrono::milliseconds>(trajectory->trajectory().at(idxMax)->timestamp()).count(), limits.at(i));
            }
        } else {
            for (size_t j = 0; j < static_cast<size_t>(data.size()); j++) {
                if (trajectory->trajectory().size() > j && static_cast<size_t>(data.at(j).size()) > idx) {
                    series->append(std::chrono::duration_cast<std::chrono::milliseconds>(trajectory->trajectory().at(j)->timestamp()).count(), data.at(j).at(idx));
                }
            }
            idx++;
        }
        series->setName(legends.at(i));
        lines.append(series);
    }

    QChart* chart = new QChart();
    for (size_t i = 0; i < static_cast<size_t>(lines.size()); i++) {
        chart->addSeries(lines.at(i));
        if (static_cast<size_t>(limited.size()) > i && limited.at(i)) {
            QPen pen = lines.at(i)->pen();
            pen.setStyle(Qt::DotLine);
            lines.at(i)->setPen(pen);
            if (static_cast<size_t>(links.size()) > i) {
                QColor color = lines.at(links.at(i))->color();
                lines.at(i)->setColor(color);
                qreal opacity = lines.at(links.at(i))->opacity();
                lines.at(i)->setOpacity(opacity / 2);
                QList<QLegendMarker*> markers = chart->legend()->markers(lines.at(i));
                for (size_t j = 0; j < static_cast<size_t>(markers.size()); j++) {
                    QBrush brush = markers.at(j)->brush();
                    brush.setStyle(Qt::Dense4Pattern);
                    markers.at(j)->setBrush(brush);
                }
            }
        }
    }
    chart->createDefaultAxes();
    chart->axisX()->setTitleText(axisX);
    chart->axisY()->setTitleText(axisY);
    chart->legend()->setVisible(true);
    chart->legend()->setAlignment(Qt::AlignBottom);
    //chart->legend()->setMarkerShape(QLegend::MarkerShapeFromSeries); // Introduced in Qt 5.9
    chart->setTitle(title);
    if (!title.isEmpty()) {
        QFont font = chart->titleFont();
        int size = font.pointSize();
        font.setPointSize(++size);
        font.setBold(true);
        chart->setTitleFont(font);
    }

    QChartView* view = new QChartView(chart);
    view->setRenderHint(QPainter::Antialiasing);

    if (window == nullptr) {
        window = new QMainWindow();
    }
    window->setCentralWidget(view);
    window->resize(size.first, size.second);
    window->show();

    return window;
}

QMainWindow* TebPlot::plotProfilePose(const Trajectory* trajectory, QMainWindow* window, const QString title)
{
    if (trajectory == nullptr) {
        return closeWindow(window);
    }

    QList<QString> legends;
    legends.append(QString("Position X"));
    legends.append(QString("Position Y"));
    legends.append(QString("Orientation"));

    QList<bool> limited = {false, false, false};
    QList<double> limits = {0, 0, 0};
    QList<size_t> links = {0, 1, 2};
    QString axis = QString("Pose [m]&[rad]");

    QList<QVector<double>> data;
    const size_t dimension = limited.count(false);
    const size_t size = trajectory->trajectory().size();
    data.reserve(size);
    std::fill_n(std::back_inserter(data), size, QVector<double>(dimension));
    for (size_t i = 0; i < size; i++) {
        for (size_t j = 0; j < dimension; j++) {
            data[i][j] = trajectory->trajectory().at(i)->pose().pose3d()[j];
        }
    }

    return plotProfile(trajectory, window, title, data, legends, limited, limits, links, axis);
}

QMainWindow* TebPlot::plotProfileSpeed(const Trajectory* trajectory, QMainWindow* window, const QString title)
{
    if (trajectory == nullptr) {
        return closeWindow(window);
    }

    QList<QString> legends;
    legends.append(QString("Left wheel speed"));
    legends.append(QString("Right wheel speed"));

    QList<bool> limited = {false, false};
    QList<double> limits = {0, 0};
    QList<size_t> links = {0, 1};
    QString axis = QString("Speed [rad/s]");

    QList<QVector<double>> data;
    const size_t dimension = limited.count(false);
    const size_t size = trajectory->trajectory().size() - 1;
    data.reserve(size);
    std::fill_n(std::back_inserter(data), size, QVector<double>(dimension));
    for (size_t i = 0; i < size; i++) {
        for (size_t j = 0; j < dimension; j++) {
            data[i][j] = trajectory->trajectory().at(i)->velocity().wheel()[j];
        }
    }

    return plotProfile(trajectory, window, title, data, legends, limited, limits, links, axis);
}

QMainWindow* TebPlot::plotProfileVelocity(const Trajectory* trajectory, QMainWindow* window, const QString title)
{
    if (trajectory == nullptr) {
        return closeWindow(window);
    }

    QList<QString> legends;
    legends.append(QString("Linear velocity"));
    legends.append(QString("Angular velocity"));
    legends.append(QString("Min. lin. vel."));
    legends.append(QString("Max. lin. vel."));
    legends.append(QString("Min. ang. vel."));
    legends.append(QString("Max. ang. vel."));

    QList<bool> limited = {false, false, true, true, true, true};
    QList<double> limits = {0, 0, -30, +30, -18, +18};
    QList<size_t> links = {0, 1, 0, 0, 1, 1};
    QString axis = QString("Velocity [cm/s]&[rad/s]");

    QList<QVector<double>> data;
    const size_t dimension = limited.count(false);
    const size_t size = trajectory->trajectory().size() - 1;
    data.reserve(size);
    std::fill_n(std::back_inserter(data), size, QVector<double>(dimension));
    for (size_t i = 0; i < size; i++) {
        for (size_t j = 0; j < dimension; j++) {
            data[i][j] = trajectory->trajectory().at(i)->velocity().velocity2d()[j];
            if (j == 0) {
                data[i][j] *= 100;
            }
        }
    }

    return plotProfile(trajectory, window, title, data, legends, limited, limits, links, axis);
}

QMainWindow* TebPlot::plotProfileAcceleration(const Trajectory* trajectory, QMainWindow* window, const QString title)
{
    if (trajectory == nullptr) {
        return closeWindow(window);
    }

    QList<QString> legends;
    legends.append(QString("Linear acceleration"));
    legends.append(QString("Angular acceleration"));
    legends.append(QString("Min. lin. acc."));
    legends.append(QString("Max. lin. acc."));
    legends.append(QString("Min. ang. acc."));
    legends.append(QString("Max. ang. acc."));

    QList<bool> limited = {false, false, true, true, true, true};
    QList<double> limits = {0, 0, -130, +130, -80, +80};
    QList<size_t> links = {0, 1, 0, 0, 1, 1};
    QString axis = QString("Acceleration [cm/s^2]&[rad/s^2]");

    QList<QVector<double>> data;
    const size_t dimension = limited.count(false);
    const size_t size = trajectory->trajectory().size() - 2;
    data.reserve(size);
    std::fill_n(std::back_inserter(data), size, QVector<double>(dimension));
    for (size_t i = 0; i < size; i++) {
        for (size_t j = 0; j < dimension; j++) {
            data[i][j] = trajectory->trajectory().at(i)->acceleration().acceleration2d()[j];
            if (j == 0) {
                data[i][j] *= 100;
            }
        }
    }

    return plotProfile(trajectory, window, title, data, legends, limited, limits, links, axis);
}

} // namespace elastic_band
