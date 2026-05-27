#include "emat_thickness_gauge/waveform_widget.h"

#include <QPainter>
#include <QPen>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <ros/ros.h>

namespace emat {

WaveformWidget::WaveformWidget(QWidget* parent)
    : QWidget(parent)
{
    setMinimumSize(400, 200);
    setWindowTitle("EMAT Waveform Viewer");
    resize(800, 400);
    setAutoFillBackground(false);

    _refresh_timer = new QTimer(this);
    connect(_refresh_timer, &QTimer::timeout, this, QOverload<>::of(&QWidget::update));
    _refresh_timer->start(25); // 40 Hz refresh
}

void WaveformWidget::pushFrame(const WaveformFrame& frame) {
    QMutexLocker lk(&_mx);
    _frames.push_back(frame);
    while (_frames.size() > _max_frames)
        _frames.pop_front();
    _frame_count++;
    ROS_INFO_ONCE("WaveformWidget: received first frame (%zu samples)", frame.raw_data.size());
}

size_t WaveformWidget::frameCount() const {
    QMutexLocker lk(&_mx);
    return _frame_count;
}

void WaveformWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);

    // background
    p.fillRect(rect(), QColor(30, 30, 30));

    QRect plotArea(kMarginLeft, kMarginTop,
                   width() - kMarginLeft - kMarginRight,
                   height() - kMarginTop - kMarginBottom);

    drawGrid(p, plotArea);
    drawWaveform(p, plotArea);
    drawInfo(p, plotArea);
}

void WaveformWidget::resizeEvent(QResizeEvent*) {
    // nothing special, paintEvent handles it
}

void WaveformWidget::drawGrid(QPainter& p, const QRect& area) {
    QPen gridPen(QColor(80, 80, 80), 1, Qt::DotLine);
    p.setPen(gridPen);

    // horizontal grid lines (5 lines)
    for (int i = 0; i <= 4; i++) {
        int y = area.top() + i * area.height() / 4;
        p.drawLine(area.left(), y, area.right(), y);
    }

    // vertical grid lines (10 lines)
    for (int i = 0; i <= 10; i++) {
        int x = area.left() + i * area.width() / 10;
        p.drawLine(x, area.top(), x, area.bottom());
    }

    // axis labels
    p.setPen(QColor(180, 180, 180));
    QFont font("Monospace", 9);
    font.setStyleHint(QFont::Monospace);
    p.setFont(font);

    // Y axis labels: -128 to +127 (DC offset 127 removed)
    for (int i = 0; i <= 4; i++) {
        int y = area.top() + i * area.height() / 4;
        float val = 127.0f - i * 254.0f / 4.0f;
        p.drawText(kMarginLeft - 55, y - 8, 50, 16,
                   Qt::AlignRight | Qt::AlignVCenter,
                   QString::number(val, 'f', 0));
    }

    // X axis label
    p.drawText(area.left(), area.bottom() + 5, area.width(), 20,
               Qt::AlignCenter, "Sample Index");
}

void WaveformWidget::drawWaveform(QPainter& p, const QRect& area) {
    std::vector<uint8_t> data;
    std::string material;
    uint32_t speed = 0;
    {
        QMutexLocker lk(&_mx);
        if (_frames.empty()) return;
        const auto& f = _frames.back();
        data = f.raw_data;
        material = f.material;
        speed = f.speed_of_voice;
    }

    if (data.empty()) return;

    const int N = static_cast<int>(data.size());
    const float dx = static_cast<float>(area.width()) / std::max(N - 1, 1);

    // DC removal: signal = raw - 127
    QVector<QPointF> points(N);
    for (int i = 0; i < N; i++) {
        float x = area.left() + i * dx;
        float signal = static_cast<float>(data[i]) - 127.0f;
        // map signal [-128, 127] to area [bottom, top]
        float y = area.bottom() - (signal + 128.0f) / 255.0f * area.height();
        points[i] = QPointF(x, y);
    }

    // waveform curve
    QPen wavePen(QColor(0, 220, 120), 1.5, Qt::SolidLine);
    p.setPen(wavePen);
    p.drawPolyline(points);

    // DC offset line
    float dcY = area.bottom() - 128.0f / 255.0f * area.height();
    p.setPen(QPen(QColor(100, 100, 100), 1, Qt::DashLine));
    p.drawLine(area.left(), dcY, area.right(), dcY);
}

void WaveformWidget::drawInfo(QPainter& p, const QRect& area) {
    std::vector<uint8_t> data;
    std::string material;
    uint32_t speed = 0;
    int totalFrames = 0;
    {
        QMutexLocker lk(&_mx);
        totalFrames = _frame_count;
        if (!_frames.empty()) {
            const auto& f = _frames.back();
            data = f.raw_data;
            material = f.material;
            speed = f.speed_of_voice;
        }
    }

    if (data.empty()) {
        ROS_INFO_THROTTLE(2, "WaveformWidget: no data yet (frames dequeued: %d)", totalFrames);
        p.setPen(QColor(180, 180, 180));
        p.drawText(area, Qt::AlignCenter, "Waiting for data...");
        return;
    }

    // compute RMS
    double sum = 0;
    for (auto v : data) {
        double s = static_cast<double>(v) - 127.0;
        sum += s * s;
    }
    double rms = std::sqrt(sum / data.size());

    QFont font("Monospace", 10);
    font.setStyleHint(QFont::Monospace);
    p.setFont(font);
    p.setPen(QColor(200, 200, 200));

    QString info = QString("Samples: %1 | RMS: %2 | Material: %3 | Speed: %4 m/s | Frame: %5")
                       .arg(data.size())
                       .arg(rms, 0, 'f', 1)
                       .arg(QString::fromStdString(material))
                       .arg(speed)
                       .arg(totalFrames);

    p.drawText(kMarginLeft, height() - 5, info);
}

} // namespace emat
