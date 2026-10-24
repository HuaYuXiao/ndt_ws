#include "emat_thickness_gauge/waveform_widget.h"

#include <QPainter>
#include <QPen>
#include <QVBoxLayout>
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
    ROS_INFO_ONCE("WaveformWidget: received first raw frame (%zu samples)", frame.raw_data.size());
}

void WaveformWidget::pushThickness(float thickness_mm) {
    QMutexLocker lk(&_mx);
    _thickness_mm = thickness_mm;
    _has_thickness = true;
    ROS_INFO_ONCE("WaveformWidget: received first thickness (%.2f mm)", thickness_mm);
}

size_t WaveformWidget::frameCount() const {
    QMutexLocker lk(&_mx);
    return _frame_count;
}

void WaveformWidget::setSlice(int start, int end) {
    QMutexLocker lk(&_mx);
    _slice_start = start;
    _slice_end = end;
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
}

void WaveformWidget::drawWaveform(QPainter& p, const QRect& area) {
    std::vector<uint8_t> data;
    int slice_s, slice_e;
    {
        QMutexLocker lk(&_mx);
        if (_frames.empty()) return;
        data = _frames.back().raw_data;
        slice_s = _slice_start;
        slice_e = _slice_end;
    }

    if (data.empty()) return;

    // Apply slice
    int full_N = static_cast<int>(data.size());
    int s = (slice_s > 0 && slice_s < full_N) ? slice_s : 0;
    int e = (slice_e > 0 && slice_e < full_N) ? slice_e : full_N;
    if (s >= e) { s = 0; e = full_N; }
    const int N = e - s;
    const float dx = static_cast<float>(area.width()) / std::max(N - 1, 1);

    QVector<QPointF> points(N);
    for (int i = 0; i < N; i++) {
        float x = area.left() + i * dx;
        float signal = static_cast<float>(data[s + i]) - 127.0f;
        float y = area.bottom() - (signal + 128.0f) / 255.0f * area.height();
        points[i] = QPointF(x, y);
    }

    QPen wavePen(QColor(0, 220, 120), 1.5, Qt::SolidLine);
    p.setPen(wavePen);
    p.drawPolyline(points);

    // DC offset line
    float dcY = area.bottom() - 128.0f / 255.0f * area.height();
    p.setPen(QPen(QColor(100, 100, 100), 1, Qt::DashLine));
    p.drawLine(area.left(), dcY, area.right(), dcY);
}

void WaveformWidget::drawInfo(QPainter& p, const QRect& area) {
    QFont font("Monospace", 10);
    font.setStyleHint(QFont::Monospace);
    p.setFont(font);
    p.setPen(QColor(200, 200, 200));

    bool has_thick;
    float thick;
    {
        QMutexLocker lk(&_mx);
        has_thick = _has_thickness;
        thick = _thickness_mm;
    }
    QString thick_str = has_thick
        ? (std::isnan(thick)
            ? QString("Thickness: --")
            : QString("Thickness: %1 mm").arg(thick, 0, 'f', 2))
        : QString("Thickness: --");

    std::vector<uint8_t> data;
    int slice_s, slice_e;
    {
        QMutexLocker lk(&_mx);
        if (!_frames.empty())
            data = _frames.back().raw_data;
        slice_s = _slice_start;
        slice_e = _slice_end;
    }

    if (data.empty()) {
        p.setPen(QColor(180, 180, 180));
        p.drawText(area, Qt::AlignCenter, "Waiting for data...");
        return;
    }

    // Apply slice for RMS computation
    int full_N = static_cast<int>(data.size());
    int s = (slice_s > 0 && slice_s < full_N) ? slice_s : 0;
    int e = (slice_e > 0 && slice_e < full_N) ? slice_e : full_N;
    if (s >= e) { s = 0; e = full_N; }

    double sum = 0;
    for (int i = s; i < e; i++) {
        double val = static_cast<double>(data[i]) - 127.0;
        sum += val * val;
    }
    double rms = std::sqrt(sum / (e - s));

    QString info = QString("[Raw] RMS: %1 | Slice: [%2,%3] | %4")
                       .arg(rms, 0, 'f', 1)
                       .arg(s)
                       .arg(e)
                       .arg(thick_str);
    p.drawText(kMarginLeft, height() - 5, info);
}

} // namespace emat
