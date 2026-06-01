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

    // toggle button (top-right corner)
    _toggle_btn = new QPushButton("Envelope", this);
    _toggle_btn->setFixedSize(80, 26);
    _toggle_btn->setStyleSheet(
        "QPushButton { background: #444; color: #ccc; border: 1px solid #666; "
        "border-radius: 3px; font-size: 11px; }"
        "QPushButton:hover { background: #555; }");
    connect(_toggle_btn, &QPushButton::clicked, this, &WaveformWidget::toggleSource);

    _refresh_timer = new QTimer(this);
    connect(_refresh_timer, &QTimer::timeout, this, QOverload<>::of(&QWidget::update));
    _refresh_timer->start(25); // 40 Hz refresh
}

void WaveformWidget::toggleSource() {
    _display_raw = !_display_raw;
    _toggle_btn->setText(_display_raw ? "Raw" : "Envelope");
}

void WaveformWidget::pushFrame(const WaveformFrame& frame) {
    QMutexLocker lk(&_mx);
    _frames.push_back(frame);
    while (_frames.size() > _max_frames)
        _frames.pop_front();
    _frame_count++;
    ROS_INFO_ONCE("WaveformWidget: received first raw frame (%zu samples)", frame.raw_data.size());
}

void WaveformWidget::pushEnvelope(const std::vector<float>& env, float sampling_rate) {
    QMutexLocker lk(&_mx);
    _env_frames.push_back(env);
    while (_env_frames.size() > _max_frames)
        _env_frames.pop_front();
    _env_frame_count++;
    ROS_INFO_ONCE("WaveformWidget: received first envelope (%zu samples, fs=%.0f)",
                   env.size(), sampling_rate);
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

    // position toggle button at top-right of plot area
    _toggle_btn->move(plotArea.right() - _toggle_btn->width(), 2);

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

    if (_display_raw) {
        // Y axis labels: -128 to +127 (DC offset 127 removed)
        for (int i = 0; i <= 4; i++) {
            int y = area.top() + i * area.height() / 4;
            float val = 127.0f - i * 254.0f / 4.0f;
            p.drawText(kMarginLeft - 55, y - 8, 50, 16,
                       Qt::AlignRight | Qt::AlignVCenter,
                       QString::number(val, 'f', 0));
        }
    } else {
        // Y axis labels: 0 to 127 (fixed, matches raw ADC range)
        for (int i = 0; i <= 4; i++) {
            int y = area.top() + i * area.height() / 4;
            float val = 127.0f * (1.0f - i / 4.0f);
            p.drawText(kMarginLeft - 55, y - 8, 50, 16,
                       Qt::AlignRight | Qt::AlignVCenter,
                       QString::number(val, 'f', 0));
        }
    }
}

void WaveformWidget::drawWaveform(QPainter& p, const QRect& area) {
    if (_display_raw) {
        // ---- Raw waveform mode ----
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

    } else {
        // ---- Envelope mode ----
        std::vector<float> env_data;
        {
            QMutexLocker lk(&_mx);
            if (_env_frames.empty()) return;
            env_data = _env_frames.back();
        }

        if (env_data.empty()) return;

        const int N = static_cast<int>(env_data.size());
        const float dx = static_cast<float>(area.width()) / std::max(N - 1, 1);

        QVector<QPointF> points(N);
        for (int i = 0; i < N; i++) {
            float x = area.left() + i * dx;
            // map [0, 127] to [bottom, top], clamp
            float y = area.bottom() - std::min(env_data[i] / 127.0f, 1.0f) * area.height();
            points[i] = QPointF(x, y);
        }

        QPen envPen(QColor(0, 180, 255), 1.5, Qt::SolidLine);
        p.setPen(envPen);
        p.drawPolyline(points);
    }
}

void WaveformWidget::drawInfo(QPainter& p, const QRect& area) {
    QFont font("Monospace", 10);
    font.setStyleHint(QFont::Monospace);
    p.setFont(font);
    p.setPen(QColor(200, 200, 200));

    if (_display_raw) {
        std::vector<uint8_t> data;
        uint32_t speed = 0;
        int slice_s, slice_e;
        {
            QMutexLocker lk(&_mx);
            if (!_frames.empty()) {
                data = _frames.back().raw_data;
                speed = _frames.back().speed_of_voice;
            }
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

        QString info = QString("[Raw] RMS: %1 | Speed: %2 m/s | Slice: [%3,%4]")
                           .arg(rms, 0, 'f', 1)
                           .arg(speed)
                           .arg(s)
                           .arg(e);
        p.drawText(kMarginLeft, height() - 5, info);

    } else {
        std::vector<float> env_data;
        {
            QMutexLocker lk(&_mx);
            if (!_env_frames.empty())
                env_data = _env_frames.back();
        }

        if (env_data.empty()) {
            p.setPen(QColor(180, 180, 180));
            p.drawText(area, Qt::AlignCenter, "Waiting for envelope data...");
            return;
        }

        float max_val = *std::max_element(env_data.begin(), env_data.end());
        double sum = 0;
        for (auto v : env_data)
            sum += static_cast<double>(v) * v;
        double rms = std::sqrt(sum / env_data.size());

        QString info = QString("[Envelope] Peak: %1 | RMS: %2 | Samples: %3")
                           .arg(max_val, 0, 'f', 1)
                           .arg(rms, 0, 'f', 1)
                           .arg(env_data.size());
        p.drawText(kMarginLeft, height() - 5, info);
    }
}

} // namespace emat
