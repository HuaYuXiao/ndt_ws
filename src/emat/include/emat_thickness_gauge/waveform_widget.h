#pragma once

#include <QWidget>
#include <QTimer>
#include <QMutex>
#include <QVector>
#include <deque>
#include <vector>
#include <cstdint>

namespace emat {

struct WaveformFrame {
    std::vector<uint8_t> raw_data;
};

class WaveformWidget : public QWidget {
    Q_OBJECT

public:
    explicit WaveformWidget(QWidget* parent = nullptr);
    ~WaveformWidget() override = default;

    void setMaxFrames(size_t n) { _max_frames = n; }
    size_t frameCount() const;

    // Thread-safe: called from ROS callback thread
    void pushFrame(const WaveformFrame& frame);
    void pushThickness(float thickness_mm);
    void setSlice(int start, int end);

protected:
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private:
    void drawGrid(QPainter& p, const QRect& area);
    void drawWaveform(QPainter& p, const QRect& area);
    void drawInfo(QPainter& p, const QRect& area);

    mutable QMutex _mx;
    std::deque<WaveformFrame> _frames;
    size_t _max_frames = 5000;
    int _frame_count = 0;
    int _slice_start = 0;
    int _slice_end = 0;  // 0 means show all
    bool _has_thickness = false;
    float _thickness_mm = 0.0f;
    QTimer* _refresh_timer;

    static constexpr int kMarginLeft   = 60;
    static constexpr int kMarginRight  = 20;
    static constexpr int kMarginTop    = 20;
    static constexpr int kMarginBottom = 40;
};

} // namespace emat
