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
    std::string material;
    uint32_t speed_of_voice;
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
    QTimer* _refresh_timer;

    static constexpr int kMarginLeft   = 60;
    static constexpr int kMarginRight  = 20;
    static constexpr int kMarginTop    = 20;
    static constexpr int kMarginBottom = 40;
};

} // namespace emat
