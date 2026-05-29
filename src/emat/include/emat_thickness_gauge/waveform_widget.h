#pragma once

#include <QWidget>
#include <QTimer>
#include <QMutex>
#include <QPushButton>
#include <QVector>
#include <deque>
#include <vector>
#include <cstdint>

namespace emat {

struct WaveformFrame {
    std::vector<uint8_t> raw_data;
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
    void pushEnvelope(const std::vector<float>& env, float sampling_rate);
    void setThickness(float mm);

public slots:
    void toggleSource();

protected:
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;

private:
    void drawGrid(QPainter& p, const QRect& area);
    void drawWaveform(QPainter& p, const QRect& area);
    void drawInfo(QPainter& p, const QRect& area);

    mutable QMutex _mx;
    std::deque<WaveformFrame> _frames;
    std::deque<std::vector<float>> _env_frames;
    size_t _max_frames = 5000;
    int _frame_count = 0;
    int _env_frame_count = 0;
    float _thickness = 0.0f;
    QTimer* _refresh_timer;
    QPushButton* _toggle_btn;
    bool _display_raw = false;  // true=raw waveform, false=envelope

    static constexpr int kMarginLeft   = 60;
    static constexpr int kMarginRight  = 20;
    static constexpr int kMarginTop    = 20;
    static constexpr int kMarginBottom = 40;
};

} // namespace emat
