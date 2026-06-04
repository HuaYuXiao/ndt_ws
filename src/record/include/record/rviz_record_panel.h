#pragma once

#include <rviz/panel.h>
#include <ros/ros.h>

#include <QLabel>
#include <QMutex>
#include <QPushButton>
#include <QProcess>
#include <QTimer>
#include <QWidget>

namespace record {

class RvizRecordPanel : public rviz::Panel {
    Q_OBJECT

public:
    explicit RvizRecordPanel(QWidget* parent = nullptr);
    ~RvizRecordPanel() override;

    void onInitialize() override;
    void save(rviz::Config config) const override;
    void load(const rviz::Config& config) override;

private slots:
    void onToggle();
    void onProcessStarted();
    void onProcessFinished(int exit_code, QProcess::ExitStatus status);
    void onTimerTick();

private:
    void setRecordingState(bool recording);

    QPushButton* _btn;
    QLabel* _status;
    QProcess* _process;
    QTimer* _timer;
    bool _recording = false;
    bool _user_stopped = false;
    int _elapsed_sec = 0;
    mutable QMutex _mx;
};

}  // namespace record
