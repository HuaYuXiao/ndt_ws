#include "record/rviz_record_panel.h"

#include <QVBoxLayout>
#include <QProcessEnvironment>
#include <ros/ros.h>

namespace record {

RvizRecordPanel::RvizRecordPanel(QWidget* parent)
    : rviz::Panel(parent)
{
    _btn = new QPushButton(this);
    _btn->setMinimumHeight(48);
    _btn->setCursor(Qt::PointingHandCursor);
    _btn->setStyleSheet(
        "QPushButton { background-color: #c0392b; color: white; "
        "font-size: 16px; font-weight: bold; border: none; border-radius: 4px; }"
        "QPushButton:hover { background-color: #e74c3c; }");

    _status = new QLabel("就绪", this);
    _status->setAlignment(Qt::AlignCenter);
    _status->setStyleSheet("QLabel { color: #888; font-size: 12px; }");

    auto* layout = new QVBoxLayout;
    layout->setContentsMargins(4, 4, 4, 4);
    layout->addWidget(_btn);
    layout->addWidget(_status);
    layout->addStretch();
    setLayout(layout);

    _process = new QProcess(this);
    _timer = new QTimer(this);
    _timer->setInterval(1000);

    connect(_btn, &QPushButton::clicked, this, &RvizRecordPanel::onToggle);
    connect(_process, &QProcess::started, this, &RvizRecordPanel::onProcessStarted);
    connect(_process,
            QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
            this, &RvizRecordPanel::onProcessFinished);
    connect(_timer, &QTimer::timeout, this, &RvizRecordPanel::onTimerTick);

    setRecordingState(false);
}

RvizRecordPanel::~RvizRecordPanel()
{
    if (_process->state() != QProcess::NotRunning) {
        _process->terminate();
        if (!_process->waitForFinished(3000)) {
            _process->kill();
            _process->waitForFinished(2000);
        }
    }
}

void RvizRecordPanel::onInitialize()
{
    ROS_INFO("RvizRecordPanel: ready");
}

void RvizRecordPanel::onToggle()
{
    QMutexLocker lk(&_mx);
    if (!_recording) {
        // 启动录制
        _elapsed_sec = 0;
        QString cmd = "bash -c \"source ~/ndt_ws/devel/setup.bash && "
                      "rosrun record multimodal_recorder\"";
        _process->start(cmd);
    } else {
        // 停止录制
        _process->terminate();
        if (!_process->waitForFinished(3000)) {
            _process->kill();
        }
    }
}

void RvizRecordPanel::onProcessStarted()
{
    setRecordingState(true);
    _timer->start();
    ROS_INFO("RvizRecordPanel: recording started");
}

void RvizRecordPanel::onProcessFinished(int exit_code,
                                        QProcess::ExitStatus status)
{
    _timer->stop();

    QString msg;
    if (status == QProcess::NormalExit) {
        msg = QString("已保存到 datasets/ (exit %1)").arg(exit_code);
    } else {
        msg = QString("进程异常退出 (exit %1)").arg(exit_code);
    }
    _status->setText(msg);
    ROS_INFO("RvizRecordPanel: %s", msg.toStdString().c_str());

    setRecordingState(false);
}

void RvizRecordPanel::onTimerTick()
{
    QMutexLocker lk(&_mx);
    ++_elapsed_sec;
    int min = _elapsed_sec / 60;
    int sec = _elapsed_sec % 60;
    _status->setText(QString("录制中... %1:%2")
                         .arg(min, 2, 10, QChar('0'))
                         .arg(sec, 2, 10, QChar('0')));
}

void RvizRecordPanel::setRecordingState(bool recording)
{
    _recording = recording;
    if (recording) {
        _btn->setText("■ 停止录制");
        _btn->setStyleSheet(
            "QPushButton { background-color: #27ae60; color: white; "
            "font-size: 16px; font-weight: bold; border: none; border-radius: 4px; }"
            "QPushButton:hover { background-color: #2ecc71; }");
    } else {
        _btn->setText("● 开始录制");
        _btn->setStyleSheet(
            "QPushButton { background-color: #c0392b; color: white; "
            "font-size: 16px; font-weight: bold; border: none; border-radius: 4px; }"
            "QPushButton:hover { background-color: #e74c3c; }");
    }
}

void RvizRecordPanel::save(rviz::Config config) const
{
    rviz::Panel::save(config);
}

void RvizRecordPanel::load(const rviz::Config& config)
{
    rviz::Panel::load(config);
}

}  // namespace record

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(record::RvizRecordPanel, rviz::Panel)
