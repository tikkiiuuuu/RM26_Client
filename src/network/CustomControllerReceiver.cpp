// SPDX-License-Identifier: MIT
/**
 * @file CustomControllerReceiver.cpp
 * @brief 自定义控制器接收器实现
 * @author Clear
 * @date 2026-09-25
 * @copyright Copyright (c) 2026 Fudan University EGA Team (复旦大学星云 EGA 机器人战队).
 */

#include "CustomControllerReceiver.h"

#include <QDateTime>
#include <QDebug>
#include <QSerialPort>
#include <QTimer>
#include <QUdpSocket>

namespace RM {

CustomControllerReceiver::CustomControllerReceiver(QObject *parent)
    : QObject(parent) {
  qRegisterMetaType<CustomControllerState>("RM::CustomControllerState");

  // 串口是默认传输形态：控制器经 RS232-USB 转换器接到主机虚拟串口。
  m_serial = new QSerialPort(this);
  connect(m_serial, &QSerialPort::readyRead, this,
          &CustomControllerReceiver::onSerialReadyRead);
  connect(m_serial, &QSerialPort::errorOccurred, this,
          &CustomControllerReceiver::onSerialError);

  // UDP 路径保留，用于旧 RJ45 链路或无硬件时的工具联调。
  m_socket = new QUdpSocket(this);
  connect(m_socket, &QUdpSocket::readyRead, this,
          &CustomControllerReceiver::onUdpReadyRead);

  m_watchdogTimer = new QTimer(this);
  m_watchdogTimer->setInterval(100);
  connect(m_watchdogTimer, &QTimer::timeout, this,
          &CustomControllerReceiver::onWatchdogTimeout);
}

CustomControllerReceiver::~CustomControllerReceiver() { stop(); }

bool CustomControllerReceiver::startSerial(const QString &portName,
                                           int baudRate) {
  stop();

  const QString trimmedPort = portName.trimmed();
  if (trimmedPort.isEmpty()) {
    reportError(QStringLiteral("自定义控制器串口未配置：请设置 controller.serial_port "
                               "或环境变量 RM_CONTROLLER_SERIAL"));
    return false;
  }

  m_serial->setPortName(trimmedPort);
  m_serial->setBaudRate(baudRate);
  m_serial->setDataBits(QSerialPort::Data8);
  m_serial->setParity(QSerialPort::NoParity);
  m_serial->setStopBits(QSerialPort::OneStop);
  m_serial->setFlowControl(QSerialPort::NoFlowControl);

  // 打开过程中的失败由 startSerial 统一报告，避免 errorOccurred 重复触发。
  m_openingSerial = true;
  const bool opened = m_serial->open(QIODevice::ReadWrite);
  m_openingSerial = false;

  if (!opened) {
    reportError(QStringLiteral("自定义控制器串口 %1 打开失败: %2")
                    .arg(trimmedPort)
                    .arg(m_serial->errorString()));
    return false;
  }

  // 控制器上行速率约 30 Hz、每帧 39 字节，数据量很小；清空历史残留即可。
  m_serial->clear(QSerialPort::AllDirections);

  m_transport = Transport::Serial;
  m_serialPortName = trimmedPort;
  m_watchdogTimer->start();

  qInfo().noquote() << QStringLiteral(
                           "[controller-link] listening serial port=%1 baud=%2 "
                           "(0x0302 over RS232-USB)")
                           .arg(trimmedPort)
                           .arg(baudRate);
  emit listeningChanged(true);
  return true;
}

bool CustomControllerReceiver::startUdp(quint16 port,
                                        const QHostAddress &bindAddress) {
  stop();

  // 与 VideoReceiver 一致：允许快速重启或与诊断工具临时共享端口。
  const bool bound =
      m_socket->bind(bindAddress, port,
                     QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);
  if (!bound) {
    reportError(QStringLiteral("自定义控制器 UDP 端口 %1 绑定失败: %2")
                    .arg(port)
                    .arg(m_socket->errorString()));
    return false;
  }

  m_transport = Transport::Udp;
  m_watchdogTimer->start();
  m_boundPort = port;
  qInfo().noquote()
      << QStringLiteral(
             "[controller-link] listening udp port=%1 (0x0302, 兼容 RJ45 链路)")
             .arg(m_boundPort);
  emit listeningChanged(true);
  return true;
}

void CustomControllerReceiver::stop() {
  if (m_watchdogTimer) {
    m_watchdogTimer->stop();
  }

  bool wasActive = false;
  if (m_serial && m_serial->isOpen()) {
    m_serial->close();
    wasActive = true;
  }
  if (m_socket && m_socket->state() != QAbstractSocket::UnconnectedState) {
    m_socket->close();
    wasActive = true;
  }

  m_boundPort = 0;
  m_serialPortName.clear();
  if (wasActive) {
    emit listeningChanged(false);
  }

  updateLinkAlive(false, QStringLiteral("接收器已停止"));
}

bool CustomControllerReceiver::isListening() const {
  if (m_transport == Transport::Serial) {
    return m_serial && m_serial->isOpen();
  }
  return m_socket && m_socket->state() == QAbstractSocket::BoundState;
}

QString CustomControllerReceiver::endpointDescription() const {
  if (m_transport == Transport::Serial) {
    return m_serialPortName;
  }
  const quint16 bound = port();
  return bound == 0 ? QStringLiteral("udp:0") : QStringLiteral("udp:%1").arg(bound);
}

quint16 CustomControllerReceiver::port() const {
  if (!m_socket) {
    return 0;
  }
  const quint16 bound = m_socket->localPort();
  return bound != 0 ? bound : m_boundPort;
}

qint64 CustomControllerReceiver::lastFrameAgeMs() const {
  if (m_lastFrameMs < 0) {
    return -1;
  }
  return QDateTime::currentMSecsSinceEpoch() - m_lastFrameMs;
}

void CustomControllerReceiver::setLinkTimeoutMs(int ms) {
  m_linkTimeoutMs = ms < 1 ? 1 : ms;
}

void CustomControllerReceiver::processBytes(const QByteArray &bytes) {
  if (bytes.isEmpty()) {
    return;
  }

  m_totalBytes += static_cast<quint64>(bytes.size());
  const QVector<CustomControllerFrame> frames = m_policy.feed(bytes);
  if (frames.isEmpty()) {
    return;
  }

  m_lastFrameMs = QDateTime::currentMSecsSinceEpoch();
  handleFrames(frames);
  updateLinkAlive(true, QString());
}

void CustomControllerReceiver::onSerialReadyRead() {
  const QByteArray chunk = m_serial->readAll();
  if (chunk.isEmpty()) {
    return;
  }
  ++m_totalDatagrams;
  processBytes(chunk);
}

void CustomControllerReceiver::onSerialError() {
  if (!m_serial) {
    return;
  }
  const QSerialPort::SerialPortError error = m_serial->error();
  if (error == QSerialPort::NoError) {
    return;
  }
  // 打开阶段的失败由 startSerial 报告，这里跳过以免重复。
  if (m_openingSerial) {
    return;
  }
  // 设备热插拔（USB 转换器被拔出）时 Qt 会报 ResourceError/DeviceNotFoundError。
  reportError(QStringLiteral("自定义控制器串口错误(%1): %2")
                  .arg(static_cast<int>(error))
                  .arg(m_serial->errorString()));
  if (error == QSerialPort::ResourceError ||
      error == QSerialPort::DeviceNotFoundError ||
      error == QSerialPort::PermissionError) {
    stop();
  }
}

void CustomControllerReceiver::onUdpReadyRead() {
  while (m_socket && m_socket->hasPendingDatagrams()) {
    QByteArray datagram;
    datagram.resize(static_cast<int>(m_socket->pendingDatagramSize()));
    m_socket->readDatagram(datagram.data(), datagram.size());
    ++m_totalDatagrams;
    processBytes(datagram);
  }
}

void CustomControllerReceiver::onWatchdogTimeout() {
  if (!m_linkAlive || m_lastFrameMs < 0) {
    return;
  }
  const qint64 ageMs = QDateTime::currentMSecsSinceEpoch() - m_lastFrameMs;
  if (ageMs > m_linkTimeoutMs) {
    updateLinkAlive(false,
                    QStringLiteral("超过 %1 ms 未收到控制器帧").arg(m_linkTimeoutMs));
  }
}

void CustomControllerReceiver::handleFrames(
    const QVector<CustomControllerFrame> &frames) {
  for (const CustomControllerFrame &frame : frames) {
    ++m_totalFrames;
    emit frameReceived(frame.cmdId, frame.seq, frame.payload);

    if (frame.cmdId != kCustomControllerCmdId) {
      continue;
    }

    m_policy.observeSequence(frame.seq);

    CustomControllerState state;
    if (!CustomControllerLinkPolicy::parseControlData(frame.payload, state)) {
      qWarning() << "[controller-link] 0x0302 控制段长度非法:"
                 << frame.payload.size();
      continue;
    }

    ++m_controlFrames;
    m_lastState = state;
    emit controlStateUpdated(state);
    emit controlPayloadReceived(frame.payload);
  }
}

void CustomControllerReceiver::updateLinkAlive(bool alive,
                                               const QString &reason) {
  if (m_linkAlive == alive) {
    return;
  }

  m_linkAlive = alive;
  if (alive) {
    qInfo().noquote() << "[controller-link] link up";
  } else {
    qWarning().noquote() << "[controller-link] link down"
                         << (reason.isEmpty() ? QString() : reason);
  }
  emit linkStateChanged(alive);
}

void CustomControllerReceiver::reportError(const QString &message) {
  m_lastError = message;
  qWarning().noquote() << "[controller-link]" << message;
  emit errorOccurred(message);
}

} // namespace RM