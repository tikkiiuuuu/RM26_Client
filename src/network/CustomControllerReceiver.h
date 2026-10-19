// SPDX-License-Identifier: MIT
/**
 * @file CustomControllerReceiver.h
 * @brief 自定义控制器接收器（RS232-USB 串口为主，UDP 保留为兼容/联调路径）
 * @details
 * 自定义控制器一侧是 RS232（STM32 UART7，115200 8N1），按裁判系统帧格式发送
 * 0x0302 控制帧。合并后的现场链路是：
 *
 *   控制器 UART7(TX) --RS232-- RS232-USB 转换器 --USB-- 客户端主机虚拟串口
 *
 * 因此本类默认按串口打开虚拟串口设备，把收到的字节交给
 * CustomControllerLinkPolicy 组帧与校验，并输出解析后的控制状态与原始控制段
 * （供 MQTT CustomControl 上行转发）。为兼容旧 RJ45 链路与无硬件联调，仍保留
 * UDP 接收路径。
 *
 * 控制器数据来源：机械臂与主控板之间改用 FDCAN 下发 mode/slow_open/slow_close；
 * 客户端只消费 0x0302 数据段，不区分其来源，因此无需随控制器改版调整。
 *
 * 线程约束：与其它网络组件一致，在 Qt 主线程使用。
 */

#ifndef CUSTOMCONTROLLERRECEIVER_H
#define CUSTOMCONTROLLERRECEIVER_H

#include "CustomControllerLinkPolicy.h"

#include <QHostAddress>
#include <QObject>
#include <QString>
#include <QtGlobal>

QT_BEGIN_NAMESPACE
class QSerialPort;
class QUdpSocket;
class QTimer;
QT_END_NAMESPACE

namespace RM {

class CustomControllerReceiver : public QObject {
  Q_OBJECT
  Q_PROPERTY(bool listening READ isListening NOTIFY listeningChanged)
  Q_PROPERTY(bool linkAlive READ isLinkAlive NOTIFY linkStateChanged)
  Q_PROPERTY(int lastMode READ lastMode NOTIFY controlStateUpdated)

public:
  /// 传输形态。
  enum class Transport { Serial, Udp };

  explicit CustomControllerReceiver(QObject *parent = nullptr);
  ~CustomControllerReceiver() override;

  /**
   * @brief 按控制器上行参数（默认 115200 8N1）打开虚拟串口。
   * @param portName 串口设备名，例如 /dev/ttyUSB0、COM5、/dev/ttyACM0。
   * @return 打开成功返回 true，失败时发出 errorOccurred。
   */
  bool startSerial(const QString &portName,
                   int baudRate = kCustomControllerDefaultBaudRate);

  /**
   * @brief 绑定 UDP 端口并开始接收（兼容旧 RJ45 链路与工具联调）。
   * @param port 端口，0 表示由系统分配。
   */
  bool startUdp(quint16 port,
                const QHostAddress &bindAddress = QHostAddress::Any);

  void stop();

  bool isListening() const;
  Transport transport() const { return m_transport; }
  /// 串口设备名，或 UDP 端口号字符串，用于日志与界面观测。
  QString endpointDescription() const;
  quint16 port() const;

  bool isLinkAlive() const { return m_linkAlive; }
  CustomControllerState lastState() const { return m_lastState; }
  int lastMode() const { return static_cast<int>(m_lastState.mode); }

  quint64 totalDatagrams() const { return m_totalDatagrams; }
  quint64 totalBytes() const { return m_totalBytes; }
  quint64 totalFrames() const { return m_totalFrames; }
  quint64 controlFrames() const { return m_controlFrames; }
  quint64 lostFrames() const { return m_policy.lostFrames(); }
  quint64 crc8Failures() const { return m_policy.crc8Failures(); }
  quint64 crc16Failures() const { return m_policy.crc16Failures(); }
  quint64 resyncBytes() const { return m_policy.resyncBytes(); }
  QString lastError() const { return m_lastError; }

  /// 距最近一次有效帧的毫秒数；尚未收到帧时返回 -1。
  qint64 lastFrameAgeMs() const;

  /// 链路超时阈值（默认 500 ms）；小于 1 时按 1 处理。
  void setLinkTimeoutMs(int ms);
  int linkTimeoutMs() const { return m_linkTimeoutMs; }

  /**
   * @brief 直接处理一段字节（等价于串口或 socket 收到的数据，供测试注入）。
   */
  void processBytes(const QByteArray &bytes);

signals:
  /// 每收到一条通过 CRC 校验的帧都会发出，含非 0x0302 命令码。
  void frameReceived(quint16 cmdId, quint8 seq, const QByteArray &payload);
  /// 0x0302 控制段解析成功。
  void controlStateUpdated(const RM::CustomControllerState &state);
  /// 0x0302 原始控制段（30 字节），用于上行转发。
  void controlPayloadReceived(const QByteArray &payload);
  /// 链路从无到有或从有到无时发出。
  void linkStateChanged(bool alive);
  void listeningChanged(bool listening);
  void errorOccurred(const QString &message);

private slots:
  void onSerialReadyRead();
  void onSerialError();
  void onUdpReadyRead();
  void onWatchdogTimeout();

private:
  void handleFrames(const QVector<CustomControllerFrame> &frames);
  void updateLinkAlive(bool alive, const QString &reason);
  void reportError(const QString &message);

  QSerialPort *m_serial = nullptr;
  QUdpSocket *m_socket = nullptr;
  QTimer *m_watchdogTimer = nullptr;
  CustomControllerLinkPolicy m_policy;
  CustomControllerState m_lastState;
  Transport m_transport = Transport::Serial;
  QString m_serialPortName;
  QString m_lastError;
  qint64 m_lastFrameMs = -1;
  int m_linkTimeoutMs = 500;
  bool m_linkAlive = false;
  /// 串口正在 open() 期间抑制 errorOccurred，避免失败报告重复。
  bool m_openingSerial = false;
  quint64 m_totalDatagrams = 0;
  quint64 m_totalBytes = 0;
  quint64 m_totalFrames = 0;
  quint64 m_controlFrames = 0;
  /// 绑定成功后记录的端口；个别 Qt 版本 localPort() 返回 0 时用于观测与测试。
  quint16 m_boundPort = 0;
};

} // namespace RM

#endif // CUSTOMCONTROLLERRECEIVER_H
