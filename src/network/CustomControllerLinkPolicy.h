#ifndef CUSTOMCONTROLLERLINKPOLICY_H
#define CUSTOMCONTROLLERLINKPOLICY_H

// SPDX-License-Identifier: MIT
/**
 * @file CustomControllerLinkPolicy.h
 * @brief 自定义控制器链路（0x0302）帧扫描与解析策略
 * @details
 * 自定义控制器把按键、编码器和达妙电机角度打包成 30 字节控制段，并按裁判系统
 * 帧格式发送：
 *
 *   SOF(0xA5) | data_length(2, 小端) | seq(1) | CRC8(1)
 *   | cmd_id(2, 小端) | data(30) | CRC16(2, 小端)
 *
 * 链路形态：控制器一侧是 RS232（STM32 UART7，115200 8N1），现场通过
 * RS232-USB 转换器接到客户端主机，客户端以虚拟串口（FTDI/CH340/CP210x 等）
 * 读取同一字节流。因此本策略只负责把字节流还原成帧和领域状态，不依赖任何
 * 传输实现；串口、虚拟串口、UDP 或测试注入都走同一个 feed() 入口。
 *
 * 注意数据来源变化：2027_controller 起机械臂与主控板之间改用 FDCAN
 * （USER/Device/src/communicate.c）下发 mode / slow_open / slow_close，
 * 数据段布局与命令码未变，因此本策略无需随控制器改版调整。
 *
 * 校验实现说明：
 * - CRC16 直接复用 CRC.{h,cpp}，该表已与协议附录和控制器发送端核对一致；
 * - CRC8 不使用 CRC.cpp 的表（该表从索引 12 起与协议附录不一致），改为在
 *   CustomControllerLinkPolicy.cpp 中从多项式 0x8C 推导并加编译期断言，
 *   与控制器工程 USER/Algorithm/Src/crc.c 字节级一致。
 */

#include <QByteArray>
#include <QMetaType>
#include <QVector>
#include <QtGlobal>

namespace RM {

/// 帧起始字节，与控制器发送端一致。
inline constexpr quint8 kCustomControllerFrameSof = 0xA5U;
/// 自定义控制器命令码（协议中英：自定义控制器数据）。
inline constexpr quint16 kCustomControllerCmdId = 0x0302U;
/// 帧头长度：SOF(1) + data_length(2) + seq(1) + CRC8(1)。
inline constexpr int kCustomControllerFrameHeaderBytes = 5;
/// 命令码长度。
inline constexpr int kCustomControllerCmdIdBytes = 2;
/// 帧尾长度：CRC16(2)。
inline constexpr int kCustomControllerFrameTailBytes = 2;
/// 0x0302 控制段固定长度（字节）。
inline constexpr int kCustomControllerControlDataBytes = 30;
/// 单帧 data_length 上限；超过视为噪声并重新同步。
inline constexpr int kCustomControllerMaxPayloadBytes = 300;
/// 解析缓冲上限；超过后只保留尾部，避免损坏数据无限累积。
inline constexpr int kCustomControllerMaxBufferBytes = 4096;
/// 完整 0x0302 帧的总字节数（5 + 2 + 30 + 2）。
inline constexpr int kCustomControllerFrameBytes =
    kCustomControllerFrameHeaderBytes + kCustomControllerCmdIdBytes +
    kCustomControllerControlDataBytes + kCustomControllerFrameTailBytes;
/// 控制器上行串口（UART7）的默认波特率，见 2027_controller/Core/Src/usart.c。
inline constexpr int kCustomControllerDefaultBaudRate = 115200;

/**
 * @brief 一条通过 CRC 校验的自定义控制器帧。
 */
struct CustomControllerFrame {
  quint16 cmdId = 0;   ///< 命令码，0x0302 为控制器控制段
  quint8 seq = 0;      ///< 发送端帧序号（0..255 回绕）
  QByteArray payload;  ///< 数据段
};

/**
 * @brief 0x0302 控制段解析结果。
 * @details 字节布局与 2026_controller/USER/Device/src/referee_system.c 的
 *          pack_Can_Message 对齐：
 *          - [0..3]   roll（float，小端）
 *          - [4..7]   encoder X（float，小端）
 *          - [8..11]  encoder Y（float，小端）
 *          - [12..15] pitch1（float，小端）
 *          - [16..19] pitch2（float，小端）
 *          - [20..23] yaw（float，小端）
 *          - [24]     mode（0=无力，1=底盘常规，2=机械臂自定义）
 *          - [25]     slow_open
 *          - [26]     slow_close
 *          - [27]     按键位（bit1/bit2 对应控制器两个按键）
 *          - [28..29] 保留
 */
struct CustomControllerState {
  float roll = 0.0f;
  float encoderX = 0.0f;
  float encoderY = 0.0f;
  float pitch1 = 0.0f;
  float pitch2 = 0.0f;
  float yaw = 0.0f;
  quint8 mode = 0;
  quint8 slowOpen = 0;
  quint8 slowClose = 0;
  quint8 buttons = 0;
  bool valid = false;
};

/**
 * @brief 增量扫描字节流，输出完整且校验通过的帧。
 *
 * 面向连续字节流设计：串口分片、虚拟串口粘连、UDP 数据报或夹带噪声都能恢复同步。
 */
class CustomControllerLinkPolicy {
public:
  /**
   * @brief 喂入一段字节，返回本次新完成的帧。
   */
  QVector<CustomControllerFrame> feed(const QByteArray &chunk);

  /**
   * @brief 解析 0x0302 控制段。
   * @return 长度合法时返回 true；越界字段保持默认值。
   */
  static bool parseControlData(const QByteArray &payload,
                               CustomControllerState &out);

  /**
   * @brief 记录帧序号，统计 uint8 回绕后的丢帧数。
   */
  void observeSequence(quint8 seq);

  void reset();

  quint64 crc8Failures() const { return m_crc8Failures; }
  quint64 crc16Failures() const { return m_crc16Failures; }
  quint64 resyncBytes() const { return m_resyncBytes; }
  quint64 oversizedFrames() const { return m_oversizedFrames; }
  quint64 lostFrames() const { return m_lostFrames; }

private:
  QByteArray m_buffer;
  quint64 m_crc8Failures = 0;
  quint64 m_crc16Failures = 0;
  quint64 m_resyncBytes = 0;
  quint64 m_oversizedFrames = 0;
  quint64 m_lostFrames = 0;
  bool m_sequenceSeen = false;
  quint8 m_lastSequence = 0;
};

} // namespace RM

Q_DECLARE_METATYPE(RM::CustomControllerState)

#endif // CUSTOMCONTROLLERLINKPOLICY_H