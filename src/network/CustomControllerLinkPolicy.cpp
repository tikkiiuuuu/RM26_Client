// SPDX-License-Identifier: MIT
/**
 * @file CustomControllerLinkPolicy.cpp
 * @brief 自定义控制器链路（0x0302）帧扫描与解析实现
 * @author Clear
 * @date 2026-09-25
 * @copyright Copyright (c) 2026 Fudan University EGA Team (复旦大学星云 EGA 机器人战队).
 */

#include "CustomControllerLinkPolicy.h"

#include "CRC.h" // 仅复用已核对的 CRC16 实现；CRC8 见下方说明

#include <array>
#include <cstring>

namespace RM {
namespace {

/**
 * 协议附录一 CRC8 直查表：poly 0x31 的反射形式 0x8C，逐位 LSB-first 推导。
 *
 * 注意：这里不能直接用 src/network/CRC.cpp 的 CRC8 表——该表从索引 12 起与
 * 协议附录以及 2026_controller/USER/Algorithm/Src/crc.c 不一致（发布版疑似
 * 转录错误）。控制器与本客户端必须使用同一张表，本实现从多项式推导并在编译期
 * 断言关键项，保证字节级一致。
 */
constexpr std::array<quint8, 256> kProtocolCrc8Table = []() constexpr {
  std::array<quint8, 256> table{};
  for (int i = 0; i < 256; ++i) {
    quint8 value = static_cast<quint8>(i);
    for (int bit = 0; bit < 8; ++bit) {
      value = (value & 0x01) ? static_cast<quint8>((value >> 1) ^ 0x8C)
                             : static_cast<quint8>(value >> 1);
    }
    table[static_cast<std::size_t>(i)] = value;
  }
  return table;
}();

static_assert(kProtocolCrc8Table[0] == 0x00, "CRC8 表推导错误");
static_assert(kProtocolCrc8Table[1] == 0x5E, "CRC8 表推导错误");
static_assert(kProtocolCrc8Table[15] == 0x41, "CRC8 表推导错误");
static_assert(kProtocolCrc8Table[255] == 0x35, "CRC8 表推导错误");

quint8 protocolCrc8(const quint8 *data, int length, quint8 init = 0xFF) {
  quint8 crc = init;
  for (int i = 0; i < length; ++i) {
    crc = kProtocolCrc8Table[static_cast<std::size_t>(crc ^ data[i])];
  }
  return crc;
}

quint16 readU16Le(const QByteArray &data, int offset) {
  return static_cast<quint16>(
      static_cast<quint8>(data.at(offset)) |
      (static_cast<quint16>(static_cast<quint8>(data.at(offset + 1))) << 8));
}

quint32 readU32Le(const QByteArray &data, int offset) {
  return static_cast<quint32>(static_cast<quint8>(data.at(offset))) |
         (static_cast<quint32>(static_cast<quint8>(data.at(offset + 1))) << 8) |
         (static_cast<quint32>(static_cast<quint8>(data.at(offset + 2))) << 16) |
         (static_cast<quint32>(static_cast<quint8>(data.at(offset + 3))) << 24);
}

float readFloatLe(const QByteArray &data, int offset) {
  const quint32 raw = readU32Le(data, offset);
  float value = 0.0f;
  static_assert(sizeof(float) == sizeof(quint32), "float 必须是 32 位");
  std::memcpy(&value, &raw, sizeof(float));
  return value;
}

} // namespace

QVector<CustomControllerFrame>
CustomControllerLinkPolicy::feed(const QByteArray &chunk) {
  QVector<CustomControllerFrame> frames;

  if (!chunk.isEmpty()) {
    m_buffer.append(chunk);
  }

  // 缓冲上限保护：损坏或异常数据不能无限累积。
  if (m_buffer.size() > kCustomControllerMaxBufferBytes) {
    const int drop = m_buffer.size() - kCustomControllerMaxBufferBytes / 2;
    m_buffer.remove(0, drop);
    m_resyncBytes += static_cast<quint64>(drop);
  }

  while (!m_buffer.isEmpty()) {
    // 1. 重新同步到帧头。
    int sofIndex = -1;
    for (int i = 0; i < m_buffer.size(); ++i) {
      if (static_cast<quint8>(m_buffer.at(i)) == kCustomControllerFrameSof) {
        sofIndex = i;
        break;
      }
    }
    if (sofIndex < 0) {
      m_resyncBytes += static_cast<quint64>(m_buffer.size());
      m_buffer.clear();
      break;
    }
    if (sofIndex > 0) {
      m_resyncBytes += static_cast<quint64>(sofIndex);
      m_buffer.remove(0, sofIndex);
    }

    // 2. 至少要有帧头 + 命令码才能继续判断。
    if (m_buffer.size() <
        kCustomControllerFrameHeaderBytes + kCustomControllerCmdIdBytes) {
      break;
    }

    // 3. 帧头 CRC8（协议附录表，与控制器发送端一致）。
    const quint8 headerCrc8 =
        protocolCrc8(reinterpret_cast<const quint8 *>(m_buffer.constData()), 4);
    if (static_cast<quint8>(m_buffer.at(4)) != headerCrc8) {
      ++m_crc8Failures;
      m_buffer.remove(0, 1);
      ++m_resyncBytes;
      continue;
    }

    // 4. 数据长度合法性。
    const quint16 dataLength = readU16Le(m_buffer, 1);
    if (dataLength > kCustomControllerMaxPayloadBytes) {
      ++m_oversizedFrames;
      m_buffer.remove(0, 1);
      ++m_resyncBytes;
      continue;
    }

    const int totalBytes = kCustomControllerFrameHeaderBytes +
                           kCustomControllerCmdIdBytes +
                           static_cast<int>(dataLength) +
                           kCustomControllerFrameTailBytes;
    if (m_buffer.size() < totalBytes) {
      break; // 等待剩余分片（串口读回调或下一次数据报）。
    }

    // 5. 整帧 CRC16（CRC.{h,cpp} 的该表已与协议附录核对一致）。
    if (Verify_CRC16_Check_Sum(
            reinterpret_cast<uint8_t *>(m_buffer.data()),
            static_cast<uint32_t>(totalBytes)) == 0) {
      ++m_crc16Failures;
      m_buffer.remove(0, 1);
      ++m_resyncBytes;
      continue;
    }

    CustomControllerFrame frame;
    frame.cmdId = readU16Le(m_buffer, kCustomControllerFrameHeaderBytes);
    frame.seq = static_cast<quint8>(m_buffer.at(3));
    frame.payload = m_buffer.mid(
        kCustomControllerFrameHeaderBytes + kCustomControllerCmdIdBytes,
        static_cast<int>(dataLength));
    frames.append(frame);

    m_buffer.remove(0, totalBytes);
  }

  return frames;
}

bool CustomControllerLinkPolicy::parseControlData(
    const QByteArray &payload, CustomControllerState &out) {
  if (payload.size() < kCustomControllerControlDataBytes) {
    return false;
  }

  out.roll = readFloatLe(payload, 0);
  out.encoderX = readFloatLe(payload, 4);
  out.encoderY = readFloatLe(payload, 8);
  out.pitch1 = readFloatLe(payload, 12);
  out.pitch2 = readFloatLe(payload, 16);
  out.yaw = readFloatLe(payload, 20);
  out.mode = static_cast<quint8>(payload.at(24));
  out.slowOpen = static_cast<quint8>(payload.at(25));
  out.slowClose = static_cast<quint8>(payload.at(26));
  out.buttons = static_cast<quint8>(payload.at(27));
  out.valid = true;
  return true;
}

void CustomControllerLinkPolicy::observeSequence(quint8 seq) {
  if (!m_sequenceSeen) {
    m_sequenceSeen = true;
    m_lastSequence = seq;
    return;
  }

  const quint8 expected = static_cast<quint8>(m_lastSequence + 1);
  if (seq == expected) {
    m_lastSequence = seq;
    return;
  }
  if (seq == m_lastSequence) {
    return; // 重复帧不计入丢帧。
  }

  m_lostFrames += static_cast<quint8>(seq - expected);
  m_lastSequence = seq;
}

void CustomControllerLinkPolicy::reset() {
  m_buffer.clear();
  m_crc8Failures = 0;
  m_crc16Failures = 0;
  m_resyncBytes = 0;
  m_oversizedFrames = 0;
  m_lostFrames = 0;
  m_sequenceSeen = false;
  m_lastSequence = 0;
}

} // namespace RM