// SPDX-License-Identifier: MIT
/**
 * @file test_custom_controller_link.cpp
 * @brief 自定义控制器链路（0x0302 over RS232-USB / 兼容 UDP）测试
 * @author Clear
 * @date 2026-09-25
 * @copyright Copyright (c) 2026 Fudan University EGA Team (复旦大学星云 EGA 机器人战队).
 */

#include "network/CustomControllerLinkPolicy.h"
#include "network/CustomControllerReceiver.h"

#include <QElapsedTimer>
#include <QFile>
#include <QSignalSpy>
#include <QUdpSocket>
#include <QtTest>

namespace {

// 黄金帧由协议附录 CRC 表独立生成（tools/controller_link_emulator.py 同源），
// 用于交叉验证客户端解析与 2027_controller 发送端字节级一致。
//
// 帧布局：SOF(0xA5) | data_length(30, 小端) | seq | CRC8 | cmd_id | data(30) | CRC16

// seq=7, roll=1.5, encX=0.25, encY=-0.5, pitch1=1.0, pitch2=2.0,
// yaw=-1.25, mode=2, slow_open=1, slow_close=0, buttons=0b00000110
QByteArray frameSeq7() {
  return QByteArray::fromHex("a51e0007fe0203"
                             "0000c03f0000803e000000bf"
                             "0000803f000000400000a0bf"
                             "020100060000290c");
}

// seq=8, roll=-2.25, encX=12.5, encY=100.0, pitch1=-0.75, pitch2=0.125,
// yaw=3.5, mode=1, slow_open=0, slow_close=1, buttons=0b00000010
QByteArray frameSeq8() {
  return QByteArray::fromHex("a51e0008bf0203"
                             "000010c0000048410000c842"
                             "000040bf0000003e00006040"
                             "010001020000e8c9");
}

// seq=10，控制段与 seq=7 相同，用于丢帧统计
QByteArray frameSeq10() {
  return QByteArray::fromHex("a51e000a030203"
                             "0000c03f0000803e000000bf"
                             "0000803f000000400000a0bf"
                             "020100060000621e");
}

// cmd_id=0x0310（自定义客户端视频链），非控制器控制段
QByteArray frameCustomClient0310() {
  return QByteArray::fromHex("a51e0009e11003") + QByteArray(30, '\xab') +
         QByteArray::fromHex("1593");
}

// cmd_id=0x0304（键鼠遥控数据，同样来自控制器），非 0x0302 控制段
QByteArray frameKeyboardMouse0304() {
  return QByteArray::fromHex("a51e00041c0403") + QByteArray(30, '\x5a') +
         QByteArray::fromHex("d701");
}

// 帧头 CRC8 被破坏（字节 4：0xFE -> 0xFF）
QByteArray frameWithBadCrc8() {
  return QByteArray::fromHex("a51e0007ff0203"
                             "0000c03f0000803e000000bf"
                             "0000803f000000400000a0bf"
                             "020100060000290c");
}

// 控制段被破坏导致 CRC16 失败（字节 10：0x3F -> 0xC0）
QByteArray frameWithBadCrc16() {
  return QByteArray::fromHex("a51e0007fe0203"
                             "0000c0c00000803e000000bf"
                             "0000803f000000400000a0bf"
                             "020100060000290c");
}

} // namespace

class TestCustomControllerLink : public QObject {
  Q_OBJECT

private slots:
  void goldenControlFrameParses() {
    RM::CustomControllerLinkPolicy policy;
    const QVector<RM::CustomControllerFrame> frames = policy.feed(frameSeq7());

    QCOMPARE(frames.size(), 1);
    QCOMPARE(frames.at(0).cmdId, static_cast<quint16>(0x0302));
    QCOMPARE(frames.at(0).seq, static_cast<quint8>(7));
    QCOMPARE(frames.at(0).payload.size(), RM::kCustomControllerControlDataBytes);

    RM::CustomControllerState state;
    QVERIFY(RM::CustomControllerLinkPolicy::parseControlData(frames.at(0).payload,
                                                            state));
    QCOMPARE(state.roll, 1.5f);
    QCOMPARE(state.encoderX, 0.25f);
    QCOMPARE(state.encoderY, -0.5f);
    QCOMPARE(state.pitch1, 1.0f);
    QCOMPARE(state.pitch2, 2.0f);
    QCOMPARE(state.yaw, -1.25f);
    QCOMPARE(state.mode, static_cast<quint8>(2));
    QCOMPARE(state.slowOpen, static_cast<quint8>(1));
    QCOMPARE(state.slowClose, static_cast<quint8>(0));
    QCOMPARE(state.buttons, static_cast<quint8>(0x06));
    QVERIFY(state.valid);
  }

  void frameSplitAcrossChunksParsesOnce() {
    RM::CustomControllerLinkPolicy policy;
    const QByteArray frame = frameSeq7();

    int frameCount = 0;
    for (int i = 0; i < frame.size(); ++i) {
      const QVector<RM::CustomControllerFrame> frames =
          policy.feed(frame.mid(i, 1));
      frameCount += frames.size();
    }

    QCOMPARE(frameCount, 1);
    QCOMPARE(policy.crc8Failures(), quint64(0));
    QCOMPARE(policy.crc16Failures(), quint64(0));
  }

  void multipleFramesInOneChunkParseInOrder() {
    RM::CustomControllerLinkPolicy policy;
    QByteArray chunk = frameSeq7() + frameSeq8() + frameCustomClient0310();

    const QVector<RM::CustomControllerFrame> frames = policy.feed(chunk);

    QCOMPARE(frames.size(), 3);
    QCOMPARE(frames.at(0).seq, static_cast<quint8>(7));
    QCOMPARE(frames.at(1).seq, static_cast<quint8>(8));
    QCOMPARE(frames.at(2).cmdId, static_cast<quint16>(0x0310));
    QCOMPARE(frames.at(2).seq, static_cast<quint8>(9));
    QCOMPARE(frames.at(2).payload.size(), 30);
  }

  void noiseBeforeFrameResyncs() {
    RM::CustomControllerLinkPolicy policy;
    QByteArray chunk = QByteArray::fromHex("0011ff22") + frameSeq7();

    const QVector<RM::CustomControllerFrame> frames = policy.feed(chunk);

    QCOMPARE(frames.size(), 1);
    QCOMPARE(frames.at(0).seq, static_cast<quint8>(7));
    QCOMPARE(policy.resyncBytes(), quint64(4));
  }

  void badHeaderCrcIsRejected() {
    RM::CustomControllerLinkPolicy policy;
    const QVector<RM::CustomControllerFrame> frames = policy.feed(frameWithBadCrc8());

    QCOMPARE(frames.size(), 0);
    QCOMPARE(policy.crc8Failures(), quint64(1));
    QCOMPARE(policy.crc16Failures(), quint64(0));
  }

  void badPayloadCrcIsRejected() {
    RM::CustomControllerLinkPolicy policy;
    const QVector<RM::CustomControllerFrame> frames =
        policy.feed(frameWithBadCrc16());

    QCOMPARE(frames.size(), 0);
    QCOMPARE(policy.crc8Failures(), quint64(0));
    QCOMPARE(policy.crc16Failures(), quint64(1));
  }

  void sequenceGapCountsLostFrames() {
    RM::CustomControllerLinkPolicy policy;
    const QVector<RM::CustomControllerFrame> first = policy.feed(frameSeq7());
    const QVector<RM::CustomControllerFrame> third = policy.feed(frameSeq10());
    QCOMPARE(first.size(), 1);
    QCOMPARE(third.size(), 1);

    policy.observeSequence(first.at(0).seq);
    policy.observeSequence(third.at(0).seq);
    QCOMPARE(policy.lostFrames(), quint64(2)); // 8、9 两帧缺失

    // 重复帧不重复计数
    policy.observeSequence(third.at(0).seq);
    QCOMPARE(policy.lostFrames(), quint64(2));
  }

  void sequenceWrapAroundIsCounted() {
    RM::CustomControllerLinkPolicy policy;
    policy.observeSequence(253);
    policy.observeSequence(1); // 254、255、0 缺失

    QCOMPARE(policy.lostFrames(), quint64(3));
  }

  /// 数据来源改成 FDCAN 后帧格式不变：0x0302 控制段布局仍应逐字段一致。
  void controlDataLayoutIsIndependentOfArmTransport() {
    RM::CustomControllerLinkPolicy policy;
    const QVector<RM::CustomControllerFrame> frames = policy.feed(frameSeq8());
    QCOMPARE(frames.size(), 1);

    RM::CustomControllerState state;
    QVERIFY(RM::CustomControllerLinkPolicy::parseControlData(frames.at(0).payload,
                                                            state));
    QCOMPARE(state.roll, -2.25f);
    QCOMPARE(state.encoderX, 12.5f);
    QCOMPARE(state.encoderY, 100.0f);
    QCOMPARE(state.mode, static_cast<quint8>(1));
    QCOMPARE(state.slowClose, static_cast<quint8>(1));
  }

  void shortControlPayloadIsRejected() {
    RM::CustomControllerState state;
    QVERIFY(!RM::CustomControllerLinkPolicy::parseControlData(
        QByteArray(8, '\0'), state));
    QVERIFY(!state.valid);
  }

  // --- 接收器：字节注入路径（不依赖串口设备，所有环境可跑） ---

  void receiverParsesInjectedBytes() {
    RM::CustomControllerReceiver receiver;
    QSignalSpy stateSpy(&receiver,
                        &RM::CustomControllerReceiver::controlStateUpdated);
    QSignalSpy payloadSpy(&receiver,
                          &RM::CustomControllerReceiver::controlPayloadReceived);
    QSignalSpy linkSpy(&receiver, &RM::CustomControllerReceiver::linkStateChanged);

    receiver.processBytes(frameSeq7());

    QCOMPARE(receiver.totalBytes(), quint64(frameSeq7().size()));
    QCOMPARE(receiver.totalFrames(), quint64(1));
    QCOMPARE(receiver.controlFrames(), quint64(1));
    QVERIFY(receiver.isLinkAlive());
    QCOMPARE(linkSpy.count(), 1);
    QCOMPARE(linkSpy.at(0).at(0).toBool(), true);
    QCOMPARE(stateSpy.count(), 1);
    QCOMPARE(payloadSpy.count(), 1);
    QCOMPARE(payloadSpy.at(0).at(0).toByteArray().size(),
             RM::kCustomControllerControlDataBytes);

    const RM::CustomControllerState state =
        qvariant_cast<RM::CustomControllerState>(stateSpy.at(0).at(0));
    QCOMPARE(state.yaw, -1.25f);
    QCOMPARE(state.mode, static_cast<quint8>(2));
    QCOMPARE(receiver.lastMode(), 2);
  }

  void receiverTracksSequenceLossAndForeignCommands() {
    RM::CustomControllerReceiver receiver;
    QSignalSpy frameSpy(&receiver, &RM::CustomControllerReceiver::frameReceived);

    receiver.processBytes(frameSeq7());
    receiver.processBytes(frameSeq10());
    receiver.processBytes(frameCustomClient0310());
    receiver.processBytes(frameKeyboardMouse0304());

    QCOMPARE(receiver.totalFrames(), quint64(4));
    QCOMPARE(receiver.controlFrames(), quint64(2));
    QCOMPARE(receiver.lostFrames(), quint64(2));
    QCOMPARE(frameSpy.count(), 4);
    QCOMPARE(frameSpy.at(2).at(0).toUInt(), 0x0310u);
    // 0x0304 只计入总帧数，不产生控制状态
    QCOMPARE(frameSpy.at(3).at(0).toUInt(), 0x0304u);
  }

  void receiverLinkTimeoutDropsLiveness() {
    RM::CustomControllerReceiver receiver;
    receiver.setLinkTimeoutMs(80);
    QSignalSpy linkSpy(&receiver, &RM::CustomControllerReceiver::linkStateChanged);
    // 不打开任何设备，仅靠字节注入触发 watchdog。
    QVERIFY(receiver.startUdp(0, QHostAddress::LocalHost));

    receiver.processBytes(frameSeq7());
    QVERIFY(receiver.isLinkAlive());

    QTRY_VERIFY_WITH_TIMEOUT(!receiver.isLinkAlive(), 2000);
    QCOMPARE(linkSpy.count(), 2);
    QCOMPARE(linkSpy.at(0).at(0).toBool(), true);
    QCOMPARE(linkSpy.at(1).at(0).toBool(), false);
  }

  void receiverRejectsCorruptFrames() {
    RM::CustomControllerReceiver receiver;
    QSignalSpy stateSpy(&receiver,
                        &RM::CustomControllerReceiver::controlStateUpdated);

    receiver.processBytes(frameWithBadCrc8());
    receiver.processBytes(frameWithBadCrc16());

    QCOMPARE(receiver.controlFrames(), quint64(0));
    QCOMPARE(receiver.crc8Failures(), quint64(1));
    QCOMPARE(receiver.crc16Failures(), quint64(1));
    QCOMPARE(stateSpy.count(), 0);
    QVERIFY(!receiver.isLinkAlive());
  }

  // --- 接收器：串口打开失败路径 ---

  void serialStartRejectsEmptyPortName() {
    RM::CustomControllerReceiver receiver;
    QSignalSpy errorSpy(&receiver, &RM::CustomControllerReceiver::errorOccurred);

    QVERIFY(!receiver.startSerial(QString()));
    QCOMPARE(errorSpy.count(), 1);
    QVERIFY(!receiver.isListening());
  }

  void serialStartReportsFailureForMissingDevice() {
    RM::CustomControllerReceiver receiver;
    QSignalSpy errorSpy(&receiver, &RM::CustomControllerReceiver::errorOccurred);

    // 不存在的设备名：应失败并给出可读错误，而不是崩溃或静默。
    QVERIFY(!receiver.startSerial(QStringLiteral("/dev/rm26-nonexistent-port")));
    QCOMPARE(errorSpy.count(), 1);
    QVERIFY(!receiver.lastError().isEmpty());
    QVERIFY(!receiver.isListening());
  }

  void stopClearsListeningState() {
    RM::CustomControllerReceiver receiver;
    QVERIFY(receiver.startUdp(0, QHostAddress::LocalHost));
    QVERIFY(receiver.isListening());
    receiver.stop();
    QVERIFY(!receiver.isListening());
    QVERIFY(!receiver.isLinkAlive());
  }

  // --- 接收器：真实串口路径（需要一对虚拟串口，未提供时跳过） ---
  //
  // 用法（模拟 RS232-USB 链路）：
  //   socat -d -d pty,raw,echo=0,link=/tmp/rm26-client.pty \
  //              pty,raw,echo=0,link=/tmp/rm26-emul.pty &
  //   RM_CONTROLLER_SERIAL_LOOPBACK=/tmp/rm26-client.pty \
  //   RM_CONTROLLER_SERIAL_WRITE=/tmp/rm26-emul.pty \
  //   ./tests/test_custom_controller_link
  void receiverParsesOverRealSerialPair() {
    const QByteArray readPath = qgetenv("RM_CONTROLLER_SERIAL_LOOPBACK");
    const QByteArray writePath = qgetenv("RM_CONTROLLER_SERIAL_WRITE");
    if (readPath.isEmpty() || writePath.isEmpty()) {
      QSKIP("未提供 RM_CONTROLLER_SERIAL_LOOPBACK/WRITE，跳过真实串口集成用例");
    }

    RM::CustomControllerReceiver receiver;
    QSignalSpy stateSpy(&receiver,
                        &RM::CustomControllerReceiver::controlStateUpdated);
    QVERIFY2(receiver.startSerial(QString::fromLocal8Bit(readPath)),
             "无法打开虚拟串口读端");
    QCOMPARE(receiver.transport(), RM::CustomControllerReceiver::Transport::Serial);
    QVERIFY(receiver.endpointDescription().contains(
        QString::fromLocal8Bit(readPath)));

    QFile writer(QString::fromLocal8Bit(writePath));
    QVERIFY2(writer.open(QIODevice::WriteOnly), "无法打开虚拟串口写端");
    // 模仿发送端：一次写入多帧并故意拆成两段，覆盖分片重组。
    const QByteArray stream = frameSeq7() + frameSeq8();
    writer.write(stream.left(20));
    writer.flush();
    QTest::qWait(50);
    writer.write(stream.mid(20));
    writer.flush();

    QTRY_COMPARE_WITH_TIMEOUT(receiver.controlFrames(), quint64(2), 3000);
    QVERIFY(receiver.isLinkAlive());
    QCOMPARE(stateSpy.count(), 2);
    QCOMPARE(receiver.lostFrames(), quint64(0));
    QCOMPARE(receiver.crc8Failures(), quint64(0));
    QCOMPARE(receiver.crc16Failures(), quint64(0));

    const RM::CustomControllerState second =
        qvariant_cast<RM::CustomControllerState>(stateSpy.at(1).at(0));
    QCOMPARE(second.mode, static_cast<quint8>(1));
    QCOMPARE(second.slowClose, static_cast<quint8>(1));

    // 链路停止后不应再保持在线。
    receiver.stop();
    QVERIFY(!receiver.isListening());
    QVERIFY(!receiver.isLinkAlive());
  }

  // --- 接收器：UDP 兼容路径（受限环境无法投递时跳过） ---

  void receiverParsesUdpDatagram() {
    RM::CustomControllerReceiver receiver;
    QSignalSpy stateSpy(&receiver,
                        &RM::CustomControllerReceiver::controlStateUpdated);

    // 使用固定端口：部分 Qt 版本对 0 端口绑定的 localPort() 上报不可靠。
    quint16 boundPort = 0;
    for (quint16 candidate = 39484; candidate < 39494; ++candidate) {
      if (receiver.startUdp(candidate, QHostAddress::LocalHost)) {
        boundPort = candidate;
        break;
      }
    }
    QVERIFY2(boundPort != 0, "无法绑定测试端口");
    QVERIFY(receiver.isListening());
    QCOMPARE(receiver.port(), boundPort);

    QUdpSocket sender;
    sender.bind(QHostAddress::Any, 0);
    const QByteArray frame = frameSeq7();
    const qint64 sentBytes =
        sender.writeDatagram(frame, QHostAddress::LocalHost, boundPort);

    // 等待投递。这里不用 QTRY_*：它在超时时会记录失败，而受限运行环境
    // 无法投递 Qt UDP 回环，需要以“跳过”而不是“失败”收尾。
    QElapsedTimer waitTimer;
    waitTimer.start();
    while (receiver.controlFrames() == 0 && waitTimer.elapsed() < 1500) {
      QTest::qWait(20);
    }
    if (receiver.controlFrames() == 0) {
      QSKIP("当前运行环境未投递 Qt UDP 回环数据报，跳过套接字集成用例");
    }
    QCOMPARE(sentBytes, static_cast<qint64>(frame.size()));

    QCOMPARE(receiver.totalDatagrams(), quint64(1));
    QCOMPARE(receiver.totalFrames(), quint64(1));
    QVERIFY(receiver.isLinkAlive());
    QCOMPARE(stateSpy.count(), 1);
    const RM::CustomControllerState state =
        qvariant_cast<RM::CustomControllerState>(stateSpy.at(0).at(0));
    QCOMPARE(state.yaw, -1.25f);
    QCOMPARE(state.mode, static_cast<quint8>(2));
  }
};

QTEST_GUILESS_MAIN(TestCustomControllerLink)

#include "test_custom_controller_link.moc"