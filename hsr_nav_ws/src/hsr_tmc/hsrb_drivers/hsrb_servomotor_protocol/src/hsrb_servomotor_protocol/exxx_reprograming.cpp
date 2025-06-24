/*
Copyright (c) 2017 TOYOTA MOTOR CORPORATION
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted (subject to the limitations in the disclaimer
below) provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may be used
  to endorse or promote products derived from this software without specific
  prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
DAMAGE.
*/
#include <algorithm>
#include <iomanip>
#include <string>
#include <vector>
#include <boost/lexical_cast.hpp>
#include <fcntl.h>
#include <linux/serial.h>
#include <poll.h>
#include <rclcpp/rclcpp.hpp>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#include <hsrb_servomotor_protocol/cti485.hpp>
#include <hsrb_servomotor_protocol/exxx_reprograming.hpp>

namespace {
const uint32_t kSendCommandMaxRetryCount      = 10;                 /// コマンド送信の最大リトライ回数
const uint32_t kFlushXmodemMaxRetryCount      = 100;                /// XMODEM通信の受信最大リトライ回数
const uint8_t  kEraseCommand                  = 0x65;               /// イレースコマンド 'e'
const uint8_t  kFlushCommand                  = 0x77;               /// ファームウェア書き込みコマンド 'w'
const uint8_t  kRunCommand                    = 0x67;               /// ファームウェア起動コマンド 'g'
const uint8_t  kExecuteCommand                = 0x79;               /// コマンド実行 'y'
const char*    kBootMessage                   = "Bootloader";       /// PROPアンプブート部起動メッセージ
const char*    kEraseCheckMessage             = "ERASE ALL FLASH";  /// イレース確認メッセージ
const char*    kEraseFinishMessage            = "COMPLETED";        /// イレース完了メッセージ
const char*    kFlushCheckMessage             = "START UPLOAD";     /// ファームウェア書き込み確認メッセージ
const char*    kFlushFinishMessage            = "SUCCESS";          /// ファームウェア書き込み完了メッセージ
const char*    kRunCheckMessage               = "GO";               /// ファームウェア起動メッセージ
const uint8_t  kXmodemSOH                     = 0x01;               /// XMODEM通信 ブロック開始
const uint8_t  kXmodemEOT                     = 0x04;               /// XMODEM通信 転送終了
const uint8_t  kXmodemACK                     = 0x06;               /// XMODEM通信 正常応答
const uint8_t  kXmodemNAK                     = 0x15;               /// XMODEM通信 送信要求および否定応答
const uint8_t  kXmodemCAN                     = 0x18;               /// XMODEM通信 中断
const uint8_t  kXmodemEOF                     = 0xFF;               /// XMODEM通信 パディング用
const uint32_t kXmodemSendSize                = 132;                /// XMODEM通信 送信データサイズ
const uint32_t kXmodemDataSize                = 128;                /// XMODEM通信 データ部サイズ
const uint32_t kXmodemHeaderSize              = 3;                  /// XMODEM通信 ヘッダ＋ブロック番号サイズ
const uint32_t kXmodemHeaderPos               = 0;                  /// XMODEM通信 送信データのヘッダ位置
const uint32_t kXmodemBlockNumberPos          = 1;                  /// XMODEM通信 送信データのブロック番号位置
const uint32_t kXmodemBlockNumberCompPos      = 2;                  /// XMODEM通信 送信データのブロック番号の1の補数位置
const uint32_t kXmodemCheckSumPos             = 131;                /// XMODEM通信 送信データのチェックサム位置
const int64_t  kBootStartUpWaitTime           = 3000000000;         /// PROPアンプブート部起動待ち時間[ns]
const int64_t  kReproIntervalTime             = 1000000000;         /// PROPアンプ起動待ちの'.'出力周期[ns]
const int64_t  kCommandIntervalTime           = 50000000;           /// コマンド送信周期[ns]
const int64_t  kEraseWaitTime                 = 1000000000;         /// イレース完了待ち時間[ns]
const int64_t  kFlushIntervalTime             = 10000000;           /// ファームウェア書き込み XMODEM通信受信周期[ns]
const uint8_t  kReceiveLineDelimiter1         = 0x0A;               /// '\n'(line feed)
const uint8_t  kReceiveLineDelimiter2         = 0x3F;               /// '?'

/**
 * @brief 指定時間待ち[ns]
 *
 * @param[in]  nsec         待ち時間[ns]
 * @return
 * boost::system::errc::success  指定時間待ち成功
 * boost::system::errc::invalid_argument  指定時間待ち失敗
 */
boost::system::error_code WaitNanoSec(int64_t nsec) {
  boost::system::error_code error(boost::system::errc::success, boost::system::system_category());
  timespec duration;
  duration.tv_sec  = nsec / 1000000000LL;
  duration.tv_nsec = nsec % 1000000000LL;

  while (clock_nanosleep(CLOCK_MONOTONIC, 0, &duration, &duration)) {
    if (errno == EINTR) {
      continue;
    } else {
      // EFAULTまたはEINVAL
      // どちらもclock_nanosleep関数の引数により発生するため、戻り値にinvalid_argumentを設定する
      error = boost::system::error_code(boost::system::errc::invalid_argument, boost::system::system_category());
      break;
    }
  }
  return error;
}
}  // anonymous namespace

namespace hsrb_servomotor_protocol {

/**
 * @brief コンストラクタ
 *
 * @param[in]   device_name   デバイス名
 * @param[in]   is_usb_rs485  USB使用有無(true:USB使用, false:USB未使用)
 * @param[in]   timeout       送受信のタイムアウト時間[ns]
 * @param[in]   sleep_tick    write関数の戻り値がEAGINの場合のリトライ待ち時間[ns]
 * @retval  void
 */
ExxxReprograming::ExxxReprograming(const std::string& device_name, bool is_usb_rs485, uint32_t baudrate,
                         int64_t timeout, int64_t sleep_tick)
    : fd_(-1), device_name_(device_name), is_usb_rs485_(is_usb_rs485), baudrate_(baudrate),
      timeout_(timeout), sleep_tick_(sleep_tick) {}

/**
 * @brief デストラクタ
 *
 * ファイルディスクリプタ(通信用デバイス)をクローズする
 *
 * @param   void
 * @retval  void
 */
ExxxReprograming::~ExxxReprograming() { (void)close(fd_); }

/**
 * @brief 通信用デバイスをオープンする
 *
 * @return
 * boost::system::errc::success  デバイスオープン成功
 * open, tcgetattr,  tcsetattr, ioctl, tcflush, cfsetispeed, tcsetattrで発生するエラーを返す
 */
ExxxReprograming::ErrorCode ExxxReprograming::Open() {
  boost::system::error_code error(boost::system::errc::success, boost::system::system_category());
  int port = open(device_name_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (port < 0) {
    error = boost::system::error_code(errno, boost::system::system_category());
  } else {
    fd_ = port;
    // デバイスを初期化する
    // エラーが発生した場合は、戻り値にerrnoを設定し、処理終了する

    // Set raw mode
    termios term = { 0 };
    if (tcgetattr(fd_, &term)) {
      error = boost::system::error_code(errno, boost::system::system_category());
    } else {
      if (!is_usb_rs485_) {
        // 8bit
        // stop bit 1
        // no parity
        // no modem control
        // enable receiving characters0
        term.c_iflag = IGNPAR;
        term.c_cflag = baudrate_ | CS8 | CLOCAL | CREAD;
        term.c_oflag = OPOST;
        term.c_lflag = 0;
        term.c_cc[VTIME] = 0;
        term.c_cc[VMIN] = 1;
        if (tcsetattr(fd_, TCSANOW, &term)) {
          error = boost::system::error_code(errno, boost::system::system_category());
        }

        // set low latency
        serial_struct serial = { 0 };
        if (error.value() == boost::system::errc::success) {
          if (ioctl(fd_, TIOCGSERIAL, &serial)) {
            error = boost::system::error_code(errno, boost::system::system_category());
          }
        }

        if (error.value() == boost::system::errc::success) {
          serial.flags |= ASYNC_LOW_LATENCY;
          if (ioctl(fd_, TIOCSSERIAL, &serial)) {
            error = boost::system::error_code(errno, boost::system::system_category());
          }
        }

        // Flushing port
        if (error.value() == boost::system::errc::success) {
          if (tcflush(fd_, TCIOFLUSH)) {
            error = boost::system::error_code(errno, boost::system::system_category());
          }
        }

        // RS485 Half-Duplexモードをセット
        int value = 0;
        if (error.value() == boost::system::errc::success) {
          if (ioctl(fd_, hsrb_servomotor_protocol::kCti485GetMode, &value) == -1) {
            error = boost::system::error_code(errno, boost::system::system_category());
          }
        }
        if (error.value() == boost::system::errc::success) {
          if (value != hsrb_servomotor_protocol::kCti485HalfDuplexMode) {
            value = hsrb_servomotor_protocol::kCti485HalfDuplexMode;
            if (ioctl(fd_, hsrb_servomotor_protocol::kCti485SetMode, &value) < 0) {
              error = boost::system::error_code(errno, boost::system::system_category());
            }
          }
        }
      } else {
        cfmakeraw(&term);
        if (cfsetispeed(&term, baudrate_)) {
          error = boost::system::error_code(errno, boost::system::system_category());
        }
        if (error.value() == boost::system::errc::success) {
          if (tcsetattr(fd_, TCSANOW, &term)) {
            error = boost::system::error_code(errno, boost::system::system_category());
          }
        }
      }
    }
  }
  return error;
}

/**
 * @brief PROPアンプブート部起動処理
 *
 * 軸番号を送信し、PROPアンプのブート部を起動する。
 * 通常リプロ時は、PROPアンプリセット後に呼び出す。
 * 強制リプロ時は、PROPアンプの電源OFF状態で呼び出す。
 *
 * 起動処理の通信
 *   ホスト -  PROPアンプ
 *  軸番号  -> 
 *  軸番号  -> 
 *  軸番号  -> 
 *  軸番号  -> 
 *  軸番号  -> 
 *         <-  \r\n====< Bootloader Ver.1.0.5 by TMC >====
 *         <-  \r\n -- for No_軸番号 -- \r\n
 *         <-  \r\n [w]:UPLOAD [g]:BOOT [e]:ERASE [x]:RESET [?]:MENU\r\n
 *         <-  >
 *
 * @param[in]  bootloader_version ブートローダのバージョン
 * @param[in]  id                 PROPアンプのブート部の軸番号
 * @param[in]  boot_timeout       PROPアンプの起動タイムアウト時間[ns]
 * @return
 * boost::system::errc::success  PROPアンプブート部起動成功
 * boost::system::errc::timed_out  送信タイムアウト
 * boost::system::errc::no_message  PROPアンプブート部起動メッセージ未受信でタイムアウト
 * boost::system::errc::value_too_large  タイムアウト計測用の変数がオーバーフローした
 * boost::system::errc::invalid_argument  指定時間待ち失敗
 * poll, read,  writeで発生するエラーを返す
 */
ExxxReprograming::ErrorCode ExxxReprograming::WaitBoot(const int32_t bootloader_version,
                                                       const uint8_t id,
                                                       const int64_t boot_timeout) {
  std::string axis_number = "No_";
  std::string boot_version = "Ver.";
  std::string receive_message;
  std::string boot_message;
  uint32_t output_count = 0;
  int64_t boot_start = Now();
  int64_t start = boot_start;
  int64_t elapsed = boot_start;
  uint8_t receive_data;
  bool is_boot_message = false;
  bool is_axis_number = false;
  boost::system::error_code error;

  axis_number.push_back(id);
  while (true) {
    error = SendData(&id, 1);
    if (error.value() != boost::system::errc::success) {
      break;
    }
    while (true) {
      // 起動メッセージ受信確認
      error = ReceiveLine(receive_message);
      if (error.value() != boost::system::errc::success) {
        break;
      } else {
        // 受信データ内に起動メッセージがあるか確認する
        // 起動メッセージを受信している場合、ブート部起動成功
        boot_message = boot_message + receive_message;
        is_boot_message = (boot_message.find(kBootMessage) != std::string::npos);
        is_axis_number = (boot_message.find(axis_number) != std::string::npos);

        if (is_boot_message && is_axis_number) {
          // 起動メッセージ受信後、ブートローダのバージョン確認を行う。
          boot_version.push_back(bootloader_version + '0');
          if (boot_message.find(boot_version) == std::string::npos) {
            error = boost::system::error_code(boost::system::errc::not_supported, boost::system::system_category());
            break;
          }
          // PROPアンプのブート部は、軸番号受信後の起動メッセージ送信まで待ち時間がある。
          // 待ち時間の間も軸番号を送信するため、ブート部は待ち時間の間に受信した軸番号に対する応答を返す。
          // 起動メッセージ以外の受信データは、以降のリプロ処理に不要なため受信データを読み捨てる。
          while (true) {
            error = ReceiveByte(receive_data);
            if (error.value() != boost::system::errc::success) {
              break;
            }
          }
          // 読み捨てが正常終了した場合、errorはtimed_outになる
          // errorがtimed_outの場合は、戻り値にsuccessを設定する
          if (error.value() == boost::system::errc::timed_out) {
            error = boost::system::error_code(boost::system::errc::success, boost::system::system_category());
          }
          break;
        }
      }
    }
    if (is_boot_message && is_axis_number) {
      // 起動メッセージを受信した場合は、処理終了する
      break;
    }
    if ((error.value() != boost::system::errc::success) &&
        (error.value() != boost::system::errc::timed_out)) {
      // 異常検出のため処理終了する
      break;
    } else {
      elapsed = Now();
      // 1秒毎に . を出力する
      if (((elapsed - start) >= kReproIntervalTime) && (output_count < (boot_timeout / kReproIntervalTime))) {
        std::cout << "." << std::flush;
        start = elapsed;
        output_count++;
      }
      if ((elapsed - boot_start) >= (boot_timeout + kBootStartUpWaitTime)) {
        // タイムアウト時間内で起動メッセージを受信できなかった場合は、戻り値にno_messageを設定し、処理終了する
        error = boost::system::error_code(boost::system::errc::no_message, boost::system::system_category());
        break;
      }
      if (elapsed < start) {
        // オーバーフロー
        error = boost::system::error_code(boost::system::errc::value_too_large, boost::system::system_category());
        break;
      }
    }
    error = WaitNanoSec(kCommandIntervalTime);
    if (error.value() != boost::system::errc::success) {
      break;
    }
  }
  return error;
}

/**
 * @brief PROPアンプイレース処理
 *
 * PROPアンプのファームウェアを消去する。
 *
 * イレース処理の通信
 *   ホスト -  PROPアンプ
 *       e  -> 
 *         <-  ERASE ALL FLASH ARE YOU SURE? (Y/N) 
 *       y  -> 
 *         <-  y\r\n
 *         <-  .
 *         <-  .
 *         <-  SUCCESS
 *
 * @return
 * boost::system::errc::success  PROPアンプイレース成功
 * boost::system::errc::timed_out  イレース処理タイムアウト
 * boost::system::errc::no_message  イレースコマンド確認メッセージ未受信でタイムアウト
 * boost::system::errc::value_too_large  タイムアウト計測用の変数がオーバーフローした
 * boost::system::errc::invalid_argument  指定時間待ち失敗
 * poll, read,  writeで発生するエラーを返す
 */
ExxxReprograming::ErrorCode ExxxReprograming::Erase() {
  int64_t start;
  int64_t elapsed;
  boost::system::error_code error;

  error = SendCommand(kEraseCommand, kEraseCheckMessage);
  if (error.value() == boost::system::errc::timed_out) {
    // イレースコマンドに対する応答がない場合は、戻り値にno_messageを設定する
    error = boost::system::error_code(boost::system::errc::no_message, boost::system::system_category());
  }
  if (error.value() == boost::system::errc::success) {
    error = SendData(&kExecuteCommand, 1);
  }
  if (error.value() == boost::system::errc::success) {
    start = Now();
    elapsed = start;
    // イレース完了待ち時間経過するまで、イレース完了メッセージの受信を確認
    while (true) {
      error = CheckReceivedMessage(kEraseFinishMessage);
      if (error.value() == boost::system::errc::success) {
        // イレース完了メッセージを受信したため、処理終了する
        break;
      } else if ((error.value() != boost::system::errc::illegal_byte_sequence) &&
                 (error.value() != boost::system::errc::timed_out)) {
        // タイムアウトおよび確認用メッセージ未受信以外のエラーが発生した場合は、処理終了する
        break;
      } else {
        // イレース完了メッセージ未受信のため処理を継続する
      }
      error = WaitNanoSec(kCommandIntervalTime);
      if (error.value() != boost::system::errc::success) {
        break;
      }
      elapsed = Now();
      if ((elapsed - start) >= kEraseWaitTime) {
        // イレース完了待ち時間経過した場合は、戻り値にtimed_outを設定し、処理終了する
        error = boost::system::error_code(boost::system::errc::timed_out, boost::system::system_category());
        break;
      }
      if (elapsed < start) {
        // オーバーフロー
        error = boost::system::error_code(boost::system::errc::value_too_large, boost::system::system_category());
        break;
      }
    }
  }
  return error;
}

/**
 * @brief PROPアンプファームウェア書き込み処理
 * 
 * PROPアンプへファームウェア書き込みデータを送信し、ファームウェアの書き換えを行う。
 * ファームウェア書き込みデータの送信は、XMODEMプロトコルで行う。
 * 対応するXMODEMの種類は、XMODEM/SUMとする。
 * 
 * ファームウェア書き込み処理の通信
 *   ホスト -  PROPアンプ
 *       w  -> 
 *         <-  START UPLOAD...\r\n
 *         <-  ARE YOU SURE? (Y/N) 
 *       y  -> 
 *         <-  y\r\n
 *
 *     (XMODEM通信)
 *
 *         <-  <SUCCESS>\r\n
 *
 * @param[in]  flush_data  ファームウェア書き込みデータ
 * @return
 * boost::system::errc::success  PROPアンプファームウェア書き込み成功
 * boost::system::errc::timed_out  PROPアンプ間通信タイムアウト
 * boost::system::errc::no_message  フラッシュ書き込みコマンド確認メッセージ未受信でタイムアウト
 * boost::system::errc::value_too_large  タイムアウト計測用の変数がオーバーフローした
 * boost::system::errc::invalid_argument  指定時間待ち失敗
 * poll, read, writeで発生するエラーを返す
 */
ExxxReprograming::ErrorCode ExxxReprograming::Flush(const std::string& flush_data) {
  boost::system::error_code error;
  uint8_t receive_data;

  error = SendCommand(kFlushCommand, kFlushCheckMessage);
  if (error.value() == boost::system::errc::timed_out) {
    // フラッシュ書き込みコマンドに対する応答がない場合は、戻り値にno_messageを設定する
    error = boost::system::error_code(boost::system::errc::no_message, boost::system::system_category());
  } else if (error.value() != boost::system::errc::success) {
    // タイムアウト以外のエラーが発生した場合は、処理終了する
  } else {
    // フラッシュ書き込みコマンドに対する応答が複数行であるため、不要な受信データを読み捨てる
    while (true) {
      error = ReceiveByte(receive_data);
      if (error.value() != boost::system::errc::success) {
        break;
      }
    }
    // 読み捨てが正常終了した場合、errorはtimed_outになる
    // errorがtimed_outの場合、コマンド送信を行う
    if (error.value() == boost::system::errc::timed_out) {
      error = SendData(&kExecuteCommand, 1);
    }
  }
  if (error.value() == boost::system::errc::success) {
    error = FlushXmodem(flush_data);
  }
  return error;
}

/**
 * @brief PROPアンプファームウェア実行処理
 *
 * PROPアンプのブート部を終了し、ファームウェアを実行する。
 *
 * ファームウェア実行処理の通信
 *   ホスト -  PROPアンプ
 *       g  -> 
 *         <-  GO !!\r\n
 *
 * @param   void
 * @return
 * boost::system::errc::success  PROPアンプファームウェア実行成功
 * boost::system::errc::no_message  ファームウェア起動コマンド応答メッセージ未受信でタイムアウト
 * boost::system::errc::value_too_large  タイムアウト計測用の変数がオーバーフローした
 * boost::system::errc::invalid_argument  指定時間待ち失敗
 * writeで発生するエラーを返す
 */
ExxxReprograming::ErrorCode ExxxReprograming::Run() {
  boost::system::error_code error;

  error = SendCommand(kRunCommand, kRunCheckMessage);
  if (error.value() == boost::system::errc::timed_out) {
    // ファームウェア起動コマンドに対する応答がない場合は、戻り値にno_messageを設定する
    error = boost::system::error_code(boost::system::errc::no_message, boost::system::system_category());
  }
  return error;
}

/**
 * @brief XMODEM通信によるファームウェア書き込み処理
 * 
 * XMODEMプロトコルでファームウェア書き込みデータを送信する。
 * 対応するXMODEMの種類は、XMODEM/SUMとする。
 * 
 * @param[in]  flush_data  ファームウェア書き込みデータ
 * @return
 * boost::system::errc::success  PROPアンプファームウェア書き込み成功
 * boost::system::errc::timed_out  PROPアンプ間通信タイムアウト, XMODEM通信中断
 * boost::system::errc::value_too_large  タイムアウト計測用の変数がオーバーフローした
 * boost::system::errc::invalid_argument  指定時間待ち失敗
 * poll, read, writeで発生するエラーを返す
 */
ExxxReprograming::ErrorCode ExxxReprograming::FlushXmodem(const std::string& flush_data) {
  boost::system::error_code error;
  uint32_t retry_count = 0;
  uint32_t send_pos = 0;
  uint32_t output_count = 0;
  uint32_t flush_ratio = 0;
  uint8_t block_number = 1;
  uint8_t receive_data;
  bool is_flush_xmodem = true;

  // NAK(送信要求)受信待ち
  while (true) {
    error = ReceiveByte(receive_data);
    if ((error.value() == boost::system::errc::success) && (receive_data == kXmodemNAK)) {
      break;
    } else if ((error.value() != boost::system::errc::success) &&
               (error.value() != boost::system::errc::timed_out)) {
      // タイムアウト以外のエラー発生時は、処理終了する
      break;
    } else {
      // NAK未受信のため処理を継続する
    }
    retry_count++;
    if (retry_count >= kFlushXmodemMaxRetryCount) {
      // リトライ回数が規定数を超えた場合は、戻り値にtimed_outを設定し処理終了する
      error = boost::system::error_code(boost::system::errc::timed_out, boost::system::system_category());
      break;
    }
  }
  // 先頭ブロックを送信
  if (error.value() == boost::system::errc::success) {
    error = SendFlushBlockData(flush_data, send_pos, block_number);
  }
  if (error.value() == boost::system::errc::success) {
    error = WaitNanoSec(kFlushIntervalTime);
  }
  if (error.value() == boost::system::errc::success) {
    retry_count = 0;
    // XMODEM通信で、ファームウェア書き込みデータを送信する
    // 受信データなし、またはNAK受信が10回続いた場合は戻り値にtimed_outを設定し、処理終了する
    // CAN(中断)受信時は、戻り値にtimed_outを設定し、処理終了する(再送等リトライはしない)
    while (is_flush_xmodem) {
      receive_data = 0;
      error = ReceiveByte(receive_data);
      if (error.value() == boost::system::errc::timed_out) {
        retry_count++;
      } else if (error.value() != boost::system::errc::success) {
        break;
      } else {
        switch (receive_data) {
          case kXmodemACK:
            // PROPからの正常応答を受信。次のデータを送信する
            retry_count = 0;
            send_pos += kXmodemDataSize;
            block_number++;
            // 10%書きこみ毎に . を出力する
            flush_ratio = ((send_pos * 10) / flush_data.length());
            if (flush_ratio > 10) {
              flush_ratio = 10;
            }
            for (uint32_t i = output_count; i < flush_ratio; ++i) {
              std::cout << "." << std::flush;
            }
            output_count = flush_ratio;
            if (send_pos >= flush_data.length()) {
              // 書き込みデータ送信終了のため、EOT(転送終了)を送信
              // PROPアンプファームウェアは、内部で残りの書き込みデータを持っている場合、書き込みを行い応答を返す。
              // 書き込みデータを持っていない場合は、直ぐに応答を返す。
              error = SendCommand(kXmodemEOT, kFlushFinishMessage);
              is_flush_xmodem = false;
            } else {
              error = SendFlushBlockData(flush_data, send_pos, block_number);
              if (error.value() != boost::system::errc::success) {
                is_flush_xmodem = false;
              }
            }
            break;
          case kXmodemNAK:
            // 再送
            retry_count++;
            if (retry_count < kFlushXmodemMaxRetryCount) {
              error = SendFlushBlockData(flush_data, send_pos, block_number);
              if (error.value() != boost::system::errc::success) {
                is_flush_xmodem = false;
              }
            }
            break;
          case kXmodemCAN:
            // 中断。戻り値にtimed_outを設定し、処理終了する
            error = boost::system::error_code(boost::system::errc::timed_out, boost::system::system_category());
            is_flush_xmodem = false;
            break;
          default :
            retry_count++;
            break;
        }
      }
      if (retry_count >= kFlushXmodemMaxRetryCount) {
        // リトライ回数が規定数を超えた場合は、戻り値にtimed_outを設定し処理終了する
        error = boost::system::error_code(boost::system::errc::timed_out, boost::system::system_category());
        break;
      }
      // XMODEM通信終了後にerrorが書き換えられないようis_flush_xmodemでガードする
      if (is_flush_xmodem) {
        error = WaitNanoSec(kFlushIntervalTime);
        if (error.value() != boost::system::errc::success) {
          break;
        }
      }
    }
  }
  return error;
}

/**
 * @brief コマンドを送信し、確認用メッセージの受信を確認
 *
 * @param[in]  command        送信コマンド
 * @param[in]  check_message  確認用メッセージ
 * @return
 * boost::system::errc::success  確認用メッセージを受信
 * boost::system::errc::timed_out  コマンド送信タイムアウトおよびメッセージ受信タイムアウト
 * boost::system::errc::value_too_large  タイムアウト計測用の変数がオーバーフローした
 * boost::system::errc::invalid_argument  指定時間待ち失敗
 * writeで発生するエラーを返す
 */
ExxxReprograming::ErrorCode ExxxReprograming::SendCommand(const uint8_t command, const std::string& check_message) {
  uint32_t retry_count = 0;
  boost::system::error_code error;

  while (true) {
    error = SendData(&command, 1);
    if (error.value() != boost::system::errc::success) {
      break;
    }
    error = CheckReceivedMessage(check_message);
    if (error.value() == boost::system::errc::success) {
      // 確認用メッセージを受信したので処理終了する
      break;
    } else if ((error.value() != boost::system::errc::timed_out) &&
               (error.value() != boost::system::errc::illegal_byte_sequence)) {
      // タイムアウトおよび確認用メッセージ未受信以外の場合は、リトライせずに処理終了する
      break;
    } else {
      retry_count++;
      if (retry_count >= kSendCommandMaxRetryCount) {
        // リトライ回数が規定数を超えた場合は、戻り値にtimed_outを設定し処理終了する
        error = boost::system::error_code(boost::system::errc::timed_out, boost::system::system_category());
        break;
      }
    }
    error = WaitNanoSec(kCommandIntervalTime);
    if (error.value() != boost::system::errc::success) {
      break;
    }
  }
  return error;
}

/**
 * @brief 確認用メッセージの受信を確認
 *
 * @param[in]  check_message  確認用メッセージ
 * @return
 * boost::system::errc::success  確認用メッセージを受信
 * boost::system::errc::illegal_byte_sequence  確認用メッセージ未受信
 * boost::system::errc::timed_out  メッセージ受信タイムアウト
 * boost::system::errc::value_too_large  受信タイムアウト計測用の変数がオーバーフローした
 * poll, readで発生するエラーを返す
 */
ExxxReprograming::ErrorCode ExxxReprograming::CheckReceivedMessage(const std::string& check_message) {
  std::string receive_message;
  boost::system::error_code error;

  error = ReceiveLine(receive_message);
  if (error.value() == boost::system::errc::success) {
    // 1行受信成功した場合は、確認用メッセージを受信しているか確認する
    // 受信データ内に確認用メッセージがない場合は、戻り値にillegal_byte_sequenceを設定する
    if (receive_message.find(check_message) == std::string::npos) {
      error = boost::system::error_code(boost::system::errc::illegal_byte_sequence, boost::system::system_category());
    }
  }
  return error;
}

/**
 * @brief XMODEM通信 ファームウェア書き込みデータ送信
 *
 * @param[in]  flush_data     ファームウェア書き込みデータ
 * @param[in]  send_pos       書き込み開始位置
 * @param[in]  block_number   ブロック番号
 * @return
 * boost::system::errc::success  書き込みデータ送信成功
 * boost::system::errc::timed_out  書き込みデータ送信タイムアウト
 * boost::system::errc::value_too_large  書き込みデータ送信タイムアウト計測用の変数がオーバーフローした
 * boost::system::errc::invalid_argument  指定時間待ち失敗
 * writeで発生するエラーを返す
 */
ExxxReprograming::ErrorCode ExxxReprograming::SendFlushBlockData(const std::string& flush_data,
                                                                 uint32_t send_pos,
                                                                 uint8_t block_number) {
  boost::system::error_code error;
  uint32_t data_size = kXmodemDataSize;
  uint8_t check_sum = 0;
  std::array<uint8_t, kXmodemSendSize> send_buffer;

  send_buffer[kXmodemHeaderPos]          = kXmodemSOH;
  send_buffer[kXmodemBlockNumberPos]     = block_number;
  send_buffer[kXmodemBlockNumberCompPos] = (~block_number & 0xFF);
  // 残りデータが128より小さいか確認
  if ((flush_data.length() - send_pos) < kXmodemDataSize) {
    data_size = flush_data.length() - send_pos;
  }
  for (uint32_t i = 0; i < data_size; ++i) {
    send_buffer[kXmodemHeaderSize + i] = flush_data[send_pos + i];
  }
  // 書き込みデータが128byteに足りない場合は、残りをEOFで埋める
  for (uint32_t i = data_size; i < kXmodemDataSize; ++i) {
    send_buffer[kXmodemHeaderSize + i] = kXmodemEOF;
  }
  // チェックサム計算 データ部分を足し算
  for (uint32_t i = 0; i < kXmodemDataSize; ++i) {
    // チェックサムは下位8ビットの2補数
    check_sum += send_buffer[kXmodemHeaderSize + i];
  }
  send_buffer[kXmodemCheckSumPos] = check_sum;
  error = SendData(&send_buffer[0], kXmodemSendSize);
  return error;
}

/**
 * @brief 指定バイト送信
 *
 * @param[in]  data        送信バッファ
 * @param[in]  data_bytes  送信バイト数
 * @return
 * boost::system::errc::success  送信バイト数のデータを送信成功
 * boost::system::errc::timed_out  送信バイト数のデータを送信できずにタイムアウト
 * boost::system::errc::value_too_large  タイムアウト計測用の変数がオーバーフローした
 * boost::system::errc::invalid_argument  指定時間待ち失敗
 * writeで発生するエラーを返す
 */
ExxxReprograming::ErrorCode ExxxReprograming::SendData(const uint8_t* data, uint32_t data_bytes) {
  boost::system::error_code error;
  int64_t start = Now();
  int64_t elapsed = start;
  uint32_t num_done = 0;

  while ((elapsed - start) < timeout_) {
    int32_t result = write(fd_, &data[num_done], data_bytes - num_done);
    if (result < 0) {
      if (errno != EAGAIN) {
        error = boost::system::error_code(errno, boost::system::system_category());
        break;
      } else {
        // 戻り値がEAGAINの場合は、指定時間待って、送信処理を継続する
        error = WaitNanoSec(sleep_tick_);
        if (error.value() != boost::system::errc::success) {
          break;
        }
      }
    } else {
      num_done += result;
      if (num_done == data_bytes) {
        // 送信バイト数が引数data_bytesと等しい場合、戻り値にsuccessを設定し、処理終了する
        error = boost::system::error_code(boost::system::errc::success, boost::system::system_category());
        break;
      } else if (num_done > data_bytes) {
        RCLCPP_FATAL(rclcpp::get_logger("exxx_reprograming"), "NOT REACHED");
      } else {
        // 送信バイト数が引数data_bytesに達していないため、送信処理継続する
      }
    }
    int64_t last_elapsed = elapsed;
    elapsed = Now();
    if (elapsed < last_elapsed) {
      // オーバーフロー
      error = boost::system::error_code(boost::system::errc::value_too_large, boost::system::system_category());
      break;
    }
  }
  if (num_done != data_bytes) {
    // 引数data_bytesの数だけ送信できなかった場合は、戻り値にtimed_outを設定する
    error = boost::system::error_code(boost::system::errc::timed_out, boost::system::system_category());
  }
  return error;
}

/**
 * @brief 区切り文字まで受信
 *
 * '\n' または '?' まで受信を行う
 *
 * @param[out] receive_line  受信データ
 * @return
 * boost::system::errc::success  区切り文字まで受信成功
 * boost::system::errc::timed_out  区切り文字を受信せずにタイムアウト（受信データはreceive_lineに設定する）
 * boost::system::errc::value_too_large  タイムアウト計測用の変数がオーバーフローした
 * poll, readで発生するエラーを返す
 */
ExxxReprograming::ErrorCode ExxxReprograming::ReceiveLine(std::string& receive_line) {
  boost::system::error_code error;
  std::string receive_message;
  uint8_t receive_data;
  int64_t start = Now();
  int64_t elapsed = start;
  uint32_t recv_size = 0;

  while ((elapsed - start) < timeout_) {
    // 1byte受信する
    error = ReceiveByte(receive_data);
    if (error.value() == boost::system::errc::timed_out) {
      // タイムアウトの場合は処理継続する
    } else if (error.value() != boost::system::errc::success) {
      // success, timed_out以外の場合は、発生したエラーを返す
      break;
    } else {
      receive_message.push_back(receive_data);
      recv_size++;
      if ((receive_data == kReceiveLineDelimiter1) || (receive_data == kReceiveLineDelimiter2)) {
        // 区切り文字を受信した場合に、引数receive_lineに受信データを設定し、戻り値にsuccessを設定する
        receive_line = receive_message;
        error = boost::system::error_code(boost::system::errc::success, boost::system::system_category());
        break;
      }
    }
    int64_t last_elapsed = elapsed;
    elapsed = Now();
    if (elapsed < last_elapsed) {
      // オーバーフロー
      error = boost::system::error_code(boost::system::errc::value_too_large, boost::system::system_category());
      break;
    }
  }
  if ((elapsed - start) >= timeout_) {
    if (recv_size != 0) {
      // タイムアウト時、データを受信していた場合は、引数receive_lineに受信データを設定し、、戻り値にsuccessを設定する
      error = boost::system::error_code(boost::system::errc::success, boost::system::system_category());
      receive_line = receive_message;
    }
  }
  return error;
}

/**
 * @brief 1byte受信
 *
 * @param[out] data_out    受信データ
 * @return
 * boost::system::errc::success  1byte受信成功
 * boost::system::errc::timed_out  受信タイムアウト
 * poll, readで発生するエラーを返す
 */
ExxxReprograming::ErrorCode ExxxReprograming::ReceiveByte(uint8_t& data_out) {
  boost::system::error_code error;
  uint8_t receive_data;
  struct pollfd poll_fd[1];
  struct timespec poll_timeout;
  poll_fd[0].fd = fd_;
  poll_fd[0].events = POLLIN | POLLPRI;
  int64_t remain_timeout = timeout_;
  poll_timeout.tv_sec = remain_timeout / 1000000000LL;
  poll_timeout.tv_nsec = remain_timeout % 1000000000LL;
  int ready = ppoll(&poll_fd[0], 1, &poll_timeout, NULL);
  if (ready == 0) {
    // timeout
    error = boost::system::error_code(boost::system::errc::timed_out, boost::system::system_category());
  } else if (ready < 0) {
    error = boost::system::error_code(errno, boost::system::system_category());
  } else {
    int32_t result = read(fd_, &receive_data, 1);
    if (result < 0) {
      error = boost::system::error_code(errno, boost::system::system_category());
    } else {
      {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("exxx_reprograming"),
                            "Read:" << std::hex  << std::setw(2) << std::setfill('0')
                            << static_cast<int32_t>(receive_data));
      }
      data_out = receive_data;
      error = boost::system::error_code(boost::system::errc::success, boost::system::system_category());
    }
  }
  return error;
}

}  // namespace hsrb_servomotor_protocol
