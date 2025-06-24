/*
Copyright (c) 2024 TOYOTA MOTOR CORPORATION
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
#ifndef HSRB_POWER_ECU_POWER_ECU_PROTOCOL_HPP_
#define HSRB_POWER_ECU_POWER_ECU_PROTOCOL_HPP_

#include <inttypes.h>

#include <string>

#include <boost/circular_buffer.hpp>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/system/error_code.hpp>
#include <boost/unordered_map.hpp>

#include <rclcpp/rclcpp.hpp>

#include "any_type_pointer_map.hpp"
#include "error_counter.hpp"
#include "power_ecu_com_frame_decoder.hpp"
#include "power_ecu_com_frame_encoder.hpp"
#include "ros2_msg_utils.hpp"

namespace hsrb_power_ecu {

/**
 * @brief 制御コマンド情報管理クラス
 *
 * CPU->電源ECUへの制御コマンド送信は各送信毎に電源ECUからのACKを確認する必要があること、
 * また通信失敗を考慮し、各制御コマンドのリトライ処理を行う必要があるため、
 * 1周期に対したかだか1個の送信しかおこなわない。
 * 一方topicからの指令は常時受け付けているため、
 * 各指令を逐次処理するために、指令をキューイングする必要がある。
 * CommandStateクラスは、キューイングされている制御コマンドの情報を保持するクラスである。
 */
class CommandState {
 private:
  CommandState(CommandState const&);             // = delete;
  CommandState& operator=(CommandState const&);  // = delete;

 public:
  typedef boost::shared_ptr<CommandState> Ptr;

  /**
   * @brief コンストラクタ
   * @param[in] command_name コマンド名
   */
  explicit CommandState(const std::string& command_name)
      : command_name_(command_name), return_value_(0), is_processing_(0), retry_count_(0) {}
  /**
   * @brief デストラクタ
   */
  ~CommandState() {}
  /**
   * @brief  コマンド名取得
   * @return コマンド名
   */
  inline const std::string& GetCommandName() const { return command_name_; }
  /**
   * @brief コマンド戻り値取得
   * コマンド戻り値の代入処理は実装済みだが、まだ利用していない。
   * @return コマンド戻り値
   */
  inline uint8_t GetReturnValue() const { return return_value_; }
  /**
   * @brief コマンド戻り値代入
   * @param[in] value コマンド戻り値
   */
  inline void SetReturnValue(const uint8_t value) { return_value_ = value; }
  /**
   * @brief コマンド処理中フラグ代入
   * コマンド送信から、ack受信までTrueを返す。
   * フラグ値の代入処理は実装済みだが、まだ利用していない。
   * @param[in] value フラグ値
   */
  inline void SetProcessingStatus(const bool value) { is_processing_ = value; }
  /**
   * @brief コマンド処理中フラグ取得
   * @return コマンド処理中 True
   */
  inline bool IsProcessing() const { return is_processing_; }
  /**
   * @brief リトライ回数インクリメント
   */
  inline void IncrementRetryCount() { ++retry_count_; }
  /**
   * @brief リトライ回数クリア
   */
  inline void ClearRetryCount() { retry_count_ = 0; }
  /**
   * @brief リトライ回数取得
   * @return リトライ回数
   */
  inline uint32_t GetRetryCount() const { return retry_count_; }

 private:
  const std::string command_name_;  //!< コマンド名
  uint8_t return_value_;            //!< 戻り値の値
  bool is_processing_;              //!< 処理中フラグ(処理中はTrue)
  uint32_t retry_count_;            //!< リトライ回数
};

// @n Protocolクラスにいれるとクラス名がVersionsだけにできる
struct PowerEcuVersions {
  std::string power_ecu_version;
  std::string power_ecu_com_version;
};

/**
 * @brief 通信コマンドのプロトコルバージョンに依存しない処理を行うクラス <br>
 * <br>
 * 通信プロトコルバージョンによらない処理は下記の通り <br>
 *   - バージョン確認 <br>
 *   - 制御コマンド送信, ack受信 <br>
 *   - ハートビート送信 <br>
 * <br>
 * また、制御コマンドの拡張の為、
 * コマンドキューとデータデコーダ/エンコーダへの登録機能を公開している。<br>
 * topicから送られて来る処理は非同期であるが、
 * 電源ECUとの通信は逐次処理する必要がある為キューイングを行う設計とした。<br>
 *
 * また、rosrunコマンドで呼び出すアプリケーションでもこのクラスを使用するため、
 * このクラスはroscore非依存の設計を取っている。
 * もし、このクラスのエラーをダイアグなどのトピックとして発信する場合、
 * このクラスを管理する上位のクラスがその機能を担う事。
 */
class PowerEcuProtocol : boost::noncopyable {
 public:
  typedef boost::shared_ptr<PowerEcuProtocol> Ptr;

  explicit PowerEcuProtocol(boost::shared_ptr<hsrb_power_ecu::INetwork> network,
                            const rclcpp::Node::SharedPtr& node);
  ~PowerEcuProtocol() { Close(); }

  /**
   * @brief オープン
   */
  bool Open();

  /**
   * @brief クローズ
   */
  void Close();

  /**
   * @brief バージョン情報をECUから取得し、プロトコルの通信準備を行う
   */
  bool Init();

  /**
   * @brief 通信開始
   *
   * @return
   */
  boost::system::error_code Start();

  /**
   * @brief 通信終了
   *
   * @return
   */
  boost::system::error_code Stop();

  /**
  * @brief 名前でコマンドに関するステータス管理情報を返す
  *
  * 受付不可能なコマンドならfalseを返す
  *
  * @param[in] command 取得するコマンド名
  * @param[out] command_state
  */
  inline bool GetCommandState(const std::string& command, hsrb_power_ecu::CommandState::Ptr& command_state) const {
    CommandMapType::const_iterator it = command_map_.find(command);
    if (it == command_map_.end()) {
      return false;
    }
    command_state = it->second;
    return true;
  }

  /**
   * @brief コマンドが有効か確認
   *
   * @param[in] nameコマンド名
   *
   * @return 有効時 true
   */
  inline bool HasCommand(const std::string& name) const { return (command_map_.find(name) != command_map_.end()); }

  /**
   * @brief データのポインタを取得
   *
   * @tparam T データの型
   * @param[in] name データの名前
   *
   * @return 成功時 : データのポインタ<br>
   *         失敗時 : NULL<br>
   *         未登録のデータ名、データの型が不一致の場合失敗となる
   */
  template <typename T>
  T* GetParamPtr(const std::string& name) const {
    T* ret = frame_decoder_.GetParamPtr<T>(name);
    if (ret != NULL) {
      return ret;
    }
    ret = frame_encoder_.GetParamPtr<T>(name);
    return ret;
  }

  /**
   * @brief  Receiveのエラーレート取得
   * @return エラーレート
   */
  inline double GetReceiveErrorRate() const { return read_error_counter_.GetErrorRate(); }

  /**
   * @brief Sendのエラーレート取得
   * @return エラーレート
   */
  inline double GetSendErrorRate() const { return write_error_counter_.GetErrorRate(); }

  /**
   * @brief バージョン情報を取得する
   *
   * @param[out] result  バージョン情報
   *
   * @return 成功時 true
   */
  bool GetPowerEcuVersions(PowerEcuVersions& result);

  /**
   * @brief コマンドキュー追加
   *
   *
   * @param[in] command_name 追加するコマンド
   *
   * @return 成功時 : true<br>
   *         受付不可能なコマンドのときfalseを返す
   */
  bool AddCommandQueue(const std::string& command_name) {
    CommandMapType::const_iterator it = command_map_.find(command_name);
    if (it == command_map_.end()) {
      return false;
    }
    AddCommandQueue(it->second);
    return true;
  }

  /**
   * @brief コマンドキューをすべて処理
   *
   * このメソッドは非リアルタイムプロセス中でコールされることを想定している
   *
   * @param[in] check_ros     ros::ok()チェック要否  true:チェック必要  false:チェック不要
   * @param[in] cycle_hz      ポーリング周期[hz]
   * @param[in] error_rate    許容エラーレート
   *
   * @return 実行結果
   * @retval 成功時           boost::system::errc::success
   * @retval 引数異常         boost::system::errc::invalid_argument
   * @retval 通信エラー       boost::system::errc::operation_canceled
   * @retval 通信タイムアウト boost::system::errc::timed_out
   * @retval roscore非起動    boost::system::errc::operation_not_permitted
   */
  boost::system::error_code ProcessCommandQueue(bool check_ros, double cycle_hz, double error_rate);

  /**
   * @brief 受信処理
   *
   * 通信プロトコル上の復帰不可能なエラー(制御コマンドリトライ許容超過、ACKタイムアウト)
   * はこの関数内でexitしている
   *
   * @return 実行結果
   * @retval 正常時 boost::system::errc::success
   * @retval networkのReceive失敗 boost::system::errc::network_down
   * @retval 受信パケットが化けている boost::system::errc::protocol_error
   * @retval 制御コマンド再送 boost::system::errc::resource_unavailable_try_again
   */
  boost::system::error_code ReceiveAll();

  /**
   * @brief 送信処理
   *
   * @return 実行結果
   * @retval 正常時 boost::system::errc::success
   * @retval networkのSend失敗 boost::system::errc::network_down
   */
  boost::system::error_code SendAll();

 private:
  typedef boost::unordered_map<std::string, hsrb_power_ecu::CommandState::Ptr>
      CommandMapType;  //!< 制御コマンドのMapの型
  typedef boost::circular_buffer<hsrb_power_ecu::CommandState::Ptr>
      CommandBuffer;  //!< 制御コマンドのコマンドキューの型

  /**
   * @brief コマンドキュー追加(内部用)
   *
   * @param[in] command command_state
   */
  void AddCommandQueue(hsrb_power_ecu::CommandState::Ptr command);

  /**
   * @brief データデコーダ登録
   *
   * @tparam T データデコーダの型
   */
  template <typename T>
  void RegisterDataDecoder() {
    boost::shared_ptr<T> const decoder = boost::make_shared<T>();
    boost::system::error_code ret = frame_decoder_.RegisterDataDecoder(decoder);
    hsrb_power_ecu::Assert(ret == boost::system::errc::success, "frame decoder regiser failed.");
  }

  /**
   * @brief データエンコーダ登録
   *
   * @tparam T データエンコーダの型
   * @param[out] name コマンド名
   */
  template <typename T>
  std::string RegisterDataEncoder() {
    boost::shared_ptr<T> const encoder = boost::make_shared<T>();
    std::string command_name = encoder->GetPacketName();
    boost::system::error_code ret = frame_encoder_.RegisterDataEncoder(encoder);
    hsrb_power_ecu::Assert(ret == boost::system::errc::success, "frame decoder regiser failed.");
    hsrb_power_ecu::Assert((command_map_.find(command_name) == command_map_.end()), "command state register failed.");
    command_map_[command_name] = boost::make_shared<hsrb_power_ecu::CommandState>(command_name);
    return command_name;
  }

  /**
   * @brief コマンド送信
   * シリアルデバイスにコマンドを送信する。
   *
   * コマンドは、コマンドキューで管理しているため、
   * 基本的にAddCommandQueueメソッドを使ってコマンド送信すること。
   *
   * コマンドキューで管理する制御コマンドは設計上、戻り値としてRxackコマンドが返ってくる必要がある。
   * Rxackコマンドが返ってこない制御コマンド(getv_コマンド等)は専用にメソッドを作成し、
   * SendCommandを使って直接コマンド送信を行う。
   *
   * @param[in] command 送信するコマンド情報
   * @return 実行結果
   * @retval 正常時 boost::system::errc::success
   * @retval networkのSend失敗 boost::system::errc::network_down
   */
  boost::system::error_code SendCommand(const hsrb_power_ecu::CommandState::Ptr command);

  // 変数
  // ネットワークインターフェース管理
  boost::shared_ptr<hsrb_power_ecu::INetwork> network_;  //!< ネットワークインターフェース
  PacketBuffer receive_buffer_;                          //!< 受信バッファ
  PacketBuffer send_buffer_;                             //!< 送信バッファ

  // トランスポート管理
  //// デコーダ
  hsrb_power_ecu::PowerEcuComFrameDecoder frame_decoder_;  //!< フレームデコーダ
  //// エンコーダ
  hsrb_power_ecu::PowerEcuComFrameEncoder frame_encoder_;  //!< フレームエンコーダ

  CommandBuffer command_queue_;       //!< コマンドキュー

  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Time last_send_command_time_;  //!< コマンド送信時刻
  rclcpp::Time last_heartbeat_time_;  //!< 最後にハートビートを送信した時刻

  bool is_waiting_ack_;            //!< ACK待ちかどうかのフラグ

  ErrorCounter read_error_counter_;   //!< Readメソッドのエラーレート
  ErrorCounter write_error_counter_;  //!< Writeメソッドのエラーレート

  // コマンドマップ
  CommandMapType command_map_;               //!< 制御コマンドのMap

  // 受信コマンドデータ
  //// rxack
  bool* is_receive_ack_;  //!< Ackが返ってきたかどうか
  uint8_t* ack_value_;    //!< 返信コマンドの戻り値
  //// ver
  std::string* ver_power_ecu_version_;      //!< 電源ECUファームVer[git hash 20byte] 16進数40桁
  std::string* ver_power_ecu_com_version_;  //!< 電源ECU通信構造HASH[hash 20byte] 16進数40桁
  bool* is_receive_version_;                //!< Verコマンドを受信したかどうか
  // 送信コマンドデータ
  //// heart
  uint32_t* counts;  //!< ハートビートのカウント値（送信ごとに+1) 16進数8桁 uint32

  // コマンド名
  std::string heart_command_name_;  //!< heartコマンド
  std::string getv_command_name_;   //!< getv_コマンド
  std::string time_command_name_;   //!< timeコマンド
  std::string start_command_name_;  //!< startコマンド
  std::string stop_command_name_;   //!< stopコマンド
  std::string mute_command_name_;   //!< muteコマンド
};

}  // namespace hsrb_power_ecu

#endif  // HSRB_POWER_ECU_POWER_ECU_PROTOCOL_HPP_
