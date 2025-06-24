/*
Copyright (c) 2015 TOYOTA MOTOR CORPORATION
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
#ifndef HSRB_SERVOMOTOR_PROTOCOL_MESSAGE_BUILDER_HPP_
#define HSRB_SERVOMOTOR_PROTOCOL_MESSAGE_BUILDER_HPP_

#include <string>

namespace hsrb_servomotor_protocol {

/// 説明(ネームスペース/名称)のようにメッセージを作成するクラス
/// 複数ある場合，";"で繋ぐ
class MessageBuilder {
 public:
  /// ネームスペースがない場合
  MessageBuilder() : dst_message_(""), space_("") {}

  /// ネームスペースがある場合
  /// @param [in] space メッセージのネームスペース，"/"は不要
  explicit MessageBuilder(const std::string& space) : dst_message_(""), space_(space + "/") {}

  /// デストラクタ
  ~MessageBuilder() {}

  /// メッセージを追加する
  /// @param [in] name 名称
  /// @param [in] message 説明文
  void Append(const std::string& name, const std::string& message) {
    dst_message_.append(message);
    dst_message_.append("(");
    dst_message_.append(space_);
    dst_message_.append(name);
    dst_message_.append("); ");
  }

  /// メッセージを生成する
  /// @param
  std::string Build() const { return dst_message_; }

 private:
  // 作成されたメッセージ
  std::string dst_message_;
  // ネームスペース
  std::string space_;
};

}  // namespace hsrb_servomotor_protocol
#endif  // HSRB_SERVOMOTOR_PROTOCOL_MESSAGE_BUILDER_HPP_
