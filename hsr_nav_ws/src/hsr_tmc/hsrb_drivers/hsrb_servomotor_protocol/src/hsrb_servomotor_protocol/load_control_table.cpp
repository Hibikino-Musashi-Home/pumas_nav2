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
#include <string>
#include <vector>
#include <boost/filesystem.hpp>
#include <boost/filesystem/operations.hpp>

#include <hsrb_servomotor_protocol/load_control_table.hpp>

namespace hsrb_servomotor_protocol {

/// @brief バージョンが一致するcontrol_tableをロードし、そのパスを返す
/// @param[in] package_path パッケージのパス
/// @param[in] hw_hash control_table(ハードウェア)のmd5
/// @param[out] control_table_out control_tableオブジェクト
/// @param[out] selected_path_out 選択されたcontrol_tableのパス
/// @return bool ロードが成功したらtrue
/// @par 振る舞い
/// - ホームにcontrol_tableがあればそれをロードする
/// - なければバージョンごとにディレクトリで管理されたcontrol_tableをロードする
/// - md5が一致したcontrol_tableのパスを返す
bool LoadControlTable(const std::string& package_path, const std::vector<uint8_t>& hw_hash,
                      ControlTable& control_table_out, std::string& selected_path_out) {
  boost::system::error_code error;
  const char* home_path = getenv("HOME");
  bool ret = false;
  std::string full_path = std::string(home_path) + "/.control_table/control_table.csv";
  const char* csv_path = full_path.c_str();

  // control_tableがhomeにある場合
  if (boost::filesystem::exists(csv_path, error)) {
    if (control_table_out.Load(csv_path) == ControlTable::kSuccess) {
      if (hw_hash == control_table_out.GetMd5Sum()) {
        selected_path_out = csv_path;
        return true;
      }
    }
    // control_tableバージョンごとに管理されたディレクトリにある場合
  } else {
    boost::filesystem::path file_path = package_path + "/control_tables";
    if (!boost::filesystem::exists(file_path)) {
      return false;
    }

    boost::filesystem::directory_iterator path_iter(file_path);
    boost::filesystem::directory_iterator last_path_iter;

    for (path_iter; path_iter != last_path_iter; path_iter++) {
      if (!boost::filesystem::exists(path_iter->path().string() + "/control_table.csv", error)) {
        continue;
      }
      if (control_table_out.CalculateMd5Sum(path_iter->path().string() + "/control_table.csv") ==
          ControlTable::kSuccess) {
        if (hw_hash == control_table_out.GetMd5Sum()) {
          if (control_table_out.Load(path_iter->path().string() + "/control_table.csv") == ControlTable::kSuccess) {
            selected_path_out = path_iter->path().string() + "/control_table.csv";
            return true;
            break;
          } else {
            break;
          }
        }
      }
    }
  }
  return false;
}

}  // namespace hsrb_servomotor_protocol
