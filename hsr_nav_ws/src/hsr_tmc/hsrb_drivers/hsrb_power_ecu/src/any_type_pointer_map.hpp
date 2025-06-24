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
#ifndef HSRB_POWER_ECU_ANY_TYPE_POINTER_MAP_HPP_
#define HSRB_POWER_ECU_ANY_TYPE_POINTER_MAP_HPP_

#include <inttypes.h>
#include <string>
#include <typeinfo>

#include <boost/make_shared.hpp>
#include <boost/noncopyable.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/unordered_map.hpp>

namespace hsrb_power_ecu {
namespace any_type_pointer_map {

/**
 * @brief Registration data interface
 * Define the interface that you down and receive a pointer to centrally manage various types of data.
 */
class IElement : boost::noncopyable {
 public:
  IElement() {}
  ~IElement() {}

  /**
   * @brief Pointer acquisition
   * Downcast and receive pointer to manage registration data at once without approaching.
   */
  virtual void* GetPtr() const = 0;

  /**
   * @brief Type information acquisition
   */
  virtual const std::type_info& GetType() const = 0;
};

/**
 * @brief Registration data
 */
template <typename T>
class Element : public IElement {
 public:
  explicit Element(T* const data) : data_(data) {}
  ~Element() {}
  virtual void* GetPtr() const { return static_cast<void*>(data_); }
  virtual const std::type_info& GetType() const { return typeid(T); }

 private:
  T* const data_;
};

class Map : boost::noncopyable {
 public:
  Map() {}
  ~Map() {}

  /**
   * @brief data registration
   * Registration fails when multiple registration
   *
   * @tparam T type
   * @param [in] name Data name
   * @param [in] value Data pointer
   *
   * When registration is completed TRUE
   */
  template <typename T>
  bool Register(std::string name, T* const value) {
    if (dic_.find(name) != dic_.end()) {
      return false;
    }
    dic_[name] = boost::make_shared<Element<T> >(value);
    return true;
  }

  /**
   * @brief Pointer acquisition
   * When the data type is uneven or the data name fails
   *
   * @tparam T type
   * @param [in] name Data name
   */
  template <typename T>
  T* GetPtr(const std::string& name) const {
    DicType::const_iterator itr = dic_.find(name);
    if (itr == dic_.end()) {
      return NULL;  // Unregistered
    }
    if (itr->second->GetType() != typeid(T)) {
      return NULL;  // Different from the registered type
    } else {
      return static_cast<T*>(itr->second->GetPtr());
    }
  }

  bool HasPtr(const std::string& name) const { return (dic_.find(name) != dic_.end()); }

 private:
  typedef boost::unordered_map<std::string, boost::shared_ptr<IElement> > DicType;
  DicType dic_;
};
}  // namespace any_type_pointer_map
}  // namespace hsrb_power_ecu
#endif  // HSRB_POWER_ECU_ANY_TYPE_POINTER_MAP_HPP_
