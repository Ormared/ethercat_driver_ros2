// Copyright 2024 ICUBE Laboratory, University of Strasbourg
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Author: Manuel YGUEL (yguel.robotics@gmail.com)

#ifndef ETHERRAT_INTERFACE__EC_PDO_SINGLE_INTERFACE_CHANNEL_MANAGER_HPP_
#define ETHERRAT_INTERFACE__EC_PDO_SINGLE_INTERFACE_CHANNEL_MANAGER_HPP_

#include "ethercat_interface/ec_pdo_channel_manager.hpp"

namespace ethercat_interface
{

/**
 * @brief Class to manage a PDO channel corresponding to a single interface
 */
class EcPdoSingleInterfaceChannelManager : public EcPdoChannelManager
{
public:
  EcPdoSingleInterfaceChannelManager();
  ~EcPdoSingleInterfaceChannelManager();

public:
//=======================
/** @name Setup methods
   * @brief Methods to setup the PDO channel manager
   * @{
   */

/// @brief Load the channel configuration from a YAML node
  bool load_from_config(YAML::Node channel_config);

/** @} */    // end of Setup methods
//=======================

public:
//=======================
/** @name Data exchange methods
   * @brief Methods to read and write data from/to the PDO
   * @{
   */

/// @brief Read the data from the PDO applying data mask, factor and offset
  double ec_read(uint8_t * domain_address);

/// @brief Perform an ec_read and update the state interface
  double ec_read_to_interface(uint8_t * domain_address);

/// @brief Write the value to the PDO applying data mask, factor and offset
  void ec_write(uint8_t * domain_address, double value);

/// @brief Perform an ec_write and update the command interface
  void ec_write_from_interface(uint8_t * domain_address);

/// @brief Update the state and command interfaces
  void ec_update(uint8_t * domain_address);

/** @} */    // < end of Data exchange methods
//=======================

public:
  uint8_t data_mask = 255;
  double default_value = std::numeric_limits<double>::quiet_NaN();
  /** last_value stores either:
   * - the last read value modified by mask, factor and offset
   * - the last written value modified by mask, factor and offset
   * */
  double last_value = std::numeric_limits<double>::quiet_NaN();
  double factor = 1;
  double offset = 0;

  bool override_command = false;

public:
  inline
  size_t number_of_managed_interfaces() const
  {
    return 1;
  }

  inline std::string interface_name(size_t i) const
  {
    if (0 == i) {
      return all_interface_names[interface_name_idx_];
    } else {
      throw std::out_of_range("EcPdoSingleInterfaceChannelManager::interface_name");
    }
  }

  inline
  bool is_interface_managed(std::string name) const
  {
    return name == all_interface_names[interface_name_idx_];
  }

public:
  int state_interface_index = -1;
  int command_interface_index = -1;

protected:
  SingleReadFunctionType read_function_;
  SingleWriteFunctionType write_function_;
  size_t state_interface_name_idx_ = 0;
  size_t command_interface_name_idx_ = 0;

};

} // < namespace ethercat_interface


#endif // < ETHERRAT_INTERFACE__EC_PDO_SINGLE_INTERFACE_CHANNEL_MANAGER_HPP_
