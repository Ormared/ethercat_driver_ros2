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

#include "ethercat_interface/ec_pdo_group_interface_channel_manager.hpp"

namespace ethercat_interface
{

#ifndef CLASSM
#define CLASSM EcPdoGroupInterfaceChannelManager
#else
#error alias CLASSM all ready defined!
#endif // < CLASSM

CLASSM::EcPdoGroupInterfaceChannelManager()
{
}

CLASSM::~EcPdoGroupInterfaceChannelManager()
{
}

std::string CLASSM::interface_name(size_t i) const
{
  if (data.size() > i) {
    if (is_interface_defined(i) ) {
      if (is_command_interface_(i)) {
        return all_command_interface_names[interface_name_ids_[i]];
      } else {
        return all_state_interface_names[interface_name_ids_[i]];
      }
    }
  }
  throw std::out_of_range("EcPdoSingleInterfaceChannelManager::interface_name unknown index");
}

std::pair<bool, size_t> CLASSM::is_interface_managed(std::string name) const
{
  for (size_t i = 0; i < data.size(); ++i) {
    if (name == interface_name(i)) {
      return std::make_pair<true, i>;
    }
  }
  return std::make_pair<false, std::numeric_limits<size_t>::max()>;   // not found
}

size_t CLASSM::channel_state_interface_index(const std::string & name) const
{
  for (size_t i = 0; i < data.size(); ++i) {
    if (is_state_interface_defined(i)) {
      if (name == all_state_interface_names[interface_name_ids_[i]]) {
        return i;
      }
    }
  }
  throw std::out_of_range(
          "EcPdoSingleInterfaceChannelManager::interface_name unknown index for state interface");
}

size_t CLASSM::channel_command_interface_index(const std::string & name) const
{
  for (size_t i = 0; i < data.size(); ++i) {
    if (is_command_interface_defined(i)) {
      if (name == all_command_interface_names[interface_name_ids_[i]]) {
        return i;
      }
    }
  }
  throw std::out_of_range(
          "EcPdoSingleInterfaceChannelManager::interface_name unknown index for command interface");
}

void CLASSM::allocate_for_new_interface()
{
  data.push_back(InterfaceDataWithAddrOffset());
  interface_ids_.push_back(std::numeric_limits<size_t>::max());
  is_command_interface_.push_back(false);
  read_functions_.push_back(nullptr);
  write_functions_.push_back(nullptr);
}

size_t CLASSM::add_command_interface(const std::string & name)
{
  // Check if the interface is already present
  auto managed = is_interface_managed(name);
  if (managed.first) {
    return managed.second;
  }

  // Add the interface
  // Stores the index of the interface in all the vectors
  size_t id = data.size();
  // Resize all the vectors
  allocate_for_new_interface();
  is_command_interface_[id] = true;
  // Add the name of the interface to the command interface names
  size_t name_idx = all_command_interface_names.size();
  all_command_interface_names.push_back(name);
  interface_name_ids_.push_back(name_idx);

  return id;
}

size_t CLASSM::add_state_interface(const std::string & name)
{
  // Check if the interface is already present
  auto managed = is_interface_managed(name);
  if (managed.first) {
    return managed.second;
  }

  // Add the interface
  // Stores the index of the interface in all the vectors
  size_t id = data.size();
  // Resize all the vectors
  allocate_for_new_interface();
  is_command_interface_[id] = false;
  // Add the name of the interface to the command interface names
  size_t name_idx = all_state_interface_names.size();
  all_state_interface_names.push_back(name);
  interface_name_ids_.push_back(name_idx);

  return id;
}

bool CLASSM::load_from_config(YAML::Node channel_config)
{
  //TODO(@yguel@unistra): Use ROS2 logging
  // index
  if (channel_config["index"]) {
    index = channel_config["index"].as<uint16_t>();
  } else {
    std::cerr << "missing channel index info" << std::endl;
  }

  // sub_index
  if (channel_config["sub_index"]) {
    sub_index = channel_config["sub_index"].as<uint8_t>();
  } else {
    std::cerr << "channel " << index << " : missing channel info" << std::endl;
  }

  // data type
  if (channel_config["type"]) {
    std::string data_type = channel_config["type"].as<std::string>();
    data_type_idx_ = typeIdx(data_type);
    if (0 == data_type_idx_) {
      std::cerr << "channel" << index << " : unknown data type " << data_type << std::endl;
      return false;
    }
    bits_ = type2bits(data_type);
  } else {
    std::cerr << "channel" << index << " : missing channel data type info" << std::endl;
  }

  size_t id = std::numeric_limits<size_t>::max();
  if (channel_config["command_interface"]) {
    throw std::runtime_error(
            "Global command_interface is not allowed in a grouped interface pdo channel, it must be defined per interface in the data_mapping");
  }

  if (channel_config["state_interface"]) {
    auto state_interface_name = channel_config["state_interface"].as<std::string>();
    id = add_state_interface(state_interface_name);
    data[id].data_type_idx = data_type_idx_;
    data[id].bits = bits_;
    read_functions_[id] = ec_pdo_single_read_functions[data_type_idx];
  }

  // factor
  if (channel_config["factor"]) {
    if (id != std::numeric_limits<size_t>::max()) {
      data[id].factor = channel_config["factor"].as<double>();
    } else {
      //Ignored
      // TODO(yguel@unistra.fr: log warning)
    }
  }
  // offset
  if (channel_config["offset"]) {
    if (id != std::numeric_limits<size_t>::max()) {
      data[id].offset = channel_config["offset"].as<double>();
    } else {
      //Ignored
      // TODO(yguel@unistra.fr: log warning)
    }
  }
  // mask
  if (channel_config["mask"]) {
    if (id != std::numeric_limits<size_t>::max()) {
      data[id].mask = channel_config["mask"].as<uint8_t>();
    } else {
      //Ignored
      // TODO(yguel@unistra: log warning)
    }
  }

  //skip
  if (channel_config["skip"]) {
    skip = channel_config["skip"].as<bool>();
    // TODO(yguel@unistra: check if skip is relevant in a grouped interface pdo channel)
  }

  // Handle data mapping
  if (channel_config["data_mapping"]) {
    auto data_mapping = channel_config["data_mapping"];
    for (auto map : data_mapping) {
      // Reset the id to skip adding data if no interface is defined
      id = std::numeric_limits<size_t>::max();
      if (map["command_interface"]) {
        auto command_interface_name = map["command_interface"].as<std::string>();
        id = add_command_interface(command_interface_name);
        if (map["default_value"]) {
          data[id].default_value = map["default_value"].as<double>();
        }
      }

      if (map["state_interface"]) {
        auto state_interface_name = map["state_interface"].as<std::string>();
        id = add_state_interface(state_interface_name);
      }

      if (std::numeric_limits<size_t>::max() == id) {
        // Skip the rest of the loop instructions since no interface is defined
        continue;
      }

      if (map["addr_offset"]) {
        data[id].addr_offset = map["addr_offset"].as<size_t>();
      }
      if (map["type"]) {
        std::string data_type = map["type"].as<std::string>();
        auto type_idx = typeIdx(data_type);
        data[id].data_type_idx = type_idx;
        if (0 == data[id].data_type_idx) {
          std::cerr << "channel" << index << " : unknown data type " << data_type << std::endl;
          return false;
        }
        data[id].bits = type2bits(data_type);
        read_functions_[id] = ec_pdo_single_read_functions[type_idx];
        write_functions_[id] = ec_pdo_single_write_functions[type_idx];
      }

      // factor
      if (map["factor"]) {
        data[id].factor = map["factor"].as<double>();
      }

      // offset
      if (map["offset"]) {
        data[id].offset = map["offset"].as<double>();
      }

      // mask
      if (map["mask"]) {
        data[id].mask = map["mask"].as<uint8_t>();
      }

    }
  }

  return true;
}

double CLASSM::ec_read(size_t i, uint8_t * domain_address)
{
  double last_value = read_functions_[i](domain_address + data[i].addr_offset, data[i].mask);
  data[i].last_value = data[i].factor * last_value + data[i].offset;
  if (is_state_interface_defined(i) ) {
    state_interface_ptr_->at(state_interface_ids_[i]) = data[i].last_value;
  }
  return data[i].last_value;
}

void CLASSM::ec_read_to_interface(uint8_t * domain_address)
{
  for (size_t i = 0; i < data.size(); ++i) {
    ec_read(i, domain_address);
    if (is_state_interface_defined(i) ) {
      state_interface_ptr_->at(state_interface_index_) = last_value;
    }
  }
}

void CLASSM::ec_write(size_t i, uint8_t * domain_address, double value)
{
  ///////////////////// TODO: Implement this function /////////////////////
  if (RPDO != pdo_type || !allow_ec_write) {
    return;
  }

  if (!std::isnan(default_value) && !override_command) {
    last_value = factor * value + offset;
    write_function_(domain_address, last_value, mask);
  } else {
    if (!std::isnan(default_value)) {
      last_value = default_value;
      write_function_(domain_address, last_value, mask);
    } else {  // Do nothing
      return;
    }
  }
}

void CLASSM::ec_write_from_interface(uint8_t * domain_address)
{
  ///////////////////// TODO: Implement this function /////////////////////
  if (is_command_interface_defined() ) {
    const auto value = command_interface_ptr_->at(command_interface_index);
    ec_write(domain_address, value);
  } else {
    if ( (RPDO == pdo_type) && allow_ec_write && !std::isnan(default_value)) {
      last_value = default_value;
      write_function_(domain_address, last_value, mask);
    }
  }
}

#undef CLASSM
