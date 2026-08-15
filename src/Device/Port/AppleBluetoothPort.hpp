// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#ifdef __APPLE__
#include <TargetConditionals.h>
#if TARGET_OS_IPHONE

#include "BufferedPort.hpp"
#include <memory>
#include <string>

class AppleBluetoothConnection;

/**
 * A serial port implementation for iOS Bluetooth connections
 */
class AppleBluetoothPort : public BufferedPort {
private:
  AppleBluetoothConnection* connection;
  std::string device_address;
  
public:
  /**
   * Create a new Bluetooth port for iOS
   * @param address The Bluetooth device UUID
   * @param listener Port listener for state changes
   * @param handler Data handler for incoming data
   */
  AppleBluetoothPort(const char *address, PortListener *listener,
                     DataHandler &handler);
  
  ~AppleBluetoothPort() noexcept override;
  
  /* virtual methods from Port */
  PortState GetState() const noexcept override;
  bool Drain() noexcept override;
  unsigned GetBaudrate() const noexcept override;
  void SetBaudrate(unsigned baud_rate) override;
  std::size_t Write(std::span<const std::byte> src) override;
  
  /**
   * Called from the CoreBluetooth delegate to push received data into the port.
   * Thread-safe — may be called from any thread/queue.
   */
  void ForwardData(const void *data, std::size_t length) noexcept {
    DataReceived({static_cast<const std::byte*>(data), length});
  }
  
  /**
   * Called from the CoreBluetooth delegate when the port state changes.
   */
  void ForwardState(PortState new_state) noexcept;
  
  PortState GetStoredState() const noexcept { return stored_state; }
  
private:
  PortState stored_state = PortState::LIMBO;
};

/**
 * Factory function to open an iOS Bluetooth port
 */
std::unique_ptr<Port>
OpenAppleBluetoothPort(const char *address, PortListener *listener,
                       DataHandler &handler);

#endif // TARGET_OS_IPHONE
#endif // __APPLE__
