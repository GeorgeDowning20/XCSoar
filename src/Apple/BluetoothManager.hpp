// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#ifdef __APPLE__
#include <TargetConditionals.h>
#if TARGET_OS_IPHONE

#include <cstdint>
#include <vector>
#include <string>

/**
 * Forward declarations for Objective-C types
 */
class AppleBluetoothManagerImpl;
class BluetoothDevice;

/**
 * iOS Bluetooth device representation
 */
struct AppleBluetoothDevice {
  std::string name;
  std::string address;
  bool is_ble;  // true for Bluetooth Low Energy, false for classic
  
  AppleBluetoothDevice(const std::string& _name, const std::string& _address, bool _is_ble = true)
    : name(_name), address(_address), is_ble(_is_ble) {}
};

/**
 * Listener for Bluetooth discovery events
 */
class AppleBluetoothListener {
public:
  virtual ~AppleBluetoothListener() = default;
  
  virtual void OnDeviceDiscovered(const AppleBluetoothDevice& device) = 0;
  virtual void OnDiscoveryFinished() = 0;
  virtual void OnError(const char* error) = 0;
};

/**
 * Manager for iOS Bluetooth connectivity
 * Provides device discovery and connection management through CoreBluetooth
 */
class AppleBluetoothManager {
private:
  AppleBluetoothManagerImpl* impl;
  
public:
  AppleBluetoothManager();
  ~AppleBluetoothManager();
  
  /**
   * Start scanning for Bluetooth devices
   * @param listener Callback for discovery events
   */
  void StartScanning(AppleBluetoothListener* listener);
  
  /**
   * Stop scanning for Bluetooth devices
   */
  void StopScanning();
  
  /**
   * Check if Bluetooth is available and powered on
   */
  bool IsBluetoothAvailable() const;
  
  /**
   * Get list of discovered/paired devices
   */
  std::vector<AppleBluetoothDevice> GetPairedDevices() const;
  
  /**
   * Get a discovered peripheral by UUID for connection
   * Returns a pointer to the peripheral object (valid as long as manager exists)
   * This is an internal implementation detail - used by port code
   */
  void* GetPeripheralByUUID(const char* uuid_string);
  
  /**
   * Get the central manager for connection operations
   * Returns a pointer to CBCentralManager (valid as long as manager exists)
   */
  void* GetCentralManager() const;
};

/**
 * Direct accessors for the global discovery cache.
 * These avoid constructing a temporary AppleBluetoothManager instance.
 */
void* GetCachedApplePeripheralByUUID(const char* uuid_string);
void* GetCachedAppleCentralManager();

#endif // TARGET_OS_IPHONE
#endif // __APPLE__
