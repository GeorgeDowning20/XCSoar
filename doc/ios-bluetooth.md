# iOS Bluetooth Device Connectivity - Implementation Guide

## Overview
iOS Bluetooth support has been implemented for XCSoar, allowing connection to external Bluetooth devices like GPS receivers, variometers, and other avionics equipment.

## Technical Details

### Architecture
The implementation follows XCSoar's existing device port abstraction pattern:
- **BluetoothManager**: Core Objective-C++ class handling CoreBluetooth API
- **AppleBluetoothPort**: Port implementation conforming to XCSoar's Port interface
- **ConfiguredPort**: Updated to open iOS Bluetooth ports when configured

### Supported Port Types
- `RFCOMM`: Standard Bluetooth RFCOMM serial emulation
- `BLE_HM10`: Bluetooth Low Energy with HM-10 protocol support

### CoreBluetooth Features
- Automatic device discovery via CBCentralManager
- BLE service and characteristic enumeration
- Support for Nordic UART service (default):
  - TX Characteristic: `6E400003-B5A3-F393-E0A9-E50E24DCCA9E`
  - RX Characteristic: `6E400002-B5A3-F393-E0A9-E50E24DCCA9E`

## Configuration

### 1. Add Device in XCSoar Settings
1. Open Device settings in XCSoar
2. Create a new device configuration
3. Select port type: "RFCOMM" or "BLE_HM10"
4. Enter device UUID (not MAC address - iOS uses UUIDs)
5. Set baud rate (defaults to 115200 for Bluetooth)

### 2. Permissions
The app requires Bluetooth permissions in Info.plist (already configured):
- `NSBluetoothPeripheralUsageDescription`
- `NSBluetoothAlwaysUsageDescription`

Users will be prompted to grant Bluetooth permissions on first use.

### 3. Pairing
Devices must be paired with the iOS device through Settings > Bluetooth before XCSoar can connect.

## Device Address Format
iOS Bluetooth uses UUIDs instead of MAC addresses. When configuring a device:
- Obtain the device's UUID (visible in iOS Settings > Bluetooth)
- Format: `550e8400-e29b-41d4-a716-446655440000`
- Copy the complete UUID into XCSoar's device address field

## Supported Devices
Any Bluetooth Low Energy device can potentially be used, including:
- Bluetooth GPS receivers
- Variometers with Bluetooth output
- Weather stations
- External sensors

Most devices following the Nordic UART service specification will work without additional configuration.

## Troubleshooting

### Device Not Connecting
1. Ensure device is paired in iOS Settings > Bluetooth
2. Verify device UUID is correct (case-insensitive)
3. Check that device is powered on and in range
4. Grant Bluetooth permissions when prompted

### Data Not Received
1. Verify device is transmitting data
2. Check that device supports BLE (not classic Bluetooth)
3. Ensure correct characteristics are being discovered
4. Review LogFile.cpp output for debug messages

### Performance Considerations
- Bluetooth connections are subject to iOS background execution limits
- Keep app in foreground for reliable data transfer
- High-frequency data may be rate-limited by iOS power management

## Implementation Files

### New Files
- `src/Apple/BluetoothManager.hpp` - Bluetooth manager header
- `src/Apple/BluetoothManager.cpp` - Bluetooth manager implementation
- `src/Device/Port/AppleBluetoothPort.hpp` - Port header
- `src/Device/Port/AppleBluetoothPort.cpp` - Port implementation

### Modified Files
- `src/Device/Port/ConfiguredPort.cpp` - Added iOS port opening
- `src/Device/Config.cpp` - Made Bluetooth available on iOS
- `build/main.mk` - Added iOS source files to build
- `Data/iOS/Info.plist.in.xml` - Added Bluetooth permissions

## Code Examples

### Opening a Bluetooth Device (C++)
```cpp
DeviceConfig config;
config.port_type = DeviceConfig::PortType::RFCOMM;
config.bluetooth_mac = "550e8400-e29b-41d4-a716-446655440000";

std::unique_ptr<Port> port = 
    OpenAppleBluetoothPort(config.bluetooth_mac.c_str(), 
                          listener, handler);
```

### Bluetooth Scanning (Objective-C++)
```cpp
AppleBluetoothManager manager;
if (manager.IsBluetoothAvailable()) {
    manager.StartScanning(listener);
    // Callback when devices discovered
}
```

## Future Enhancements
- Device scanning UI for automatic device discovery
- Classic Bluetooth RFCOMM support (currently BLE only)
- Multiple characteristic support beyond Nordic UART
- Connection state UI indicators
- Automatic reconnection on disconnect
- Power management optimizations

## Compatibility
- **iOS Version**: iOS 13.0+
- **Architecture**: ARM64 (iPhone 6s+)
- **Simulator**: Supported via iOS Simulator with BLE emulation

## Known Limitations
1. iOS doesn't provide API to list all previously paired devices
2. Classic Bluetooth (non-BLE) requires additional implementation
3. Background data transfer is limited by iOS system policies
4. Device must maintain connection state (iOS handles disconnects)

## Technical References
- [Apple CoreBluetooth Documentation](https://developer.apple.com/documentation/corebluetooth)
- [Nordic UART Service (nRF Connect)](https://github.com/NordicSemiconductor/nRF-UART-Central-iOS)
- [XCSoar Device Port Architecture](src/Device/Port/Port.hpp)
