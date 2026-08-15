// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#ifdef __APPLE__
#include <TargetConditionals.h>
#if TARGET_OS_IPHONE

#include "BluetoothManager.hpp"
#include "LogFile.hpp"

#import <CoreBluetooth/CoreBluetooth.h>
#import <UIKit/UIKit.h>

// Static cache for discovered peripherals (used by port connections)
static NSMutableDictionary* g_discoveredPeripherals = nil;
static CBCentralManager* g_centralManager = nil;
static NSObject* g_cacheLock = nil;

/**
 * Objective-C delegate for CoreBluetooth central manager
 */
@interface XCSoarCentralManagerDelegate : NSObject <CBCentralManagerDelegate, CBPeripheralDelegate>
{
@private
  AppleBluetoothListener* listener;
  CBCentralManager* centralManager;
  NSMutableArray* discoveredPeripherals;
  NSMutableDictionary* peripheralData;
  BOOL isScanning;
  BOOL shouldScanWhenReady;
}

- (instancetype)initWithListener:(AppleBluetoothListener*)_listener;
- (void)updateListener:(AppleBluetoothListener*)_listener;
- (void)startScanning;
- (void)stopScanning;
- (BOOL)isBluetoothAvailable;
- (NSArray*)getDiscoveredPeripherals;

@end

@implementation XCSoarCentralManagerDelegate

- (instancetype)initWithListener:(AppleBluetoothListener*)_listener {
  self = [super init];
  if (self) {
    listener = _listener;
    discoveredPeripherals = [[NSMutableArray alloc] init];
    peripheralData = [[NSMutableDictionary alloc] init];
    isScanning = NO;
    shouldScanWhenReady = NO;
    
    // Initialize Central Manager - check if we're already on main thread
    auto initBlock = ^{
      NSDictionary *options = @{
        CBCentralManagerOptionShowPowerAlertKey: @NO
      };
      centralManager = [[CBCentralManager alloc] initWithDelegate:self 
                                                            queue:dispatch_get_main_queue()
                                                          options:options];
      LogFormat("iOS Bluetooth: CBCentralManager initialized, state=%ld", (long)centralManager.state);
    };
    
    if ([NSThread isMainThread]) {
      // Already on main thread, call directly
      initBlock();
    } else {
      // Not on main thread, dispatch to main thread synchronously
      dispatch_sync(dispatch_get_main_queue(), initBlock);
    }
  }
  return self;
}

- (void)updateListener:(AppleBluetoothListener*)_listener {
  listener = _listener;
  LogFormat("iOS Bluetooth: Updated listener to %p", listener);
}

- (void)startScanning {
  if (!centralManager) {
    LogFormat("iOS Bluetooth: ERROR - Central manager not initialized");
    if (listener)
      listener->OnError("Bluetooth manager not initialized");
    return;
  }
  
  // Log current state
  const char* stateStr = "Unknown";
  switch (centralManager.state) {
    case CBManagerStateUnknown:
      stateStr = "Unknown";
      break;
    case CBManagerStatePoweredOff:
      stateStr = "PoweredOff";
      break;
    case CBManagerStatePoweredOn:
      stateStr = "PoweredOn";
      break;
    case CBManagerStateUnauthorized:
      stateStr = "Unauthorized";
      break;
    case CBManagerStateUnsupported:
      stateStr = "Unsupported";
      break;
    case CBManagerStateResetting:
      stateStr = "Resetting";
      break;
  }
  
  LogFormat("iOS Bluetooth: Starting scan (state: %s)", stateStr);
  
  if (centralManager.state != CBManagerStatePoweredOn) {
    LogFormat("iOS Bluetooth: Bluetooth not ready (state: %s) - will scan when powered on", stateStr);
    shouldScanWhenReady = YES;
    if (listener && centralManager.state == CBManagerStatePoweredOff)
      listener->OnError("Bluetooth is not powered on. Enable it in Settings.");
    else if (listener && centralManager.state == CBManagerStateUnauthorized)
      listener->OnError("Bluetooth permission not granted. Check Settings > Privacy.");
    return;
  }
  
  [discoveredPeripherals removeAllObjects];
  [peripheralData removeAllObjects];
  
  // Scan with options for better discovery
  NSDictionary *options = @{
    CBCentralManagerScanOptionAllowDuplicatesKey: @NO
  };
  [centralManager scanForPeripheralsWithServices:nil options:options];
  isScanning = YES;
  shouldScanWhenReady = NO;
  
  LogFormat("iOS Bluetooth: ✓ Scan started - devices will appear below...");
}

- (void)stopScanning {
  if (centralManager) {
    [centralManager stopScan];
    isScanning = NO;
    shouldScanWhenReady = NO;
  }
  
  if (listener)
    listener->OnDiscoveryFinished();
  
  LogFormat("iOS Bluetooth: Stopped device scan");
}

- (BOOL)isBluetoothAvailable {
  if (!centralManager)
    return NO;
    
  return centralManager.state == CBManagerStatePoweredOn;
}

- (NSArray*)getDiscoveredPeripherals {
  return [NSArray arrayWithArray:discoveredPeripherals];
}

- (CBPeripheral*)getPeripheralByUUID:(NSString*)uuidString {
  for (CBPeripheral* periph in discoveredPeripherals) {
    if ([periph.identifier.UUIDString isEqualToString:uuidString]) {
      LogFormat("iOS Bluetooth: Found peripheral for UUID: %s", [uuidString UTF8String]);
      return periph;
    }
  }
  LogFormat("iOS Bluetooth: Peripheral not found for UUID: %s", [uuidString UTF8String]);
  return nil;
}

- (CBCentralManager*)getCentralManager {
  return centralManager;
}

#pragma mark - CBCentralManagerDelegate

- (void)centralManagerDidUpdateState:(CBCentralManager *)central {
  switch (central.state) {
    case CBManagerStatePoweredOn:
      LogFormat("iOS Bluetooth: ✓ Powered ON - Ready to scan");
      // If scanning was requested before, start it now
      if (shouldScanWhenReady) {
        [self startScanning];
      }
      break;
    case CBManagerStatePoweredOff:
      LogFormat("iOS Bluetooth: ✗ Powered OFF - Enable in Settings > Bluetooth");
      isScanning = NO;
      if (listener)
        listener->OnError("Bluetooth is powered off. Enable it in Settings > Bluetooth.");
      break;
    case CBManagerStateUnsupported:
      LogFormat("iOS Bluetooth: ✗ NOT SUPPORTED on this device");
      isScanning = NO;
      if (listener)
        listener->OnError("Bluetooth is not supported on this device");
      break;
    case CBManagerStateUnauthorized:
      LogFormat("iOS Bluetooth: ✗ NOT AUTHORIZED - Grant permission in Settings");
      isScanning = NO;
      if (listener)
        listener->OnError("Bluetooth permission denied. Grant permission in Settings > Privacy.");
      break;
    case CBManagerStateUnknown:
      LogFormat("iOS Bluetooth: ? State unknown - initializing...");
      break;
    case CBManagerStateResetting:
      LogFormat("iOS Bluetooth: ! Resetting - wait a moment...");
      isScanning = NO;
      break;
  }
}

- (void)centralManager:(CBCentralManager *)central
 didDiscoverPeripheral:(CBPeripheral *)peripheral
     advertisementData:(NSDictionary<NSString *, id> *)advertisementData
                  RSSI:(NSNumber *)RSSI {
  
  // Accept devices even without names - some devices don't advertise names
  if (!peripheral || !peripheral.identifier) {
    return;
  }
  
  // Check if we already have this peripheral
  BOOL alreadyDiscovered = NO;
  for (CBPeripheral* p in discoveredPeripherals) {
    if ([p.identifier isEqual:peripheral.identifier]) {
      alreadyDiscovered = YES;
      break;
    }
  }
  
  if (!alreadyDiscovered) {
    [discoveredPeripherals addObject:peripheral];
    peripheral.delegate = self;
    
    // Get device name or use UUID if no name available
    NSString* deviceName = peripheral.name ? peripheral.name : 
                          [NSString stringWithFormat:@"Unnamed (%@)", peripheral.identifier.UUIDString];
    NSString* deviceUUID = peripheral.identifier.UUIDString;
    
    // Log advertisement data for debugging
    NSString* advertisementInfo = @"";
    if (advertisementData) {
      NSData* manufacturerData = advertisementData[CBAdvertisementDataManufacturerDataKey];
      NSArray* serviceUUIDs = advertisementData[CBAdvertisementDataServiceUUIDsKey];
      if (manufacturerData) {
        advertisementInfo = [NSString stringWithFormat:@", MfrData:%lu bytes", (unsigned long)manufacturerData.length];
      }
      if (serviceUUIDs && serviceUUIDs.count > 0) {
        advertisementInfo = [advertisementInfo stringByAppendingFormat:@", Services:%lu", (unsigned long)serviceUUIDs.count];
      }
    }
    
    // Notify listener
    if (listener) {
      AppleBluetoothDevice device(
        [deviceName UTF8String],
        [deviceUUID UTF8String],
        true  // BLE
      );
      listener->OnDeviceDiscovered(device);
    }
    
    LogFormat("iOS Bluetooth: Discovered device: %s [%s] RSSI:%ld%s",
              [deviceName UTF8String],
              [deviceUUID UTF8String],
              (long)[RSSI integerValue],
              [advertisementInfo UTF8String]);    
    // Cache the peripheral for later connection
    if (!g_cacheLock) {
      @synchronized(self) {
        if (!g_cacheLock) {
          g_cacheLock = [[NSObject alloc] init];
          g_discoveredPeripherals = [[NSMutableDictionary alloc] init];
        }
      }
    }
    @synchronized(g_cacheLock) {
      [g_discoveredPeripherals setObject:peripheral forKey:deviceUUID];
      g_centralManager = centralManager;
      LogFormat("iOS Bluetooth: Cached peripheral - total cached: %lu", (unsigned long)g_discoveredPeripherals.count);
    }  } else {
    // Update RSSI for already discovered device
    if ([RSSI integerValue] > -100) {
      // Periodically log RSSI updates for strong signals
      LogFormat("iOS Bluetooth: Update %s RSSI:%ld",
                peripheral.name ? [peripheral.name UTF8String] : "Unnamed",
                (long)[RSSI integerValue]);
    }
  }
}

- (void)centralManager:(CBCentralManager *)central
didConnectPeripheral:(CBPeripheral *)peripheral {
  LogFormat("iOS Bluetooth: Connected to %s", [peripheral.name UTF8String]);
  
  // Discover services
  [peripheral discoverServices:nil];
}

- (void)centralManager:(CBCentralManager *)central
didFailToConnectPeripheral:(CBPeripheral *)peripheral
                 error:(NSError *)error {
  LogFormat("iOS Bluetooth: Failed to connect to %s: %s",
            [peripheral.name UTF8String],
            [[error localizedDescription] UTF8String]);
  
  if (listener)
    listener->OnError([[error localizedDescription] UTF8String]);
}

- (void)centralManager:(CBCentralManager *)central
didDisconnectPeripheral:(CBPeripheral *)peripheral
                 error:(NSError *)error {
  LogFormat("iOS Bluetooth: Disconnected from %s", [peripheral.name UTF8String]);
}

#pragma mark - CBPeripheralDelegate

- (void)peripheral:(CBPeripheral *)peripheral
didDiscoverServices:(NSError *)error {
  if (error) {
    LogFormat("iOS Bluetooth: Error discovering services: %s",
              [[error localizedDescription] UTF8String]);
    return;
  }
  
  LogFormat("iOS Bluetooth: Discovered %lu services", 
            (unsigned long)peripheral.services.count);
  
  // Discover characteristics for each service
  for (CBService* service in peripheral.services) {
    [peripheral discoverCharacteristics:nil forService:service];
  }
}

- (void)peripheral:(CBPeripheral *)peripheral
didDiscoverCharacteristicsForService:(CBService *)service
              error:(NSError *)error {
  if (error) {
    LogFormat("iOS Bluetooth: Error discovering characteristics: %s",
              [[error localizedDescription] UTF8String]);
    return;
  }
  
  LogFormat("iOS Bluetooth: Discovered %lu characteristics",
            (unsigned long)service.characteristics.count);
}

@end

/**
 * Implementation class for AppleBluetoothManager
 */
class AppleBluetoothManagerImpl {
public:
  XCSoarCentralManagerDelegate* delegate;
  
  AppleBluetoothManagerImpl() : delegate(nil) {}
  
  ~AppleBluetoothManagerImpl() {
    // ARC handles cleanup
    delegate = nil;
  }
};

/**
 * AppleBluetoothManager implementation
 */
AppleBluetoothManager::AppleBluetoothManager()
  : impl(new AppleBluetoothManagerImpl()) {
  impl->delegate = [[XCSoarCentralManagerDelegate alloc] initWithListener:nullptr];
}

AppleBluetoothManager::~AppleBluetoothManager() {
  delete impl;
}

void AppleBluetoothManager::StartScanning(AppleBluetoothListener* listener) {
  if (!impl) {
    LogFormat("iOS Bluetooth: ERROR - impl is null");
    return;
  }
  
  if (!impl->delegate) {
    LogFormat("iOS Bluetooth: ERROR - delegate is null");
    if (listener)
      listener->OnError("Bluetooth manager not properly initialized");
    return;
  }
  
  LogFormat("iOS Bluetooth: StartScanning called with listener=%p", listener);
  
  // Update listener for this scan session
  [impl->delegate updateListener:listener];
  [impl->delegate startScanning];
}

void AppleBluetoothManager::StopScanning() {
  if (!impl || !impl->delegate)
    return;
  
  [impl->delegate stopScanning];
}

bool AppleBluetoothManager::IsBluetoothAvailable() const {
  if (!impl || !impl->delegate)
    return false;
  
  return [impl->delegate isBluetoothAvailable];
}

std::vector<AppleBluetoothDevice> AppleBluetoothManager::GetPairedDevices() const {
  std::vector<AppleBluetoothDevice> devices;
  
  if (!impl || !impl->delegate)
    return devices;
  
  NSArray* peripherals = [impl->delegate getDiscoveredPeripherals];
  
  for (CBPeripheral* peripheral in peripherals) {
    if (peripheral.name) {
      NSString* uuidStr = peripheral.identifier.UUIDString;
      AppleBluetoothDevice device(
        [peripheral.name UTF8String],
        [uuidStr UTF8String],
        true  // iOS primarily uses BLE
      );
      devices.push_back(device);
    }
  }
  
  return devices;
}

void* AppleBluetoothManager::GetPeripheralByUUID(const char* uuid_string) {
  if (!uuid_string)
    return nullptr;
  
  NSString* uuidNS = [NSString stringWithUTF8String:uuid_string];
  
  if (g_cacheLock && g_discoveredPeripherals) {
    @synchronized(g_cacheLock) {
      CBPeripheral* periph = [g_discoveredPeripherals objectForKey:uuidNS];
      if (periph) {
        LogFormat("iOS Bluetooth: Found cached peripheral for UUID: %s", uuid_string);
        return (__bridge void*)periph;
      }
    }
  }
  
  LogFormat("iOS Bluetooth: Peripheral not found in cache for UUID: %s", uuid_string);
  
  // Fallback: try using the delegate method
  if (!impl || !impl->delegate)
    return nullptr;
  
  CBPeripheral* periph = [impl->delegate getPeripheralByUUID:uuidNS];
  return (__bridge void*)periph;
}

void* AppleBluetoothManager::GetCentralManager() const {
  if (g_cacheLock && g_centralManager) {
    @synchronized(g_cacheLock) {
      if (g_centralManager) {
        LogFormat("iOS Bluetooth: Returning cached central manager");
        return (__bridge void*)g_centralManager;
      }
    }
  }
  
  if (!impl || !impl->delegate)
    return nullptr;
  
  CBCentralManager* manager = [impl->delegate getCentralManager];
  return (__bridge void*)manager;
}

void* GetCachedApplePeripheralByUUID(const char* uuid_string) {
  if (!uuid_string || !g_cacheLock || !g_discoveredPeripherals)
    return nullptr;

  NSString* uuidNS = [NSString stringWithUTF8String:uuid_string];
  @synchronized(g_cacheLock) {
    CBPeripheral* periph = [g_discoveredPeripherals objectForKey:uuidNS];
    if (periph) {
      LogFormat("iOS Bluetooth: Direct cache hit for peripheral UUID: %s", uuid_string);
      return (__bridge void*)periph;
    }
  }

  LogFormat("iOS Bluetooth: Direct cache miss for peripheral UUID: %s", uuid_string);
  return nullptr;
}

void* GetCachedAppleCentralManager() {
  if (!g_cacheLock || !g_centralManager)
    return nullptr;

  @synchronized(g_cacheLock) {
    if (g_centralManager) {
      LogFormat("iOS Bluetooth: Direct cache hit for central manager");
      return (__bridge void*)g_centralManager;
    }
  }

  return nullptr;
}

#endif // TARGET_OS_IPHONE
#endif // __APPLE__
