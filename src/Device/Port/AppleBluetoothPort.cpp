// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#ifdef __APPLE__
#include <TargetConditionals.h>
#if TARGET_OS_IPHONE

#include "AppleBluetoothPort.hpp"
#include "Apple/BluetoothManager.hpp"
#include "BufferedPort.hpp"
#include "Listener.hpp"
#include "LogFile.hpp"

#import <CoreBluetooth/CoreBluetooth.h>
#import <Foundation/Foundation.h>

// BLE_LOG writes to both XCSoar's internal log file AND NSLog so that
// messages are visible in idevicesyslog during live debugging.
#define BLE_LOG(fmt, ...) do { \
  NSLog(@"[XCSoar-BLE] " fmt, ##__VA_ARGS__); \
  LogFormat("iOS BLE: " fmt, ##__VA_ARGS__); \
} while(0)

class AppleBluetoothPort;

// Nordic UART Service (NUS) UUIDs
static NSString* const kNUSServiceUUID = @"6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
// NUS RX: central writes to peripheral
static NSString* const kNUSRxCharUUID  = @"6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
// NUS TX: peripheral notifies central
static NSString* const kNUSTxCharUUID  = @"6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

// HM-10 / CC41 module UUIDs (very common on BLE-NMEA devices)
static NSString* const kHM10ServiceUUID = @"0000FFE0-0000-1000-8000-00805F9B34FB";
static NSString* const kHM10CharUUID    = @"0000FFE1-0000-1000-8000-00805F9B34FB"; // bidirectional notify+write

// ISSC (Microchip) Transparent BLE UART — used by LXNAV Nano and similar devices
static NSString* const kISSCServiceUUID  = @"49535343-FE7D-4AE5-8FA9-9FAFD205E455";
static NSString* const kISSCRxCtrlUUID   = @"49535343-6DAA-4D02-ABF6-19569ACA69FE"; // write: flow-control enable (write 0x01)
static NSString* const kISSCTxNotifyUUID = @"49535343-1E4D-4BD9-BA61-23C647249616"; // notify: device→phone
static NSString* const kISSCTxNotify2UUID= @"49535343-ACA3-481C-91EC-D85E28A60318"; // notify+write
static NSString* const kISSCRxWriteUUID  = @"49535343-8841-43F4-A8D4-ECBE34729BB3"; // write: phone→device
static NSString* const kISSCCtrlUUID     = @"49535343-026E-3A9B-954C-97DAEF17E26E"; // notify+write (ctrl)

/**
 * Handles both the CBCentralManager and CBPeripheral delegates for a single
 * BLE connection. Owns the central manager for this connection.
 */
@interface XCSoarBLEConnection : NSObject <CBCentralManagerDelegate, CBPeripheralDelegate>
{
@private
  CBCentralManager*    centralManager;
  CBPeripheral*        peripheral;
  CBCharacteristic*    txChar;      // write (phone → device)
  CBCharacteristic*    rxChar;      // notify (device → phone)
  CBCharacteristic*    isscCtrlChar; // ISSC flow-control: write 0x01 to enable data
  BOOL                 rxCharFromNamedProtocol;
  BOOL                 txCharFromNamedProtocol;
  NSString*            targetUUID;
  AppleBluetoothPort*  port;
  PortListener*        listener;
  BOOL                 ready;
  BOOL                 reconnectAttempted;
  BOOL                 pairingPrompted;
}

- (instancetype)initWithUUID:(NSString*)uuid
                        port:(AppleBluetoothPort*)p
                    listener:(PortListener*)l;
- (void)disconnect;
- (NSInteger)write:(const uint8_t*)data length:(size_t)length;
- (BOOL)isReady;
- (BOOL)isPairingError:(NSError*)error;
- (void)handlePairingError:(NSError*)error context:(const char*)context;

@end

@implementation XCSoarBLEConnection

- (instancetype)initWithUUID:(NSString*)uuid
                        port:(AppleBluetoothPort*)p
                    listener:(PortListener*)l {
  self = [super init];
  if (self) {
    targetUUID = uuid;
    port = p;
    listener = l;
    ready = NO;
    reconnectAttempted = NO;
    pairingPrompted = NO;
    rxCharFromNamedProtocol = NO;
    txCharFromNamedProtocol = NO;
    isscCtrlChar = nil;

    // Always create a dedicated CBCentralManager for this connection.
    // Do NOT steal the global scan manager — that causes delegate races and
    // double-calls to startConnect from two threads simultaneously.
    // iOS keeps a system-level registry of seen peripherals, so
    // retrievePeripheralsWithIdentifiers: works on a brand-new manager as
    // long as the device has been scanned before (which it has, since the
    // user just selected it from the picker).
    NSDictionary* opts = @{CBCentralManagerOptionShowPowerAlertKey: @NO};
    dispatch_block_t create = ^{
      centralManager = [[CBCentralManager alloc] initWithDelegate:self
                                                            queue:dispatch_get_main_queue()
                                                          options:opts];
      BLE_LOG("Created dedicated connection manager for UUID %s",
                [uuid UTF8String]);
    };
    if ([NSThread isMainThread])
      create();
    else
      dispatch_sync(dispatch_get_main_queue(), create);
    // centralManagerDidUpdateState: will fire on the main queue and call
    // startConnect when/if Bluetooth is powered on.  Do NOT call startConnect
    // here — that would race with the delegate callback.
  }
  return self;
}

- (BOOL)isPairingError:(NSError*)error {
  if (!error)
    return NO;

  if ([error.domain isEqualToString:CBATTErrorDomain]) {
    switch ((CBATTError)(error.code)) {
      case CBATTErrorInsufficientAuthentication:
      case CBATTErrorInsufficientEncryption:
      case CBATTErrorInsufficientEncryptionKeySize:
        return YES;
      default:
        break;
    }
  }

  if ([error.domain isEqualToString:CBErrorDomain]) {
    switch ((CBError)(error.code)) {
      case CBErrorConnectionFailed:
      case CBErrorPeerRemovedPairingInformation:
      case CBErrorEncryptionTimedOut:
        return YES;
      default:
        break;
    }
  }

  NSString *desc = error.localizedDescription.lowercaseString;
  return [desc containsString:@"authentication"] ||
         [desc containsString:@"pair"] ||
         [desc containsString:@"encrypt"] ||
         [desc containsString:@"not permitted"];
}

- (void)handlePairingError:(NSError*)error context:(const char*)context {
  const char *err = error ? [[error localizedDescription] UTF8String] : "unknown";
  BLE_LOG("Pairing/auth required in %s: %s", context, err);

  if (!pairingPrompted) {
    pairingPrompted = YES;
    if (listener) {
      listener->PortError("BLE pairing required. When prompted by iOS, enter PIN 1234.");
    }
  }

  if (port)
    port->ForwardState(PortState::FAILED);
  if (listener)
    listener->PortStateChanged();
}

- (void)startConnect {
  if (!centralManager) {
    BLE_LOG("No central manager available for connection");
    if (port) port->ForwardState(PortState::FAILED);
    if (listener) listener->PortStateChanged();
    return;
  }

  if (centralManager.state != CBManagerStatePoweredOn) {
    BLE_LOG("Manager not powered on yet (state=%ld)", (long)centralManager.state);
    return;
  }

  NSUUID* uuid = [[NSUUID alloc] initWithUUIDString:targetUUID];
  if (!uuid) {
    // UUID is malformed (e.g. truncated by old StaticString<32> profile) —
    // fall through to a scan so we can find the device anyway.
    BLE_LOG("Invalid UUID '%s' — falling back to name-based scan", [targetUUID UTF8String]);
  }
  
  if (uuid) {
  // Step 1: try retrievePeripheralsWithIdentifiers — works for any peripheral
  //         iOS has seen before (i.e. appeared in a prior scan).
  if (!peripheral) {
    NSArray* known = [centralManager retrievePeripheralsWithIdentifiers:@[uuid]];
    if (known.count > 0) {
      peripheral = known[0];
      BLE_LOG("Retrieved known peripheral from system registry");
    }
  }

  // Step 2: check already-connected peripherals carrying a UART service.
  if (!peripheral) {
    CBUUID* nusSvc  = [CBUUID UUIDWithString:kNUSServiceUUID];
    CBUUID* hm10Svc = [CBUUID UUIDWithString:kHM10ServiceUUID];
    CBUUID* isscSvc = [CBUUID UUIDWithString:kISSCServiceUUID];
    for (CBPeripheral* p in [centralManager retrieveConnectedPeripheralsWithServices:
                              @[nusSvc, hm10Svc, isscSvc]]) {
      if ([p.identifier.UUIDString isEqualToString:targetUUID]) {
        peripheral = p;
        break;
      }
    }
  }

  if (peripheral) {
    peripheral.delegate = self;
    BLE_LOG("Using peripheral '%s' state=%ld",
              peripheral.name ? [peripheral.name UTF8String] : "(unnamed)",
              (long)peripheral.state);

    if (peripheral.state == CBPeripheralStateConnected) {
      [self centralManager:centralManager didConnectPeripheral:peripheral];
      return;
    }
    if (peripheral.state == CBPeripheralStateConnecting) {
      BLE_LOG("Already connecting — waiting for callback");
      return;
    }
    [centralManager connectPeripheral:peripheral options:nil];
  } else {
    // Peripheral not yet known to this manager — scan briefly to find it.
    BLE_LOG("Peripheral unknown, starting targeted scan for %s",
              [targetUUID UTF8String]);
    NSDictionary* scanOpts = @{CBCentralManagerScanOptionAllowDuplicatesKey: @NO};
    [centralManager scanForPeripheralsWithServices:nil options:scanOpts];
  }
}  // end if (uuid)
}  // end startConnect

- (void)disconnect {
  if (peripheral && centralManager) {
    [centralManager stopScan];
    [centralManager cancelPeripheralConnection:peripheral];
  }
  ready = NO;
  if (port) port->ForwardState(PortState::FAILED);
  if (listener) listener->PortStateChanged();
}

- (BOOL)isReady {
  return ready;
}

- (NSInteger)write:(const uint8_t*)data length:(size_t)length {
  if (!ready || !txChar || !peripheral)
    return -1;

  // Peripheral writes must be dispatched to the main queue (the queue the
  // CBCentralManager was created on).  Calling from a worker thread directly
  // can silently fail on some iOS versions.
  NSData* d = [NSData dataWithBytes:data length:length];
  CBCharacteristicWriteType wtype = (txChar.properties & CBCharacteristicPropertyWriteWithoutResponse)
    ? CBCharacteristicWriteWithoutResponse
    : CBCharacteristicWriteWithResponse;
  CBPeripheral* p = peripheral;
  CBCharacteristic* c = txChar;
  dispatch_async(dispatch_get_main_queue(), ^{
    [p writeValue:d forCharacteristic:c type:wtype];
  });
  return (NSInteger)length;
}

#pragma mark - CBCentralManagerDelegate

- (void)centralManagerDidUpdateState:(CBCentralManager*)central {
  switch (central.state) {
    case CBManagerStatePoweredOn:
      BLE_LOG("Manager powered on — starting connection");
      reconnectAttempted = NO;
      [self startConnect];
      break;
    case CBManagerStatePoweredOff:
      BLE_LOG("Bluetooth is off");
      if (port) port->ForwardState(PortState::FAILED);
      if (listener) listener->PortStateChanged();
      break;
    case CBManagerStateUnauthorized:
      BLE_LOG("Bluetooth not authorized");
      if (port) port->ForwardState(PortState::FAILED);
      if (listener) listener->PortStateChanged();
      break;
    default:
      BLE_LOG("Central state changed: %ld", (long)central.state);
      break;
  }
}

- (void)centralManager:(CBCentralManager*)central
didDiscoverPeripheral:(CBPeripheral*)periph
    advertisementData:(NSDictionary*)advData
                 RSSI:(NSNumber*)RSSI {
  if ([periph.identifier.UUIDString isEqualToString:targetUUID]) {
    peripheral = periph;
    peripheral.delegate = self;
    [centralManager stopScan];
    BLE_LOG("Found target peripheral during scan, connecting...");
    [centralManager connectPeripheral:peripheral options:nil];
  }
}

- (void)centralManager:(CBCentralManager*)central
  didConnectPeripheral:(CBPeripheral*)periph {
  BLE_LOG("Connected to '%s', discovering ALL services...",
            periph.name ? [periph.name UTF8String] : "(unnamed)");
  periph.delegate = self;
  // Discover ALL services — matches the Android approach of gatt.discoverServices()
  // Narrowing to NUS only misses HM-10 and other BLE-NMEA protocols
  [periph discoverServices:nil];
}

- (void)centralManager:(CBCentralManager*)central
didFailToConnectPeripheral:(CBPeripheral*)periph
                 error:(NSError*)error {
  BLE_LOG("Failed to connect: %s",
            [[error localizedDescription] UTF8String]);

  if ([self isPairingError:error]) {
    [self handlePairingError:error context:"didFailToConnectPeripheral"];
    return;
  }

  // One immediate retry handles transient CoreBluetooth race conditions that
  // are common when reconnecting quickly.
  if (!reconnectAttempted) {
    reconnectAttempted = YES;
    dispatch_after(dispatch_time(DISPATCH_TIME_NOW, (int64_t)(500 * NSEC_PER_MSEC)),
                   dispatch_get_main_queue(), ^{
      BLE_LOG("Retrying connection once after didFailToConnect");
      [self startConnect];
    });
    return;
  }

  if (port) port->ForwardState(PortState::FAILED);
  if (listener) listener->PortStateChanged();
}

- (void)centralManager:(CBCentralManager*)central
didDisconnectPeripheral:(CBPeripheral*)periph
                 error:(NSError*)error {
  ready = NO;
  reconnectAttempted = NO;
  pairingPrompted = NO;
  if (error) {
    BLE_LOG("Disconnected with error: %s",
              [[error localizedDescription] UTF8String]);
  } else {
    BLE_LOG("Disconnected cleanly");
  }
  if (port) port->ForwardState(PortState::FAILED);
  if (listener) listener->PortStateChanged();
}

#pragma mark - CBPeripheralDelegate

- (void)peripheral:(CBPeripheral*)periph
didDiscoverServices:(NSError*)error {
  if (error) {
    BLE_LOG("Error discovering services: %s",
              [[error localizedDescription] UTF8String]);
    if ([self isPairingError:error]) {
      [self handlePairingError:error context:"didDiscoverServices"];
      return;
    }
    if (port) port->ForwardState(PortState::FAILED);
    if (listener) listener->PortStateChanged();
    return;
  }
  
  BLE_LOG("Discovered %lu services", (unsigned long)periph.services.count);
  
  if (periph.services.count == 0) {
    BLE_LOG("No services found — failing");
    if (port) port->ForwardState(PortState::FAILED);
    if (listener) listener->PortStateChanged();
    return;
  }
  
  // Discover ALL characteristics for EVERY service — mirrors Android's approach
  for (CBService* svc in periph.services) {
    BLE_LOG("Found service %s", [[svc.UUID UUIDString] UTF8String]);
    [periph discoverCharacteristics:nil forService:svc];
  }
}

- (void)peripheral:(CBPeripheral*)periph
didDiscoverCharacteristicsForService:(CBService*)svc
              error:(NSError*)error {
  if (error) {
    BLE_LOG("Error discovering characteristics: %s",
              [[error localizedDescription] UTF8String]);
    if ([self isPairingError:error]) {
      [self handlePairingError:error context:"didDiscoverCharacteristicsForService"];
    }
    return;
  }
  
  // For NUS, TX (003) is notify from device -> app, RX (002) is write from app -> device.
  CBUUID* nusNotifyUUID    = [CBUUID UUIDWithString:kNUSTxCharUUID];
  CBUUID* nusWriteUUID     = [CBUUID UUIDWithString:kNUSRxCharUUID];
  CBUUID* hm10UUID         = [CBUUID UUIDWithString:kHM10CharUUID];       // 0000FFE1 (notify+write)
  // ISSC / Transparent UART (LXNAV Nano etc.)
  CBUUID* isscRxCtrlUUID    = [CBUUID UUIDWithString:kISSCRxCtrlUUID];    // flow control (write 0x01)
  CBUUID* isscTxNotifyUUID  = [CBUUID UUIDWithString:kISSCTxNotifyUUID];  // primary notify
  CBUUID* isscTxNotify2UUID = [CBUUID UUIDWithString:kISSCTxNotify2UUID]; // secondary notify
  CBUUID* isscRxWriteUUID   = [CBUUID UUIDWithString:kISSCRxWriteUUID];   // write
  CBUUID* isscCtrlUUID      = [CBUUID UUIDWithString:kISSCCtrlUUID];      // ctrl (notify+write)

  // Helper blocks: named-protocol matches always override a fallback that was
  // set by an earlier service callback (e.g. Generic Attribute "Service Changed"
  // has CBCharacteristicPropertyIndicate and can accidentally become the fallback
  // rxChar before ISSC service characteristics are processed).
  void (^setRxNamed)(CBCharacteristic*) = ^(CBCharacteristic* c) {
    if (!rxCharFromNamedProtocol) {
      // Cancel any subscription on the previously-set fallback char
      if (rxChar && !rxCharFromNamedProtocol)
        [periph setNotifyValue:NO forCharacteristic:rxChar];
      rxChar = c;
      rxCharFromNamedProtocol = YES;
      [periph setNotifyValue:YES forCharacteristic:c];
    }
  };
  void (^setTxNamed)(CBCharacteristic*) = ^(CBCharacteristic* c) {
    if (!txCharFromNamedProtocol) {
      txChar = c;
      txCharFromNamedProtocol = YES;
    }
  };

  for (CBCharacteristic* c in svc.characteristics) {
    BOOL canNotify = (c.properties & (CBCharacteristicPropertyNotify | CBCharacteristicPropertyIndicate)) != 0;
    BOOL canWrite  = (c.properties & (CBCharacteristicPropertyWrite |
                                      CBCharacteristicPropertyWriteWithoutResponse)) != 0;

    BLE_LOG("Characteristic %s props=0x%02x notify=%d write=%d",
              [[c.UUID UUIDString] UTF8String], (unsigned)c.properties,
              (int)canNotify, (int)canWrite);

    // === NUS TX (6E400003): notify, device → phone ===
    if ([c.UUID isEqual:nusNotifyUUID] && canNotify) {
      setRxNamed(c);
      BLE_LOG("NUS TX notify");
    }
    // === NUS RX (6E400002): write, phone → device ===
    else if ([c.UUID isEqual:nusWriteUUID] && canWrite) {
      setTxNamed(c);
      BLE_LOG("NUS RX write");
    }
    // === HM-10 (FFE1): bidirectional ===
    else if ([c.UUID isEqual:hm10UUID]) {
      if (canNotify) { setRxNamed(c); BLE_LOG("HM-10 notify"); }
      if (canWrite)  { setTxNamed(c); BLE_LOG("HM-10 write"); }
    }
    // === ISSC Transparent UART — LXNAV Nano, Microchip RN-series ===
    else if ([c.UUID isEqual:isscRxCtrlUUID]) {
      // Flow-control char: we write 0x01 after subscribing to enable data output
      isscCtrlChar = c;
      BLE_LOG("ISSC RX_CTRL found");
    }
    else if ([c.UUID isEqual:isscTxNotifyUUID] && canNotify) {
      setRxNamed(c);
      BLE_LOG("ISSC TX notify (primary)");
    }
    else if ([c.UUID isEqual:isscRxWriteUUID] && canWrite) {
      setTxNamed(c);
      BLE_LOG("ISSC RX write");
    }
    else if ([c.UUID isEqual:isscTxNotify2UUID] && canNotify) {
      // Always subscribe; use as primary only if primary not yet named
      [periph setNotifyValue:YES forCharacteristic:c];
      BLE_LOG("ISSC TX notify2 (extra subscribe)");
      if (!rxCharFromNamedProtocol) { rxChar = c; }
    }
    else if ([c.UUID isEqual:isscCtrlUUID]) {
      if (canNotify) [periph setNotifyValue:YES forCharacteristic:c];
      if (canWrite)  { setTxNamed(c); BLE_LOG("ISSC CTRL write fallback"); }
    }
    // === Generic fallback — only if no named match won yet ===
    else {
      if (canNotify && !rxChar) {
        rxChar = c;
        [periph setNotifyValue:YES forCharacteristic:c];
        BLE_LOG("Fallback notify char %s", [[c.UUID UUIDString] UTF8String]);
      }
      if (canWrite && !txChar) {
        txChar = c;
        BLE_LOG("Fallback write char %s", [[c.UUID UUIDString] UTF8String]);
      }
    }
  }
  // NOTE: port state goes to READY in didUpdateNotificationStateForCharacteristic:
  //       once iOS confirms the subscription — mirrors Android's onDescriptorWrite
}

- (void)peripheral:(CBPeripheral*)periph
didUpdateValueForCharacteristic:(CBCharacteristic*)c
              error:(NSError*)error {
  if (error) {
    BLE_LOG("Data error: %s", [[error localizedDescription] UTF8String]);
    if ([self isPairingError:error]) {
      [self handlePairingError:error context:"didUpdateValueForCharacteristic"];
    }
    return;
  }
  
  if (c.value && port) {
    // *** THE KEY FIX: push data directly into the BufferedPort FIFO ***
    static int dataCount = 0;
    if (++dataCount <= 5 || dataCount % 100 == 0) {
      BLE_LOG("DATA recv #%d: %lu bytes on %s", dataCount,
              (unsigned long)c.value.length, [[c.UUID UUIDString] UTF8String]);
    }
    port->ForwardData(c.value.bytes, c.value.length);
  }
}

- (void)peripheral:(CBPeripheral*)periph
didUpdateNotificationStateForCharacteristic:(CBCharacteristic*)c
              error:(NSError*)error {
  if (error) {
    BLE_LOG("Notification enable error for %s: %s",
              [[c.UUID UUIDString] UTF8String],
              [[error localizedDescription] UTF8String]);
    if ([self isPairingError:error]) {
      [self handlePairingError:error context:"didUpdateNotificationStateForCharacteristic"];
      return;
    }
    // Don't fail the whole connection — another characteristic might work
    return;
  }
  
  BLE_LOG("Notifications %s for %s",
            c.isNotifying ? "ENABLED" : "disabled",
            [[c.UUID UUIDString] UTF8String]);
  
  // Port becomes READY the moment the first notification subscription is confirmed
  // Mirrors Android's onDescriptorWrite → setStateSafe(STATE_READY)
  if (c.isNotifying && !ready) {
    ready = YES;
    BLE_LOG("✓ Port READY — notification confirmed on %s",
              [[c.UUID UUIDString] UTF8String]);

    // ISSC flow-control: write 0x01 to RX_CTRL to enable data output from device
    // This is mandatory for the ISSC transparent UART — without it the device
    // stays silent even though notifications are subscribed.
    if (isscCtrlChar) {
      uint8_t enable = 0x01;
      NSData* d = [NSData dataWithBytes:&enable length:1];
      [periph writeValue:d forCharacteristic:isscCtrlChar
                    type:CBCharacteristicWriteWithResponse];
      BLE_LOG("ISSC flow-control 0x01 sent to RX_CTRL");
    }

    if (port) port->ForwardState(PortState::READY);
    if (listener) listener->PortStateChanged();
  }
}

- (void)peripheral:(CBPeripheral*)periph
didWriteValueForCharacteristic:(CBCharacteristic*)characteristic
              error:(NSError*)error {
  if (!error)
    return;

  BLE_LOG("Write error on %s: %s",
            [[characteristic.UUID UUIDString] UTF8String],
            [[error localizedDescription] UTF8String]);

  if ([self isPairingError:error]) {
    [self handlePairingError:error context:"didWriteValueForCharacteristic"];
  }
}

@end

// -----------------------------------------------------------------------
// C++ wrappers
// -----------------------------------------------------------------------

/**
 * Thin C++ wrapper around XCSoarBLEConnection so we can use it from
 * AppleBluetoothPort without exposing Objective-C headers.
 */
class AppleBluetoothConnection {
  __strong XCSoarBLEConnection* conn;
  
public:
  AppleBluetoothConnection(const char* address,
                           PortListener* listener,
                           AppleBluetoothPort* port) {
    NSString* uuidStr = [NSString stringWithUTF8String:address];
    conn = [[XCSoarBLEConnection alloc] initWithUUID:uuidStr
                                                port:port
                                            listener:listener];
  }
  
  ~AppleBluetoothConnection() {
    if (conn) {
      [conn disconnect];
      conn = nil;
    }
  }
  
  std::size_t Write(std::span<const std::byte> src) {
    if (!conn || ![conn isReady])
      return 0;
    NSInteger n = [conn write:(const uint8_t*)src.data() length:src.size()];
    return n > 0 ? (std::size_t)n : 0;
  }
};

// -----------------------------------------------------------------------
// AppleBluetoothPort
// -----------------------------------------------------------------------

void AppleBluetoothPort::ForwardState(PortState new_state) noexcept {
  stored_state = new_state;
}

AppleBluetoothPort::AppleBluetoothPort(const char *address,
                                       PortListener *listener,
                                       DataHandler &handler)
  : BufferedPort(listener, handler), device_address(address) {
  connection = new AppleBluetoothConnection(address, listener, this);
}

AppleBluetoothPort::~AppleBluetoothPort() noexcept {
  delete connection;
}

PortState AppleBluetoothPort::GetState() const noexcept {
  return stored_state;
}

bool AppleBluetoothPort::Drain() noexcept {
  return true;
}

unsigned AppleBluetoothPort::GetBaudrate() const noexcept {
  return 115200;
}

void AppleBluetoothPort::SetBaudrate([[maybe_unused]] unsigned baud_rate) {
  // No baud rate for BLE
}

std::size_t AppleBluetoothPort::Write(std::span<const std::byte> src) {
  if (!connection)
    return 0;
  return connection->Write(src);
}

std::unique_ptr<Port>
OpenAppleBluetoothPort(const char *address, PortListener *listener,
                       DataHandler &handler) {
  return std::make_unique<AppleBluetoothPort>(address, listener, handler);
}

#endif // TARGET_OS_IPHONE
#endif // __APPLE__
