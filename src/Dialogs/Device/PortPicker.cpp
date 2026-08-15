// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "PortPicker.hpp"
#include "PortDataField.hpp"
#include "Look/DialogLook.hpp"
#include "Form/Form.hpp"
#include "Dialogs/WidgetDialog.hpp"
#include "Widget/ListWidget.hpp"
#include "Renderer/TextRowRenderer.hpp"
#include "Form/DataField/Enum.hpp"
#include "Form/DataField/ComboList.hpp"
#include "ui/event/Notify.hpp"
#include "Language/Language.hpp"
#include "UIGlobals.hpp"
#include "LogFile.hpp"

#ifdef ANDROID
#include "java/Global.hxx"
#include "Android/Main.hpp"
#include "Android/BluetoothHelper.hpp"
#include "Android/UsbSerialHelper.hpp"
#include "Android/DetectDeviceListener.hpp"
#include "thread/Mutex.hxx"
#include <list>
#endif

#ifdef __APPLE__
#include <TargetConditionals.h>
#if TARGET_OS_IPHONE
#include "Apple/BluetoothManager.hpp"
#include "thread/Mutex.hxx"
#include <list>
#endif
#endif

#include <cassert>

class PortListItemRenderer final {
  TextRowRenderer row_renderer;

public:
  unsigned CalculateLayout(const DialogLook &look) noexcept {
    return row_renderer.CalculateLayout(*look.list.font);
  }

  void PaintItem(Canvas &canvas, PixelRect rc,
                 const ComboList::Item &item) noexcept {
    if (const char *text = ToDisplayString(DeviceConfig::PortType(item.int_value >> 16));
        text != nullptr)
      rc.right = row_renderer.DrawRightColumn(canvas, rc, text);

    row_renderer.DrawTextRow(canvas, rc, item.display_string.c_str());
  }

private:
  static const char *ToDisplayString(DeviceConfig::PortType type) noexcept {
    switch (type) {
    case DeviceConfig::PortType::RFCOMM:
      return "Bluetooth";

    case DeviceConfig::PortType::BLE_SERIAL:
      return _("BLE port");

    case DeviceConfig::PortType::BLE_SENSOR:
      return _("BLE sensor");

    case DeviceConfig::PortType::ANDROID_USB_SERIAL:
      return _("USB serial");

    default:
      return nullptr;
    }
  }
};

class PortPickerWidget
  : public ListWidget
#ifdef ANDROID
  , DetectDeviceListener
#endif
{
  WndForm &dialog;

  PortListItemRenderer item_renderer;

  DataFieldEnum &df;

  ComboList combo_list;

#ifdef ANDROID
  Java::LocalObject detect_listener;
  Java::LocalObject usb_serial_detect_listener;

  struct DetectedPort {
    DeviceConfig::PortType type;
    std::string address, name;
  };

  Mutex detected_mutex;
  std::list<DetectedPort> detected_list;

  UI::Notify detected_notify{[this]{ OnDetectedNotification(); }};
#endif

#ifdef __APPLE__
#if TARGET_OS_IPHONE
  struct DetectedPort {
    DeviceConfig::PortType type;
    std::string address, name;
  };

  std::unique_ptr<AppleBluetoothManager> bt_manager;
  std::unique_ptr<class iOSBluetoothListener> bt_listener;

  Mutex detected_mutex;
  std::list<DetectedPort> detected_list;
  bool is_scanning = false;

  UI::Notify detected_notify{[this]{ OnDetectedNotification(); }};
#endif
#endif

public:
  PortPickerWidget(WndForm &_dialog, DataFieldEnum &_df) noexcept
    :dialog(_dialog),
     df(_df) {}

private:
  const auto &GetSelectedItem() const noexcept {
    assert(!combo_list.empty());
    return combo_list[GetList().GetCursorIndex()];
  }

  void ReloadComboList() noexcept;

public:
  /* virtual methods from class Widget */

  void Prepare(ContainerWindow &parent,
               const PixelRect &rc) noexcept override {
    combo_list = df.CreateComboList(nullptr);

    const auto &look = dialog.GetLook();
    ListControl &list = CreateList(parent, look, rc,
                                   item_renderer.CalculateLayout(look));
    list.SetLength(combo_list.size());
    list.SetCursorIndex(combo_list.current_index);
  }

  bool Save(bool &changed) noexcept override {
    if (combo_list.empty())
      return true;

    const int old_value = df.GetValue();
    const auto &item = GetSelectedItem();
    if (item.int_value == old_value)
      /* no change */
      return true;

    changed = true;
    df.SetFromCombo(item.int_value, item.string_value.c_str());
    return true;
  }

  void Show(const PixelRect &rc) noexcept override {
    ListWidget::Show(rc);

#ifdef ANDROID
    if (bluetooth_helper != nullptr) {
      const auto env = Java::GetEnv();
      if (bluetooth_helper->HasLe(env))
        detect_listener =
          bluetooth_helper->AddDetectDeviceListener(env, *this);
    }

    if (usb_serial_helper != nullptr) {
      const auto env = Java::GetEnv();
      usb_serial_detect_listener =
        usb_serial_helper->AddDetectDeviceListener(env, *this);
    }
#endif

#ifdef __APPLE__
#if TARGET_OS_IPHONE
    is_scanning = true;
    OnStartIOSBluetoothScanning();
#endif
#endif
  }

  void Hide() noexcept override {
#ifdef ANDROID
    if (detect_listener) {
      bluetooth_helper->RemoveDetectDeviceListener(detect_listener.GetEnv(),
                                                   detect_listener);
      detect_listener = {};
    }

    if (usb_serial_detect_listener) {
      usb_serial_helper->RemoveDetectDeviceListener(usb_serial_detect_listener.GetEnv(),
                                                    usb_serial_detect_listener);
      usb_serial_detect_listener = {};
    }
#endif

#ifdef __APPLE__
#if TARGET_OS_IPHONE
    is_scanning = false;
    if (bt_manager) {
      bt_manager->StopScanning();
      bt_manager = nullptr;
      bt_listener = nullptr;
    }
#endif
#endif

    ListWidget::Hide();
  }

  /* virtual methods from class ListControl::Handler */

  void OnPaintItem(Canvas &canvas, const PixelRect rc,
                   unsigned idx) noexcept override {
    item_renderer.PaintItem(canvas, rc, combo_list[idx]);
  }

  bool CanActivateItem([[maybe_unused]] unsigned index) const noexcept override {
    return true;
  }

  void OnActivateItem([[maybe_unused]] unsigned index) noexcept override {
    dialog.SetModalResult(mrOK);
  }

#ifdef __APPLE__
#if TARGET_OS_IPHONE
  friend class iOSBluetoothListener;
  
  void OnIOSDeviceDiscovered(const AppleBluetoothDevice &device) noexcept {
    if (!is_scanning) {
      LogFormat("iOS Bluetooth: Ignoring device discovered callback - not scanning");
      return;
    }
    
    // Check if this device is already in the detected_list
    {
      const std::lock_guard lock{detected_mutex};
      for (const auto &existing : detected_list) {
        if (existing.address == device.address) {
          LogFormat("iOS Bluetooth: Device already discovered: %s", device.address.c_str());
          return;  // Device already discovered, skip it
        }
      }
    }
    
    // BLE devices broadcasting NMEA likely use BLE_HM10 protocol
    DetectedPort detected{
      DeviceConfig::PortType::BLE_SERIAL,
      device.address,
      device.name
    };
    
    {
      const std::lock_guard lock{detected_mutex};
      detected_list.emplace_back(detected);
    }
    
    detected_notify.SendNotification();
  }

  void OnStartIOSBluetoothScanning() noexcept;

private:
  void UpdateItem(DetectedPort &&detected) noexcept;
  void OnDetectedNotification() noexcept;
#endif
#endif

#ifdef ANDROID
private:
  /* virtual methods from class DetectDeviceListener */
  void OnDeviceDetected(Type type, const char *address,
                        const char *name,
                        uint64_t features) noexcept override;

  void UpdateItem(DetectedPort &&detected) noexcept;
  void OnDetectedNotification() noexcept;
#endif
};

#ifdef __APPLE__
#if TARGET_OS_IPHONE
/**
 * iOS Bluetooth listener for device discovery
 */
class iOSBluetoothListener : public AppleBluetoothListener {
  PortPickerWidget &widget;

public:
  explicit iOSBluetoothListener(PortPickerWidget &_widget) : widget(_widget) {}

  void OnDeviceDiscovered(const AppleBluetoothDevice &device) noexcept override {
    widget.OnIOSDeviceDiscovered(device);
  }

  void OnDiscoveryFinished() noexcept override {
    // Optional: notify when discovery is done
  }

  void OnError(const char *) noexcept override {
    // Errors are logged, continue scanning
  }
};

void
PortPickerWidget::OnStartIOSBluetoothScanning() noexcept
{
  if (!bt_manager) {
    bt_manager = std::make_unique<AppleBluetoothManager>();
    bt_listener = std::make_unique<iOSBluetoothListener>(*this);
    bt_manager->StartScanning(bt_listener.get());
  }
}
#endif
#endif

void
PortPickerWidget::ReloadComboList() noexcept
{
  const int old_value = combo_list.empty()
    ? -1
    : GetSelectedItem().int_value;

  combo_list = df.CreateComboList(nullptr);

  auto &list = GetList();
  list.SetLength(combo_list.size());

  int new_cursor = old_value >= 0 ? combo_list.Find(old_value) : -1;
  if (new_cursor >= 0)
    list.SetCursorIndex(new_cursor);

  list.Invalidate();
}

#ifdef ANDROID

void
PortPickerWidget::OnDeviceDetected(Type type, const char *address,
                                   const char *name,
                                   uint64_t features) noexcept
{
  if (name == nullptr)
    name = "";

  DeviceConfig::PortType port_type;
  switch (type) {
  case Type::IOIO:
    port_type = DeviceConfig::PortType::IOIOUART;
    break;

  case Type::BLUETOOTH_CLASSIC:
    port_type = DeviceConfig::PortType::RFCOMM;
    break;

  case Type::BLUETOOTH_LE:
    port_type = (features & DetectDeviceListener::FEATURE_BLE_SERIAL) != 0
      ? DeviceConfig::PortType::BLE_SERIAL
      : DeviceConfig::PortType::BLE_SENSOR;
    break;

  case Type::USB_SERIAL:
    port_type = DeviceConfig::PortType::ANDROID_USB_SERIAL;
    break;
  }

  {
    const std::lock_guard lock{detected_mutex};
    detected_list.emplace_back(DetectedPort{port_type, address, name});
  }

  detected_notify.SendNotification();
}

inline void
PortPickerWidget::UpdateItem(DetectedPort &&detected) noexcept
{
  UpdatePortEntry(df, detected.type, detected.address.c_str(),
                  detected.name.empty() ? nullptr : detected.name.c_str());
}

inline void
PortPickerWidget::OnDetectedNotification() noexcept
{
  {
    const std::lock_guard lock{detected_mutex};

    while (!detected_list.empty()) {
      UpdateItem(std::move(detected_list.front()));
      detected_list.pop_front();
    }
  }

  ReloadComboList();
}

#endif

#ifdef __APPLE__
#if TARGET_OS_IPHONE

inline void
PortPickerWidget::UpdateItem(DetectedPort &&detected) noexcept
{
  UpdatePortEntry(df, detected.type, detected.address.c_str(),
                  detected.name.empty() ? nullptr : detected.name.c_str());
}

inline void
PortPickerWidget::OnDetectedNotification() noexcept
{
  {
    const std::lock_guard lock{detected_mutex};

    while (!detected_list.empty()) {
      UpdateItem(std::move(detected_list.front()));
      detected_list.pop_front();
    }
  }

  // Reload combo list to reflect new devices
  // This preserves the current selection if it still exists
  auto &list = GetList();
  const int old_cursor = list.GetCursorIndex();
  const int old_value = (old_cursor >= 0 && old_cursor < (int)combo_list.size())
    ? combo_list[old_cursor].int_value
    : -1;

  combo_list = df.CreateComboList(nullptr);
  list.SetLength(combo_list.size());

  // Try to restore the old selection
  if (old_value >= 0) {
    int new_cursor = combo_list.Find(old_value);
    if (new_cursor >= 0) {
      list.SetCursorIndex(new_cursor);
    }
  }

  list.Invalidate();
}

#endif
#endif

bool
PortPicker(DataFieldEnum &df, const char *caption) noexcept
{
  TWidgetDialog<PortPickerWidget> dialog(WidgetDialog::Full{},
                                         UIGlobals::GetMainWindow(),
                                         UIGlobals::GetDialogLook(),
                                         caption);

  dialog.SetWidget(dialog, df);
  dialog.AddButton(_("Select"), mrOK);
  dialog.AddButton(_("Cancel"), mrCancel);
  dialog.ShowModal();

  return dialog.GetChanged();
}
