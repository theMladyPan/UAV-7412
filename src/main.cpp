#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <BLEServer.h>
#include <BLEAdvertisedDevice.h>
#include <BLE2902.h>

#define STEAM_CONTROLLER_BUTTON_A                  0x800000
#define STEAM_CONTROLLER_BUTTON_X                  0x400000
#define STEAM_CONTROLLER_BUTTON_B                  0x200000
#define STEAM_CONTROLLER_BUTTON_Y                  0x100000
#define STEAM_CONTROLLER_BUTTON_LEFT_UPPER_PADDLE  0x080000
#define STEAM_CONTROLLER_BUTTON_RIGHT_UPPER_PADDLE 0x040000
#define STEAM_CONTROLLER_BUTTON_LEFT_PADDLE        0x020000
#define STEAM_CONTROLLER_BUTTON_RIGHT_PADDLE       0x010000
#define STEAM_CONTROLLER_BUTTON_LEFT_INNER_PADDLE  0x008000
#define STEAM_CONTROLLER_BUTTON_NAV_RIGHT          0x004000
#define STEAM_CONTROLLER_BUTTON_STEAM              0x002000
#define STEAM_CONTROLLER_BUTTON_NAV_LEFT           0x001000
#define STEAM_CONTROLLER_BUTTON_JOYSTICK           0x000040
#define STEAM_CONTROLLER_BUTTON_RIGHT_TOUCH        0x000010
#define STEAM_CONTROLLER_BUTTON_LEFT_TOUCH         0x000008
#define STEAM_CONTROLLER_BUTTON_RIGHT_PAD          0x000004
#define STEAM_CONTROLLER_BUTTON_LEFT_PAD           0x000002
#define STEAM_CONTROLLER_BUTTON_RIGHT_INNER_PADDLE 0x000001

#define STEAM_CONTROLLER_FLAG_REPORT               0x0004
#define STEAM_CONTROLLER_FLAG_BUTTONS              0x0010
#define STEAM_CONTROLLER_FLAG_PADDLES              0x0020
#define STEAM_CONTROLLER_FLAG_JOYSTICK             0x0080
#define STEAM_CONTROLLER_FLAG_LEFT_PAD             0x0100
#define STEAM_CONTROLLER_FLAG_RIGHT_PAD            0x0200


// enable notifications:
uint8_t startReportCommand[] = { 0xC0, 0x87, 0x03, 0x08, 0x07, 0x00 };  // 192, 135, 3, 8, 7, 0

BLEScan* pBLEScan;
static bool ble_connected = false;

static BLEUUID     hidUUID("00001812-0000-1000-8000-00805f9b34fb");
static BLEUUID serviceUUID("100F6C32-1735-4313-B402-38567131E5F3");  // Steam Controller Custom service UUID
static BLEUUID   inputUUID("100F6C33-1735-4313-B402-38567131E5F3");  // Steam Controller Input characteristic UUID
static BLEUUID    rprtUUID("100F6C34-1735-4313-B402-38567131E5F3");  // Steam Controller Report characteristic UUID

void connectToController();

BLEClient* pClient;
BLERemoteService* pService;
BLERemoteCharacteristic* pReportCharacteristic;
BLERemoteCharacteristic* pInputCharacteristic;
BLEAddress* pSteamControllerMac;
esp_ble_addr_type_t steamControllerType;

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    // print the mac address of the discovered device:
    ESP_LOGI("BLE Device", "Discovered device name: %s", advertisedDevice.getName().c_str());
    // if device name is SteamController, connect to it:
    if (advertisedDevice.getName() == "SteamController") {
      ESP_LOGI("BLE Device", "Discovered device: %s", advertisedDevice.getAddress().toString().c_str());

      // stop scanning:
      pBLEScan->stop();
      // connect to the device:
      pSteamControllerMac = new BLEAddress(advertisedDevice.getAddress().toString());
      steamControllerType = advertisedDevice.getAddressType();

      connectToController();
    }
  }
};

class ConnectCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pClient) {
    log_i("Connected to BLE client!");
  }

  void onDisconnect(BLEClient* pClient) {
    log_i("Disconnected from BLE client!");
    ble_connected = false;
  }
};

void setup() {
  Serial.begin(1500000);
  log_i("Scanning for BLE devices...");

  BLEDevice::init("ESP32");

  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(30);

  log_i("Scanning complete!");
}

class InputCallback : public BLECharacteristicCallbacks {
  void onNotify(BLECharacteristic* pCharacteristic, esp_ble_gatts_cb_param_t* param) {
    std::string value = pCharacteristic->getValue();
    log_i("Notified with value: %s", value.c_str());
  }

  void onRead(BLECharacteristic* pCharacteristic, esp_ble_gatts_cb_param_t* param) {
    log_i("Read from Steam Controller!");
  }
};


class ReportCallback : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) {
    std::string value = pCharacteristic->getValue();
    log_i("Got value: %s", value.c_str());
  }

  void onRead(BLECharacteristic* pCharacteristic, esp_ble_gatts_cb_param_t* param) {
    log_i("Read from Steam Controller!");
  }
};

static void parsePacket(uint8_t* buf, size_t len) {
  // print the packet:
  log_i("Got packet: ");
  for (int i = 0; i < len; i++) {
    log_i("%02X ", buf[i]);
  }
}

static void report_cbr(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* buf, size_t len, bool isNotify)
{
  parsePacket(buf, len);
}

void connectToController() {
  pClient = BLEDevice::createClient();
  log_i("Created BLE client!");

  pClient->setClientCallbacks(new ConnectCallback());

  log_i("Connecting to Steam Controller...");
  pClient->connect(*pSteamControllerMac, steamControllerType);
  log_i("Connected to Steam Controller!");

  pService = pClient->getService(serviceUUID);
  if(pService == nullptr) {
    log_i("Service is null!");
    esp_restart();
  }

  log_i("Created service!");

  pInputCharacteristic = pService->getCharacteristic(inputUUID);
  if(pInputCharacteristic == nullptr) {
    log_i("Input characteristic is null!");
    esp_restart();
  }
  log_i("Got input characteristic!");

  if(pInputCharacteristic->canNotify()) {
    log_i("Input characteristic can notify!");
      pInputCharacteristic->registerForNotify(report_cbr);
  } else {
    log_i("Input characteristic cannot notify!");
  }

  pReportCharacteristic = pService->getCharacteristic(rprtUUID);
  if(pReportCharacteristic == nullptr) {
    log_i("Report characteristic is null!");
    esp_restart();
  }
  log_i("Got report characteristic!");

  if(pReportCharacteristic && pReportCharacteristic->canWrite())
  {
    pReportCharacteristic->writeValue(startReportCommand, sizeof(startReportCommand));
  }
  else
  {
    log_i("[BLE]   FAILED to start reporting");
    pClient->disconnect();
    esp_restart();
  }
  ble_connected = true;
}

void loop() {
  // Empty loop, BLE scanning and connection handled in setup()
  while(ble_connected) {
  }
  esp_restart();
}
