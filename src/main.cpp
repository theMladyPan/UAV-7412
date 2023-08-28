#include <Arduino.h>
#include <BLEDevice.h>
#include <esp_log.h>

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

BLEScan* pBLEScan;
BLEAddress* steam_controller_mac = NULL;
static bool ble_connected = false;


static BLEUUID     hidUUID("00001812-0000-1000-8000-00805f9b34fb"); /* controller announces this service */
static BLEUUID serviceUUID("100F6C32-1735-4313-B402-38567131E5F3"); /* but instead use this one */
static BLEUUID   inputUUID("100F6C33-1735-4313-B402-38567131E5F3"); /* and this characterstic within it to receive the reports */
static BLEUUID    rprtUUID("100F6C34-1735-4313-B402-38567131E5F3"); /* plus this one to configure the controller for sending */

void connect_to_controller(BLEAddress steam_controller_mac);

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    // print the mac address of the discovered device:
    ESP_LOGI("BLE Device", "Discovered device name: %s", advertisedDevice.getName().c_str());
    // if device name is SteamController, connect to it:
    if (advertisedDevice.getName() == "SteamController") {
      ESP_LOGI("BLE Device", "Discovered device: %s", advertisedDevice.getAddress().toString().c_str());
      // keep the address:
      steam_controller_mac = new BLEAddress(advertisedDevice.getAddress());
      // stop scanning:
      pBLEScan->stop();
    }
  }
};


class MyClientCallback : public BLEClientCallbacks
{
  void onConnect(BLEClient* pclient) 
  {
  }

  void onDisconnect(BLEClient* pclient)
  {
    ble_connected = false;
  }
};

void setup() {
  Serial.begin(1500000);
  ESP_LOGI("BLE Device", "Scanning for BLE devices...");

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(30);

  ESP_LOGI("BLE Device", "Scanning complete!");

  // if we found the Steam Controller, connect to it:
  if (steam_controller_mac != NULL) {
    connect_to_controller(*steam_controller_mac);
  }
}

void connect_to_controller(BLEAddress steam_controller_mac) {
  ESP_LOGI("BLE Device", "Connecting to Steam Controller...");  
  // stop scanning:
  pBLEScan->stop();
  // connect to the Steam Controller:
  BLEClient*  pClient  = BLEDevice::createClient();
  pClient->setClientCallbacks(new MyClientCallback());
  pClient->connect(steam_controller_mac);
  // get the HID service:
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  // get the input characteristic:
  BLERemoteCharacteristic* pRemoteCharacteristic = pRemoteService->getCharacteristic(inputUUID);
  // enable notifications:
  pRemoteCharacteristic->registerForNotify([](BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
    // print the received data:
    ESP_LOGI("BLE Device", "Received data: %s", pData);
  });
  // get the report characteristic:
  pRemoteCharacteristic = pRemoteService->getCharacteristic(rprtUUID);
  
  ESP_LOGI("BLE Device", "Connected to Steam Controller!");
  delay(1e3);
}

void loop() {
  // This loop is intentionally left empty
  // The BLE scanning process occurs in the background
  // if *steam_controller_mac != NULL, then we have found the Steam Controller
  // and can connect to it
  
}
