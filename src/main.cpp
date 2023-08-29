#include "BLEDevice.h"
//#include "BLEScan.h"
#include <Arduino.h>

// The remote service we wish to connect to.
static BLEUUID serviceUUID("4fafc201-1fb5-459e-8fcc-c5c9c331914b");
// The characteristic of the remote service we are interested in.
static BLEUUID    txUUID("beb5483e-36e1-4688-b7f5-ea07361b26a8");
static BLEUUID    rxUUID("beb5483e-36e1-4688-b7f5-ea07361b26a9");

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLERemoteCharacteristic* pRemoteRxCCharacteristic;
static BLEAdvertisedDevice* myDevice;

#define LOG_TAG "BLE_CLIENT"

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    ESP_LOGI(LOG_TAG, "Notify callback for characteristic %s of data length %d", pBLERemoteCharacteristic->getUUID().toString().c_str(), length);
    ESP_LOGI(LOG_TAG, "data: %s", pData);

}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    ESP_LOGI(LOG_TAG, "Connected to %s", myDevice->getAddress().toString().c_str());
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    ESP_LOGI(LOG_TAG, "Disconected from %s", myDevice->getAddress().toString().c_str());
  }
};

bool connectToServer() {
    // Serial.print("Forming a connection to ");
    // Serial.println(myDevice->getAddress().toString().c_str());
    ESP_LOGI(LOG_TAG, "Forming a connection to %s", myDevice->getAddress().toString().c_str());
    
    BLEClient*  pClient  = BLEDevice::createClient();
    ESP_LOGI(LOG_TAG, " - Created client");

    pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the remove BLE Server.
    pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    ESP_LOGI(LOG_TAG, " - Connected to server");
    pClient->setMTU(517); //set client to request maximum MTU from server (default is 23 otherwise)
    
    delay(3e3);
    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
    if (pRemoteService == nullptr) {
      ESP_LOGI(LOG_TAG, "Failed to find our service UUID: %s", serviceUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    ESP_LOGI(LOG_TAG, " - Found our service: %s", serviceUUID.toString().c_str());


    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteCharacteristic = pRemoteService->getCharacteristic(txUUID);
    if (pRemoteCharacteristic == nullptr) {
      ESP_LOGI(LOG_TAG, "Failed to find our characteristic UUID: %s", txUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    ESP_LOGI(LOG_TAG, " - Found our characteristic: %s", txUUID.toString().c_str());


    // Read the value of the characteristic.
    if(pRemoteCharacteristic->canRead()) {
      std::string value = pRemoteCharacteristic->readValue();
      ESP_LOGI(LOG_TAG, "The characteristic value was: %s", value.c_str());
    }

    if(pRemoteCharacteristic->canNotify())
      pRemoteCharacteristic->registerForNotify(notifyCallback);

    pRemoteRxCCharacteristic = pRemoteService->getCharacteristic(rxUUID);
    if(pRemoteRxCCharacteristic == nullptr){
      ESP_LOGI(LOG_TAG, "Failed to find our characteristic UUID: %s", rxUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    ESP_LOGI(LOG_TAG, " - Found our characteristic: %s", rxUUID.toString().c_str());


    connected = true;
    return true;
}
/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    ESP_LOGI(LOG_TAG, "BLE Advertised Device found: %s", advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() 
        && advertisedDevice.isAdvertisingService(serviceUUID)
        && advertisedDevice.getName() == "UAV_ESP32"
        ) {

      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;

    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks


void setup() {
  Serial.begin(115200);
  BLEDevice::init("");
  ESP_LOGI(LOG_TAG, "BLE Client Started");

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
} // End of setup.


// This is the Arduino main loop function.
void loop() {

  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are 
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    if (connectToServer()) {
        ESP_LOGI(LOG_TAG, "We are now connected to the BLE Server.");
    } else {
        ESP_LOGI(LOG_TAG, "We have failed to connect to the server; there is nothin more we will do.");
    }
    doConnect = false;
  }

  // If we are connected to a peer BLE Server, update the characteristic each time we are reached
  // with the current time since boot.
  if (connected) {
    String newValue = "Time since boot: " + String(millis()/1000);
    ESP_LOGI(LOG_TAG, "Setting new characteristic value to \"%s\"", newValue.c_str());
    
    // Set the characteristic's value to be the array of bytes that is actually a string.
    // pRemoteCharacteristic->writeValue(newValue.c_str(), newValue.length());
        // if can write:
    if(pRemoteRxCCharacteristic->canWrite()){
        auto timestamp = esp_timer_get_time();
      std::string string_timestamp = std::to_string(timestamp);
      pRemoteRxCCharacteristic->writeValue(string_timestamp.c_str(), string_timestamp.length());
      delay(1000);
    }
  }else if(doScan){
    BLEDevice::getScan()->start(0);  // this is just example to start scan after disconnect, most likely there is better way to do it in arduino
  }
  
  delay(1000); // Delay a second between loops.
} // End of loop