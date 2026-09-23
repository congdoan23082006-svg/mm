#ifndef BLE_MANAGER_H
#define BLE_MANAGER_H

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// UUIDs tiêu chuẩn cho Nordic UART Service (NUS)
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class BLEManager : public BLEServerCallbacks, public BLECharacteristicCallbacks {
public:
    BLEManager();

    void begin(const char* deviceName = "MM-Robot-BLE");
    bool isConnected() const;

    void print(const String &msg);
    void println(const String &msg);

    bool hasCommand();
    String readCommand();

    // Callbacks của BLEServerCallbacks
    void onConnect(BLEServer* pServer) override;
    void onDisconnect(BLEServer* pServer) override;

    // Callbacks của BLECharacteristicCallbacks
    void onWrite(BLECharacteristic *pCharacteristic) override;

private:
    BLEServer *_pServer;
    BLECharacteristic *_pTxCharacteristic;
    BLECharacteristic *_pRxCharacteristic;
    bool _deviceConnected;
    bool _oldDeviceConnected;

    String _rxBuffer;
    bool _commandReady;
    String _lastCommand;
};

extern BLEManager bleManager;

#endif
