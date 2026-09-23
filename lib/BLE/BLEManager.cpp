#include "BLEManager.h"

BLEManager bleManager;

BLEManager::BLEManager() {
    _pServer = nullptr;
    _pTxCharacteristic = nullptr;
    _pRxCharacteristic = nullptr;
    _deviceConnected = false;
    _oldDeviceConnected = false;
    _commandReady = false;
    _rxBuffer = "";
    _lastCommand = "";
}

void BLEManager::begin(const char* deviceName) {
    // 1. Khởi tạo BLE Device
    BLEDevice::init(deviceName);

    // 2. Tạo BLE Server
    _pServer = BLEDevice::createServer();
    _pServer->setCallbacks(this);

    // 3. Tạo BLE Service (Nordic UART Service)
    BLEService *pService = _pServer->createService(SERVICE_UUID);

    // 4. Tạo TX Characteristic (Dùng gửi dữ liệu / Telemetry lên App)
    _pTxCharacteristic = pService->createCharacteristic(
                            CHARACTERISTIC_UUID_TX,
                            BLECharacteristic::PROPERTY_NOTIFY
                          );
    _pTxCharacteristic->addDescriptor(new BLE2902());

    // 5. Tạo RX Characteristic (Dùng nhận lệnh từ App)
    _pRxCharacteristic = pService->createCharacteristic(
                            CHARACTERISTIC_UUID_RX,
                            BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
                          );
    _pRxCharacteristic->setCallbacks(this);

    // 6. Bắt đầu Service
    pService->start();

    // 7. Cấu hình & Bắt đầu Quảng bá (Advertising)
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06); // Dành cho kết nối iPhone
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();

    Serial.println(">> BLE MANAGER DA KHOI TAO SAN SANG (Nordic UART Service)");
}

bool BLEManager::isConnected() const {
    return _deviceConnected;
}

void BLEManager::onConnect(BLEServer* pServer) {
    _deviceConnected = true;
    Serial.println(">> BLE DEVICE DA KET NOI!");
}

void BLEManager::onDisconnect(BLEServer* pServer) {
    _deviceConnected = false;
    Serial.println(">> BLE DEVICE DA NGAT KET NOI! BAT DAU QUANG BA LAI...");
    // Tự động quảng bá lại khi bị ngắt kết nối
    pServer->startAdvertising();
}

void BLEManager::onWrite(BLECharacteristic *pCharacteristic) {
    String rxValue = pCharacteristic->getValue().c_str();
    if (rxValue.length() > 0) {
        rxValue.trim();
        _lastCommand = rxValue;
        _commandReady = true;
        Serial.print(">> NHAN LENH BLE: ");
        Serial.println(_lastCommand);
    }
}

void BLEManager::print(const String &msg) {
    if (_deviceConnected && _pTxCharacteristic != nullptr) {
        _pTxCharacteristic->setValue((uint8_t*)msg.c_str(), msg.length());
        _pTxCharacteristic->notify();
    }
}

void BLEManager::println(const String &msg) {
    String fullMsg = msg + "\r\n";
    print(fullMsg);
}

bool BLEManager::hasCommand() {
    return _commandReady;
}

String BLEManager::readCommand() {
    _commandReady = false;
    String cmd = _lastCommand;
    _lastCommand = "";
    return cmd;
}
