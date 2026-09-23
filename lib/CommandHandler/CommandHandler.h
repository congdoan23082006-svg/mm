#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include <Arduino.h>
#include "BLEManager.h"
#include "RobotNav.h"

class CommandHandler {
public:
    CommandHandler();

    void update();
    void processBLECommands();
    void sendTelemetry();
    void printStatus();

private:
    unsigned long _lastTelemetryTime;
};

extern CommandHandler commandHandler;

#endif
