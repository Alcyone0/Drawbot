#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>

// Log functions
void addLog(String message);
String getAllLogs();

// Function to initialize the logger
void setupLogger();

#endif // LOGGER_H
