#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>
#include <string>
#include <ctime>
#include <cstdarg> 

using namespace std;

enum LogLevel {
    INFO,
    WARNING,
    ERROR
};

class Logger {
public:
    Logger(const std::string& tag);
    void log(LogLevel level, const char* format, ...);

private:
    string currentDateTime();
    std::string tag_;
};

#endif // LOGGER_H