#include "simpĺe_logger.h"

Logger::Logger(const std::string& tag) : tag_(tag) {}

void Logger::log(LogLevel level, const char* format, ...) {
    currentDateTime();

    // Construir el mensaje formateado
    va_list args;
    va_start(args, format);
    char buffer[4000]; // Tamaño del buffer ajustable
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    // Imprimir el mensaje según el nivel
    switch (level) {
        case INFO:
            std::cout << "[INFO] ";
            break;
        case WARNING:
            std::cout << "[WARNING] ";
            break;
        case ERROR:
            std::cout << "[ERROR] ";
            break;
    }
    std::cout << buffer << std::endl;
}

string Logger::currentDateTime() {
    time_t now = time(0);
    struct tm tstruct;
    char buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);
    return buf;
}
