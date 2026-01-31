#ifndef UBLOX_NMEA_H
#define UBLOX_NMEA_H

#include "py/obj.h"
#include "py/runtime.h"
#include <math.h>

// Структура для хранения GPS данных
typedef struct {
    double latitude;
    double longitude;
    double altitude;
    double speed;
    double course;
    uint8_t satellites_used;
    uint8_t satellites_visible;
    uint8_t fix_type;
    double hdop;
    double vdop;
    double pdop;
    double accuracy;  // точность в метрах
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t valid;
    uint8_t has_gga;
    uint8_t has_gsa;
    uint8_t has_gsv;
    uint8_t has_vtg;
    uint8_t has_satellites_used;
    uint8_t has_satellites_visible;
    uint8_t has_accuracy;
    char timestamp[25]; // Формат: "2024-01-15T14:30:45Z" + null terminator
} gps_data_t;

#endif