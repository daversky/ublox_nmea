#include "ublox_nmea.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Константы для расчета расстояния
#define EARTH_RADIUS_M 6371000.0
#define DEG_TO_RAD (M_PI / 180.0)

// Глобальная структура для хранения текущих GPS данных
static gps_data_t current_gps_data;
static uint8_t gps_data_initialized = 0;

// Прототипы функций
static void gps_data_init(gps_data_t* gps_data);
static double calculate_distance_haversine(double lat1, double lon1, double lat2, double lon2);
static double parse_coordinate(const char* coord, char direction);
static int checksum_valid(const char* sentence);
static int parse_fields(const char* sentence, char fields[][16], int max_fields);
static void parse_gga(const char* sentence, gps_data_t* gps_data);
static void parse_rmc(const char* sentence, gps_data_t* gps_data);
static void parse_gsa(const char* sentence, gps_data_t* gps_data);
static void parse_gsv(const char* sentence, gps_data_t* gps_data);
static void parse_vtg(const char* sentence, gps_data_t* gps_data);
static mp_obj_t create_gps_dict_from_data(gps_data_t* gps_data);
static void update_timestamp(gps_data_t* gps_data);
static double calculate_accuracy(double hdop, uint8_t satellites_used);
static void update_accuracy(gps_data_t* gps_data);

// Инициализация структуры GPS данных
static void gps_data_init(gps_data_t* gps_data) {
    gps_data->latitude = NAN;
    gps_data->longitude = NAN;
    gps_data->altitude = NAN;
    gps_data->speed = NAN;
    gps_data->course = NAN;
    gps_data->satellites_used = 0;
    gps_data->satellites_visible = 0;
    gps_data->fix_type = 0;
    gps_data->hdop = NAN;
    gps_data->vdop = NAN;
    gps_data->pdop = NAN;
    gps_data->accuracy = NAN;
    gps_data->year = 0;
    gps_data->month = 0;
    gps_data->day = 0;
    gps_data->hour = 0;
    gps_data->minute = 0;
    gps_data->second = 0;
    gps_data->valid = 0;
    gps_data->has_gga = 0;
    gps_data->has_gsa = 0;
    gps_data->has_gsv = 0;
    gps_data->has_vtg = 0;
    gps_data->has_satellites_used = 0;
    gps_data->has_satellites_visible = 0;
    gps_data->has_accuracy = 0;
    gps_data->timestamp[0] = '\0';  // Инициализируем пустой строкой
}

// Функция расчета accuracy на основе HDOP и количества спутников
static double calculate_accuracy(double hdop, uint8_t satellites_used) {
    double base_accuracy = hdop * 4.9;  // базовый множитель

    // Корректировка по количеству спутников
    if (satellites_used >= 8) {
        base_accuracy *= 0.7;  // высокая точность
    } else if (satellites_used >= 5) {
        base_accuracy *= 0.9;  // средняя точность
    } else if (satellites_used <= 3) {
        base_accuracy *= 1.5;  // низкая точность
    }

    return base_accuracy;
}

// Обновление accuracy
static void update_accuracy(gps_data_t* gps_data) {
    if (!isnan(gps_data->hdop) && gps_data->has_satellites_used) {
        gps_data->accuracy = calculate_accuracy(gps_data->hdop, gps_data->satellites_used);
        gps_data->accuracy = round(gps_data->accuracy * 10.0) / 10.0; // округление
        gps_data->has_accuracy = 1;
    } else {
        gps_data->accuracy = NAN;
        gps_data->has_accuracy = 0;
    }
}

// Функция для обновления timestamp в структуре
static void update_timestamp(gps_data_t* gps_data) {
    // Проверяем наличие полной даты и времени
    if (gps_data->year > 0 && gps_data->month > 0 && gps_data->day > 0) {
        int result = snprintf(gps_data->timestamp, sizeof(gps_data->timestamp),
                "%04d-%02d-%02dT%02d:%02d:%02dZ",
                gps_data->year,
                gps_data->month,
                gps_data->day,
                gps_data->hour,
                gps_data->minute,
                gps_data->second);

        // Проверяем успешность форматирования
        if (result <= 0 || result >= (int)sizeof(gps_data->timestamp)) {
            gps_data->timestamp[0] = '\0'; // сбрасываем при ошибке
        }
    } else {
        gps_data->timestamp[0] = '\0';
    }
}

// Функция для создания словаря из текущих GPS данных
static mp_obj_t create_gps_dict_from_data(gps_data_t* gps_data) {
    mp_obj_dict_t* dict = mp_obj_new_dict(0);

    // Основные поля (всегда присутствуют)
    mp_obj_dict_store(dict, mp_obj_new_str("valid", 5), mp_obj_new_bool(gps_data->valid));

    // Координаты (только если есть данные)
    if (!isnan(gps_data->latitude)) {
        mp_obj_dict_store(dict, mp_obj_new_str("latitude", 8), mp_obj_new_float(gps_data->latitude));
    }

    if (!isnan(gps_data->longitude)) {
        mp_obj_dict_store(dict, mp_obj_new_str("longitude", 9), mp_obj_new_float(gps_data->longitude));
    }

    // Высота (только если есть данные из GGA)
    if (gps_data->has_gga && !isnan(gps_data->altitude)) {
        mp_obj_dict_store(dict, mp_obj_new_str("altitude", 8), mp_obj_new_float(gps_data->altitude));
    }

    // Скорость (только если есть данные)
    if (!isnan(gps_data->speed)) {
        double rounded_speed = round(gps_data->speed * 10.0) / 10.0;
        mp_obj_dict_store(dict, mp_obj_new_str("speed", 5), mp_obj_new_float(rounded_speed));
    }

    // Курс (только если есть данные)
    if (!isnan(gps_data->course)) {
        double rounded_course = round(gps_data->course * 10.0) / 10.0;
        mp_obj_dict_store(dict, mp_obj_new_str("course", 6), mp_obj_new_float(rounded_course));
    }

    // Спутники used (только если есть данные)
    if (gps_data->has_satellites_used) {
        mp_obj_dict_store(dict, mp_obj_new_str("satellites_used", 15), mp_obj_new_int(gps_data->satellites_used));
    }

    // Спутники visible (только если есть данные)
    if (gps_data->has_satellites_visible) {
        mp_obj_dict_store(dict, mp_obj_new_str("satellites_visible", 18), mp_obj_new_int(gps_data->satellites_visible));
    }

    // Тип фикса (только если есть данные из GGA)
    if (gps_data->has_gga) {
        mp_obj_dict_store(dict, mp_obj_new_str("fix_type", 8), mp_obj_new_int(gps_data->fix_type));
    }

    // DOP параметры (только если есть данные из GSA)
    if (gps_data->has_gsa && !isnan(gps_data->hdop)) {
        double rounded_hdop = round(gps_data->hdop * 10.0) / 10.0;
        mp_obj_dict_store(dict, mp_obj_new_str("hdop", 4), mp_obj_new_float(rounded_hdop));
    }

    if (gps_data->has_gsa && !isnan(gps_data->vdop)) {
        double rounded_vdop = round(gps_data->vdop * 10.0) / 10.0;
        mp_obj_dict_store(dict, mp_obj_new_str("vdop", 4), mp_obj_new_float(rounded_vdop));
    }

    if (gps_data->has_gsa && !isnan(gps_data->pdop)) {
        double rounded_pdop = round(gps_data->pdop * 10.0) / 10.0;
        mp_obj_dict_store(dict, mp_obj_new_str("pdop", 4), mp_obj_new_float(rounded_pdop));
    }

    // Accuracy (только если есть данные)
    if (gps_data->has_accuracy && !isnan(gps_data->accuracy)) {
        mp_obj_dict_store(dict, mp_obj_new_str("accuracy", 8), mp_obj_new_float(gps_data->accuracy));
    }

    // Дата (только если есть данные)
    if (gps_data->year > 0 && gps_data->month > 0 && gps_data->day > 0) {
        mp_obj_t date_items[3];
        date_items[0] = mp_obj_new_int(gps_data->day);
        date_items[1] = mp_obj_new_int(gps_data->month);
        date_items[2] = mp_obj_new_int(gps_data->year);
        mp_obj_dict_store(dict, mp_obj_new_str("date", 4), mp_obj_new_list(3, date_items));
    }

    // Время (только если есть данные)
    if (gps_data->year > 0) { // используем year как индикатор наличия времени
        mp_obj_t time_items[3];
        time_items[0] = mp_obj_new_int(gps_data->hour);
        time_items[1] = mp_obj_new_int(gps_data->minute);
        time_items[2] = mp_obj_new_int(gps_data->second);
        mp_obj_dict_store(dict, mp_obj_new_str("time", 4), mp_obj_new_list(3, time_items));
    }

    // Timestamp в формате ISO 8601 (только если есть данные)
    if (gps_data->timestamp[0] != '\0') {
        mp_obj_dict_store(dict, mp_obj_new_str("timestamp", 9),
                         mp_obj_new_str(gps_data->timestamp, strlen(gps_data->timestamp)));
    }

    return MP_OBJ_FROM_PTR(dict);
}

static double parse_coordinate(const char* coord, char direction) {
    if (strlen(coord) < 7) return NAN;

    // Находим позицию десятичной точки
    const char* dot_pos = strchr(coord, '.');
    if (!dot_pos) return NAN;

    int dot_index = dot_pos - coord;

    // Определяем количество цифр в градусах по позиции точки
    // Форматы NMEA:
    // DDM.MMMMM   - точка на позиции 2 (нестандартный)
    // DDMM.MMMMM  - точка на позиции 4 (стандартный для широты)
    // DDDMM.MMMMM - точка на позиции 5 (стандартный для долготы)
    // DDDDMM.MMMMM - точка на позиции 6 (расширенный)

    int degree_digits;
    if (dot_index == 2) {
        degree_digits = 1;  // DDM.MMMMM
    } else if (dot_index == 3) {
        degree_digits = 2;  // DDM.MMMMM (редкий)
    } else if (dot_index == 4) {
        degree_digits = 2;  // DDMM.MMMMM (широта)
    } else if (dot_index == 5) {
        degree_digits = 3;  // DDDMM.MMMMM (долгота)
    } else if (dot_index == 6) {
        degree_digits = 4;  // DDDDMM.MMMMM (высокая точность)
    } else {
        return NAN;  // Неизвестный формат
    }

    // Извлекаем градусы
    char degrees_str[5] = {0};  // увеличен буфер для 4 цифр
    strncpy(degrees_str, coord, degree_digits);
    degrees_str[degree_digits] = '\0';

    // Извлекаем минуты (всё после градусов)
    char minutes_str[16] = {0};
    strncpy(minutes_str, coord + degree_digits, sizeof(minutes_str) - 1);

    double degrees = atof(degrees_str);
    double minutes = atof(minutes_str);

    double result = degrees + minutes / 60.0;

    if (direction == 'S' || direction == 'W') {
        result = -result;
    }

    return result;
}

static int checksum_valid(const char* sentence) {
    if (sentence[0] != '$') return 0;

    const char* checksum_start = strchr(sentence, '*');
    if (!checksum_start) return 0;

    uint8_t calculated_checksum = 0;
    for (const char* p = sentence + 1; p < checksum_start; p++) {
        calculated_checksum ^= *p;
    }

    uint8_t received_checksum = (uint8_t)strtol(checksum_start + 1, NULL, 16);
    return calculated_checksum == received_checksum;
}

// Улучшенный парсинг полей
static int parse_fields(const char* sentence, char fields[][16], int max_fields) {
    int field_count = 0;
    const char* start = sentence;

    while (*sentence && field_count < max_fields) {
        if (*sentence == ',') {
            size_t len = sentence - start;
            if (len < sizeof(fields[0]) - 1) {
                memcpy(fields[field_count], start, len);
                fields[field_count][len] = '\0';
            } else {
                fields[field_count][0] = '\0';
            }
            field_count++;
            start = sentence + 1;
        }
        sentence++;
    }

    // Обработка последнего поля
    if (*start && field_count < max_fields) {
        const char* end = strchr(start, '*');
        if (!end) end = sentence;
        size_t len = end - start;
        if (len < sizeof(fields[0]) - 1) {
            memcpy(fields[field_count], start, len);
            fields[field_count][len] = '\0';
        } else {
            fields[field_count][0] = '\0';
        }
        field_count++;
    }

    return field_count;
}

// Парсинг GGA сообщения
static void parse_gga(const char* sentence, gps_data_t* gps_data) {
    char fields[20][16] = {0};
    int field_count = parse_fields(sentence, fields, 20);

    if (field_count < 14) return;

    // Время
    if (strlen(fields[1]) >= 6) {
        gps_data->hour = (fields[1][0] - '0') * 10 + (fields[1][1] - '0');
        gps_data->minute = (fields[1][2] - '0') * 10 + (fields[1][3] - '0');
        gps_data->second = (fields[1][4] - '0') * 10 + (fields[1][5] - '0');
    }

    // Координаты (GGA имеет приоритет для высоты и точности)
    if (strlen(fields[2]) > 0 && strlen(fields[3]) > 0) {
        gps_data->latitude = parse_coordinate(fields[2], fields[3][0]);
    }

    if (strlen(fields[4]) > 0 && strlen(fields[5]) > 0) {
        gps_data->longitude = parse_coordinate(fields[4], fields[5][0]);
    }

    // Качество фикса
    if (strlen(fields[6]) > 0) {
        gps_data->fix_type = atoi(fields[6]);
    }

    // Количество спутников
    if (strlen(fields[7]) > 0) {
        gps_data->satellites_used = atoi(fields[7]);
        gps_data->has_satellites_used = 1;
    }

    // HDOP
    if (strlen(fields[8]) > 0) {
        gps_data->hdop = atof(fields[8]);
    }

    // Высота (округляем до 1 знака после запятой)
    if (strlen(fields[9]) > 0) {
        gps_data->altitude = round(atof(fields[9]) * 10.0) / 10.0;
    }

    gps_data->has_gga = 1;

    // Обновляем accuracy и timestamp
    update_accuracy(gps_data);
    update_timestamp(gps_data);
}

// Парсинг RMC сообщения
static void parse_rmc(const char* sentence, gps_data_t* gps_data) {
    char fields[20][16] = {0};
    int field_count = parse_fields(sentence, fields, 20);

    if (field_count < 12) return;

    // ВАЖНО: RMC НЕ сбрасывает флаги других предложений
    // Каждое предложение дополняет общую картину

    // Время (RMC имеет приоритет для даты и общего статуса)
    if (strlen(fields[1]) >= 6) {
        gps_data->hour = (fields[1][0] - '0') * 10 + (fields[1][1] - '0');
        gps_data->minute = (fields[1][2] - '0') * 10 + (fields[1][3] - '0');
        gps_data->second = (fields[1][4] - '0') * 10 + (fields[1][5] - '0');
    }

    // Статус - основной индикатор валидности позиции
    gps_data->valid = (fields[2][0] == 'A') ? 1 : 0;

    // Координаты (используем если нет от GGA или GGA невалидны)
    if (strlen(fields[3]) > 0 && strlen(fields[4]) > 0 &&
        (isnan(gps_data->latitude) || !gps_data->has_gga)) {
        gps_data->latitude = parse_coordinate(fields[3], fields[4][0]);
    }

    if (strlen(fields[5]) > 0 && strlen(fields[6]) > 0 &&
        (isnan(gps_data->longitude) || !gps_data->has_gga)) {
        gps_data->longitude = parse_coordinate(fields[5], fields[6][0]);
    }

    // Скорость (узлы -> м/с, округляем до 1 знака после запятой)
    if (strlen(fields[7]) > 0) {
        double speed_knots = atof(fields[7]);
        gps_data->speed = round(speed_knots * 0.514444 * 10.0) / 10.0;
    }

    // Курс из RMC (округляем до 1 знака после запятой)
    if (strlen(fields[8]) > 0) {
        gps_data->course = round(atof(fields[8]) * 10.0) / 10.0;
    }

    // Дата (RMC - основной источник даты)
    if (strlen(fields[9]) >= 6) {
        gps_data->day = (fields[9][0] - '0') * 10 + (fields[9][1] - '0');
        gps_data->month = (fields[9][2] - '0') * 10 + (fields[9][3] - '0');
        gps_data->year = 2000 + (fields[9][4] - '0') * 10 + (fields[9][5] - '0');
    }

    // Обновляем timestamp
    update_timestamp(gps_data);
}

// Парсинг GSA сообщения
static void parse_gsa(const char* sentence, gps_data_t* gps_data) {
    char fields[20][16] = {0};
    int field_count = parse_fields(sentence, fields, 20);

    if (field_count < 17) return;

    // PDOP, HDOP, VDOP (округляем до 1 знака после запятой)
    // GSA имеет приоритет для DOP параметров
    if (strlen(fields[15]) > 0) {
        gps_data->pdop = round(atof(fields[15]) * 10.0) / 10.0;
    }

    if (strlen(fields[16]) > 0) {
        gps_data->hdop = round(atof(fields[16]) * 10.0) / 10.0;
    }

    if (strlen(fields[17]) > 0) {
        gps_data->vdop = round(atof(fields[17]) * 10.0) / 10.0;
    }

    gps_data->has_gsa = 1;

    // Обновляем accuracy
    update_accuracy(gps_data);
}

// Парсинг GSV сообщения - подсчет видимых спутников
static void parse_gsv(const char* sentence, gps_data_t* gps_data) {
    char fields[20][16] = {0};
    int field_count = parse_fields(sentence, fields, 20);

    if (field_count < 4) return;

    // ЛОГИКА: В GSV предложениях:
    // Поле 1 - общее количество GSV сообщений для полного набора данных
    // Поле 2 - номер текущего GSV сообщения
    // Поле 3 - общее количество видимых спутников
    // Поля 4-7, 8-11, 12-15, 16-19 - данные по 4 спутникам за сообщение

    // Используем поле 3 - общее количество видимых спутников
    if (strlen(fields[3]) > 0) {
        gps_data->satellites_visible = atoi(fields[3]);
        gps_data->has_satellites_visible = 1;
        gps_data->has_gsv = 1;
    }
}

// Парсинг VTG сообщения - курс и скорость относительно земли
static void parse_vtg(const char* sentence, gps_data_t* gps_data) {
    char fields[20][16] = {0};
    int field_count = parse_fields(sentence, fields, 20);

    if (field_count < 8) return;

    // VTG используется как дополнение к RMC
    // Курс относительно истинного севера (поле 1)
    if (strlen(fields[1]) > 0 && isnan(gps_data->course)) {
        gps_data->course = round(atof(fields[1]) * 10.0) / 10.0;
    }

    // Скорость в км/ч (поле 7) - конвертируем в м/с
    // Используем только если в RMC не было скорости
    if (strlen(fields[7]) > 0 && (isnan(gps_data->speed) || gps_data->speed < 0.1)) {
        double speed_kmh = atof(fields[7]);
        gps_data->speed = round((speed_kmh / 3.6) * 10.0) / 10.0;
    }

    gps_data->has_vtg = 1;
}

// Функция расчета расстояния между двумя точками (формула гаверсинуса)
static double calculate_distance_haversine(double lat1, double lon1, double lat2, double lon2) {
    double lat1_rad = lat1 * DEG_TO_RAD;
    double lon1_rad = lon1 * DEG_TO_RAD;
    double lat2_rad = lat2 * DEG_TO_RAD;
    double lon2_rad = lon2 * DEG_TO_RAD;

    double dlat = lat2_rad - lat1_rad;
    double dlon = lon2_rad - lon1_rad;

    double a = sin(dlat/2) * sin(dlat/2) +
               cos(lat1_rad) * cos(lat2_rad) *
               sin(dlon/2) * sin(dlon/2);

    double c = 2 * atan2(sqrt(a), sqrt(1-a));

    return EARTH_RADIUS_M * c;
}

// Универсальная функция для расчета расстояния
static mp_obj_t calculate_distance(size_t n_args, const mp_obj_t *args) {
    // Проверяем количество аргументов
    if (n_args < 1 || n_args > 2) {
        mp_raise_TypeError(MP_ERROR_TEXT("calculate_distance() takes 1 or 2 arguments"));
    }

    double lat1, lon1, lat2, lon2;

    if (n_args == 1) {
        // Один аргумент - расчет от текущей позиции до указанной точки
        mp_obj_t target_obj = args[0];

        // Проверяем есть ли текущие координаты
        if (!gps_data_initialized || isnan(current_gps_data.latitude) || isnan(current_gps_data.longitude)) {
            return mp_const_none;
        }

        // Текущая позиция - стандартный порядок [lat, lon]
        lat1 = current_gps_data.latitude;
        lon1 = current_gps_data.longitude;

        // Целевая точка - ожидаем [lat, lon]
        if (!mp_obj_is_type(target_obj, &mp_type_tuple) && !mp_obj_is_type(target_obj, &mp_type_list)) {
            mp_raise_TypeError(MP_ERROR_TEXT("target must be tuple or list [lat, lon]"));
        }

        size_t target_len = 0;
        mp_obj_t *target_items;
        if (mp_obj_is_type(target_obj, &mp_type_tuple)) {
            mp_obj_tuple_get(target_obj, &target_len, &target_items);
        } else {
            mp_obj_list_get(target_obj, &target_len, &target_items);
        }

        if (target_len < 2) {
            mp_raise_ValueError(MP_ERROR_TEXT("target must have at least 2 elements [lat, lon]"));
        }

        // Извлекаем координаты цели в порядке [lat, lon]
        if (mp_obj_is_float(target_items[0])) {
            lat2 = mp_obj_get_float(target_items[0]);
        } else if (mp_obj_is_int(target_items[0])) {
            lat2 = (double)mp_obj_get_int(target_items[0]);
        } else {
            mp_raise_TypeError(MP_ERROR_TEXT("latitude must be float or int"));
        }

        if (mp_obj_is_float(target_items[1])) {
            lon2 = mp_obj_get_float(target_items[1]);
        } else if (mp_obj_is_int(target_items[1])) {
            lon2 = (double)mp_obj_get_int(target_items[1]);
        } else {
            mp_raise_TypeError(MP_ERROR_TEXT("longitude must be float or int"));
        }
    } else {
        // Два аргумента - расчет между двумя точками
        mp_obj_t a_obj = args[0];
        mp_obj_t b_obj = args[1];

        // Проверяем что a и b - кортежи или списки в формате [lat, lon]
        if (!mp_obj_is_type(a_obj, &mp_type_tuple) && !mp_obj_is_type(a_obj, &mp_type_list)) {
            mp_raise_TypeError(MP_ERROR_TEXT("first argument must be tuple or list [lat, lon]"));
        }

        if (!mp_obj_is_type(b_obj, &mp_type_tuple) && !mp_obj_is_type(b_obj, &mp_type_list)) {
            mp_raise_TypeError(MP_ERROR_TEXT("second argument must be tuple or list [lat, lon]"));
        }

        // Получаем размеры
        size_t a_len = 0;
        mp_obj_t *a_items;
        if (mp_obj_is_type(a_obj, &mp_type_tuple)) {
            mp_obj_tuple_get(a_obj, &a_len, &a_items);
        } else {
            mp_obj_list_get(a_obj, &a_len, &a_items);
        }

        size_t b_len = 0;
        mp_obj_t *b_items;
        if (mp_obj_is_type(b_obj, &mp_type_tuple)) {
            mp_obj_tuple_get(b_obj, &b_len, &b_items);
        } else {
            mp_obj_list_get(b_obj, &b_len, &b_items);
        }

        // Проверяем размеры
        if (a_len < 2) {
            mp_raise_ValueError(MP_ERROR_TEXT("first point must have at least 2 elements [lat, lon]"));
        }

        if (b_len < 2) {
            mp_raise_ValueError(MP_ERROR_TEXT("second point must have at least 2 elements [lat, lon]"));
        }

        // Первая точка: [lat1, lon1]
        if (mp_obj_is_float(a_items[0])) {
            lat1 = mp_obj_get_float(a_items[0]);
        } else if (mp_obj_is_int(a_items[0])) {
            lat1 = (double)mp_obj_get_int(a_items[0]);
        } else {
            mp_raise_TypeError(MP_ERROR_TEXT("latitude must be float or int"));
        }

        if (mp_obj_is_float(a_items[1])) {
            lon1 = mp_obj_get_float(a_items[1]);
        } else if (mp_obj_is_int(a_items[1])) {
            lon1 = (double)mp_obj_get_int(a_items[1]);
        } else {
            mp_raise_TypeError(MP_ERROR_TEXT("longitude must be float or int"));
        }

        // Вторая точка: [lat2, lon2]
        if (mp_obj_is_float(b_items[0])) {
            lat2 = mp_obj_get_float(b_items[0]);
        } else if (mp_obj_is_int(b_items[0])) {
            lat2 = (double)mp_obj_get_int(b_items[0]);
        } else {
            mp_raise_TypeError(MP_ERROR_TEXT("latitude must be float or int"));
        }

        if (mp_obj_is_float(b_items[1])) {
            lon2 = mp_obj_get_float(b_items[1]);
        } else if (mp_obj_is_int(b_items[1])) {
            lon2 = (double)mp_obj_get_int(b_items[1]);
        } else {
            mp_raise_TypeError(MP_ERROR_TEXT("longitude must be float or int"));
        }
    }

    // Проверяем валидность координат
    if (lat1 < -90.0 || lat1 > 90.0 || lat2 < -90.0 || lat2 > 90.0) {
        mp_raise_ValueError(MP_ERROR_TEXT("latitude must be between -90 and 90 degrees"));
    }

    if (lon1 < -180.0 || lon1 > 180.0 || lon2 < -180.0 || lon2 > 180.0) {
        mp_raise_ValueError(MP_ERROR_TEXT("longitude must be between -180 and 180 degrees"));
    }

    // Вычисляем расстояние
    double distance = calculate_distance_haversine(lat1, lon1, lat2, lon2);

    // Округляем до 1 знака после запятой
    distance = round(distance * 10.0) / 10.0;

    return mp_obj_new_float(distance);
}

// Основная функция парсинга NMEA строк
static mp_obj_t parse_nmea_string(mp_obj_t string_obj) {
    const char* nmea_string = mp_obj_str_get_str(string_obj);

    if (!nmea_string || strlen(nmea_string) < 6) {
        return mp_const_none;
    }

    if (!checksum_valid(nmea_string)) {
        return mp_const_none;
    }

    // Инициализация при первом вызове
    if (!gps_data_initialized) {
        gps_data_init(&current_gps_data);
        gps_data_initialized = 1;
    }

    // Определяем тип сообщения и парсим
    // ЛОГИКА: Каждое предложение дополняет общую картину данных
    if (strstr(nmea_string, "$GPRMC") == nmea_string ||
        strstr(nmea_string, "$GNRMC") == nmea_string) {
        parse_rmc(nmea_string, &current_gps_data);
    }
    else if (strstr(nmea_string, "$GPGGA") == nmea_string ||
             strstr(nmea_string, "$GNGGA") == nmea_string) {
        parse_gga(nmea_string, &current_gps_data);
    }
    else if (strstr(nmea_string, "$GPGSA") == nmea_string ||
             strstr(nmea_string, "$GNGSA") == nmea_string) {
        parse_gsa(nmea_string, &current_gps_data);
    }
    else if (strstr(nmea_string, "$GPGSV") == nmea_string ||
             strstr(nmea_string, "$GLGSV") == nmea_string ||
             strstr(nmea_string, "$GNGSV") == nmea_string ||
             strstr(nmea_string, "$GBGSV") == nmea_string) {
        parse_gsv(nmea_string, &current_gps_data);
    }
    else if (strstr(nmea_string, "$GPVTG") == nmea_string ||
             strstr(nmea_string, "$GNVTG") == nmea_string) {
        parse_vtg(nmea_string, &current_gps_data);
    }

    return create_gps_dict_from_data(&current_gps_data);
}

// Функция сброса данных
static mp_obj_t reset_gps_data(void) {
    gps_data_init(&current_gps_data);
    gps_data_initialized = 1;
    return mp_const_none;
}

// Функция для получения текущих данных
static mp_obj_t get_current(void) {
    // Проверяем инициализацию
    if (!gps_data_initialized) {
        // Возвращаем только valid: False
        mp_obj_dict_t* dict = mp_obj_new_dict(1);
        mp_obj_dict_store(dict, mp_obj_new_str("valid", 5), mp_obj_new_bool(0));
        return MP_OBJ_FROM_PTR(dict);
    }

    // Возвращаем текущие данные (только существующие поля)
    return create_gps_dict_from_data(&current_gps_data);
}

// Определение функций для модуля
MP_DEFINE_CONST_FUN_OBJ_1(parse_nmea_string_obj, parse_nmea_string);
MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(calculate_distance_obj, 1, 2, calculate_distance);
MP_DEFINE_CONST_FUN_OBJ_0(reset_gps_data_obj, reset_gps_data);
MP_DEFINE_CONST_FUN_OBJ_0(get_current_obj, get_current);

// Определение модуля
static const mp_rom_map_elem_t ublox_nmea_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_ublox_nmea) },
    { MP_ROM_QSTR(MP_QSTR_parse), MP_ROM_PTR(&parse_nmea_string_obj) },
    { MP_ROM_QSTR(MP_QSTR_calculate_distance), MP_ROM_PTR(&calculate_distance_obj) },
    { MP_ROM_QSTR(MP_QSTR_reset), MP_ROM_PTR(&reset_gps_data_obj) },
    { MP_ROM_QSTR(MP_QSTR_current), MP_ROM_PTR(&get_current_obj) },
};

static MP_DEFINE_CONST_DICT(ublox_nmea_globals, ublox_nmea_globals_table);

// Определение модуля
const mp_obj_module_t ublox_nmea_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t*)&ublox_nmea_globals,
};

// Регистрация модуля
MP_REGISTER_MODULE(MP_QSTR_ublox_nmea, ublox_nmea_module);