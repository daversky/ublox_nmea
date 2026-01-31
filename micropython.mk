# micropython.mk
# Ublox NMEA module for MicroPython
SRC_USERMOD += $(USERMOD_DIR)/ublox_nmea.c

CFLAGS_USERMOD += -I$(USERMOD_DIR)
CFLAGS_USERMOD += -Wno-unused-variable -Wno-unused-function -Wno-unused-parameter