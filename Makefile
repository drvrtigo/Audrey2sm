# Project Name
TARGET = Audrey2sm

# Sources
CPP_SOURCES = Audrey2sm.cpp \
              src/FeedbackSynthEngine.cpp \
              src/BiquadFilters.cpp \
              src/KarplusString.cpp \
			  src/PitchCalibration.cpp \
			  src/memory/sdram_alloc.cpp


# Explicit LGPL usage for DaisySP, which is required for the reverb code.
USE_DAISYSP_LGPL=1

# Library Locations
LIBDAISY_DIR = lib/libDaisy/
DAISYSP_DIR = lib/DaisySP/

# Core location, and generic Makefile.
SYSTEM_FILES_DIR = $(LIBDAISY_DIR)/core
include $(SYSTEM_FILES_DIR)/Makefile
