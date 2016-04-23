# Makefile taken from the dRonin project
.DEFAULT_GOAL := all

WHEREAMI := $(dir $(lastword $(MAKEFILE_LIST)))
export ROOT_DIR := $(realpath $(WHEREAMI)/ )
export BUILD_ALL_DEPENDENCIES := $(BUILD_ALL_DEPENDENCIES)
# import macros common to all supported build systems
include $(ROOT_DIR)/make/system-id.mk

# configure some directories that are relative to wherever ROOT_DIR is located
TOOLS_DIR := $(ROOT_DIR)/tools
BUILD_DIR := $(ROOT_DIR)/build
DL_DIR := $(ROOT_DIR)/downloads

SIZE_LOG := sizes.log

export RM := rm

# import macros that are OS specific
#include $(ROOT_DIR)/make/$(OSFAMILY).mk

# include the tools makefile
include $(ROOT_DIR)/make/tools.mk

# Decide on a verbosity level based on the V= parameter
export AT := @

export VERBOSE_FLAG :=

ifndef V
export V0    :=
export V1    := $(AT)
else ifeq ($(V), 0)
export V0    := $(AT)
export V1    := $(AT)
else ifeq ($(V), 1)
export VERBOSE_FLAG := -verbose
endif

$(DL_DIR):
	mkdir -p $@

$(TOOLS_DIR):
	mkdir -p $@

$(BUILD_DIR):
	mkdir -p $@

# instructions
.PHONY: help
help:
	@echo
	@echo "   This Makefile is tested on macosx by @nathantsoi, please updated this message if you're using another platform"
	@echo "   Shamelessly copied from the dRonin project. Mad cred to them."
	@echo
	@echo "   Here is a summary of the available targets:"
	@echo
	@echo "   [Tool Installers]"
	@echo "     arduino_builder_install      - Install the arduino_builder toolchain"
	@echo "     uncrustify_install           - Install the uncrustify code formatter"
	@echo
	@echo "   [Firmware]"
	@echo "     arduino                      - Build firmware for the arduino based MinimOsd"
	@echo
	@echo "   Hint: Add V=1 to your command line to see verbose build output."
	@echo
	@echo "   Note: All tools will be installed into $(TOOLS_DIR)"
	@echo "         All build output will be placed in $(BUILD_DIR)"
	@echo


SRC_DIR := $(ROOT_DIR)/MW_OSD

# ARDUINO CONFIG
MAIN_INO := MW_OSD.ino
ARDUINO_DEVICE_TARGET := arduino:avr:pro:cpu=16MHzatmega328

arduino: $(BUILD_DIR)
arduino:
	$(V1) $(ARDUINO_BUILDER_BIN) \
		-hardware $(ARDUINO_APP_RESOURCE_DIR)/hardware \
		-tools $(ARDUINO_APP_RESOURCE_DIR)/hardware/tools \
		-tools $(ARDUINO_BUILDER_DIR)/tools \
		-libraries $(SRC_DIR) \
		-fqbn=$(ARDUINO_DEVICE_TARGET) \
		-build-path $(BUILD_DIR) \
		$(VERBOSE_FLAG) \
		-compile \
	  $(SRC_DIR)/$(MAIN_INO)

.PHONY: arduino-size
arduino-size:
	$(V1) SIZE=$(ARDUINO_TOOLS_BIN)/avr-size $(BUILD_DIR)/$(MAIN_INO).elf | tee -a $(SIZE_LOG)

all: arduino arduino-size

clean:
	$(V1) $(RM) -rf $(BUILD_DIR)/*


# uncrustify
ifneq ($(strip $(filter uncrustify,$(MAKECMDGOALS))),)
  ifeq ($(FILE),)
    $(error pass files to uncrustify by adding FILE=<file> to the make command line)
  endif
endif

.PHONY: uncrustify
uncrustify: UNCRUSTIFY_OPTIONS := -c make/uncrustify.cfg --replace
uncrustify:
	$(V1) $(UNCRUSTIFY) $(UNCRUSTIFY_OPTIONS) $(SRC_DIR)/$(FILE)
