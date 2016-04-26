# Set up Arduino builder SDK
ARDUINO_BUILDER_VERSION := 1.3.13
ARDUINO_BUILDER_DIR := $(TOOLS_DIR)/arduino-builder-$(ARDUINO_BUILDER_VERSION)
ARDUINO_BUILDER_BIN := $(TOOLS_DIR)/arduino-builder-$(ARDUINO_BUILDER_VERSION)/arduino-builder

# Arduino builder depends on files found in the Arduino app download
ARDUINO_APP_VERSION := 1.6.8
ARDUINO_APP_DIR:= $(TOOLS_DIR)/arduino-app-$(ARDUINO_APP_VERSION)

ARDUINO_TOOLS_BIN =: $(ARDUINO_APP_RESOURCE_DIR)/hardware/tools/avr/bin

# from: https://github.com/arduino/arduino-builder/releases
ifdef LINUX
	# or linux64
	ARDUINO_BUILDER_URL := https://github.com/arduino/arduino-builder/releases/download/$(ARDUINO_BUILDER_VERSION)/arduino-builder-$(OSFAMILY)32-$(ARDUINO_BUILDER_VERSION).tar.bz2
	ARDUINO_APP_URL := https://downloads.arduino.cc/arduino-$(ARDUINO_APP_VERSION)-$(OSFAMILY).tr.xz
	$(error linux platform support: please set ARDUINO_APP_RESOURCE_DIR in make/tooks.mk to the directory containing the arduino hardware folder and remove this error message. see MACOSX section for an example)
endif
ifdef MACOSX
	ARDUINO_BUILDER_URL := https://github.com/arduino/arduino-builder/releases/download/$(ARDUINO_BUILDER_VERSION)/arduino-builder-$(OSFAMILY)-$(ARDUINO_BUILDER_VERSION).tar.bz2
	ARDUINO_APP_URL := https://downloads.arduino.cc/arduino-$(ARDUINO_APP_VERSION)-$(OSFAMILY).zip
	ARDUINO_APP_RESOURCE_DIR := $(ARDUINO_APP_DIR)/Arduino.app/Contents/Java/
endif
ifdef WINDOWS
	ARDUINO_BUILDER_URL := https://github.com/arduino/arduino-builder/releases/download/$(ARDUINO_BUILDER_VERSION)/arduino-builder-$(OSFAMILY)-$(ARDUINO_BUILDER_VERSION).zip
	ARDUINO_APP_URL := https://downloads.arduino.cc/arduino-$(ARDUINO_APP_VERSION)-$(OSFAMILY).zip
	ARDUINO_APP_RESOURCE_DIR := $(ARDUINO_APP_DIR)/arduino-$(ARDUINO_APP_VERSION)/
endif

.PHONY: arduino_app_install

arduino_app_install: ARDUINO_APP_FILE := $(notdir $(ARDUINO_APP_URL))
# order-only prereq on directory existance:
arduino_app_install: | $(DL_DIR) $(TOOLS_DIR)
arduino_app_install: arduino_app_clean
ifneq ($(OSFAMILY), linux)
  # download the source only if it's newer than what we already have
ifneq ($(OSFAMILY), windows)
	$(V1) wget --no-check-certificate -N -P "$(DL_DIR)" "$(ARDUINO_APP_URL)"
else
	$(V1) curl -L -k -o "$(DL_DIR)/$(ARDUINO_APP_FILE)" "$(ARDUINO_APP_URL)"
endif
	$(V1) unzip -q -d $(ARDUINO_APP_DIR) "$(DL_DIR)/$(ARDUINO_APP_FILE)"
else
	$(V1) curl -L -k -o "$(DL_DIR)/$(ARDUINO_APP_FILE)" "$(ARDUINO_APP_URL)"
	$(V1) tar -C $(TOOLS_DIR) -xf "$(DL_DIR)/$(ARDUINO_APP_FILE)"
endif

.PHONY: arduino_app_clean
arduino_app_clean:
	$(V1) [ ! -d "$(ARDUINO_APP_DIR)" ] || $(RM) -r $(ARDUINO_APP_DIR)



.PHONY: arduino_builder_install

arduino_builder_install: arduino_app_install
arduino_builder_install: ARDUINO_BUILDER_FILE := $(notdir $(ARDUINO_BUILDER_URL))
# order-only prereq on directory existance:
arduino_builder_install: | $(DL_DIR) $(TOOLS_DIR)
arduino_builder_install: arduino_builder_clean
ifneq ($(OSFAMILY), windows)
	$(V1) wget --no-check-certificate -N -P "$(DL_DIR)" "$(ARDUINO_BUILDER_URL)"
	$(V1) mkdir -p $(ARDUINO_BUILDER_DIR)
	$(V1) tar -C $(ARDUINO_BUILDER_DIR) -xjf "$(DL_DIR)/$(ARDUINO_BUILDER_FILE)"
else
	$(V1) curl -L -k -o "$(DL_DIR)/$(ARDUINO_BUILDER_FILE)" "$(ARDUINO_BUILDER_URL)"
	$(V1) unzip -q -d $(ARDUINO_BUILDER_DIR) "$(DL_DIR)/$(ARDUINO_BUILDER_FILE)"
endif

.PHONY: arduino_builder_clean
arduino_builder_clean:
	$(V1) [ ! -d "$(ARDUINO_BUILDER_DIR)" ] || $(RM) -r $(ARDUINO_BUILDER_DIR)

# make sure we have the right dependencies installed
ifneq ($(MAKECMDGOALS), arduino_app_install)
ifeq ($(wildcard $(ARDUINO_APP_RESOURCE_DIR)),)
  $(error please run: `make arduino_app_install`, missing "$(ARDUINO_APP_RESOURCE_DIR)", install the arduino app
endif
endif
ifneq ($(MAKECMDGOALS), arduino_builder_install)
ifeq ($(wildcard $(ARDUINO_BUILDER_DIR)),)
  $(error please run: `make arduino_builder_install`, missing "$(ARDUINO_BUILDER_DIR)", install arduino builder)
endif
endif


# Set up uncrustify tools
UNCRUSTIFY_DIR := $(TOOLS_DIR)/uncrustify-0.61
UNCRUSTIFY_BUILD_DIR := $(DL_DIR)/uncrustify

.PHONY: uncrustify_install
uncrustify_install: | $(DL_DIR) $(TOOLS_DIR)
uncrustify_install: UNCRUSTIFY_URL := http://downloads.sourceforge.net/project/uncrustify/uncrustify/uncrustify-0.61/uncrustify-0.61.tar.gz
uncrustify_install: UNCRUSTIFY_FILE := uncrustify-0.61.tar.gz
uncrustify_install: UNCRUSTIFY_OPTIONS := prefix=$(UNCRUSTIFY_DIR)
uncrustify_install: uncrustify_clean
ifneq ($(OSFAMILY), windows)
	$(V0) @echo " DOWNLOAD     $(UNCRUSTIFY_URL)"
	$(V1) wget --no-check-certificate -N -P "$(DL_DIR)" "$(UNCRUSTIFY_URL)"
else
	$(V1) curl -L -k -o "$(DL_DIR)/$(UNCRUSTIFY_FILE)" "$(UNCRUSTIFY_URL)"
endif
        # extract the src
	$(V0) @echo " EXTRACT      $(UNCRUSTIFY_FILE)"
	$(V1) tar -C $(TOOLS_DIR) -xf "$(DL_DIR)/$(UNCRUSTIFY_FILE)"

	$(V0) @echo " BUILD        $(UNCRUSTIFY_DIR)"
	$(V1) ( \
	  cd $(UNCRUSTIFY_DIR) ; \
	  ./configure --prefix="$(UNCRUSTIFY_DIR)" ; \
	  $(MAKE) ; \
	  $(MAKE) install ; \
	)
	      # delete the extracted source when we're done
	$(V1) [ ! -d "$(UNCRUSTIFY_BUILD_DIR)" ] || $(RM) -r "$(UNCRUSTIFY_BUILD_DIR)"

.PHONY: uncrustify_clean
uncrustify_clean:
	$(V0) @echo " CLEAN        $(UNCRUSTIFY_DIR)"
	$(V1) [ ! -d "$(UNCRUSTIFY_DIR)" ] || $(RM) -r "$(UNCRUSTIFY_DIR)"
	$(V0) @echo " CLEAN        $(UNCRUSTIFY_BUILD_DIR)"
	$(V1) [ ! -d "$(UNCRUSTIFY_BUILD_DIR)" ] || $(RM) -r "$(UNCRUSTIFY_BUILD_DIR)"

ifeq ($(shell [ -d "$(UNCRUSTIFY_DIR)" ] && echo "exists"), exists)
  UNCRUSTIFY := $(UNCRUSTIFY_DIR)/bin/uncrustify
else
  # not installed, hope it's in the path...
  UNCRUSTIFY ?= uncrustify
endif

