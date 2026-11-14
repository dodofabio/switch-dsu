ifeq ($(strip $(DEVKITPRO)),)
$(error "DEVKITPRO n'est pas defini. Installe devkitPro puis : export DEVKITPRO=/opt/devkitpro")
endif

include $(DEVKITPRO)/libnx/switch_rules

TARGET		:= switch-dsu
APP_TITLE	:= Switch GamePad
APP_AUTHOR	:= dodofabio
APP_VERSION	:= 1.0.0

ARCH		:= -march=armv8-a+crc+crypto -mtune=cortex-a57 -mtp=soft -fPIE
CFLAGS		:= -g -Wall -O2 -ffunction-sections $(ARCH) -D__SWITCH__ -I$(DEVKITPRO)/libnx/include -I$(DEVKITPRO)/portlibs/switch/include
LDFLAGS		:= -specs=$(DEVKITPRO)/libnx/switch.specs -g $(ARCH) -L$(DEVKITPRO)/portlibs/switch/lib -L$(DEVKITPRO)/libnx/lib
LIBS		:= -lturbojpeg -lm -lnx

TOOLS		:= $(DEVKITPRO)/tools/bin
OFILES		:= build/main.o

.PHONY: all clean

all: $(TARGET).nro

build/%.o: source/%.c
	@mkdir -p build
	$(CC) $(CFLAGS) -c $< -o $@

$(TARGET).elf: $(OFILES)
	$(CC) $(LDFLAGS) $(OFILES) $(LIBS) -o $@

$(TARGET).nacp:
	$(TOOLS)/nacptool --create "$(APP_TITLE)" "$(APP_AUTHOR)" "$(APP_VERSION)" $@

$(TARGET).nro: $(TARGET).elf $(TARGET).nacp
	$(TOOLS)/elf2nro $< $@ --nacp=$(TARGET).nacp

clean:
	rm -rf build $(TARGET).elf $(TARGET).nacp $(TARGET).nro
