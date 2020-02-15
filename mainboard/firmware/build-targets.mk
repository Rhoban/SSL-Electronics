# Project targets
# Defines here your cpp source files
# Ex : main.cpp test.cpp ...

ifeq ($(COMPILE_MASTER),yes)
SRC_FILES = main_master.cpp com_master.cpp com.cpp buzzer.cpp \
            mux.cpp infos.cpp hardware.cpp assert.cpp com_proto.cpp
CFLAGS+= -DMASTER_FIRMWARE
else
SRC_FILES = main_robot.cpp com_robot.cpp drivers.cpp com.cpp buzzer.cpp hardware.cpp \
        kicker.cpp voltage.cpp ir.cpp kinematic.cpp mux.cpp infos.cpp odometry.cpp assert.cpp\
        com_proto.cpp
CFLAGS+= -DROBOT_FIRMWARE
endif

ifeq ($(ENABLE_RHOCK),yes)
SRC_FILES += rhock-functions.cpp rhock-stream.cpp
endif

# Uncomment to disable robot campus commands
CFLAGS += -DHAS_TERMINAL -DDISABLE_SERVOS_COMMANDS
# CFLAGS += -DDXL_VERSION_1

OBJ_FILES_CPP = $(SRC_FILES:.cpp=.o)
OBJ_FILES = $(addprefix $(BUILD_PATH)/,$(OBJ_FILES_CPP:.c=.o))


$(BUILD_PATH)/%.o: %.cpp
	$(CXX) $(CFLAGS) $(CXXFLAGS) $(LIBMAPLE_INCLUDES) $(WIRISH_INCLUDES) -o $@ -c $<

$(BUILD_PATH)/libmaple.a: $(BUILDDIRS) $(TGT_BIN)
	- rm -f $@
	$(AR) crv $(BUILD_PATH)/libmaple.a $(TGT_BIN)

library: $(BUILD_PATH)/libmaple.a

.PHONY: library

$(BUILD_PATH)/$(BOARD).elf: $(BUILDDIRS) $(TGT_BIN) $(OBJ_FILES)
	$(SILENT_LD) $(CXX) $(LDFLAGS) -o $@ $(TGT_BIN) $(OBJ_FILES) -Wl,-Map,$(BUILD_PATH)/$(BOARD).map

$(BUILD_PATH)/$(BOARD).bin: $(BUILD_PATH)/$(BOARD).elf
	$(SILENT_OBJCOPY) $(OBJCOPY) -v -Obinary $(BUILD_PATH)/$(BOARD).elf $@ 1>/dev/null
	$(SILENT_DISAS) $(DISAS) -d $(BUILD_PATH)/$(BOARD).elf > $(BUILD_PATH)/$(BOARD).disas
	@echo " "
	@echo "Object file sizes:"
	@find $(BUILD_PATH) -iname "*.o" | xargs $(SIZE) -t > $(BUILD_PATH)/$(BOARD).sizes
	@cat $(BUILD_PATH)/$(BOARD).sizes
	@echo " "
	@echo "Final Size:"
	@$(SIZE) $<
	@echo $(MEMORY_TARGET) > $(BUILD_PATH)/build-type

$(BUILDDIRS):
	@mkdir -p $@

MSG_INFO:
	@echo "================================================================================"
	@echo ""
	@echo "  Build info:"
	@echo "     BOARD:          " $(BOARD)
	@echo "     MCU:            " $(MCU)
	@echo "     MEMORY_TARGET:  " $(MEMORY_TARGET)
	@echo ""
	@echo "  See 'make help' for all possible targets"
	@echo ""
	@echo "================================================================================"
	@echo ""
