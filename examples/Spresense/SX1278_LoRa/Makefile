-include $(TOPDIR)/Make.defs

# Application name
APPNAME = sx1278_lora

# Application source files
CSRCS = main.c

# Include RadioLib source files
CSRCS += $(wildcard $(TOPDIR)/src/*.cpp)
CSRCS += $(wildcard $(TOPDIR)/src/modules/*.cpp)
CSRCS += $(wildcard $(TOPDIR)/src/protocols/*.cpp)
CSRCS += $(wildcard $(TOPDIR)/src/hal/Spresense/*.cpp)

# Include paths
CFLAGS += -I$(TOPDIR)/src
CFLAGS += -I$(TOPDIR)/src/modules
CFLAGS += -I$(TOPDIR)/src/protocols
CFLAGS += -I$(TOPDIR)/src/hal/Spresense

# Link with C++ standard library
LDFLAGS += -lstdc++

# Build the application
include $(APPDIR)/Application.mk 