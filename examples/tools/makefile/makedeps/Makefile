ROOT_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
SRC_FOLDER    = $(ROOT_DIR)/src
BUILD_FOLDER  = $(ROOT_DIR)/build

include $(BUILD_FOLDER)/conandeps.mk

CXXFLAGS      += $(CONAN_CXXFLAGS) -std=c++11
CPPFLAGS      += $(addprefix -I, $(CONAN_INCLUDE_DIRS))
CPPFLAGS      += $(addprefix -D, $(CONAN_DEFINES))
LDFLAGS       += $(addprefix -L, $(CONAN_LIB_DIRS))
LDLIBS        += $(addprefix -l, $(CONAN_LIBS))
EXELINKFLAGS  += $(CONAN_EXELINKFLAGS)


all:
	$(CXX) $(SRC_FOLDER)/main.cpp $(CPPFLAGS) $(CXXFLAGS) $(LDFLAGS) $(LDLIBS) $(EXELINKFLAGS) -o $(BUILD_FOLDER)/string_digest_hex
