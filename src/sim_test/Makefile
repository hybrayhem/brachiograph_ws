# Compiler
CXX = g++

# Compiler flags
CXXFLAGS = -std=c++11 -Wall

# Source files
SRCS = joint_position_sender.cpp joint_position_receiver.cpp

# Object files
OBJS = $(SRCS:.cpp=.o)

# Executables
TARGETS = $(SRCS:.cpp=)

# # Executable
# FIRST_SRC := $(firstword $(wildcard $(SRCS))) # first item of SRCS
# TARGET := $(basename $(FIRST_SRC)) # remove .cpp extension

# Build directory
BUILD_DIR = build

# Build targets
all: $(BUILD_DIR) $(TARGETS)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(TARGETS): %: $(BUILD_DIR)/%.o
	$(CXX) $(CXXFLAGS) -o $(BUILD_DIR)/$@ $<

# Compile source files
$(BUILD_DIR)/%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean
clean:
	rm -rf $(BUILD_DIR) $(OBJS) $(TARGETS)
