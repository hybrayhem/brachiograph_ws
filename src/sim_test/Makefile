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

# Build targets
all: $(TARGETS)

$(TARGETS): %: %.o
	$(CXX) $(CXXFLAGS) -o $@ $<

# Compile source files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean
clean:
	rm -f $(OBJS) $(TARGETS)
