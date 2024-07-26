# Compiler settings - use g++ for C++ code
CXX = g++
CXXFLAGS = -Wall -Wextra -O2 

# Linker settings
LDFLAGS = -Linclude 

# Include paths
INCLUDE = -Iinclude

# Source files
SOURCES = simple_logger.cpp pwm.cpp main.cpp

# Object files (change extension to .o)
OBJECTS = $(SOURCES:.cpp=.o)

# Executable name
EXEC = main

# Default build
all: $(EXEC)

# Rule to link the executable
$(EXEC): $(OBJECTS)
	$(CXX) $(OBJECTS) -o $@
	
# Compile step
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDE) -c $< -o $@ 

# Clean up
clean:
	rm -f $(OBJECTS) $(EXEC)

run:
	sudo ./$(EXEC)

# Phony targets
.PHONY: all clean