# Compiler settings - use g++ for C++ code
CXX = g++
CXXFLAGS = -Wall -Wextra -O2 

# Linker settings
LDFLAGS =

# Source files
SOURCES = main.cpp

# Object files (change extension to .o)
OBJECTS = $(SOURCES:.cpp=.o)

# Executable name
EXEC = main

# Default build
all: $(EXEC)

# Rule to link the executable
$(EXEC): $(OBJECTS)
	$(CXX) $(LDFLAGS) -o $@ $^ -lgpiod

# Compile step
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@ 

# Clean up
clean:
	rm -f $(OBJECTS) $(EXEC)

run:
	sudo ./$(EXEC)

# Phony targets
.PHONY: all clean
