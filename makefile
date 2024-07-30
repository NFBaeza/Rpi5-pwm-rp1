# VARIABLES
BUILDDIR = build

# COMPILER CONFIG
CXX = g++
CXXFLAGS = -pthread -lm

OBJ_DIR = pwm_module
OBJ_INCLUDES = ${OBJ_DIR}/include
INCLUDES = -I${OBJ_INCLUDES} -I.

# RULES
DEPS = $(wildcard $(OBJ_DIR)/*.h)
SOURCES = $(wildcard $(OBJ_DIR)/*.cpp)
OBJECTS = $(SOURCES:$(OBJ_DIR)/%.cpp=$(BUILDDIR)/%.o)

# Regla para compilar el ejecutable
$(BUILDDIR)/main.out: $(OBJECTS) main.cpp
	@mkdir -p $(BUILDDIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -o $@ $^

# Regla para compilar los archivos fuente a objetos
$(BUILDDIR)/%.o: $(OBJ_DIR)/%.cpp
	@mkdir -p $(BUILDDIR)
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Regla para compilar el proyecto principal
main: $(BUILDDIR)/main.out
	mv $(BUILDDIR)/main.out $(BUILDDIR)

clean:
	rm -rf $(BUILDDIR)

run:
	sudo $(BUILDDIR)/*.out

$(BUILDDIR):
	mkdir $@

help:
	@echo "make: compile code which send timestamp from Pico (Required Pico has connected)"
	@echo "make clean: clean directories and previous compilations"
	@echo "make help: shows this guide"

.PHONY: main clean run help

.SECONDARY:

