# Compiler and Flags
CXX      := g++
CXXFLAGS := $(shell pkg-config --cflags ibex) -frounding-math -ffloat-store
LIBS     := $(shell pkg-config --libs ibex) -libex -lprim -lz -lm -std=c++14

# Debug Mode
ifeq ($(DEBUG), yes)
    CXXFLAGS += -O0 -g -pg -Wall -std=c++14
else
    CXXFLAGS += -O2 -DNDEBUG -Wno-deprecated -Wno-logical-op-parentheses -g -std=c++14
endif

# Source and Header Files
SRCS     := simulation.cpp C_STL.cpp
BINS     := simulation.out

# Default Target
all: $(BINS)

# Rule for building the executable
$(BINS): $(SRCS)
	$(CXX) $(CXXFLAGS) -o $@ $(SRCS) $(LIBS)

# Clean Up
clean:
	rm -f $(BINS)

