# Compiler to use
CXX = g++

# Compiler flags
CXXFLAGS = -std=c++17 -Wall -O2

# Eigen include path (adjust based on architecture)
# Apple Silicon (arm64)
EIGEN_PATH = /opt/homebrew/include/eigen3
# Uncomment the following line for Intel Macs instead
# EIGEN_PATH = /usr/local/include/eigen3

# Include directories
INCLUDES = -I$(EIGEN_PATH) -I.

# Target executable name
TARGET = balloon_ekf

# Source files
SOURCES =  test_balloonEKF.cpp
#SOURCES = BalloonEKF.h test_balloonEKF.cpp

test_balloonEKF.cpp: BalloonEKF.h

# Object files
OBJECTS = $(SOURCES:.cpp=.o)

# Default target
all: $(TARGET)

# Link object files to create executable
$(TARGET): $(OBJECTS)
	$(CXX) $(OBJECTS) -o $(TARGET)

# Compile source files to object files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) $(INCLUDES) -c $< -o $@

# Clean up
clean:
	rm -f $(OBJECTS) $(TARGET)

# Phony targets
.PHONY: all clean