#!/bin/bash
# Build script for Smart Autonomous Car

echo "Building Smart Autonomous Car..."

# Create build directory
mkdir -p build

# Run make
make all

if [ $? -eq 0 ]; then
    echo "Build successful!"
    echo "Output files:"
    ls -la *.elf *.hex
else
    echo "Build failed!"
    exit 1
fi
