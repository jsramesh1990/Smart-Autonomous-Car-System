#!/bin/bash
# Flash script for Smart Autonomous Car

echo "Flashing Smart Autonomous Car to microcontroller..."

# Check if hex file exists
if [ ! -f "smart_autonomous_car.hex" ]; then
    echo "Error: HEX file not found. Run build first."
    exit 1
fi

# Flash using avrdude
make flash

if [ $? -eq 0 ]; then
    echo "Flash successful!"
else
    echo "Flash failed!"
    exit 1
fi
