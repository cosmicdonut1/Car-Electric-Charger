# Project Title

ChargeMonitor

## Project Description

ChargeMonitor is a desktop application to monitor electrical parameters like voltage, current, and power through a serial connection. This project utilizes PyQt6 for the graphical interface and handles serial communication to collect and display real-time data on the main window.

## Content

- **AverageFilter**: Implements a simple averaging filter.
- **MedianFilter**: Implements a median filter to smooth the data.
- **EnergyMeter**: Computes the energy consumed (in watt-hours).
- **SerialThread**: Handles serial communication in a separate thread.
- **NeChargeMonitor**: Main window class that sets up the GUI and updates the displayed values with real-time data from the serial port.

## Requirements

- Python 3.x
- PyQt6
- pyserial

## Contributors

Copying and usage in any form are prohibited.
