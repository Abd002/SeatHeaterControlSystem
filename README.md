# Seat Heater Control System

This project implements a real-time seat heater control system for the front two seats of a car using FreeRTOS and a Tiva C microcontroller. The system controls the seat temperature based on user input, provides diagnostics, and displays system data on a shared screen via UART.

## Features

- **Temperature Control**: Supports three heating levels (Low, Medium, High) and an Off state. The heating intensity adjusts based on the difference between the current and desired temperatures.
- **Diagnostics**: Detects temperature sensor failures and logs them in memory. A red LED indicates sensor issues.
- **User Interface**: A shared screen displays the current temperature, heating level, and heater status.
- **Event-based Architecture**: Button presses trigger corresponding heating events, and tasks manage the heating levels accordingly.
- **Real-time Measurements**: The system monitors CPU load, task execution time, and resource lock time using FreeRTOS and the GPTM module.

## Hardware Components

Each seat (driver and passenger) includes:
- A temperature sensor (simulated as a potentiometer).
- LEDs to simulate the heater intensity (green for low, blue for medium, cyan for high).
- A button to control the heating level.

Additional components:
- A red LED for error reporting.
- An extra button on the steering wheel to control the driver's seat heater.

## Task Architecture

The system is designed with the following tasks:

1. **PassengerButtonHandler**
   - **Description**: Handles the passenger button press and sets corresponding event bits.
   - **Type**: Periodic
   - **Periodicity**: 100ms
   - **Events Set**: `PASSENGER_EVENT_BIT_BUTTON_PRESSED_LOW`, `PASSENGER_EVENT_BIT_BUTTON_PRESSED_MEDIUM`, `PASSENGER_EVENT_BIT_BUTTON_PRESSED_HIGH`

2. **DriverButtonHandler**
   - **Description**: Handles the driver button press and sets corresponding event bits.
   - **Type**: Periodic
   - **Periodicity**: 100ms
   - **Events Set**: `DRIVER_EVENT_BIT_BUTTON_PRESSED_LOW`, `DRIVER_EVENT_BIT_BUTTON_PRESSED_MEDIUM`, `DRIVER_EVENT_BIT_BUTTON_PRESSED_HIGH`

3. **Temperature_Task**
   - **Description**: Monitors seat temperatures and checks for out-of-range values.
   - **Type**: Periodic
   - **Periodicity**: 100ms

4. **Heating_level_LOW**
   - **Description**: Manages heating at the low level and adjusts LED indicators.
   - **Type**: Event-based
   - **Events Waited**: `DRIVER_EVENT_BIT_BUTTON_PRESSED_LOW`, `PASSENGER_EVENT_BIT_BUTTON_PRESSED_LOW`

5. **Heating_level_MEDIUM**
   - **Description**: Manages heating at the medium level and adjusts LED indicators.
   - **Type**: Event-based
   - **Events Waited**: `DRIVER_EVENT_BIT_BUTTON_PRESSED_MEDIUM`, `PASSENGER_EVENT_BIT_BUTTON_PRESSED_MEDIUM`

6. **Heating_level_HIGH**
   - **Description**: Manages heating at the high level and adjusts LED indicators.
   - **Type**: Event-based
   - **Events Waited**: `DRIVER_EVENT_BIT_BUTTON_PRESSED_HIGH`, `PASSENGER_EVENT_BIT_BUTTON_PRESSED_HIGH`

7. **vRunTimeMeasurementsTask**
   - **Description**: Periodically calculates CPU load and outputs it via UART.
   - **Type**: Periodic
   - **Periodicity**: 500ms

## Shared Resources

- **UART**: Shared by multiple tasks, access protected by a mutex to prevent data corruption.
- **Event Group**: Used to synchronize button press events and heating level tasks, managed using FreeRTOS APIs.
- **Temperature Sensors**: Read by the `Temperature_Task`, with sequential access by other tasks dependent on the latest readings.

## Project Structure

- `src/`: Contains all source code for the application tasks, FreeRTOS configuration, and MCAL modules (GPIO, UART, GPTM, ADC).
- `simso/`: Contains the Simso simulation project for deadline analysis.
- `docs/`: Contains project documentation, including task design, resource sharing, system output screenshots, and runtime measurement results.

## Setup and Usage

1. Clone the repository:
   ```bash
   git clone https://github.com/yourusername/SeatHeaterControlSystem.git
