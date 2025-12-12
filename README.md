# Torom

This is the Torom project.

## Project Summary

This project is a single motor controller designed for precise control of a stepper motor. It is implemented using the Arduino framework and includes the following features:

### Features
- **Motor Control**: Supports movement to specific positions, relative movements, and sequences like blowout and tip dispense.
- **Calibration**: Includes a calibration routine to set the home position using a limit switch.
- **Command Processing**: Processes commands via serial communication to control motor actions.
- **JSON Reporting**: Outputs status and error messages in JSON format for easy integration.

### Key Components
- **StepperMotor Class**: Encapsulates motor control logic, including movement, calibration, and status reporting.
- **TMC2209 Driver**: Configured for motor control with adjustable current and microsteps.
- **Serial Communication**: Handles commands via USB and software serial interfaces.

### Commands
The project supports various commands for motor control, including:
- `c`: Calibrate the motor.
- `u<val>`: Aspirate a specific volume.
- `b`: Perform a blowout sequence.
- `k`: Perform a tip dispense sequence.
- `a<val>`: Set acceleration.
- `x<val>`: Set maximum speed.
- `s`: Stop the motor.
- `e`: Emergency stop.
- `p`: Display the current position.
- `?`: Display help information.

### Setup and Loop
- **Setup**: Initializes serial communication, the motor driver, and the stepper motor.
- **Loop**: Continuously updates the motor state, reads commands, and processes them.