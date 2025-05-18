#  Implementation and Evaluation of Advanced Motor Control Algorithms: FOC & DTC

This project presents the implementation and performance evaluation of advanced motor control algorithms — Field-Oriented Control (FOC) and Direct Torque Control (DTC) — using STM32 microcontrollers for BLDC motors.

##  Objective

- Achieve accurate torque and speed control in BLDC motors.
- Implement Clarke and Park transformations for efficient control.
- Evaluate performance through MATLAB simulations and hardware testing.

## Algorithms Implemented

- **FOC (Field-Oriented Control)**:
  - Clarke and Park Transformations
  - d-q axis decoupling
  - PID Control
  - Space Vector Modulation (SVM)

- **DTC (Direct Torque Control)**:
  - Torque and flux estimation
  - Hysteresis controllers
  - Optimal voltage vector selection

##  Tools and Technologies

- **Microcontroller**: STM32 (ARM Cortex-M)
- **Software**: 
  - STM32CubeMX + HAL
  - Keil uVision / Arduino IDE
  - MATLAB for simulation
- **Hardware**: 
  - BLDC motor
  - Inverter
  - Rotary encoder
  - Current sensors

##  Results

- FOC offered better **torque ripple reduction** and **efficiency** than traditional methods.
- Achieved <2% steady-state error in speed tracking.
- STM32 handled real-time control with no extra hardware modules.
- MATLAB waveforms validated speed, torque, and PWM performance.

## Future Scope

- Add predictive/adaptive control for dynamic loads
- Sensorless FOC using estimators
- IoT-based monitoring & diagnostics
- Expansion to industrial-grade high-power motors


