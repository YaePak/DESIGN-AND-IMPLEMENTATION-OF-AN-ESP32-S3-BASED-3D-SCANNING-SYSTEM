# Weekly Progress Report — Week 4

## 1. Work Completed

- Following the component tests and the overall system test from Week 3, ran the integrated prototype under load and found that the hardware works but is not yet stable.
- Identified the power supply as the cause: the 12 V / 1 A supply cannot provide the current the system needs.
- Analyzed the problem by estimating the current demand of each load and comparing it with the supply rating.
- Used the issues found (the insufficient supply and the voltage sag under load) to complete and verify the calculations against real operating conditions, and on that basis completed the Definitions_and_Theory section.

## 2. Issues Encountered

- The prototype was powered by a 12 V, 1 A supply, while the estimated maximum current demand is approximately 5 A. This estimate is the sum of the maximum rated values of the loads:
  - Two stepper motors, each up to 1.8 A (maximum value published by the manufacturer).
  - The ESP32-S3, about 1 A in total: 600–800 mA when Wi-Fi is used, plus a few tens of mA for the GPIO pins in use.
  - The TF-Luna distance sensor.
- At the start the turntable rotated, but it moved slowly and with difficulty.
- The supply was then overloaded. Its output sagged and was clamped at 4–6 V instead of holding the nominal 12 V.
- The low supply voltage caused the system to operate unstably under load.

## 3. Current Work and Next Steps

- Measuring the input voltage and current at the supply output and at the motor driver supply pins, to separate supply overload from voltage drop in the wiring.
- Selecting a power supply with enough current capacity for the estimated peak demand, with headroom for start-up and acceleration peaks.
- Checking the wiring, power distribution, and the motor driver current settings.
- Repeating the tests after the power supply is improved, to verify motor movement and overall system stability.

## 4. Overall Progress

The components and the overall system were tested in Week 3. This week the cause of the instability was found: the 12 V / 1 A supply cannot support a peak demand of approximately 5 A, and its output sags to 4–6 V under load. The Definitions_and_Theory section has been completed using the conditions observed. The main priority is to improve the power supply and verify the power distribution before moving on to full scanning tests.