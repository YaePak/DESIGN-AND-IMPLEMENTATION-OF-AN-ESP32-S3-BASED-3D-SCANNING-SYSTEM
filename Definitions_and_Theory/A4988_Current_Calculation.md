# A4988 Current Calculation

## 1. A4988 VREF and Current Limit Calculation

The A4988 stepper motor driver limits the current through the motor windings. The current limit is adjusted using the reference voltage $V_{REF}$.

### 1.1. Calculation Formula

The current limit is:

$$I_{LIMIT} = \frac{V_{REF}}{8 \times R_{CS}}$$

Rearranging gives:

$$V_{REF} = 8 \times I_{LIMIT} \times R_{CS}$$

Where:

- $I_{LIMIT}$: configured current limit (A).
- $V_{REF}$: reference voltage measured at the driver (V).
- $R_{CS}$: current-sense resistor on the A4988 module (Ω).

The value of $R_{CS}$ depends on the board. Common values are 0.05 Ω (marking R050), 0.068 Ω (R068) and 0.1 Ω (R100). The actual resistor value must be verified before calculating $V_{REF}$. [1], [2]

### 1.2. Example Calculation

Assume the motor's rated phase current is 1.0 A.

For $R_{CS} = 0.068$ Ω:

$$V_{REF} = 8 \times 1.0 \times 0.068 = 0.544 \text{ V}$$

For $R_{CS} = 0.05$ Ω:

$$V_{REF} = 8 \times 1.0 \times 0.05 = 0.400 \text{ V}$$

For $R_{CS} = 0.1$ Ω:

$$V_{REF} = 8 \times 1.0 \times 0.1 = 0.800 \text{ V}$$

The required $V_{REF}$ differs between modules even when the target current is identical.

### 1.3. Application to the Two Stepper Motors

Each stepper motor has a rated phase current of 1.8 A.

| Sense resistor | VREF for 1.8 A |
|---|---:|
| 0.05 Ω | 0.720 V |
| 0.068 Ω | 0.979 V |
| 0.1 Ω | 1.440 V |

These are theoretical values, not recommended settings. Many A4988 modules cannot sustain 1.8 A per phase without substantial cooling, and some cannot reach it reliably. Check the module's thermal performance, cooling and power stage first. Do not set the driver to 1.8 A simply because it is the motor's rating.

In full-step mode both windings conduct, so each winding current is about $0.707 \times I_{LIMIT}$. In microstepping modes the peak winding current reaches $I_{LIMIT}$. [1]

### 1.4. Practical Measurement Procedure

1. Power off and identify the sense resistors on the module.
2. Calculate the target $V_{REF}$ using the verified $R_{CS}$.
3. Power the logic supply and measure $V_{REF}$ between the potentiometer wiper (or its metal screw) and GND.
4. Adjust the potentiometer slowly while watching the multimeter.
5. Check motor and driver temperatures during operation.

Safety notes:

- Never connect or disconnect the motor while VMOT is powered.
- Avoid shorting adjacent pins with the multimeter probe.
- Fit a heat sink, and add airflow when running near 1 A or above.

## References

 Pololu, *A4988 Stepper Motor Driver Carrier* and A4988 datasheet: https://www.pololu.com/product/1182