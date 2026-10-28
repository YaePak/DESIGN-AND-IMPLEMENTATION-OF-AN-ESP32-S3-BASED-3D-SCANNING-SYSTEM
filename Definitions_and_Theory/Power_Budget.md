# Power Supply Current and Power Calculation

The prototype uses a nominal 12 V supply rated at 1 A. During testing, the system operated slowly and the supply voltage dropped to approximately 4–6 V under load.

This behavior indicates that the supply may be overloaded or that excessive voltage drop exists in the wiring or connections. Further measurements are required to confirm the cause.

## 1. Available Supply Power

The rated power available from the existing supply is:

$$P_{available} = V \times I$$

$$P_{available} = 12 \times 1 = 12 \text{ W}$$

If the output sags to 4–6 V while still delivering 1 A, the output power is only $4 \text{ to } 6 \text{ W}$. This is an illustrative calculation, because the actual current during the voltage drop has not yet been measured.

The A4988 requires a motor supply voltage $V_{MOT}$ of at least 8 V. At 4–6 V the driver is below its minimum operating voltage, so the motors would produce weak torque and move slowly. This matches the observed behavior. [1]

## 2. Load Assessment

The loads are supplied through different power paths:

| Load | Supply rail | Assumed current | Power |
|---|---|---|---|
| Turntable stepper motor | 12 V through A4988 | Set by the driver current limit | See Section 2.3 |
| Z-axis stepper motor | 12 V through A4988 | Set by the driver current limit | See Section 2.3 |
| ESP32-S3 | 3.3 V | About 0.3 A peak during Wi-Fi transmission (assumed, board dependent) | 3.3 × 0.3 ≈ 1.0 W |
| TF-Luna | 5 V | Average about 70 mA, peak up to 150 mA | 5 × 0.15 = 0.75 W (peak) |
| GP2Y0A51SK0F | 5 V | Up to about 22 mA | 5 × 0.022 ≈ 0.11 W |

The motor phase currents must not be added directly to the logic currents, because the A4988 regulates the winding current with a chopping circuit. The current drawn from the 12 V supply is different from the winding current. The logic loads are supplied from 3.3 V and 5 V rails through a regulator or converter, so their currents at those rails are not currents at 12 V.

The logic power is:

$$P_{logic} = 1.0 + 0.75 + 0.11 \approx 1.9 \text{ W}$$

## 3. Power Budget Method

**Stepper motors.** The power dissipated in the motor windings is:

$$P_{cu} = N_{ph} \times R_{ph} \times I_{ph}^{2}$$

Where $N_{ph} = 2$ is the number of energized phases, $R_{ph}$ is the resistance of one winding, and $I_{ph}$ is the actual (RMS) winding current. The power drawn from the supply includes the driver losses:

$$P_{motor,in} = \frac{P_{cu}}{\eta_{drv}}$$

where $\eta_{drv}$ is the driver efficiency.

**Logic loads.** The power drawn from 12 V through a converter with efficiency $\eta_{conv}$ is:

$$P_{logic,in} = \frac{P_{logic}}{\eta_{conv}}$$

**Total and supply current.**

$$P_{total} = \sum P_{motor,in} + P_{logic,in}$$

$$I_{12V} = \frac{P_{total}}{12}$$

The efficiency is applied only to the loads that pass through the corresponding stage, and not to the whole sum.

**Numerical example.** The following uses an assumed winding resistance $R_{ph} = 1.5\ \Omega$, $\eta_{drv} = 0.85$, and $\eta_{conv} = 0.85$. These values are illustrative and must be replaced with the datasheet values of the actual motors.

| Case | $I_{ph}$ | $P_{cu}$ per motor | $P_{motor,in}$ per motor | Two motors | $P_{logic,in}$ | $P_{total}$ | $I_{12V}$ |
|---|---:|---:|---:|---:|---:|---:|---:|
| Driver set to 1.0 A | 1.0 A | 2 × 1.5 × 1.0² = 3.0 W | 3.0 / 0.85 ≈ 3.5 W | 7.1 W | 1.9 / 0.85 ≈ 2.2 W | 9.3 W | 0.77 A |
| Driver set to 1.8 A | 1.8 A | 2 × 1.5 × 1.8² = 9.7 W | 9.7 / 0.85 ≈ 11.4 W | 22.9 W | 2.2 W | 25.1 W | 2.09 A |

In full-step mode with the A4988, each winding carries about $0.707 \times I_{LIMIT}$, so the real $P_{cu}$ can be about half of the values above. The table uses the worst case, where both windings carry the full current.

The result shows that:

- With the driver set to 1.0 A, the demand is about 0.77 A at 12 V. This is close to the 1 A rating of the existing supply, and there is no headroom for start-up and acceleration peaks.
- With the driver set to 1.8 A, the demand is about 2.1 A at 12 V, which is twice the rating of the existing supply (12 W against about 25 W).

Both cases show that a 12 V / 1 A supply is not suitable for two stepper motors, and this can explain the voltage drop to 4–6 V (overload or current limiting of the supply).

## 4. Voltage Drop in Wiring

Voltage drop in the wiring and connectors is:

$$\Delta V = I \times R_{wire}$$

and the power lost in the wiring is:

$$P_{loss} = I^{2} \times R_{wire}$$

For example, 2 A through a total wire and contact resistance of 0.5 Ω gives $\Delta V = 1 \text{ V}$ and $P_{loss} = 2 \text{ W}$.

To distinguish the cause of the 4–6 V reading, measure the voltage at two points while the motors are running:

| Measurement | Meaning |
|---|---|
| Voltage at the supply output terminals is low (4–6 V) | The supply is overloaded, or its current limit or protection is active |
| Voltage at the supply terminals is about 12 V, but low at the driver $V_{MOT}$ pin | Excessive resistance in the wiring, connectors, or PCB traces |

## 5. Power Supply Selection

A replacement supply should be selected based on the measured input current, start-up and acceleration peaks, and the requirements of all connected devices. A headroom factor of about 1.5 is commonly used:

$$I_{supply} \geq 1.5 \times I_{12V}$$

With the illustrative values above, this gives about 1.2 A for the 1.0 A driver setting and about 3.1 A for the 1.8 A driver setting. A 12 V / 2 A supply is the minimum for the 1.0 A setting, and a 12 V / 3–4 A supply is suitable for the 1.8 A setting.

Recommended checks include:

- Measure the 12 V input voltage and current during motor start-up, rotation, and Z-axis movement.
- Inspect power connectors, PCB traces, solder joints, and wires for excessive resistance or heating.
- Verify the A4988 current limits and thermal conditions. Many A4988 modules cannot sustain 1.8 A per phase without substantial cooling.
- Place an electrolytic capacitor of at least 47 µF across $V_{MOT}$ and GND close to each driver to absorb voltage spikes. [1]
- Ensure the ESP32-S3, TF-Luna, and GP2Y0A51SK0F receive the correct supply voltages.

The 4–6 V output under load should be investigated before further extended operation. A higher-current supply alone will not resolve the issue if faulty wiring, excessive resistance, or an incorrect power connection is present.

---

## 6. Conclusion

The preliminary budget shows that the existing 12 V / 1 A supply is insufficient for two stepper motors. The next steps are to verify the A4988 modules, measure the prototype's 12 V input current under load, inspect the wiring, and select a supply with sufficient current and headroom.
