# Component Description — TF-Luna ToF LiDAR Sensor

## What It Is

The TF-Luna is a compact, solid-state single-point LiDAR distance sensor made by Benewake. Unlike simple infrared (IR) distance sensors that estimate distance through triangulation of a reflected light spot, the TF-Luna uses the **Time-of-Flight (ToF) principle**: it emits a modulated near-infrared laser pulse and measures the time it takes for the light to reflect off a surface and return to the sensor. That time is then converted directly into a distance value.

## Technical Specifications

| Parameter | Value |
|---|---|
| Measuring principle | Time-of-Flight (ToF) |
| Light source | 850 nm VCSEL laser (Class 1 — eye-safe) |
| Operating range | 0.2 m – 8 m |
| Distance resolution | 1 cm |
| Accuracy | ±6 cm (0.2 m – 3 m), ±2% (3 m – 8 m) |
| Field of View (FOV) | 2° |
| Frame rate | 1 Hz – 250 Hz (100 Hz default, adjustable) |
| Supply voltage | 3.7 V – 5.2 V (5 V typical) |
| Average / peak current | ~70 mA / 150 mA |
| Power consumption | ≤ 0.35 W |
| Communication interface | UART or I2C (UART is the default; I2C must be set manually) |
| Communication level | LVTTL, 3.3 V (directly compatible with ESP32-S3 logic) |
| Default UART baud rate | 115200 |
| Dimensions / weight | 35 × 21.25 × 13.5 mm, < 5 g |
| Operating temperature | −20 °C to 75 °C |

## Why TF-Luna Fits This 3D Scanner Project

The scanner's design compares an IR distance sensor against a ToF sensor as part of its sensor sub-study, and TF-Luna is the ToF side of that comparison:

- **Consistent accuracy over the working range** — because it measures actual light travel time rather than triangulating a reflected spot, its accuracy stays relatively stable across the 0.2 m–8 m range and is less affected by the color or reflectivity of the scanned object's surface than a basic IR sensor is.
- **Narrow 2° field of view** — helps the sensor pick out a single, well-defined point on the object at each turntable angle/Z-height, which matters for building a clean point cloud.
- **Digital UART/I2C output** — the sensor returns a ready-to-use distance value over UART or I2C, so the ESP32-S3 doesn't need to do analog-to-digital conversion or apply a distance-conversion formula, unlike a typical analog IR sensor.
- **Low power and small footprint** — at under 0.35 W and 5 g, it is easy to mount on the scanner's moving Z-axis arm without adding significant load.
- **3.3 V-compatible logic level** — its LVTTL communication level matches the ESP32-S3 directly, simplifying wiring compared to sensors that need level shifting.

In the project's IR-vs-ToF comparison, TF-Luna represents the higher-accuracy, more consistent option, while the IR sensor represents the lower-cost, simpler-interface alternative — letting the report evaluate the practical trade-off between the two for a DIY 3D scanning application.
