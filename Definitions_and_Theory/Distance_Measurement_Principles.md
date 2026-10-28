# Distance Measurement Principles

## 1. TF-Luna Distance Measurement Principle

The TF-Luna is a single-point LiDAR sensor that measures distance using the Time-of-Flight (ToF) principle. It emits modulated near-infrared light and determines distance from the phase difference between the emitted and reflected signals. [2]

### 1.1. Basic ToF Formula

For a simplified pulsed time-of-flight model, the distance is:

$$d = \frac{c \cdot \Delta t}{2}$$

Where:

- $d$: distance to the target (m).
- $c$: speed of light, approximately $3.0 \times 10^{8}$ m/s.
- $\Delta t$: round-trip travel time of the light (s).
- The factor 2 accounts for the outgoing and returning paths.

For example, a target at $d = 1$ m gives $\Delta t = 2d/c \approx 6.67$ ns, which is too short to measure directly with low-cost electronics.

For this reason, the TF-Luna uses a modulated signal with modulation frequency $f_{mod}$ and measures the phase shift $\Delta\varphi$ (rad) between the emitted and received signals:

$$d = \frac{c \cdot \Delta\varphi}{4\pi f_{mod}}$$

Because the phase repeats every $2\pi$, the unambiguous measurement range is:

$$d_{max} = \frac{c}{2 f_{mod}}$$

The phase is computed inside the sensor, so the optical phase is not treated as a directly measured pulse travel time in the application.

### 1.2. Distance Data from the Sensor

The ESP32-S3 receives an already calculated distance value from the TF-Luna through UART. The firmware only needs to parse the data frame and does not calculate distance from the optical signal.

The sensor's nominal operating range is approximately 0.2 to 8 m under specified test conditions, with a blind zone below 0.2 m. Actual performance depends on target reflectivity, ambient lighting, configuration, and operating environment. [2]

### 1.3. UART Configuration and Baud Rate

The TF-Luna uses a 3.3 V TTL UART interface with 8 data bits, no parity, and 1 stop bit (8N1). The default baud rate is 115200 bps, and the default output frame rate is 100 Hz.

Each output frame has 9 bytes, and each byte is transmitted as 10 bits (1 start bit, 8 data bits, 1 stop bit). The number of bits per frame is:

$$N_{bits} = 9 \times 10 = 90 \text{ bits}$$

The minimum baud rate required for a frame rate $f_{frame}$ is:

$$B_{min} = N_{bits} \times f_{frame}$$

The time to transmit one frame at baud rate $B$ is:

$$T_{frame} = \frac{N_{bits}}{B}$$

The bus utilization is:

$$U = \frac{N_{bits} \times f_{frame}}{B} \times 100\%$$

| Frame rate | $B_{min}$ | Utilization at 115200 bps | Utilization at 19200 bps |
|---:|---:|---:|---:|
| 100 Hz | 9,000 bps | 7.8 % | 46.9 % |
| 250 Hz | 22,500 bps | 19.5 % | 117 % (insufficient) |

At 115200 bps, one frame takes $T_{frame} = 90 / 115200 \approx 0.78$ ms, which is much shorter than the 4 ms period at the maximum frame rate of 250 Hz. Therefore, **115200 bps, 8N1** is selected: it is the sensor's default, it leaves a large margin at every supported frame rate, and the ESP32-S3 UART supports it without error. A lower baud rate such as 19200 bps cannot carry 250 Hz data and leaves too little margin at 100 Hz.

If the baud rate is changed, it must be changed with the sensor's configuration command, saved to the sensor, and the sensor restarted. The ESP32-S3 UART must then be set to the same value. The command format is given in the Benewake integration manual.

### 1.4. Data Frame and Signal Strength (Flux)

Each UART frame contains the following bytes:

| Byte | Content |
|---:|---|
| 0 | Header, 0x59 |
| 1 | Header, 0x59 |
| 2 | Dist_L |
| 3 | Dist_H |
| 4 | Flux_L |
| 5 | Flux_H |
| 6 | Temp_L |
| 7 | Temp_H |
| 8 | Checksum |

The checksum is the lower 8 bits of the sum of bytes 0 to 7:

$$\text{Checksum} = \left( \sum_{i=0}^{7} B_i \right) \bmod 256$$

The distance and signal strength are assembled from the low and high bytes:

$$D = 256 \times \text{Dist\_H} + \text{Dist\_L} \quad (\text{cm})$$

$$\Phi = 256 \times \text{Flux\_H} + \text{Flux\_L}$$

Here $\Phi$ is the signal strength (Flux, also called Amp) of the reflected light, in raw units with no physical unit. A higher value means a stronger return signal, which depends on target reflectivity, target distance, and incidence angle. In this report, Flux is used as a quality indicator and is not a photometric quantity.

A measurement is accepted only if all of the following conditions hold:

- The header bytes are 0x59 0x59.
- The checksum is correct.
- $\Phi \geq \Phi_{min}$, where $\Phi_{min} = 100$ (below this value the distance is unreliable).
- $\Phi \neq 65535$ (this value means the receiver is saturated).
- $0.2 \le D/100 \le 8$ m.

If a frame fails any condition, it is discarded and the previous valid point is not reused. Points with low Flux, such as dark or glossy surfaces, are marked invalid in the point cloud.

### 1.5. Using Distance in the 3D Scanner

The TF-Luna is mounted so that its optical axis points horizontally at the rotation axis, at a known distance $L$ (m) from the sensor's optical origin to the rotation axis. For a rotating object, the radial distance of the surface point is:

$$r = L - d$$

where $d = D/100$ is the measured distance in meters. The point coordinates are:

$$x = r\cos\theta$$

$$y = r\sin\theta$$

$$z = z_{axis}$$

Where:

- $r$: radial distance from the rotation axis to the measured surface point (m).
- $\theta$: turntable angle (rad).
- $z_{axis}$: vertical position of the sensor (m).

Because of the 0.2 m blind zone, the sensor must be placed so that $L - r_{max} \geq 0.2$ m, where $r_{max}$ is the largest radius of the scanned object. The value of $L$, the sensor height, and the angle offset must be calibrated using objects of known dimensions. Otherwise the point-cloud coordinates will be inaccurate.

## 2. GP2Y0A51SK0F Distance Measurement Principle

The Sharp GP2Y0A51SK0F is an infrared distance sensor that estimates distance using optical triangulation. It emits infrared light toward the target and detects the reflected light with an internal position-sensitive detector. The sensor converts the detected light position into an analog output voltage corresponding to the target distance. [3]

### 2.1. Basic Measurement Principle

In triangulation, the light spot position $s$ on the detector depends on the distance $d$ as:

$$s = \frac{f \cdot b}{d}$$

Where $f$ is the focal length of the receiving lens and $b$ is the baseline between the emitter and the lens. The distance is therefore inversely related to the spot position, so the output voltage is nonlinear in $d$. A good model of the output is:

$$V = \frac{a}{d + k} + b_0$$

Where $a$, $k$, and $b_0$ are constants that depend on the individual sensor and the target surface. This model is the basis of the calibration in Section 1.2.3. Solving for the distance gives:

$$d = \frac{a}{V - b_0} - k$$

### 2.2. Distance Data from the Sensor (Single-Wire Analog Output to ADC)

The sensor has one analog output wire (Vo) in addition to the power and ground lines. This wire is connected directly to an ADC input of the ESP32-S3. The sensor is powered from 5 V, and its output voltage stays below the 3.3 V limit of the ESP32-S3 pin.

The firmware reads the output as follows:

1. Use an ADC1 channel, because ADC2 cannot be used reliably while Wi-Fi is active on the ESP32-S3.
2. Set the attenuation to the highest setting (about 12 dB) so that the full output range is covered.
3. Convert the raw reading to millivolts with the ESP-IDF ADC calibration functions, because the ESP32-S3 ADC is not perfectly linear.
4. Place a capacitor of about 10 µF close to the sensor's supply pins to reduce supply noise.

The ideal conversion from a raw reading $N$ of a 12-bit ADC is:

$$V = \frac{N}{4095} \times V_{FS}$$

where $V_{FS}$ is the full-scale voltage at the selected attenuation. The sensor updates its output about every 16.5 ms, so readings taken faster than this are repeated values. For averaging, samples should be spaced by at least 20 ms.

The GP2Y0A51SK0F has a nominal measurement range of approximately 2 to 15 cm. Below 2 cm, the output voltage decreases again, so the same voltage can correspond to two different distances. Therefore, the sensor must be mounted so that the surface being measured is never closer than 2 cm. [3]

### 2.3. Calibration for Accurate Near-Range Measurement

Near-range accuracy depends on how accurately $a$, $k$, and $b_0$ are determined, so calibration is done with reference distances at the near end of the range.

**Sensitivity analysis.** Differentiating the model gives:

$$\frac{dV}{dd} = -\frac{a}{(d + k)^2}$$

For a voltage noise $\delta V$, the distance error is:

$$\delta d = \delta V \cdot \frac{(d + k)^2}{a}$$

The error grows with $(d + k)^2$, so the sensor is most accurate at short distances and loses accuracy at long distances. For this reason, calibration points are concentrated in the near range.

**Calibration procedure.**

1. Fix the sensor and a flat target (use the same material and color as the scanned objects) on a rail with a ruler or caliper.
2. Measure at the reference distances $d_i$: 2, 2.5, 3, 3.5, 4, 5, 6, 8, 10, 12, and 15 cm. Use more points between 2 and 6 cm.
3. At each point, collect $M$ readings (for example, $M = 50$) spaced at least 20 ms apart, discard outliers with a median filter, and use the mean voltage $\bar{V}_i$.
4. Calibrate with the same ambient light and the same supply voltage as in the scanner.

**Model fitting.** Define $u_i = 1/(d_i + k)$. For a fixed $k$, the model $V = a u + b_0$ is linear, so $a$ and $b_0$ are obtained by linear least squares:

$$a = \frac{\sum_i (u_i - \bar{u})(\bar{V}_i - \bar{V})}{\sum_i (u_i - \bar{u})^2}, \qquad b_0 = \bar{V} - a\bar{u}$$

The value of $k$ is searched (for example, from 0 to 3 cm in steps of 0.05 cm) to minimize the distance error:

$$E = \sqrt{\frac{1}{n}\sum_{i=1}^{n}\left(d_i - \hat{d}_i\right)^2}$$

where $\hat{d}_i = \dfrac{a}{\bar{V}_i - b_0} - k$. To favor the near range, each point can be weighted by $w_i = 1/d_i^2$.

**Lookup table with interpolation.** As an alternative, or in addition, the points $(\bar{V}_i, d_i)$ can be stored as a lookup table sorted by voltage. For a measured voltage $V$ between two table points, the distance is interpolated linearly:

$$d = d_i + \frac{(V - V_i)(d_{i+1} - d_i)}{V_{i+1} - V_i}$$

A dense table between 2 and 6 cm gives a smaller error than a single global fit, because the fit error is largest where the curve bends most. A practical choice is to use the lookup table for 2 to 6 cm and the fitted model above 6 cm.

**Verification.** After calibration, measure objects at distances that were not used for fitting and compute the error $E$. Repeat the calibration if the error is larger than the accuracy required by the scanner.

### 2.4. Using Distance in the 3D Scanner

The estimated distance is associated with the corresponding scanning position in the same way as for the TF-Luna. If the sensor is at a distance $L_s$ from the rotation axis, the radial distance is:

$$r = L_s - d$$

and the point coordinates are:

$$x = r\cos\theta$$

$$y = r\sin\theta$$

$$z = z_{axis}$$

Where:

- $r$: radial distance from the rotation axis to the measured surface point (cm).
- $\theta$: turntable angle (rad).
- $z_{axis}$: vertical position of the sensor (cm).

The sensor mounting position, orientation, and the distance $L_s$ must be calibrated. Because the sensor range is only 2 to 15 cm, $L_s - r_{max} \geq 2$ cm and $L_s - r_{min} \leq 15$ cm must both hold for every scanned object.

Measurement accuracy is affected by the nonlinear output, ADC resolution and calibration, noise, and the target's reflectivity. Therefore, the scanner should be tested with objects of known dimensions before evaluating the reconstructed models.


## References
1. Benewake, *TF-Luna Datasheet and Technical Documentation*: https://ai.benewake.com/en/resources/technical-documents/tf-luna-datasheet/
2. Sharp, *GP2Y0A51SK0F Distance Measuring Sensor Unit Datasheet*.