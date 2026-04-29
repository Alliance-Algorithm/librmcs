## BMI088

## 6-axis Motion Tracking for High-performance Applications

![](https://cdn.mathpix.com/cropped/20159896-9fb0-48a0-8609-2aca341babdf-01.jpg?height=565&width=743&top_left_y=1199&top_left_x=660)

## BMI088: Data Sheet

Document revision
Document release date
Document number
Technical reference code
Notes
1.9

January 2024
BST-BMI088-DS000-19
0273141365
Data and descriptions in this document are subject to change without notice. Product photos and pictures are for illustration purposes only and may differ from the real product appearance

## Basic Description

BMI088 is an inertial measurement unit (IMU) for the detection of movements and rotations in 6 degrees of freedom (6DoF). It combines the functionality of two inertial sensors in one device: an advanced triaxial 16-bit gyroscope and a versatile, leading-edge triaxial 16-bit accelerometer.

BMI088 is designed to meet all requirements for high performance consumer applications in harsh vibration environments such as those encountered in drones and robotics applications. The IMU is designed to effectively suppress vibrations above a few hundred Hz that could occasionally occur due to resonances on the pcb or the structure of the total system.

The sensor has an extended measurement range of up to $\pm 24 \mathrm{~g}$ to avoid signal clipping under strong signal exposure.

An evaluation circuitry (ASIC) converts the output of the micro-electro-mechanical sensing structures (MEMS), which are developed, produced and tested in BOSCH facilities. The corresponding chip-sets are packed into one single LGA $3.0 \mathrm{~mm} \times 4.5 \mathrm{~mm} \times 0.95 \mathrm{~mm}$ housing. For optimum system integration, BMI088 is fitted with digital interfaces (SPI or I2C), offering a wide VDDIO voltage range from 1.2 V to 3.6 V . To provide maximum performance and reliability, each device is tested and is ready-to-use calibrated.

To increase flexibility, both gyroscope and accelerometer can not only be operated individually, but tied together for data synchronization purposes. The on-chip features comprise FIFOs for acceleration and gyroscope data and interrupt controllers.

The BMI088 has an excellent temperature behavior with an outstanding low temperature coefficient of the offset (TCO) and temperature coefficient of the sensitivity (TCS).

## Index of Contents

Basic Description ..... 2

1. Specification ..... 7
1.1 Electrical Specifications ..... 7
1.1.1 Electrical Specifications: Accelerometer/Gyroscope ..... 8
1.2 Accelerometer Specifications ..... 9
1.3 Gyroscope Specifications ..... 10
1.4 Temperature Sensor Specifications ..... 11
1.5 Absolute Maximum Ratings ..... 12
2. Block Diagram ..... 13
3. Quick Start Guide - Device Initialization ..... 13
4. Functional Description ..... 14
4.1 Power Management and Power Modes ..... 14
4.1.1 Power Modes: Accelerometer ..... 14
4.1.2 Power Modes: Gyroscope ..... 15
4.2 Sensor Data ..... 15
4.3 Sensor Time ..... 16
4.4 Output Data Rate (ODR) and Low-pass Filter ..... 16
4.4.1 Accelerometer ..... 16
4.4.2 Gyroscope ..... 16
4.5 Range Settings ..... 16
4.6 Self-test ..... 16
4.6.1 Accelerometer ..... 17
4.6.2 Gyroscope ..... 17
4.7 New Data Interrupt ..... 18
4.7.1 Accelerometer ..... 18
4.7.2 Gyroscope ..... 18
4.8 Soft-Reset ..... 18
4.8.1 Soft-Reset Accelerometer ..... 18
4.9 FIFO ..... 19
4.9.1 FIFO operating modes ..... 19
4.9.2 FIFO interrupts ..... 19
4.9.3 Accelerometer sensor FIFO buffer ..... 19
4.9.4 Gyroscope sensor FIFO buffer ..... 22
5. Register Maps ..... 24
5.1 Communication with the sensor ..... 24
5.2 Register Map: Accelerometer ..... 25
5.3 Register Description: Accelerometer ..... 26
5.3.1 Register 0x00: ACC_CHIP_ID ..... 26
5.3.2 Register 0x02: ACC_ERR_REG ..... 26
5.3.3 Register 0x03: ACC_STATUS ..... 26
5.3.4 Register 0x12-0x17: ACC data ..... 27
5.3.5 Register 0x18-0x1A: Sensortime data ..... 27
5.3.6 Register 0x1D: ACC_INT_STAT_1 ..... 27
5.3.7 Register $0 \times 22-0 \times 23$ : Temperature sensor data ..... 28
5.3.8 Register $0 \times 24-0 \times 25$ : FIFO_LENGTH ..... 28
5.3.9 Register 0x26: FIFO_DATA ..... 28
5.3.10 Register 0x40: ACC_CONF ..... 29
5.3.11 Register 0x41: ACC_RANGE ..... 30
5.3.12 Register 0x45: FIFO_DOWNS ..... 30
5.3.13 Register 0x46-0x47: FIFO_WTM ..... 30
5.3.14 Register 0x48: FIFO_CONFIG_0 ..... 31
5.3.15 Register 0x49: FIFO_CONFIG_1 ..... 31
5.3.16 Register 0x53: INT1_IO_CONF ..... 32
5.3.17 Register 0x54: INT2_IO_CONF ..... 33
5.3.18 Register 0x58: INT1_INT2_MAP_DATA ..... 33
5.3.19 Register 0x6D: ACC_SELF_TEST ..... 34
5.3.20 Register 0x7C: ACC_PWR_CONF ..... 34
5.3.21 Register 0x7D: ACC_PWR_CTRL ..... 34
5.3.22 Register 0x7E: ACC_SOFTRESET ..... 35
5.4 Register Map: Gyroscope ..... 36
5.5 Register Description: Gyroscope ..... 37
5.5.1 Register 0x00: GYRO_CHIP_ID ..... 37
5.5.2 Register 0x02-0x07: Rate data ..... 37
5.5.3 Register 0x0A: GYRO_INT_STAT_1 ..... 38
5.5.4 Register 0x0E: FIFO_STATUS ..... 38
5.5.5 Register 0x0F: GYRO_RANGE ..... 39
5.5.6 Register 0x10: GYRO_BANDWIDTH ..... 39
5.5.7 Register 0x11: GYRO_LPM1 ..... 40
5.5.8 Register 0x14: GYRO_SOFTRESET ..... 40
5.5.9 Register 0x15: GYRO_INT_CTRL ..... 40
5.5.10 Register 0x16: INT3_INT4_IO_CONF ..... 41
5.5.11 Register 0x18: INT3_INT4_IO_MAP ..... 41
5.5.12 Register 0x1E: FIFO_WM_ENABLE ..... 42
5.5.13 Register 0x34: FIFO_EXT_INT_S ..... 42
5.5.14 Register 0x3C: GYRO_SELF_TEST ..... 42
5.5.15 Register 0x3D: FIFO_CONFIG_0 ..... 43
5.5.16 Register 0x3E: FIFO_CONFIG_1 ..... 43
5.5.17 Register 0x3F: FIFO_DATA ..... 43
6. Digital Interface ..... 44
6.1 Serial Peripheral Interface (SPI) ..... 45
6.1.1 SPI interface of gyroscope part ..... 47
6.1.2 SPI interface of accelerometer part ..... 48
6.2 Inter-Integrated Circuit (I ${ }^{2} \mathrm{C}$ ) ..... 48
7. Pin-out and Connection Diagram ..... 52
7.1 Pin-out ..... 52
7.2 Connection diagram SPI ..... 53
7.3 Connection diagram $\mathrm{I}^{2} \mathrm{C}$ ..... 53
8. Package ..... 54
8.1 Outline Dimensions ..... 54
8.2 Landing pattern ..... 55
8.3 Sensing axes orientation ..... 56
8.4 Marking ..... 57
8.4.1 Mass production samples ..... 57
8.4.2 Engineering samples ..... 57
8.5 PCB layout and soldering guidelines ..... 57
8.6 Handling instructions ..... 58
8.7 Tape and Reel specification ..... 58
8.7.1 Orientation within the reel ..... 59
8.8 Environmental safety ..... 59
8.8.1 Halogen content ..... 59
9. Legal Disclaimer ..... 60
9.1 Engineering samples ..... 60
9.2 Product use ..... 60
9.3 Application examples and hints ..... 60
10. Document History and Modification ..... 61

## 1. Specification

If not stated otherwise, the given values are over lifetime and full performance temperature and voltage ranges, minimum/maximum values are $\pm 3 \sigma$.

### 1.1 Electrical Specifications

Table 1: Electrical parameter specification
| Parameter | Symbol | Condition | Min | Max | Unit |
| :--- | :--- | :--- | :--- | :--- | :--- |
| Supply Voltage Internal Domains | VDD |  | 2.4 | 3.6 | V |
| Supply Voltage I/O Domain | VDDIO |  | 1.2 | 3.6 | v |
| Voltage Input Low Level | VIL,a | SPI \& I ${ }^{\mathbf{2}} \mathbf{C}$ |  | 0.3 VDDIO | - |
| Voltage Input High Level | VIH,a | SPI \& I ${ }^{\mathbf{2}} \mathbf{C}$ | 0.7 VDDIO |  | - |
| Voltage Output Low Level | Vol,a | $\mathrm{lol}<=2 \mathrm{~mA}$, SPI |  | 0.23 VDDIO | - |
| Voltage Output High Level | Vон | Іон $<=2 \mathrm{~mA}$, SPI | 0.8 VDDIO |  | - |
| Operating Temperature | T A |  | -40 | +85 | ${ }^{\circ} \mathrm{C}$ |


### 1.1.1 Electrical Specifications: Accelerometer/Gyroscope

Table 2: Electrical parameter specification accelerometer
| Parameter | Symbol | Condition | Min | Typ | Max | Units |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| Total Supply Current in Normal mode | $\mathrm{l}_{\mathrm{DD}}$ | $\mathrm{VDD}=\mathrm{VDDIO}=3.0 \mathrm{~V}$, $25^{\circ} \mathrm{C}, \mathrm{g}_{\mathrm{FS} 4 \mathrm{~g}}$ |  | 150 |  | $\mu \mathrm{A}$ |
| Total Supply Current in Suspend Mode | $\mathrm{I}_{\text {DDsum }}$ | $\mathrm{VDD}=\mathrm{VDDIO}=3.0 \mathrm{~V}$, $25^{\circ} \mathrm{C}$ |  | 3 |  | $\mu \mathrm{A}$ |
| Power-up time | $\mathrm{t}_{\text {s_up }}$ | Time to first valid sample from suspend mode |  |  | 1 | ms |


Table 3: Electrical parameter specification gyroscope
| Parameter | Symbol | Condition | Min | Typ | Max | Unit |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| Supply Current in Normal Mode | $\mathrm{I}_{\mathrm{DD}}$ | $\begin{gathered} \mathrm{VDD}=\mathrm{VDDIO}=3.0 \mathrm{~V}, \\ 25^{\circ} \mathrm{C}, \mathrm{ODR}=1 \mathrm{kHz} \end{gathered}$ |  | 5 |  | mA |
| Supply Current in Suspend Mode | $\mathrm{I}_{\text {DDsum }}$ | $\begin{gathered} \mathrm{VDD}=\mathrm{VDDIO}=3.0 \mathrm{~V}, \\ 25^{\circ} \mathrm{C} \end{gathered}$ |  | 25 |  | $\mu \mathrm{A}$ |
| Supply Current in Deep Suspend Mode | IDDdsum | $\mathrm{VDD}=\mathrm{VDDIO}=3.0 \mathrm{~V}$, $25^{\circ} \mathrm{C}$ |  | <5 |  | $\mu \mathrm{A}$ |
| Start-up time | $\mathrm{t}_{\mathrm{su}}$ | to $\pm 1^{\circ} / \mathrm{s}$ of final value; from power-off |  | 30 |  | ms |
| Wake-up time | $\mathrm{t}_{\text {wusm }}$ | From suspend- and deep suspend-modes |  | 30 |  | ms |
| Wake-up time | $\mathrm{t}_{\text {wufpm }}$ | From fast power-up mode |  | 10 |  | ms |


### 1.2 Accelerometer Specifications

Table 4: Accelerometer specifications
| Parameter | Symbol | Condition | Min | Typ | Max | Units |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| Acceleration Range | gfS3g | Selectable via serial digital interface |  | $\pm 3$ |  | g |
|  | gFS6g |  |  | $\pm 6$ |  | g |
|  | gFS12g |  |  | $\pm 12$ |  | g |
|  | gFS24g |  |  | $\pm 24$ |  | g |
| Sensitivity | $\mathrm{S}_{3 \mathrm{~g}}$ | gfS3g, $\mathrm{T}_{\mathrm{A}}=25^{\circ} \mathrm{C}$ |  | 10920 |  | LSB/g |
|  | $\mathrm{S}_{6 \mathrm{~g}}$ | gfS6g, $\mathrm{T}_{\mathrm{A}}=25^{\circ} \mathrm{C}$ |  | 5460 |  | LSB/g |
|  | $\mathrm{S}_{12 \mathrm{~g}}$ | $\mathrm{g}_{\text {FS12g }}, \mathrm{T}_{\mathrm{A}}=25^{\circ} \mathrm{C}$ |  | 2730 |  | LSB/g |
|  | $\mathrm{S}_{24 \mathrm{~g}}$ | $\mathrm{g}_{\mathrm{F} 524 \mathrm{~g}}, \mathrm{~T}_{\mathrm{A}}=25^{\circ} \mathrm{C}$ |  | 1365 |  | LSB/g |
| Sensitivity Temperature Drift | TCS |  |  | 0.002 |  | \%/K |
| Zero-g Offset | Off | Nominal VDD and VDDIO, $25^{\circ} \mathrm{C}$, g $_{\text {FS6g }}$ |  | 20 |  | mg |
| Zero-g Offset Temperature Drift | TCO |  |  | <0.2 |  | mg/K |
| Output Data Rate | ODR |  | 12.5 |  | 1600 | Hz |
| Bandwidth range | BW | 3 dB cut-off frequency of the accelerometer depends on ODR and OSR | 5 |  | 280 (245 for Z axis) | Hz |
| Nonlinearity | NL | best fit straight line, $\mathrm{g}_{\text {FS3g }}$ |  | 0.5 |  | \%FS |
| Output Noise Density | $\mathrm{n}_{\text {rms }}$ | $\mathrm{g}_{\text {FS3g }}, \mathrm{T}_{\mathrm{A}}=25^{\circ} \mathrm{C}$ Nominal VDD supplies Normal mode |  | 190 (Z-axis) <br> 160 (X- \& Yaxis) |  | $\mu \mathrm{g} / \sqrt{ } \mathrm{Hz}$ |
| Cross Axis Sensitivity | S | relative contribution between any two of the three axes |  | 0.5 |  | \% |
| Alignment Error | $\mathrm{E}_{\mathrm{A}}$ | relative to package outline |  | 0.5 |  | ∘ |


### 1.3 Gyroscope Specifications

Table 5: Gyroscope specifications
| Parameter | Symbol | Condition | Min | Typ | Max | Unit |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| Range | $\mathrm{R}_{\mathrm{FS} 125}$ | Selectable via serial digital interface |  | 125 |  | $\% / \mathrm{s}$ |
|  | $\mathrm{R}_{\mathrm{FS} 250}$ |  |  | 250 |  | $\% / \mathrm{s}$ |
|  | Rf5500 |  |  | 500 |  | $\% / \mathrm{s}$ |
|  | Rfs1000 |  |  | 1000 |  | $\% / \mathrm{s}$ |
|  | Rfs2000 |  |  | 2000 |  | $\% \mathrm{~s}$ |
| Sensitivity |  | $\mathrm{Ta}=25^{\circ} \mathrm{C}, \mathrm{R}_{\mathrm{FS} 125}$ |  | 262.144 |  | LSB/ $\%$ / s |
|  |  | $\mathrm{Ta}=25^{\circ} \mathrm{C}, \mathrm{R}_{\mathrm{FS} 250}$ |  | 131.072 |  | LSB/ $\%$ / $\frac{\mathrm{s}}{\mathrm{s}}$ |
|  |  | $\mathrm{Ta}=25^{\circ} \mathrm{C}, \mathrm{R}_{\mathrm{FS} 500}$ |  | 65.536 |  | LSB/ $\%$ / s |
|  |  | $\mathrm{Ta}=25^{\circ} \mathrm{C}$, RfS1000 |  | 32.768 |  | LSB/ $\%$ / s |
|  |  | $\mathrm{Ta}=25^{\circ} \mathrm{C}$, RfS2000 |  | 16.384 |  | LSB/ $\%$ / s |
| Sensitivity tolerance |  | $\mathrm{Ta}=25^{\circ} \mathrm{C}, \mathrm{R}_{\mathrm{FS} 2000}$ |  | $\pm 1$ |  | \% |
| Sensitivity Change over Temperature | TCS | Nominal VDD supplies $-40^{\circ} \mathrm{C} \leq \mathrm{T}_{\Delta} \leq+85^{\circ} \mathrm{C}$ <br> Rfs2000 |  | $\pm 0.03$ |  | \%/K |
| Sensitivity Supply Volt. Drift | Svdd | $\mathrm{T}_{\mathrm{A}}=25^{\circ} \mathrm{C}$, $\mathrm{VDD}_{\text {min }} \leq \mathrm{VDD} \leq \mathrm{VDD}_{\text {max }}$ |  | <0.4 |  | \%/V |
| Nonlinearity | NL | best fit straight line Rfs1000, Rfs2000 |  | $\pm 0.05$ |  | \%FS |
| g-Sensitivity |  | Sensitivity to acceleration stimuli in all three axis (frequency $<20 \mathrm{kHz}$ ) |  |  | 0.1 | $\% / \mathrm{s} / \mathrm{g}$ |
| Zero-rate Offset | Off $\Omega_{\mathrm{x}} \Omega_{\mathrm{y}}$ and $\Omega_{\mathrm{z}}$ | Nominal VDD supplies $\mathrm{T}_{\wedge}=25^{\circ} \mathrm{C}$, slow and fast offset cancellation off |  | $\pm 1$ |  | $\% / \mathrm{s}$ |
| Zero-rate Offset Change over Temperature | TCO | Nominal VDD supplies $-40^{\circ} \mathrm{C} \leq \mathrm{T}_{\Delta} \leq+85^{\circ} \mathrm{C}$ <br> Rfs2000 |  | $\pm 0.015$ |  | \% s per K |
| Zero-rate Offset Supply Volt. Drift | Off $\Omega$ vdd | $\mathrm{T}_{\mathrm{A}}=25^{\circ} \mathrm{C}$, $\mathrm{VDD}_{\text {min }} \leq \mathrm{VDD} \leq \mathrm{VDD}_{\text {max }}$ |  | <0.1 |  | \%/s/V |
| Output Noise | $\mathrm{n}_{\mathrm{rms}}$ | rms, $\mathrm{BW}=47 \mathrm{~Hz}$ (@ $0.014 \% / \mathrm{s} / \sqrt{ } \mathrm{Hz}$ ) |  | 0.1 |  | $\% / \mathrm{s}$ |


| Bandwidth BW | $\mathrm{f}_{-3 \mathrm{~dB}}$ |  |  | 523 |  | Hz |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
|  |  |  |  | 116 |  |  |
|  |  |  |  | 64 |  |  |
|  |  |  |  | 47 |  |  |
|  |  |  |  | 32 |  |  |
|  |  |  |  | 23 |  |  |
|  |  |  |  | 12 |  |  |
| Data rate (set of $x, y, z$ rate) |  |  |  | 2000 |  | Hz |
|  |  |  |  | 1000 |  |  |
|  |  |  |  | 400 |  |  |
|  |  |  |  | 200 |  |  |
|  |  |  |  | 100 |  |  |
| Data rate tolerance (set of $x, y, z$ rate) |  |  |  | $\pm 0.3$ |  | \% |
| Cross Axis Sensitivity |  | Sensitivity to stimuli in non-sense-direction |  | $\pm 1$ |  | \% |

### 1.4 Temperature Sensor Specifications

Table 6: Temperature sensor specifications
| Parameter | Symbol | Condition | Min | Typ | Max | Units |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| Temperature Sensor Measurement Range | Ts |  | -104 |  | 150 | ${ }^{\circ} \mathrm{C}$ |
| Temperature Sensor Slope | dTs |  |  | 0.125 |  | K/LSB |
| Temperature Sensor Offset error | OTs | at $25^{\circ} \mathrm{C}$ |  | $\pm 1$ |  | K |


### 1.5 Absolute Maximum Ratings

Table 7: Absolute maximum ratings
| Parameter | Condition | Min | Max | Units |
| :--- | :--- | :--- | :--- | :--- |
| Voltage at Supply Pin | VDD Pin | -0.3 | 4 | v |
|  | VDDIO Pin | -0.3 | 4 | V |
| Voltage at any Logic Pin | Non-Supply Pin | -0.3 | $\mathrm{VDDIO}+0.3$ | V |
| Passive Storage Temp. Range | $\leq 65 \%$ rel. H. | -50 | +150 | ${ }^{\circ} \mathrm{C}$ |
| Mechanical Shock | Duration $\leq 200 \mu \mathrm{~s}$ |  | 10,000 | g |
|  | Duration $\leq 1.0 \mathrm{~ms}$ |  | 2,000 | g |
|  | Free fall onto hard surfaces |  | 1.8 | m |
| ESD | HBM, at any Pin |  | 2 | kV |
|  | CDM |  | 500 | V |
|  | MM |  | 200 | V |


Note: Stress above these limits may cause damage to the device. Exceeding the specified electrical limits may affect the device reliability or cause malfunction.

## 2. Block Diagram

Figure 1 shows the basic building blocks of the BMI088:

![](https://cdn.mathpix.com/cropped/20159896-9fb0-48a0-8609-2aca341babdf-13.jpg?height=975&width=1581&top_left_y=424&top_left_x=242)
Figure 1: Block diagram of BMI088

## 3. Quick Start Guide - Device Initialization

For a proper device initialization, the following steps should be considered:

- The user must decide on the interface (I2C or SPI) already during HW design: with the PS pin the user determines which interface the sensor should listen to (see chapter 6).
- The gyroscope part of the BMI088 initializes its I/O pins according to the selection given by the PS pin.
- The accelerometer part starts in I2C mode. It will stay in I2C mode until it detects a rising edge on the CSB1 pin (chip select of the accelerometer), on which the accelerometer part switches to SPI mode and stays in this mode until the next power-up-reset.
- To change the accelerometer to SPI mode in the initialization phase, the user could perform a dummy SPI read operation, e.g. of register ACC_CHIP_ID (the obtained value will be invalid). After the POR the gyroscope is in normal mode, while the accelerometer is in suspend mode. To switch the accelerometer into normal mode, the user must perform the following steps:
a. Power up the sensor
b. Wait 1 ms
c. Enter normal mode by writing '4' to ACC_PWR_CTRL
d. Wait for 450 microseconds


## 4. Functional Description

### 4.1 Power Management and Power Modes

The BMI088 has two distinct power supply pins:

- VDD is the main power supply for the internal blocks
- VDDIO is a separate power supply pin mainly used for the supply of the interface

There are no limitations on the voltage levels of both pins relative to each other, as long as each of them lies within its operating range. Furthermore, the device can be completely switched off (VDD $=0 \mathrm{~V}$ ) while keeping the VDDIO supply on (VDDIO $>0 \mathrm{~V}$ ) or vice versa.

When the VDDIO supply is switched off, all interface pins (CSB, SDI, SCK, PS) must be kept close to $\mathrm{GND}_{\text {Io }}$ potential.

The device contains a power-on reset (POR) generator. It resets the logic part and the register values after powering-on VDD and VDDIO. This means that all application specific settings which are not equal to the default settings (refer to 6.2 register map accelerometer and to 8.2 register map gyroscope), must be changed back to their designated values after POR.

Please note: the POR resets also the interface. For the gyroscope part, the interface is defined by the voltage level on the PS pin. The interface of the accelerometer part is defined by the voltage level of the CSB1 pin at the moment when the POR is initiated (see chapter 3).

### 4.1.1 Power Modes: Accelerometer

The power state of the BMI088 accelerometer is controlled through the register ACC_PWR_CTRL. The register ACC_PWR_CTRL enables and disables the accelerometer and the temperature sensor.

To enter normal mode, the value $0 \times 04$ must be written to ACC_PWR_CTRL.

To enter suspend mode, register ACC_PWR_CTRL must be cleared.

Note: The sensor is in suspend mode after reset (POR or soft-reset), thus the user actively needs to enter normal mode in order to obtain acceleration values.
Note: After POR or soft-reset, the acceleration sensor needs up to 1 ms boot time ( $\mathrm{t}_{\mathrm{s} \_ \text {up }}$ )

### 4.1.2 Power Modes: Gyroscope

The gyroscope has 3 different power modes. Besides normal mode, which represents the fully operational state of the device, there are 2 energy saving modes: suspend mode and deep-suspend mode.

After power-up gyro is in normal mode so that all parts of the device are held powered-up and data acquisition is performed continuously.

In suspend mode the whole analog part is powered down. No data acquisition is performed. While in suspend mode the latest rate data and the content of all configuration registers are kept. The registers can still be read (though they are not updated).

Suspend mode is entered by writing $0 \times 80$ to the register GYRO_LPM1. It can be left by writing $0 \times 00$ to GYRO_LPM1 or by a soft reset (see 4.8).

Although write access to registers is supported at the full interface clock speed (SCL or SCK), a waiting period must be inserted between two consecutive write cycles (please refer also to section 9.2.1).

In deep suspend mode_the device reaches the lowest possible power consumption. Only the interface section is kept alive. No data acquisition is performed and the content of the configuration registers is lost.

Deep suspend mode is entered by writing $0 \times 20$ to the register GYRO_LPM1. It can be left by writing 0x00 to GYRO_LPM1 or by a soft reset (see 4.8).

Please note, that all application specific settings, which are not equal to the default settings, must be reset to its designated values after leaving deep-suspend mode.

Note: after POR or soft-reset, or when switching between the different power modes, the gyroscope sensor needs up to 30 ms time to reach the new state. Any communication with the sensor during this time should be avoided.

### 4.2 Sensor Data

The width of the gyroscope and accelerometer sensor data is 16 bits (11 bits for the temperature sensor) given in two's complement representation.
The bits for each axis are split into an MSB upper part and an LSB lower part. Reading the sensor data registers shall always start with the LSB part. In order to ensure the integrity of the sensor data, the content of an MSB register is locked by reading the corresponding LSB register (shadowing procedure).

For details regarding the registers and the interpretation of the data found in these registers see:

- chapter 5.5.2 for the gyroscope part
- chapter 5.3.4 or the accelerometer part
- chapter 5.3.7 for the temperature sensor

The burst-access mechanism provides an efficient way to read out the angular rate data in $\mathrm{I}^{2} \mathrm{C}$ or SPI mode. During a burst-access, the sensor automatically increments the starting read address after each byte. The burst-access allows data to be transferred over the $\mathrm{I}^{2} \mathrm{C}$ bus with an up to $50 \%$ reduced data density. The sensor data (angular rate or acceleration data) in all read-out registers is locked as long as the burst read access is active. Reading the sensor data registers of each gyroscope and accelerometer part in burst read access mode ensures that the sensor values in all readout registers belong to the same sample.

### 4.3 Sensor Time

The accelerometer part of BMI088 has a built-in counter with a width of 24 bits. It increments periodically with a resolution of $39.0625 \mu \mathrm{~s}$. Details can be found in chapter 5.3.5.

### 4.4 Output Data Rate (ODR) and Low-pass Filter

The sensor signals from the acceleration sensor and gyroscope analog front-end are each routed through a low-pass filter.

### 4.4.1 Accelerometer

The 3db cut-off frequency of the digital low-pass filter depends on the chosen ODR as well as on the over-sampling-ratio (OSR). Both can be configured in register ACC_CONF. The following table lists the possible options:

Table 8: 3dB cutoff frequency of the accelerometer according to ODR and OSR settings in ACC_CONF register
| Accelerometer ODR [Hz] | Normal (acc_bwp $=0 \times A$ ) | OSR2 (acc_bwp = 0x9) | OSR4 (acc_bwp = 0x8) |
| :--- | :--- | :--- | :--- |
| 12.5 | 5 Hz | 2 Hz | 1 Hz |
| 25 | 10 Hz | 5 Hz | 3 Hz |
| 50 | 20 Hz | 9 Hz | 5 Hz |
| 100 | 40 Hz | 19 Hz | 10 Hz |
| 200 | 80 Hz | 38 Hz | 20 Hz |
| 400 | 145 Hz | 75 Hz | 40 Hz |
| 800 | 230 Hz ( 200 Hz for z channel) | 140 Hz | 80 Hz |
| 1600 | 280 Hz (245 Hz for z channel) | 234 Hz ( 215 Hz for z channel) | 145 Hz |


### 4.4.2 Gyroscope

The user can choose between 8 different ODR and low pass filter bandwidth settings (see section 5.5.6).

### 4.5 Range Settings

The measurement range can be set through the registers described in section 5.3.11 for the accelerometer and in section 5.5.5 for the gyroscope.

### 4.6 Self-test

The BMI088 incorporates a self-test feature for both the accelerometer and the gyroscope, indicating whether the sensor is still ok.

### 4.6.1 Accelerometer

The self-test feature allows for checking the sensor functionality by applying electrostatic forces to the sensor core instead of external accelerations. By physically deflecting the seismic mass, the entire signal path of the sensor is tested. Activation of the self-test results in a static offset in the acceleration data. Any external acceleration or gravitational force, which is applied to the sensor during a self-test, will be observed in the sensor output as a superposition of the acceleration and the self-test signal. This means that the self-test signal depends on the orientation of the sensor. To overcome this, the full self-test procedure should be performed under static circumstances, e.g. when the part is not excited to any acceleration except gravity.

The recommended self-test procedure is as follows:

1) Set $\pm 24 \mathrm{~g}$ range by writing $0 \times 03$ to register ACC_RANGE ( $0 \times 41$ )
2) Set ODR $=1.6 \mathrm{kHz}$, continuous sampling mode, "normal mode" (norm_avg4) by writing 0xA7 to register ACC_CONF ( $0 \times 40$ )

- Continuous filter function: set bit7 in ACC_CONF
- "normal avg4 mode": ACC_CONF |= 0x02<<4
- $\mathrm{ODR}=1.6 \mathrm{kHz}$ : $\mathrm{ACC} \_\mathrm{CONF} \mathrm{I}=0 \times 0 \mathrm{C}$

3) Wait for $>2 \mathrm{~ms}$
4) Enable the positive self-test polarity (i.e. write 0x0D to register ACC_SELF_TEST (0x6D))
5) Wait for $>50 \mathrm{~ms}$
6) Read the accelerometer offset values for each axis (positive self-test response)
7) Enable the negative self-test polarity (i.e. write $0 \times 09$ to register ACC_SELF_TEST (0x6D))
8) Wait for $>50 \mathrm{~ms}$
9) Read the accelerometer offset values for each axis (negative self-test response)
10) Disable the self-test (i.e. write $0 \times 00$ to register ACC_SELF_TEST (0x6D))
11) Calculate difference of positive and negative self-test response and compare with the expected values (see table below)
12) Wait for $>50 \mathrm{~ms}$ to let the sensor settle to normal mode steady state operation

Table 9: Accelerometer self-test: resulting minimum difference signal between positive and negative self-test signal
| x -axis signal | y -axis signal | z -axis signal |
| :---: | :---: | :---: |
| $\geq 1000 \mathrm{mg}$ | $\geq 1000 \mathrm{mg}$ | $\geq 500 \mathrm{mg}$ |


It is recommended to perform a reset of the device after a self-test has been performed, since the selftest response also affects the interrupt generation. If the reset cannot be performed, the following sequence must be kept to prevent unwanted interrupt generation: disable interrupts, change parameters of interrupts, wait for at least 50 ms , and enable desired interrupts.

### 4.6.2 Gyroscope

A built-in self-test facility of the gyro does not deflect the mechanical MEMS structure (as the accelerometer self-test does), but this test also provides a quick way to determine if the gyroscope is operational within the specified conditions.

To trigger the self-test, bit \#0 ('bite_trig') in address GYRO_SELF_TEST must be set. When the test is finished, bit \#1 ('bist_rdy') will be set by the gyro and the test result can then be found in bit \#2 ('bist_fail'). A ' 0 ' indicates that the test was passed without issues. If a failure occurred, the bit 'bist_fail' will be set to '1'.

A further test which is running continuously in the background can be checked by reading bit \#4 in address GYRO_SELF_TEST. Proper sensor function is indicated if the bit is set to ' 1 '.

### 4.7 New Data Interrupt

Both accelerometer and gyroscope part offer a new data ready interrupt, which fires whenever a new data sample set is complete and made available in the corresponding sensor data registers. This allows a low latency data readout.

### 4.7.1 Accelerometer

The new data interrupt flag can be found in the register ACC_INT_STAT_1 (bit \#7). It is set whenever new data is available in the data registers and cleared automatically.

The interrupt can be mapped to the interrupt pins INT1 and/or INT2 in register INT1_INT2_MAP_DATA.

Both interrupt pins INT1 and INT2 can be configured regarding their electrical behavior (see INT1_IO_CONF and INT2_IO_CONF).

### 4.7.2 Gyroscope

The gyroscope provides a new data interrupt, which will generate an interrupt every time after storing a new value of z-axis angular rate data in the data register. The interrupt is cleared automatically after $280-400 \mu \mathrm{~s}$.

In contrast to the accelerometer part, for the gyro the new data interrupt must be explicitly enabled by writing 0x80 to the register GYRO_INT_CTRL.

The interrupt can be mapped to the interrupt pins INT3 and/or INT4 in register INT3_INT4_IO_MAP.

Both interrupt pins INT3 and INT4 can be configured regarding their electrical behavior (see INT3_INT4_IO_CONF).

### 4.8 Soft-Reset

A soft-reset can be initiated at any time

- for the accelerometer part by writing the command soft-reset (0xB6) to register ACC_SOFTRESET (see 5.3.22)
- for the gyroscope part by writing the command soft-reset (0xB6) to register GYRO_SOFTRESET (see 5.5.8)

The soft-reset performs a fundamental reset to the device, which is largely equivalent to a power cycle. Following a delay, all user configuration settings are overwritten with their default state (setting stored in the NVM) wherever applicable. This command is functional in all operation modes but must not be performed while NVM writing operation is in progress.

### 4.8.1 Soft-Reset Accelerometer

## Soft reset timing:

In I2C mode, the rising edge of the SCL line for the ACK bit of the soft reset transfer determines the reception of soft reset. The execution of the soft rest is delayed by $0.5-1$ us if the sensor is not in advanced power save mode i.e. PWR_CONF.adv_power_save=0b0.
The execution of the soft reset causes that the SDA line to be released immediately by the device, independent of the state of the SCL line.

If the host requires an explicit I2C ACK for the soft reset (i.e. SDA line remains low while the SCL line is high) the host must put the sensor in advance power save mode (PWR_CONF.adv_power_save=0b1), disable the sensor (PWR_CTRL.acc_en=0b0) and the auxillary interface (PWR_CTRL.aux_en=0b0) before issuing a soft reset.

### 4.9 FIFO

BMI088 offers two integrated FIFO (First In, First Out) buffers for accelerometer and gyroscope sensor signals, helping the user to reduce or even omit time critical read access to the sensor in order to obtain data with a high timing precision.

### 4.9.1 FIFO operating modes

The FIFO can be operated in different modes: FIFO (or stop-at-full) mode and STREAM mode.

- FIFO or stop-at-full mode: In FIFO or stop-at-full mode, the sensor values are stored in the FIFO buffer subsequently until it is full.
- STREAM mode: The STREAM mode works like the FIFO mode with the difference that once the buffer is full, the oldest data in the FIFO will be overwritten with the newest data from the sensor.


### 4.9.2 FIFO interrupts

The FIFO buffers support two different types of interrupts:

- Watermark interrupt: Triggered, when the fill level of the FIFO buffer reaches a user-defined level.
- FIFO-full interrupt: Triggered, when the FIFO is full.


### 4.9.3 Accelerometer sensor FIFO buffer

The accelerometer part of BMI088 has an integrated 1024 byte data FIFO. The FIFO captures data from the data registers in frames, and each frame contains only one sample of a sensor.

### 4.9.3.1 Enabling FIFO and selecting the mode

The FIFO for accelerometer sensor data is enabled by setting bit \#6 in register 0x49 (see 5.3.15).

### 4.9.3.1.1 Mode selection

When STREAM mode is desired, then the bit \#0 in register 0x48 has to be cleared (set to '0') (default value on power up reset, see 5.3.14).

For FIFO or stop-at-full mode, bit \#0 has to be set to '1' in register 0x48.

### 4.9.3.1.2 FIFO data sampling rate

The input data rate to the FIFO is the same as the configured ODR of the sensor. However, it can be reduced selecting a down-sampling factor of $2^{\mathrm{k}}$ with $\mathrm{k}=[0,1, \ldots 7]$. The factor k must be written to bits \#4-6 of register 0x45 (see 5.3.12).
4.9.3.1.3 FIFO synchronization with external interrupts (tag application) for the accel

If the INT1 and/or INT2 pin is configured as input pin (by setting int2_io in register INT2_IO_CTRL and/or setting int1_io in register INT1_IO_CTRL), signals on these pins can also be recorded in the FIFO, and the frames are "tagged" accordingly. Therefore the pins need to be activated for FIFO recording in register FIFO_CONFIG_1 (see 5.3.15).

### 4.9.3.2 Data format in FIFO

The FIFO captures data in frames. The first byte is a header byte, defining the type of frame. From this, the number of consecutive bytes and their content can be derived.
The header byte consists of the header signature (first 6 bits) and two bits indicating the status of the interrupt pins INT1 and INT2 if configured accordingly (see 4.9.3.1.3).
4.9.3.2.1 Acceleration sensor data frame

- Frame length: 7 bytes ( 1 byte header +6 bytes payload)
- Header:

| Bit | $\mathbf{7}$ | $\mathbf{6}$ | $\mathbf{5}$ | $\mathbf{4}$ | $\mathbf{3}$ | $\mathbf{2}$ | $\mathbf{1}$ | $\mathbf{0}$ |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
|  | 1 | 0 | 0 | 0 | 0 | 1 | [INT2 tag] | [INT1 tag] |

- Payload: the next bytes contain the sensor data in the same order as defined in the register map (addresses $0 \times 12-0 \times 17$ ).


### 4.9.3.2.2 Skip Frame

In the case of FIFO overflows, in both FIFO and STREAM mode, a skip_frame is prepended to the FIFO content, when read out next time. A skip frame does not consume memory in the FIFO.

- Frame length: 2 bytes ( 1 byte header +1 byte payload)
- Header:

| Bit | $\mathbf{7}$ | $\mathbf{6}$ | $\mathbf{5}$ | $\mathbf{4}$ | $\mathbf{3}$ | $\mathbf{2}$ | $\mathbf{1}$ | $\mathbf{0}$ |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
|  | 0 | 1 | 0 | 0 | 0 | 0 | reserved | reserved |

- Payload: one byte containing the number of skipped frames. When more than 0xFF frames have been skipped, $0 \times F F$ is returned.


### 4.9.3.2.3 Sensortime Frame

A sensortime frame is only sent if the FIFO becomes empty during the burst read. A sensortime frame does not consume memory in the FIFO.

- Frame length: 4 bytes ( 1 byte header +3 bytes payload)
- Header:

| Bit | $\mathbf{7}$ | $\mathbf{6}$ | $\mathbf{5}$ | $\mathbf{4}$ | $\mathbf{3}$ | $\mathbf{2}$ | $\mathbf{1}$ | $\mathbf{0}$ |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
|  | 0 | 1 | 0 | 0 | 0 | 1 | reserved | reserved |

- Payload: Sensortime (content of registers $0 \times 18-0 \times 1 \mathrm{~A}$ ), taken when the last byte of the last frame is read.


### 4.9.3.2.4 FIFO input config Frame

Whenever the filter configuration or the range of the accelerometer sensor is changed, a FIFO input config frame is inserted into the FIFO, before the configuration change becomes active. E.g. when the bandwidth for the accelerometer filter is changed in Register ACC_CONF, a FIFO input config frame is inserted before the first frame with accelerometer data with the new bandwidth configuration.

- Frame length: 2 bytes ( 1 byte header +1 byte payload)
- Header:

| Bit | $\mathbf{7}$ | $\mathbf{6}$ | $\mathbf{5}$ | $\mathbf{4}$ | $\mathbf{3}$ | $\mathbf{2}$ | $\mathbf{1}$ | $\mathbf{0}$ |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
|  | 0 | 1 | 0 | 0 | 1 | 0 | reserved | reserved |

- Payload: The FIFO input config frame contains one byte of data, of which the following bits have a meaning (the content of the other bits can safely be ignored):
- Bit \#1: indicates that a configuration change through register ACC_RANGE becomes active (means for example that the range of the accelerometer was changed).
- Bit \#0: indicates that a configuration change through the registers ACC_CONF or FIFO_DOWNS becomes active (means of example that the filter settings where changed or the FIFO sampling rate was modified).


### 4.9.3.2.5 Sample drop Frame

After a reconfiguration, indicated by the fifo_Input_Config frame, the next sample may be dropped, until the sensor delivers valid data again. Instead, a drop frame is inserted at the ODR tick at which a sample was to be expected without reconfiguration.

- Frame length: 2 bytes ( 1 byte header +1 byte payload)
- Header:

| Bit | $\mathbf{7}$ | $\mathbf{6}$ | $\mathbf{5}$ | $\mathbf{4}$ | $\mathbf{3}$ | $\mathbf{2}$ | $\mathbf{1}$ | $\mathbf{0}$ |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
|  | 0 | 1 | 0 | 1 | 0 | 0 | reserved | reserved |

- Payload: The sample drop frame contains one byte of data, whose content can be ignored.


### 4.9.3.2.6 FIFO partial frame reads and overreads

When a frame is only partially (uncompletely) read through the register FIFO_DATA it will be repeated completely with the next access. In the case of a FIFO overflow between the first partial read and the second read attempt, the frame may be deleted.

When more data is read from the FIFO than it contains valid data, $0 \times 8000$ is returned.

### 4.9.3.3 FIFO Interrupts

The FIFO supports two interrupts, a FIFO full interrupt and a watermark interrupt:

- The FIFO full interrupt is issued when the FIFO fill level is above the full threshold. The full threshold is reached just before the last two frames are stored in the FIFO.
- The FIFO watermark is issued when the FIFO fill level is superior or equal to the watermark level defined in register FIFO_WTM (see 5.3.13).

In order to enable/use the FIFO full or watermark interrupts they need to be mapped on the desired interrupt pin via INT1_INT2_MAP_DATA (see 5.3.18).

Both interrupts are suppressed when a read operation on the register FIFO_DATA is ongoing. Latched FIFO interrupts will only get cleared, if the status register gets read and the fill level is below the corresponding FIFO interrupt (full or watermark).

### 4.9.3.4 FIFO Reset

The user can trigger a FIFO reset by writing 0xB0 to ACC_SOFTRESET (register 0x7E).

### 4.9.4 Gyroscope sensor FIFO buffer

The gyroscope part of BMI088 features an integrated FIFO memory capable of storing up to 100 frames of data in FIFO mode. Each frame consists of three 16-bit rate_x,y,z data words, and 16 bits of interrupt data sampled at the same point in time.

### 4.9.4.1 Enabling FIFO and selecting the mode

The FIFO for gyroscope sensor data is enabled by setting the appropriate FIFO mode in Register 0x3E: FIFO_CONFIG_1.

### 4.9.4.1.1 FIFO data sampling rate

The input data rate to the FIFO is the same as the configured ODR of the sensor.
4.9.4.1.2 FIFO sync with external interrupts (tag application) for the gyroscope

The FIFO of the gyroscope features a mode that allows the precise synchronization of external events with the gyroscope angular rate saved in the FIFO. This synchronization can be used for example for image and video stabilization applications.

Any of the gyroscope interrupt pins (INT3 or INT4) can be reconfigured to act as input pin, but not both. In addition, the tag mode has to be enabled. The so configured interrupt pin will then behave as an input pin and not as an interrupt pin. The working principle is shown in below figure:
![](https://cdn.mathpix.com/cropped/20159896-9fb0-48a0-8609-2aca341babdf-22.jpg?height=200&width=1461&top_left_y=1585&top_left_x=328)

Timing diagram for external FIFO synchronization. EFS-pin is the Interrupt pin configured to capture external events. FIFO $z(0)$ is the least significant bit of the $z$-axis gyro data stored in the FIFO.

In order to enable the tag mode, bit 5 must be set in register $0 \times 34$ (see 5.5.13). The pin can be chosen in the same register, bit 4.

In this mode, the least significant bit of the $z$-axis is used as tag-bit, therefore losing its meaning as gyroscope data bit. The remaining 15 bits of the $z$-axis gyroscope data keep the same meaning as in standard mode.

Once the pin, which is configured for the tag mode, is set to high level, the next FIFO word will be marked with a tag ( $z$-axis LSB $=1$ ). While pin is kept at a high level, the corresponding FIFO words will continuously be tagged. After the pin is reset to low level, the immediate next FIFO word could still be tagged, and only after this word, the next tag will be reset (z-axis LSB=0). This is shown in the above diagram.

The tag synchronizes external events with the same time precision as the FIFO update rate. Therefore update rate of the tag is determined by the output data rate.

### 4.9.4.2 FIFO Data Readout

The FIFO stores the data that are also available at the read-out registers $0 \times 02-0 \times 07$. Thus, all configuration settings apply to the FIFO data as well as the data readout registers. The FIFO read out is possible through register 0x3F (FIFO_DATA). The readout can be performed using burst mode. A single burst can read out one or more frames at a time. If a frame is not read completely due to an incomplete read operation, the remaining part of the frame is lost. In this case the FIFO aligns to the next frame during the next read operation.

The data format is described in 5.5.17.

### 4.9.4.2.1 Interface speed requirements for Gyroscope FIFO use

In order to use the FIFO effectively, larger blocks of data need to be read out quickly. Depending on the output data rate of the sensor, this can impose requirements on the interface.

The output data rate of the gyroscope is determined by the filter configuration (see the data sheet of the sensor). What interface speed is required depends on the selected rate.

- For an $\mathrm{I}^{2} \mathrm{C}$ speed of 400 kHz , every filter mode can be used.
- For an $\mathrm{I}^{2} \mathrm{C}$ speed of 200 kHz , only modes with an output data rate of 1 KHz and below are recommended.
- For an $\mathrm{I}^{2} \mathrm{C}$ speed of 100 kHz , only modes with an output data rate of 400 Hz and below are recommended.


### 4.9.4.3 FIFO Frame Counter and Overrun Flag

The frame counter (address $0 \times 0 \mathrm{E}$ bits<6:0>, see 5.5.4) indicates the current fill level of the buffer. If additional frames are written to the buffer although the FIFO is full, the overrun flag (register 0x0E bit 7) is set. If the FIFO is reset, the FIFO fill level indicated in the frame_counter<6:0> is set to ' 0 ' and the overrun flag is reset each time a write operation happens to the FIFO configuration registers.

Note: the overrun bit is not reset when the FIFO fill level frame_counter<6:0> has decremented to ' 0 ' due to reading from the FIFO_DATA register, but only when a write operation is performed on FIFO configuration registers.

### 4.9.4.4 FIFO Interrupts

The FIFO supports two interrupts, a FIFO full interrupt and a watermark interrupt:

- The FIFO full interrupt is issued when the buffer has been fully filled with samples. In FIFO mode this occurs after 100 samples, and in STREAM mode after 99 samples, have been stored in a previously empty FIFO.
The status of the FIFO-full interrupt may be read back through the status bit in INT_STATUS_1 register 0x0A.
- The watermark interrupt is issued when the fill level in the buffer has reached the frame number defined by the water mark level trigger in 0x3D. The status of the watermark may be read back through the address $0 \times 0 \mathrm{~A}$ bit 4 (fifo_int) status bit. Writing to water mark level trigger in register 0x3D clears the FIFO buffer.


## 5. Register Maps

### 5.1 Communication with the sensor

The entire communication with the device is performed by reading from and writing to registers. Registers have a width of 8 bits; they are mapped to an 8 -bit address space. Accelerometer and gyroscope have individual register maps. The selection of the appropriate register map is done on digital interface level by either selecting the corresponding chip select pin (SPI mode) or $I^{2} \mathrm{C}$ address (I ${ }^{2} \mathrm{C}$ mode). For details regarding the digital interface, see chapter 0 .

The functional registers and the register addresses containing functional bits are marked in the following register maps. All non-functional registers are marked as reserved and should be completely ignored by the user.

It is recommended to mask out (logical and with zero) non-functional bits (marked with '-') of registers which partially contain functional bits (i.e. read the register content first, changing bit by means of bitwise operations, and write the modified byte back to the register).

### 5.2 Register Map: Accelerometer

| Legend |  |  | Read-only |  | Read/Write |  | Write-only |  | Reserved |  |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| Addr | Name | Reset value | bit7 | bit6 | bit5 | bit4 | bit3 | bit2 | bit1 | bit0 |
| 0x7E | ACC_SOFTRESET | 0x00 | softreset_cmd (0xb6) |  |  |  |  |  |  |  |
| 0x7D | ACC_PWR_CTRL | 0x00 | acc_enable |  |  |  |  |  |  |  |
| 0x7C | ACC PWR CONF | 0x03 | pwr_save_mode |  |  |  |  |  |  |  |
| 0x7B - 0x6E: reserved |  |  | - |  |  |  |  |  |  |  |
| 0x6D | ACC SELF TEST | 0x00 | acc_self_test |  |  |  |  |  |  |  |
| 0x6B-0x59: reserved |  |  |  |  |  |  |  |  |  |  |
| $0 \times 58$ | INT_MAP_DATA | $0 \times 00$ | - | int2 drdy | int2 fwm | int2 ffull | - | int1_drdy | Int1 fwm | Int1 ffull |
| 0x57-0x55: reserved |  |  |  |  |  |  |  |  |  |  |
| 0x54 | INT2_IO_CTRL | 0x00 | - |  |  | int2 in | int2 out | int2 od | int2 _lvl | - |
| 0x53 | INT1 IO CTRL | 0x00 | - |  |  | int1 _in | int1 out | int1 od | int1 _\|vl | - |
|  | 0x52-0x4A: reserved |  |  |  |  |  |  |  |  |  |
| 0x49 | FIFO_CONFIG_1 | $0 \times 10$ | - | acc_en | - | 1 | int1_en | int2_en | - |  |
| $0 \times 48$ | FIFO_CONFIG_0 | 0x02 | - |  |  |  |  |  | 1 | mode |
| $0 \times 47$ | FIFO_WTM_1 | 0x02 | - |  |  | fifo_water_mark[1 2:8] |  |  |  |  |
| 0x46 | FIFO_WTM_0 | $0 \times 00$ | fifo_water_mark[7:0] |  |  |  |  |  |  |  |
| 0x45 | FIFO_DOWNS | 0x80 | 1 | fifo_downs |  | - |  |  |  |  |
| 0x44-0x42: reserved |  |  | - |  |  |  |  |  |  |  |
| 0x41 | ACC_RANGE | 0x01 | - |  |  |  |  |  | acc_range |  |
| 0x40 | ACC_CONF | 0xA8 | acc_bwp |  |  |  | acc_odr |  |  |  |
| 0x3F-0x27: reserved |  |  | - |  |  |  |  |  |  |  |
| 0x26 | FIFO DATA | 0x00 | fifo_data |  |  |  |  |  |  |  |
| 0x25 | FIFO LENGTH 1 | 0x00 | - |  | fifo_byte_counter[13:8] |  |  |  |  |  |
| 0x24 | FIFO LENGTH 0 | 0x00 | fifo_byte_counter[7:0] |  |  |  |  |  |  |  |
| 0x23 | TEMP_LSB | 0x00 | temperature[2:0] |  |  | - |  |  |  |  |
| 0x22 | TEMP_MSB | 0x00 | temperature[10:3] |  |  |  |  |  |  |  |
| 0x21-0x1E: reserved |  |  | - |  |  |  |  |  |  |  |
| $0 \times 1 \mathrm{D}$ | ACC_INT_STAT_1 | 0x00 | acc_drdy | - |  |  |  |  |  |  |
| 0x1C-0x1B: reserved |  |  | - |  |  |  |  |  |  |  |
| $0 \times 1 \mathrm{~A}$ | SENSORTIME 2 | 0x00 | sensor_time[23:16] |  |  |  |  |  |  |  |
| 0x19 | SENSORTIME 1 | 0x00 | sensor_time[15:8] |  |  |  |  |  |  |  |
| 0x18 | SENSORTIME 0 | 0x00 | sensor_time[7:0] |  |  |  |  |  |  |  |
| 0x17 | ACC_Z_MSB | 0x00 | acc_z[15:8] |  |  |  |  |  |  |  |
| $0 \times 16$ | ACC_Z_LSB | 0x00 | acc_z[7:0] |  |  |  |  |  |  |  |
| 0x15 | ACC Y MSB | 0x00 | acc_y15:8] |  |  |  |  |  |  |  |
| $0 \times 14$ | ACC Y LSB | 0x00 | acc_47:0] |  |  |  |  |  |  |  |
| 0x13 | ACC_X_MSB | 0x00 | acc_x15:8] |  |  |  |  |  |  |  |
| 0x12 | ACC X LSB | 0x00 | acc_47:0] |  |  |  |  |  |  |  |
| 0x11-0x04: reserved |  |  | - |  |  |  |  |  |  |  |
| 0x03 | ACC_STATUS | 0x10 | drdy_acc | - |  |  |  |  |  |  |
| 0x02 | ACC_ERR_REG | 0x00 | - |  |  | error_code |  |  | - | fatal_err |
| 0x01 | - | - | - |  |  |  |  |  |  |  |
| 0x00 | ACC_CHIP_ID | 0x1E | acc_chip_id |  |  |  |  |  |  |  |

### 5.3 Register Description: Accelerometer

### 5.3.1 Register 0x00: ACC_CHIP_ID

| Bit | Access | Reset <br> value | Description |
| :--- | :--- | :--- | :--- |
| $[7: 0]$ | RO | $0 \times 1 \mathrm{E}$ | Contains identifier code of acceleration sensor |

### 5.3.2 Register 0x02: ACC_ERR_REG

Reports sensor error conditions.

| Bit | Name | Access | Reset value | Description |
| :--- | :--- | :--- | :--- | :--- |
| [7:5] | reserved |  |  |  |
| [4:2] | error_code | RO | 0x00 | Error codes for persistent errors: <br> 0x00: no error <br> 0x01: error occurred in accelerometer configuration (unvalid data in register ACC_CONF) |
| [1] | reserved |  |  |  |
| [0] | fatal_err | RO | $0 \times 0$ | Fatal Error, chip is not in operational state (Boot-, power-system). This flag will be reset only by power-on-reset or soft-reset. |

### 5.3.3 Register 0x03: ACC_STATUS

Sensor status flag.

| Bit | Name | Access | Reset value | Description |
| :--- | :--- | :--- | :--- | :--- |
| [7] | acc_drdy | RO | 0x0 | Data ready for Accelerometer. Reset when one acceleration data register is read out. |
| [6:0] | reserved |  |  |  |

### 5.3.4 Register 0x12-0x17: ACC data

Registers containing the acceleration sensor output. The sensor output is stored as signed 16 bit number in 2's complement format in each 2 registers. From the registers, the acceleration values can be calculated as follows:
Accel_X_int16 $=$ ACC_X_MSB ${ }^{*} 256+$ ACC_X_LSB
Accel_Y_int16 $=$ ACC_Y_MSB ${ }^{*} 256+$ ACC_Y_LSB
Accel_Z_int16 $=$ ACC_Z_MSB ${ }^{*} 256+$ ACC_Z_LSB

When a register is read containing the LSB value of an acceleration value, the corresponding MSB register is locked internally, until it is read. By this mechanism, it is ensured that both LSB and MSB values belong to the same acceleration value and are not updated between the readouts of the individual registers.

The unit is in LSB. The conversion from LSB to acceleration ( mg ) is based on the range settings and can be calculated as follows (<0x41>: content of the ACC_RANGE register):
Accel_X_in_mg $=$ Accel_X_int16 $/ 32768$ * 1000 * $2^{\wedge}(<0 \times 41>+1)$ * 1.5
Accel_Y_in_mg $=$ Accel_Y_int16 $/ 32768$ * 1000 * $2^{\wedge}(<0 \times 41>+1)$ * 1.5
Accel_Z_in_mg $=$ Accel_Z_int16 $/ 32768$ * 1000 * $2^{\wedge}(<0 \times 41>+1)$ * 1.5

### 5.3.5 Register $0 \times 18-0 \times 1 \mathrm{~A}$ : Sensortime data

Registers containing the value of the internal 24-bit counter.

- Register $0 \times 18$ (SENSORTIME_0) contains the lower 8 bits of the counter. This register is incremented every $39.0625 \mu \mathrm{~s}$.
- Register 0x19 (SENSORTIME_1) contains the middle 8 bits of the counter. This register is incremented on SENSORTIME_0 overflow, which is every 10 ms .
- Register 0x1A (SENSORTIME_2) contains the higher 8 bits of the counter. This register is incremented on SENSORTIME_1 overflow, which is every 2.56 s .

The complete 24-bit counter overflows after 655.36 s or almost 11 minutes.

### 5.3.6 Register 0x1 D: ACC_INT_STAT_1

Interrupt status register.

| Bit | Name | Access | Reset value | Description |
| :--- | :--- | :--- | :--- | :--- |
| [7] | acc_drdy | RO | $0 \times 00$ | Acceleration data ready interrupt. Cleared on read of this register |
| [6:0] | reserved |  |  |  |

### 5.3.7 Register $0 \times 22-0 \times 23$ : Temperature sensor data

Registers containing the temperature sensor data output. The data is stored in an 11-bit value in 2's complement format. The resolution is $0.125^{\circ} \mathrm{C} / \mathrm{LSB}$, thus the temperature can be obtained as follows:

```
Temp_uint11 = (TEMP_MSB * 8) + (TEMP_LSB / 32)
if Temp_uint11 > 1023:
    Temp_int11 = Temp_uint11 - 2048
else:
    Temp_int11 = Temp_uint11
Temperature = Temp_int11 * 0,125 c/LSB + 23 }\mp@subsup{}{}{\circ}\textrm{C
```

| TEMP_MSB | TEMP_LSB | Temp_int11 | Temperature |
| :--- | :--- | :--- | :--- |
| 0x3E | 0x00 | 496 | $85^{\circ} \mathrm{C}$ |
| ... | ... | ... | ... |
| 0x00 | $0 \times 60$ | 3 | $23.375^{\circ} \mathrm{C}$ |
| 0x00 | $0 \times 40$ | 2 | $23.250^{\circ} \mathrm{C}$ |
| 0x00 | $0 \times 20$ | 1 | $23.125^{\circ} \mathrm{C}$ |
| 0x00 | 0x00 | 0 | $23.0^{\circ} \mathrm{C}$ |
| ... | ⋯ | ... | ... |
| 0xC1 | $0 \times 00$ | -504 | $-40^{\circ} \mathrm{C}$ |
| 0x80 |  |  | Invalid |

The temperature sensor data is updated every 1.28 s .

### 5.3.8 Register $0 \times 24-0 \times 25$ : FIFO_LENGTH

The FIFO length registers FIFO_LENGTH_1 and FIFO_LENGTH_0 contain the 14 bit FIFO byte counter. The counter represents the current fill level of the FIFO buffer.
An empty FIFO corresponds to $0 \times 8000$. A FIFO content reset can be triggered by reading out all frames from the FIFO buffer or by writing 0xB0 into register 0x7E. The byte counter is updated when a complete frame is read or written.

### 5.3.9 Register 0x26: FIFO_DATA

When reading out data from FIFO, burst read access must be used. The address will not increase when burst read at the address of FIFO_DATA. The FIFO data is organized in frames as described in section 4.9.3.2.
When a frame is partially read through FIFO Data Register 0x26, it will be repeated completely with the next access. However, in case of a FIFO overflow between the first partial read and the second read attempt, the frame may be deleted.

### 5.3.10 Register 0x40: ACC_CONF

Accelerometer configuration register.

| Bit | Name | Access | Reset value | Description |
| :--- | :--- | :--- | :--- | :--- |
| [7:4] | acc_bwp | RW | 0x0A | This parameter influences the bandwidth of the accelerometer low pass filter. For details, see section 4.4.1. Possible values:acc_bwp Filter setting <br> $0 \times 08$ OSR4 (4-fold oversampling) <br> $0 \times 09$ OSR2 (2-fold oversampling) <br> $0 \times 0 \mathrm{~A}$ Normal <br> others reserved |
| [3:0] | acc_odr | RW | $0 \times 08$ | This parameter sets the output data rate ODR. Possible values: ![](https://cdn.mathpix.com/cropped/20159896-9fb0-48a0-8609-2aca341babdf-29.jpg?height=724&width=724&top_left_y=1242&top_left_x=1028) |

### 5.3.11 Register 0x41: ACC_RANGE

Accelerometer range setting register.

| Bit | Name | Access | Reset value | Description |
| :--- | :--- | :--- | :--- | :--- |
| [7:2] | reserved |  |  |  |
| [1:0] | acc_range | RW | $0 \times 01$ | This parameter sets the measurement range. Possible values:acc_range Range setting <br> $0 \times 00$ $\pm 3 \mathrm{~g}$ <br> $0 \times 01$ $\pm 6 \mathrm{~g}$ <br> $0 \times 02$ $\pm 12 \mathrm{~g}$ <br> $0 \times 03$ $\pm 24 \mathrm{~g}$ |

### 5.3.12 Register 0x45: FIFO_DOWNS

Reduction of sample rate.

| Bit | Name | Access | Reset value | Description |
| :--- | :--- | :--- | :--- | :--- |
| [7] | Reserved | RW | 0x01 | This bit must always be '1'. |
| [6:4] | fifo_downs | RO | $0 \times 00$ | Reduction of sample rate by a factor 2**fifo_downs. <br> Example: fifo_downs=5 will reduce the FIFO sampling rate by a factor of $2^{* *} 5=32$ in relation to the chosen ODR of the sensor signal. |
| [3:0] | reserved |  |  |  |

### 5.3.13 Register 0x46-0x47: FIFO_WTM

Registers containing the 13 bit FIFO watermark level value. A FIFO water mark interrupt signal is active if the FIFO fill level is equal or greater than fifo_water_mark [12:0] (unit of the fifo water mark is one byte).

### 5.3.14 Register 0x48: FIFO_CONFIG_0

Sets the FIFO mode.

| Bit | Name | Access | Reset value | Description |
| :--- | :--- | :--- | :--- | :--- |
| [7:2] | reserved |  |  |  |
| [1] | Reserved | $R W$ | 0x01 | This bit must always be '1'. |
| [0] | mode | RW | 0x00 | This parameter sets the FIFO mode. Possible values:value mode <br> $0 \times 00$ STREAM mode <br> $0 \times 01$ FIFO mode |

### 5.3.15 Register 0x49: FIFO_CONFIG_1

Selects sources for the FIFO buffer.

| Bit | Name | Access | Reset value | Description |
| :--- | :--- | :--- | :--- | :--- |
| [7] | reserved |  |  |  |
| [6] | Acc_en | RW | 0x00 | Enables storing of accelerometer sensor data |
| [5] | Reserved | RW | 0x00 |  |
| [4] | Reserved | $R W$ | 0x01 | This bit must always be '1'. |
| [3] | Int1_en | RW | 0x00 | Enables storing of captured interrupt events at pin INT1 (pin needs to be configured as input pin accordingly) |
| [2] | Int2_en | RW | 0x00 | Enables storing of captured interrupt events at pin INT2 (pin needs to be configured as input pin accordingly) |
| [1:0] | reserved |  |  |  |

### 5.3.16 Register 0x53: INT1_IO_CONF

Configures the input/output pin INT1.

| Bit | Name | Access | Reset value | Description |
| :--- | :--- | :--- | :--- | :--- |
| [7:5] | reserved |  |  |  |
| [4] | int1_in | RW | $0 \times 00$ | Enable INT1 as input pin. |
| [3] | int1_out | RW | 0x00 | Enable INT1 as output pin. |
| [2] | int1_od | RW | 0x00 | int1_od Pin behavior <br> $0 \times 00$ Push-pull <br> $0 \times 01$ Open-drain |
| [1] | int1_Ivl | RW | 0x00 | int1_lvl Active state <br> $0 \times 00$ Active low <br> $0 \times 01$ Active high |
| [0] | reserved |  |  |  |

### 5.3.17 Register 0x54: INT2_IO_CONF

Configures the input/output pin INT2.

| Bit | Name | Access | Reset value | Description |
| :--- | :--- | :--- | :--- | :--- |
| [7:5] | reserved |  |  |  |
| [4] | int2_io | RW | 0x00 | Enable INT2 as input pin. |
| [3] | int2_out | RW | 0x00 | Enable INT2 as output pin. |
| [2] | int2_od | RW | 0x00 | int2_od Pin behavior <br> $0 \times 00$ Push-pull <br> $0 \times 01$ Open-drain |
| [1] | int2_Ivl | RW | $0 \times 00$ | int2_Ivl Active state <br> $0 \times 00$ Active low <br> $0 \times 01$ Active high |
| [0] | reserved |  |  |  |

### 5.3.18 Register 0x58: INT1_INT2_MAP_DATA

Map data ready interrupt to output pin INT1 and/or INT2.

| Bit | Name | Access | Reset value | Description |
| :--- | :--- | :--- | :--- | :--- |
| [7] | reserved |  |  |  |
| [6] | Int2_drdy | RW | 0x00 | Map data ready interrupt to pin INT2 |
| [5] | int2_fwm | RW | 0x00 | Map FIFO watermark interrupt to pin INT2 |
| [4] | int2_ffull | RW | 0x00 | Map FIFO full interrupt to pin INT2 |
| [5:3] | reserved |  |  |  |
| [2] | Int1_drdy | RW | 0x00 | Map data ready interrupt to pin INT1 |
| [1] | int1_fwm | RW | 0x00 | Map FIFO watermark interrupt to pin INT1 |
| [0] | int1_ffull | RW | $0 \times 00$ | Map FIFO full interrupt to pin INT1 |

### 5.3.19 Register 0x6D: ACC_SELF_TEST

Enables the sensor self-test signal, occurring as a steady offset to the sensor output. Note that the self-test needs to be switched off actively by the user (details see 4.6.1).

| Bit | Access | Reset value | Description |  |
| :--- | :--- | :--- | :--- | :--- |
| [7:0] | RW | 0x00 | □ <br> self_test <br> 0x00 <br> Self-test is switched off. <br> Enable positive self-test signal. <br> Enable negative self-test signal. |  |
|  |  |  |  |  |
|  |  |  |  |  |
|  |  |  |  |  |
|  |  |  |  |  |

### 5.3.20 Register 0x7C: ACC_PWR_CONF

Switches accelerometer into suspend mode for saving power. In this mode the data acquisition is stopped.

| Bit | Name | Access | Reset value | Description |
| :--- | :--- | :--- | :--- | :--- |
| [7:0] | acc_pwr_save | RW | $0 \times 03$ | acc_pwr_save Filter setting <br> $0 \times 03$ Suspend mode <br> $0 \times 00$ Active mode |

### 5.3.21 Register 0x7D: ACC_PWR_CTRL

Switches accelerometer ON or OFF. Required to do after every reset in order to obtain acceleration values.

| Bit | Name | Access | Reset value | Description |  |
| :--- | :--- | :--- | :--- | :--- | :--- |
| [7:0] | acc_enable | RW | $0 \times 00$ | acc_enable Filter setting <br> $0 \times 00$ Accelerometer off <br> $0 \times 04$ Accelerometer on |  |

### 5.3.22 Register 0x7E: ACC_SOFTRESET

| Bit | Access | Reset value | Description |
| :--- | :--- | :--- | :--- |
| [7:0] | W | N/A | Writing a value of 0xB6 to this register resets the sensor. Following a delay of 1 ms , all configuration settings are overwritten with their reset value. <br> The soft-reset can be triggered from any operation mode. |

### 5.4 Register Map: Gyroscope

| Legend |  |  | Read-only |  | Read/Write |  | Write-only |  | Reserved |  |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| Addr | Name | Reset value | bit7 | bit6 | bit5 | bit4 | bit3 | bit2 | bit1 | bit0 |
| $0 \times 3 \mathrm{~F}$ | FIFO_DATA | N/A | fifo_data_output_register |  |  |  |  |  |  |  |
| 0x3E | FIFO_CONFIG_1 | 0x00 | fifo_mode | - |  |  |  |  |  |  |
| 0x3D | FIFO_CONFIG_0 | 0x00 |  | fifo_water_mark_level_trigger_retain |  |  |  |  |  |  |
| 0x3C | GYRO_SELF_TEST | N/A | - |  |  | rate_ok | - | bist_fail | bist_rdy | trig_bist |
| 0x3B-0x35: reserved |  |  |  |  |  |  |  |  |  |  |
| 0x34 | FIFO_EXT_INT_S | 0x00 |  |  | ext_fifo_s_en | ext_fifo_s_sel |  |  |  |  |
| 0x33-0x1F: reserved |  |  |  |  |  |  |  |  |  |  |
| $0 \times 1 \mathrm{E}$ | FIFO_WM_EN | 0x00 | fifo_watermark_enable |  |  |  |  |  |  |  |
| 0x1D-0x19: reserved |  |  | - |  |  |  |  |  |  |  |
| 0x18 | INT3_INT4_IO_MAP | 0x00 | Int4_data | - | Int4_fifo |  | - | Int3_fifo | - | Int3_data |
| 0x17: reserved |  |  |  |  |  |  |  |  |  |  |
| $0 \times 16$ | INT3_INT4_IO_CONF | 0x0F | - |  |  |  | Int4_od | Int4_IV | Int3_od | Int3_IM |
| $0 \times 15$ | GYRO_INT_CTRL | 0x00 | data_en | fifo_en | - |  |  |  |  |  |
| 0x14 | GYRO_SOFTRESET | N/A | softreset |  |  |  |  |  |  |  |
| 0x13-0x12: reserved |  |  | - |  |  |  |  |  |  |  |
| $0 \times 11$ | GYRO_LPM1 | 0x00 | gyro_pm |  |  |  |  |  |  |  |
| 0x10 | GYRO_BANDWIDTH | 0x80 | gyro_bw |  |  |  |  |  |  |  |
| 0x0F | GYRO_RANGE | 0x00 | gyro_range |  |  |  |  |  |  |  |
| 0x0E | FIFO_STATUS | N/A | fifo_overrun | fifo_frame_counter |  |  |  |  |  |  |
| 0x0D-0x0B: reserved |  |  |  |  |  |  |  |  |  |  |
| 0x0A | GYRO_INT_STAT_1 | N/A | gyro_drdy | - |  | fifo_int |  |  |  |  |
| 0x09-0x08: reserved |  |  | - |  |  |  |  |  |  |  |
| $0 \times 07$ | RATE_Z_MSB | N/A | rate_z[15:8] |  |  |  |  |  |  |  |
| $0 \times 06$ | RATE_Z_LSB | N/A | rate_z[7:0] |  |  |  |  |  |  |  |
| 0x05 | RATE_Y_MSB | N/A | rate_y[15:8] |  |  |  |  |  |  |  |
| 0x04 | RATE_Y_LSB | N/A | rate_y[7:0] |  |  |  |  |  |  |  |
| 0x03 | RATE_X_MSB | N/A | rate_x[15:8] |  |  |  |  |  |  |  |
| $0 \times 02$ | RATE_X_LSB | N/A | rate_x[7:0] |  |  |  |  |  |  |  |
| 0x01 | Reserved | N/A | - |  |  |  |  |  |  |  |
| 0x00 | GYRO_CHIP_ID | 0x0F | gyro_chip_id |  |  |  |  |  |  |  |

### 5.5 Register Description: Gyroscope

5.5.1 Register 0x00: GYRO_CHIP_ID

| Bit | Access | Reset <br> value | Description |
| :--- | :--- | :--- | :--- |
| $[7: 0]$ | RO | 0x0F | Contains identifier code of gyroscope. |

### 5.5.2 Register 0x02-0x07: Rate data

Registers containing the angular velocity sensor output. The sensor output is stored as signed 16-bit number in 2's complement format in each 2 registers. From the registers, the gyro values can be calculated as follows:

Rate_X: RATE_X_MSB * 256 + RATE_X_LSB
Rate_Y: RATE_Y_MSB * 256 + RATE_Y_LSB
Rate_Z: RATE_Z_MSB * 256 + RATE_Z_LSB

When a register is read containing the LSB value of a rate value, the corresponding MSB register is locked internally, until it is read. By this mechanism, it is ensured that both LSB and MSB values belong to the same rate range value and are not updated between the readouts of the individual registers.

The unit is in LSB. The conversion from LSB to angular velocity (degree per second) is based on the range settings (see 5.5.5). For example, for the default range setting of 0x00 in register 0x0F, the following conversion table applies:

| Sensor output [LSB] | Angular rate (in $2000 \boldsymbol{\%} \boldsymbol{/} \mathbf{s}$ range mode) |
| :--- | :--- |
| +32767 | $+2000 \%$ |
| ... | ... |
| 0 | $0 \% \mathrm{~s}$ |
| ... | … |
| -32767 | $-2000^{\circ} / \mathrm{s}$ |

### 5.5.3 Register 0x0A: GYRO_INT_STAT_1

| Bit | Name | Access | Reset value | Description |
| :--- | :--- | :--- | :--- | :--- |
| [7] | gyro_drdy | RO | N/A | Data ready interrupt status. The interrupt is cleared automatically after $280-400 \mu \mathrm{~s}$. |
| [6:5] | reserved |  |  |  |
| [4] | fifo_int | RO | N/A | FIFO interrupt status |
| [3:0] | reserved |  |  |  |

### 5.5.4 Register 0x0E: FIFO_STATUS

The register contains FIFO status information.

| Bit | Name | Access | Reset value | Description |
| :--- | :--- | :--- | :--- | :--- |
| [7] | Fifo_overrun | RO | N/A | If set, FIFO overrun condition has occurred. Note: flag can only be cleared by writing to the FIFO configuration register FIFO_CONFIG_1 |
| [6:0] | Fifo_frame_counter | RO | N/A | Current fill level of FIFO buffer. An empty FIFO corresponds to 0x00. The frame counter can be cleared by reading out all frames from the FIFO buffer or writing to the FIFO configuration register FIFO_CONFIG_1. |

### 5.5.5 Register 0x0F: GYRO_RANGE

| Bit | Access | Reset value | Description |  |  |
| :--- | :--- | :--- | :--- | :--- | :--- |
| [7:0] | RW | 0x00 | Angular rate range and resolution. Possible values: |  |  |
|  |  |  | □ <br> gyro_range | Full scale [ s ] | Resolution |
|  |  |  | $0 \times 00$ | $\pm 2000$ | 16.384 LSB/ ${ }^{\circ} / \mathrm{s} \Leftrightarrow 61.0 \mathrm{~m} / \mathrm{s} / \mathrm{LSB}$ |
|  |  |  | $0 \times 01$ | $\pm 1000$ | $32.768 \mathrm{LSB} /{ }^{\circ} / \mathrm{s} \Leftrightarrow 30.5 \mathrm{~m} / \mathrm{s} / \mathrm{LSB}$ |
|  |  |  | $0 \times 02$ | $\pm 500$ | 65.536 LSB/ ${ }^{\circ} / \mathrm{s} \Leftrightarrow 15.3 \mathrm{~m} / \mathrm{s} / \mathrm{LSB}$ |
|  |  |  | $0 \times 03$ | $\pm 250$ | $131.072 \mathrm{LSB} /{ }^{\circ} / \mathrm{s} \Leftrightarrow 7.6 \mathrm{~m}^{\circ} / \mathrm{s} / \mathrm{LSB}$ |
|  |  |  | $0 \times 04$ | $\pm 125$ | 262.144 LSB/ ${ }^{\circ} / \mathrm{s} \Leftrightarrow 3.8 \mathrm{~m} / \mathrm{s} / \mathrm{LSB}$ |

### 5.5.6 Register 0x10: GYRO_BANDWIDTH

| Bit | Access | Reset value | Description |
| :--- | :--- | :--- | :--- |
| [7:0] | RW | $0 \times 80^{1}$ | The register allows the selection of the rate data filter bandwidth and output data rate (ODR). Possible values: ![](https://cdn.mathpix.com/cropped/20159896-9fb0-48a0-8609-2aca341babdf-39.jpg?height=589&width=1108&top_left_y=1691&top_left_x=718) |

[^0]
### 5.5.7 Register 0x11: GYRO_LPM1

Selection of the main power modes. Please note that only switching between normal mode and the suspend modes is allowed, it is not possible to switch between suspend and deep suspend and vice versa.

| Bit | Access | Reset value | Description |
| :--- | :--- | :--- | :--- |
| [7:0] | RW | 0x00 | Switch to the main power modes.gyro_pm Power mode <br> $0 \times 00$ normal <br> $0 \times 80$ suspend <br> $0 \times 20$ deep suspend |

### 5.5.8 Register 0x14: GYRO_SOFTRESET

| Bit | Access | Reset value | Description |
| :--- | :--- | :--- | :--- |
| [7:0] | W | N/A | Writing a value of 0xB6 to this register resets the sensor. (Other values are ignored.) <br> Following a delay of 30 ms , all configuration settings are overwritten with their reset value. <br> The soft reset can be triggered from any operation mode. |

### 5.5.9 Register 0x15: GYRO_INT_CTRL

| Bit | Access | Reset value | Description |
| :--- | :--- | :--- | :--- |
| [7] | RW | $0 \times 0$ | Enables the new data interrupt to be triggered on new data. |
| [6] | RW | $0 \times 0$ | Enables the FIFO interrupt. |
| [5:0] | reserved |  |  |

### 5.5.10 Register 0x16: INT3_INT4_IO_CONF

Sets electrical and logical properties of the interrupt pins.

| Bit | Name | Access | Reset value | Description |
| :--- | :--- | :--- | :--- | :--- |
| [3] | Int4_od | RW | '1' | Int4_od Pin INT4 output configuration <br> '0' Push-pull <br> '1' Open-drain |
| [2] | Int4_Ivl | RW | '1' | Int4_lvl Pin INT4 active state <br> '0' Active low <br> '1' Active high |
| [1] | Int3_od | RW | '1' | Int3_od Pin INT3 output configuration <br> '0' Push-pull <br> '1' Open-drain |
| [0] | Int3_Ivl | RW | '1' | Int3_Ivl Pin INT3 active state <br> '0' Active low <br> '1' Active high |

### 5.5.11 Register 0x18: INT3_INT4_IO_MAP

Map the data ready interrupt pin to one of the interrupt pins INT3 and/or INT4.

| Bit | Access | Reset value | Description |
| :--- | :--- | :--- | :--- |
| [7] | RW | $0 \times 0$ | Data ready interrupt is mapped to INT4 pin. |
| [6] | reserved |  |  |
| [5] | RW | 0x0 | FIFO interrupt is mapped to INT4. |
| [4:3] | reserved |  |  |
| [2] | RW | $0 \times 0$ | FIFO interrupt is mapped to INT3. |
| [1] | reserved |  |  |
| [0] | RW | 0x0 | Data ready interrupt is mapped to INT3 pin. |

### 5.5.12 Register 0x1 E: FIFO_WM_ENABLE

Enables FIFO watermark level interrupt.

| Bit | Access | Reset value | Description |
| :--- | :--- | :--- | :--- |
| [7:0] | RW | 0x08 | Value description <br> $0 \times 08$ FIFO watermark level interrupt disabled <br> $0 \times 88$ FIFO watermark level interrupt enabled |


5.5.13 Register 0x34: FIFO_EXT_INT_S
| Bit | Access | Reset value | Description |
| :--- | :--- | :--- | :--- |
| [7:6] | reserved |  |  |
| [5] | RW | 0x00 | It set, enables external FIFO synchronization mode |
| [4] | RW | 0x00 | Selects source for external FIFO synchronizationext_fifo Behavior <br> _s_sel  <br> $0 \times 0$ Source is pin INT3 <br> $0 \times 1$ Source is pin INT4 |
| [3:0] | reserved |  |  |


### 5.5.14 Register 0x3C: GYRO_SELF_TEST

Built-in self-test of gyroscope.

| Bit | Access | Name | Reset value | Description |
| :--- | :--- | :--- | :--- | :--- |
| [4] | R | rate_ok | '0' | A value of '1' indicates proper sensor function. |
| [2] | R | bist_fail | '0' | If ' 0 ' and bist_rdy = ' 1 ': built-in self-test is ok, sensor is ok <br> If ' 1 ' and bist_rdy = ' 1 ': built-in self-test is not ok, sensor values may not be in expected range |
| [1] | R | bist_rdy | '0' | If bit is ' 1 ', built-in self-test has been performed and finished |
| [0] | W | trig_bist | N/A | Setting this bit to '1' (i.e. writing 0x01 to this register) starts the built-in self-test. |


5.5.15 Register 0x3D: FIFO_CONFIG_0
| Bit | Access | Reset value | Description |
| :--- | :--- | :--- | :--- |
| [7] | Reserved |  |  |
| [6:0] | RW | 0x00 | fifo_water_mark_level_trigger_retain<6:0> defines the FIFO watermark level. An interrupt will be generated, when the number of entries in the FIFO exceeds <br> fifo_water_mark_level_trigger_retain<6:0>. <br> Writing to this register clears the FIFO buffer. |


### 5.5.16 Register 0x3E: FIFO_CONFIG_1

Contains FIFO configuration settings. The FIFO buffer memory is cleared and the fifo-full flag is cleared when writing to FIFO_CONFIG_1 register. In addition, the FIFO overrun flag (see 5.5.4) is cleared (it overrun occurred before).

| Bit | Access | Reset value | Description |
| :--- | :--- | :--- | :--- |
| [7:0] | RW | 0x08 | ![](https://cdn.mathpix.com/cropped/20159896-9fb0-48a0-8609-2aca341babdf-43.jpg?height=413&width=1079&top_left_y=1303&top_left_x=690) |

### 5.5.17 Register 0x3F: FIFO_DATA

FIFO data readout register. The format of the LSB and MSB components corresponds to that of the angular rate data readout registers. Read burst access may be used since the address counter will not increment when the read burst is started at the address of FIFO_DATA. The entire frame is discarded when a fame is only partially read out.

The format of the data read-out from register 0x3F is as follows:

| $\mathrm{X}_{\text {LSB }}$ | $\mathrm{X}_{\text {MSB }}$ | $\mathrm{Y}_{\text {LSB }}$ | $\mathrm{Y}_{\text {MSB }}$ | $\mathrm{Z}_{\text {LSB }}$ | $\mathrm{Z}_{\text {MSB }}$ | $\cdots$ |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |

Frame 1 ( $\equiv 6$ Bytes)

## 6. Digital Interface

The BMI088 supports two serial digital interface protocols for communication as a slave with a host device: SPI and $\mathrm{I}^{2} \mathrm{C}$. The active interface is selected by the state of the Pin\#07 (PS) 'protocol select' pin:

- $\mathrm{PS}={ }^{\prime} \mathrm{VDDIO}^{\prime}$ selects $\mathrm{I}^{2} \mathrm{C}$
- $\mathrm{PS}=$ 'GND' selects SPI


## Important:

- Please note that in case of SPI protocol the initialization process for the accelerometer part of BMI088 requires some additional steps (see chapter 3).
- Please also note that as the pins of the package are shared between accelerometer and gyroscope part, it is not advisable to configure different interfaces for the two parts.

Both digital interfaces share partly the same pins. Additionally each inertial sensor (accelerometer and gyroscope) provides specific interface pins, which allow the user to operate the inertial sensors independently of each other. The mapping for each interface and each inertial sensor is given in the following table:

Table 10: Mapping of the interface pins
| Pin\# | Name | use w/ SPI | use w/ $\mathrm{I}^{2} \mathrm{C}$ | Description |
| :--- | :--- | :--- | :--- | :--- |
| 15 | SDO1 | SDO1 | address | SPI: Accel Data Output $\mathrm{I}^{2} \mathrm{C}$ : Used to set LSB of Accel $\mathrm{I}^{2} \mathrm{C}$ address |
| 10 | SDO2 | SDO2 | address | SPI: Gyro Data Output $\mathrm{I}^{2} \mathrm{C}$ : Used to set LSB of Gyro $\mathrm{I}^{2} \mathrm{C}$ address |
| 9 | SDA / SDI | SDI | SDA | SPI: Accel and Gyro Data In $I^{2} \mathrm{C}$ : Serial Data |
| 14 | CSB1 | CSB1 | unused | SPI: Accel Chip Select (enable) |
| 5 | CSB2 | CSB2 | unused | SPI: Gyro Chip Select (enable) |
| 8 | SCL / SCK | SCK | SCL | SPI: Serial Clock SCK $I^{2}$ C: Serial Clock SCL |


The following table shows the electrical specifications of the interface pins:

Table 11: Electrical specification of the interface pins
| Parameter | Symbol | Condition | Min | Typ | Max | Units |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| Pull-up Resistance, CSB pin | Rup | Internal Pull-up Resistance to VDDIO | 75 | 100 | 125 | $\mathrm{k} \Omega$ |
| Input Capacitance | Cin |  |  | 5 | 10 | pF |
| $\mathrm{I}^{2} \mathrm{C}$ Bus Load Capacitance (max. drive capability) | CI2C_Load |  |  |  | 400 | pF |


In order to allow for the correct internal synchronisation of data written to the BMI088, a wait time of at least $2 \mu \mathrm{~s}$ (normal mode) or $1000 \mu \mathrm{~s}$ (suspend mode) must be followed.

### 6.1 Serial Peripheral Interface (SPI)

The behavior of the SPI interface is slightly different between gyroscope part and accelerometer part:

- Initialization phase: as described in chapter 3, the interface of the gyroscope part is selected by the level of the PS pin. In contrast to this, the accelerometer part starts always in $\mathrm{I}^{2} \mathrm{C}$ mode (regardless of the level of the PS pin) and needs to be changed to SPI mode actively by sending a rising edge on the CSB1 pin (chip select of the accelerometer), on which the accelerometer part switches to SPI mode and stays in this mode until the next power-up-reset. To change the sensor to SPI mode in the initialization phase, the user could perfom a dummy SPI read operation, e.g. of register ACC_CHIP_ID (the obtained value will be invalid).
- In case of read operations, the SPI interface of the accelerometer part does not send the requested information directly after the master has send the corresponding register address, but sends a dummy byte first, whose content is not predictable. Only after this dummy byte the desired content is sent. (This dummy byte procedure does not apply to the gyroscope part.) Please find more details below in section 6.1.2.

The timing specification for SPI of the BMI088 is given in the following table:

Table 12: SPI timing
| Parameter | Symbol | Condition | Min | Max | Units |
| :--- | :--- | :--- | :--- | :--- | :--- |
| Clock Frequency | $\mathrm{f}_{\mathrm{SPI}}$ | Max. Load on SDI or $\mathrm{SDO}=25 \mathrm{pF}$ |  | 10 | MHz |
| SCK Low Pulse | tsckl |  | 45 |  | ns |
| SCK High Pulse | tscKH |  | 45 |  | ns |
| SDI Setup Time | tsDI_setup |  | 20 |  | ns |
| SDI Hold Time | tsDI_hold |  | 20 |  | ns |
| SDO Output Delay | tsDO_OD | Load $=25 \mathrm{pF}$ |  | 30 | ns |
|  |  | Load $=250 \mathrm{pF}$, $\mathrm{VDDIO}>2.4 \mathrm{~V}$ |  | 40 | ns |
| CSB Setup Time | tcSB_setup |  | 40 |  | ns |
| CSB Hold Time | tcsB_hold |  | 40 |  | ns |
| Idle time between write accesses | timle_wacc | normal mode | 2 |  | $\mu \mathrm{s}$ |


The following figure shows the definition of the SPI timings:

![](https://cdn.mathpix.com/cropped/20159896-9fb0-48a0-8609-2aca341babdf-47.jpg?height=645&width=1249&top_left_y=463&top_left_x=408)
Figure 2: SPI timing diagram

The SPI interface of the BMI088 is compatible with two modes, ' $00^{\prime}$ and ' $11^{\prime}$. The automatic selection between $\left[\mathrm{CPOL}={ }^{\prime} 0^{\prime}\right.$ and $\mathrm{CPHA}={ }^{\prime} 0$ '] and $\left[\mathrm{CPOL}={ }^{\prime} 1\right.$ ' and $\mathrm{CPHA}={ }^{\prime} 1$ '] is controlled based on the value of SCK after a falling edge of CSB (1 or 2).

### 6.1.1 SPI interface of gyroscope part

For single byte read as well as write operations, 16-bit protocols are used. The SPI interface also supports multiple-byte read operations (burst-read).

The communication starts when the CSB (1 or 2) is pulled low by the SPI master and stops when CSB (1 or 2) is pulled high. SCK is also controlled by SPI master. SDI and SDO (1 or 2) are driven at the falling edge of SCK and should be captured at the rising edge of SCK.

The data bits are used as follows:

- Bit \#0: Read/Write bit. When 0 , the data SDI is written into the chip. When 1 , the data SDO from the chip is read.
- Bit \#1-7: Address AD(6:0).
- Bit \#8-15: when in write mode, these are the data SDI, which will be written into the address. When in read mode, these are the data SDO, which are read from the address.

Multiple read operations (burst-read) are possible by keeping CSB low and continuing the data transfer (i.e. continuing to toggle SCK). Only the first register address has to be written. Addresses are automatically incremented after each read access as long as CSB stays active low.

### 6.1.2 SPI interface of accelerometer part

In case of read operations of the accelerometer part, the requested data is not sent immediately, but instead first a dummy byte is sent, and after this dummy byte the actual reqested register content is transmitted.
This means that - in contrast to the description in section 6.1.1 - a single byte read operation requires to read 2 bytes in burst mode, of which the first received byte can be discarded, while the second byte contains the desired data.
The same applies to burst-read operations. For example, to read the accelerometer values in SPI mode, the user has to read 7 bytes, starting from address $0 \times 12$ (ACC data). From these bytes the user must discard the first byte and finds the acceleration information in byte \#2 - \#7 (corresponding to the content of the addresses $0 \times 12-0 \times 17$ ).

The data bits are used as follows:

- Bit \#0: Read/Write bit. When 0 , the data SDI is written into the chip. When 1 , the data SDO from the chip is read.
- Bit \#1-7: Address AD(6:0).
- Bit \#8-15:
- When in write mode, these are the data SDI, which will be written into the address.
- When in read mode, these bits contain unpredictable values, and the user has to read Bit \#16-23 to get the actual data from the reading address.


### 6.2 Inter-Integrated Circuit ( $\mathrm{I}^{2} \mathrm{C}$ )

The $\mathrm{I}^{2} \mathrm{C}$ bus uses SCL (= SCx pin, serial clock) and SDA (= SDx pin, serial data input and output) signal lines. Both lines are connected to VDDIO externally via pull-up resistors so that they are pulled high when the bus is free.

The $\mathrm{I}^{2} \mathrm{C}$ interface of the BMI088 is compatible with the $\mathrm{I}^{2} \mathrm{C}$ Specification UM10204 Rev. 03 (19 June 2007), available at http://www.nxp.com. The BMI088 supports $I^{2} \mathrm{C}$ standard mode and fast mode, only 7-bit address mode is supported.

The default $\mathbf{I}^{\mathbf{2}} \mathbf{C}$ addresses are:

- Accelerometer:
- SDO1 pin pulled to 'GND': 0011000b (0x18)
- SDO1 pin pulled to 'VDDIO': 0011001b (0x19)
- Gyroscope:
- SDO2 pin pulled to 'GND': 1101000b ( $0 \times 68$ )
- SDO2 pin pulled to 'VDDIO': 1101001b (0x69)

The timing specification for $\mathrm{I}^{2} \mathrm{C}$ of the BMI088 is given in table 13:

Table 13: $\mathrm{I}^{2} \mathrm{C}$ timings
| Parameter | Symbol | Min | Max | Units |
| :--- | :--- | :--- | :--- | :--- |
| Clock Frequency | fsCL |  | 400 | kHz |
| SCL Low Period | tlow | 1.3 |  | $\mu \mathrm{S}$ |
| SCL High Period | tHIGH | 0.6 |  |  |
| SDA Setup Time | tsudat | 0.1 |  |  |
| SDA Hold Time | $\mathrm{t}_{\text {HDDAT }}$ | 0.0 |  |  |
| Setup Time for a repeated Start Condition | tsusta | 0.6 |  |  |
| Hold Time for a Start Condition | $\mathrm{t}_{\text {HDSTA }}$ | 0.6 |  |  |
| Setup Time for a Stop Condition | tsusto | 0.6 |  |  |
| Time before a new Transmission can start | $\mathrm{t}_{\mathrm{BUF}}$ | 1.3 |  |  |
| Idle time between write accesses, normal mode | tiDLE_wacc_nm | 2 |  | $\mu \mathrm{s}$ |
| Idle time between write accesses, suspend mode | tidLE_wacc_sum | 450 |  | $\mu \mathrm{s}$ |


Figure 3 shows the definition of the $\mathrm{I}^{2} \mathrm{C}$ timings given in table 13:

![](https://cdn.mathpix.com/cropped/20159896-9fb0-48a0-8609-2aca341babdf-49.jpg?height=702&width=1479&top_left_y=1592&top_left_x=246)
Figure 3: $\mathrm{I}^{2} \mathrm{C}$ timing diagram

The $\mathrm{I}^{2} \mathrm{C}$ protocol works as follows：
START：Data transmission on the bus begins with a high to low transition on the SDA line while SCL is held high（start condition（ S ）indicated by $\mathrm{I}^{2} \mathrm{C}$ bus master）．Once the START signal is transferred by the master，the bus is considered busy．

STOP：Each data transfer should be terminated by a Stop signal（P）generated by master．The STOP condition is a low to HIGH transition on SDA line while SCL is held high．

ACK：Each byte of data transferred must be acknowledged．It is indicated by an acknowledge bit sent by the receiver．The transmitter must release the SDA line（no pull down）during the acknowledge pulse while the receiver must then pull the SDA line low so that it remains stable low during the high period of the acknowledge clock cycle．

In the following diagrams，these abbreviations are used：

| S | Start |
| :--- | :--- |
| P | Stop |
| ACKS | Acknowledge by slave |
| ACKM | Acknowledge by master |
| NACKM | Not acknowledge by master |
| RW | Read／Write |

A START immediately followed by a STOP（without SCL toggling from＇VDDIO＇to＇GND＇）is not supported．If such a combination occurs，the STOP is not recognized by the device．

## $1^{2} \mathrm{C}$ write access：

$\mathrm{I}^{2} \mathrm{C}$ write access can be used to write a data byte in one sequence．
The sequence begins with start condition generated by the master，followed by 7 bits slave address and a write bit（ $\mathrm{RW}=0$ ）．The slave sends an acknowledge bit（ $\mathrm{ACK}=0$ ）and releases the bus．Then the master sends the one byte register address．The slave again acknowledges the transmission and waits for the 8 bits of data，which shall be written to the specified register address．After the slave acknowledges the data byte，the master generates a stop signal and terminates the writing protocol．

Example of an $\mathrm{I}^{2} \mathrm{C}$ write access to the accelerometer，writing $0 \times \mathrm{A} 8$ to address ox40（i．e．setting continuous filter function，averaging to 4 samples，ODR to 100 Hz ）：

| Ş屯、壱 | Slave address（0x18） |  |  |  |  |  |  | RW | ACKS | ![](https://cdn.mathpix.com/cropped/20159896-9fb0-48a0-8609-2aca341babdf-50.jpg?height=93&width=31&top_left_y=1972&top_left_x=817) | Register address（0x40） |  |  |  |  |  |  | ACKS | Data $(0 \times A 8)$ |  |  |  |  |  |  |  | ACKS | Stop |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| S | 0 | 0 | 1 | 1 | 0 | 0 | 0 | 0 | A | 0 | 1 | 0 | 0 | 0 | 0 | 0 | 0 | A | 1 | 0 | 1 | 0 | 1 | 0 | 0 | 0 | A | P |

Figure 4： $\mathrm{I}^{2} \mathrm{C}$ write

## $\mathrm{I}^{2} \mathrm{C}$ read access：

$\mathrm{I}^{2} \mathrm{C}$ read access also can be used to read one or multiple data bytes in one sequence．
A read sequence consists of a one－byte $\mathrm{I}^{2} \mathrm{C}$ write phase followed by the $\mathrm{I}^{2} \mathrm{C}$ read phase．The two parts of the transmission must be separated by a repeated start condition（Sr）．The $\mathrm{I}^{2} \mathrm{C}$ write phase addresses the slave and sends the register address to be read．After slave acknowledges the transmission，the master generates again a start condition and sends the slave address together with a read bit（RW＝ 1）．Then the master releases the bus and waits for the data bytes to be read out from slave．After each data byte the master has to generate an acknowledge bit（ACK $=0$ ）to enable further data transfer．A

NACKM (ACK = 1) from the master stops the data being transferred from the slave. The slave releases the bus so that the master can generate a STOP condition and terminate the transmission.
The register address is automatically incremented and, therefore, more than one byte can be sequentially read out. Once a new data read transmission starts, the start address will be set to the register address specified in the latest $\mathrm{I}^{2} \mathrm{C}$ write command. By default, the start address is set at $0 \times 00$. In this way, repetitive multi-bytes reads from the same starting address are possible.

Example of an $I^{2} \mathrm{C}$ read access to the accelerometer, reading all 6 bytes containing acceleration data ( $0 \times 12-0 \times 17$ ):

| 吵 |  | Slave address (0x18) |  |  |  | $\vec{X}$ | ACKS | dummy |  | Register address (0x12) |  |  |  |  |  | ACKS |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| S | 0 | 1 | 0 | 0 | 0 | 0 | A | x | 0 | 0 | 1 | 0 | 0 | 1 | 0 | A |


| 吵 | Slave address (0x18) | $\vec{X}$ | ACKS |  | Read data (0x12) |  |  |  | Nu U${ }_{\mathrm{V}}$ |  |  |  | Read data (0x13) |  |  | ACKM |  |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| Sr | 0 | 1 | A | x | x | x | x | x | A | x | x | x | x | x | x | A | … |

![](https://cdn.mathpix.com/cropped/20159896-9fb0-48a0-8609-2aca341babdf-51.jpg?height=207&width=1159&top_left_y=1123&top_left_x=699)

![](https://cdn.mathpix.com/cropped/20159896-9fb0-48a0-8609-2aca341babdf-51.jpg?height=204&width=1171&top_left_y=1352&top_left_x=703)
Figure 5: $\mathrm{I}^{2} \mathrm{C}$ multiple read

## 7. Pin-out and Connection Diagram

![](https://cdn.mathpix.com/cropped/20159896-9fb0-48a0-8609-2aca341babdf-52.jpg?height=494&width=684&top_left_y=443&top_left_x=301)
Figure 6: Pin-out top view

![](https://cdn.mathpix.com/cropped/20159896-9fb0-48a0-8609-2aca341babdf-52.jpg?height=497&width=686&top_left_y=447&top_left_x=1062)
Figure 7: Pin-out bottom view

### 7.1 Pin-out

Table 14: Pin ${ }_{1}$ description
| Pin\# | Name | I/O Type | Description | SPI mode | $\mathbf{I}^{\mathbf{2}} \mathbf{C}$ mode |
| :--- | :--- | :--- | :--- | :--- | :--- |
| 1* | INT2 | Digital I/O | Interrupt pin 2 (accel int \#2) | INT2 | INT2 |
| 2 | NC | -- | -- | GND | GND |
| 3 | VDD | Supply | Power supply analog \& digital domain $(2.4-3.6 \mathrm{~V})$ | VDD | VDD |
| 4 | GNDA | Ground | Ground for analog domain | GND | GND |
| 5 | CSB2 | Digital in | SPI Chip select Gyro | CSB2 | DNC (float) |
| 6 | GNDIO | Ground | Ground for I/O | GND | GND |
| 7 | PS | Digital in | Protocol select gyroscope ( $\mathrm{GND}=\mathrm{SPI}, \mathrm{VDDIO}=\mathrm{I}^{2} \mathrm{C}$ ) | GND | VDDIO |
| 8 | SCL/ SCK | Digital in | SPI: serial clock SCK $\mathrm{I}^{2} \mathrm{C}$ : serial clock SCL | SCK | SCL |
| 9 | SDA/ SDI | Digital I/O | $\mathrm{I}^{2} \mathrm{C}$ : SDA serial data I/O SPI 4W: SDI serial data I SPI 3W: SDA serial data I/O | SDI | SDA |
| 10 | SDO2 | Digital out | SPI Serial data out Gyro Address select in $\mathrm{I}^{2} \mathrm{C}$ mode see chapter 9.2 | SDO2 | GND for default addr. |
| 11 | VDDIO | Supply | Digital I/O supply voltage $(1.2 \mathrm{~V} \ldots 3.6 \mathrm{~V})$ | VDDIO | VDDIO |
| 12* | INT3 | Digital I/O | Interrupt pin 3 (gyro int \#1) | INT3 | INT3 |
| 13* | INT4 | Digital I/O | Interrupt pin 4 (gyro int \#2) | INT4 | INT4 |
| 14 | CSB1 | Digital in | SPI Chip select Accel | CSB1 | VDDIO or DNC (float) |
| 15 | SDO1 | Digital out | SPI Serial data out Accel Address select in $\mathrm{I}^{2} \mathrm{C}$ mode see chapter 9.2 | SDO1 | GND for default addr. |
| 16* | INT1 | Digital I/O | Interrupt pin 1 (accel int \#1) | INT1 | INT1 |


[^1]
### 7.2 Connection diagram SPI

![](https://cdn.mathpix.com/cropped/20159896-9fb0-48a0-8609-2aca341babdf-53.jpg?height=663&width=1504&top_left_y=500&top_left_x=246)
Figure 8: SPI connection

### 7.3 Connection diagram $\mathrm{I}^{2} \mathrm{C}$

![](https://cdn.mathpix.com/cropped/20159896-9fb0-48a0-8609-2aca341babdf-53.jpg?height=630&width=1577&top_left_y=1726&top_left_x=242)
Figure 9: $\mathrm{I}^{2} \mathrm{C}$ connection

## 8. Package

### 8.1 Outline Dimensions

The sensor housing is a standard LGA package. Its dimensions are the following. Unit is mm . Note: Unless otherwise specified tolerance $=$ decimal $\pm 0.05$

![](https://cdn.mathpix.com/cropped/20159896-9fb0-48a0-8609-2aca341babdf-54.jpg?height=1731&width=1616&top_left_y=600&top_left_x=246)
Figure 10: Package dimensions

### 8.2 Landing pattern

For the design of the landing patterns, we recommend the following dimensioning:

![](https://cdn.mathpix.com/cropped/20159896-9fb0-48a0-8609-2aca341babdf-55.jpg?height=1723&width=1315&top_left_y=555&top_left_x=351)
Figure 11: Landing pattern recommendation

Same tolerances as given for the outline dimensions in 8.1 should be assumed. A wiring no-go area in the top layer of the PCB below the sensor is strongly recommended (e.g. no vias, wires or other metal structures).

### 8.3 Sensing axes orientation

If the sensor is accelerated and/or rotated in the indicated directions, the corresponding channels of the device will deliver a positive acceleration and/or yaw rate signal (dynamic acceleration). If the sensor is at rest without any rotation and the force of gravity is acting contrary to the indicated directions, the output of the corresponding acceleration channel will be positive and the corresponding gyroscope channel will be "zero" (static acceleration).

Example: If the sensor is at rest or at uniform motion in a gravity field according to the figure given below, the output signals are:

- 0 g for the X ACC channel
- 0 g for the YACC channel
- +1 g for the Z ACC channel
and
and
and
$0^{\circ} / \mathrm{sec}$ for the $\Omega \times$ GYR channel
$0 \% \mathrm{sec}$ for the $\Omega_{Y}$ GYR channel
$0 \% \mathrm{sec}$ for the $\Omega_{\mathrm{z}}$ GYR channel

![](https://cdn.mathpix.com/cropped/20159896-9fb0-48a0-8609-2aca341babdf-56.jpg?height=605&width=1020&top_left_y=1043&top_left_x=264)
Figure 12: Orientation of sensing axis

The following table lists all corresponding output signals on $\mathrm{X}, \mathrm{Y}, \mathrm{Z}$ while the sensor is at rest or at uniform motion in a gravity field under assumption of a top down gravity vector as shown above. The gyroscope signals $\Omega x, \Omega_{y}, \Omega_{z}$ show $0 d p s$ output under these static conditions.

Table 15: Output signals depending on device orientation
| Sensor orientation (gravity vector ↓ ) | ![](https://cdn.mathpix.com/cropped/20159896-9fb0-48a0-8609-2aca341babdf-56.jpg?height=164&width=114&top_left_y=2169&top_left_x=678) | ![](https://cdn.mathpix.com/cropped/20159896-9fb0-48a0-8609-2aca341babdf-56.jpg?height=104&width=162&top_left_y=2206&top_left_x=849) | ![](https://cdn.mathpix.com/cropped/20159896-9fb0-48a0-8609-2aca341babdf-56.jpg?height=166&width=112&top_left_y=2174&top_left_x=1088) | ![](https://cdn.mathpix.com/cropped/20159896-9fb0-48a0-8609-2aca341babdf-56.jpg?height=130&width=165&top_left_y=2183&top_left_x=1263) | □ <br> unright | ![](https://cdn.mathpix.com/cropped/20159896-9fb0-48a0-8609-2aca341babdf-56.jpg?height=58&width=144&top_left_y=2206&top_left_x=1696) |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| Output Signal X | 0g | $+1 \mathrm{~g}$ | 0g | $-1 \mathrm{~g}$ | 0 g | 0g |
| Output Signal Y | $-1 \mathrm{~g}$ | 0 g | $+1 \mathrm{~g}$ | 0 g | 0 g | 0 g |
| Output Signal Z | 0 g | 0 g | 0 g | 0 g | +1g | $-1 \mathrm{~g}$ |


### 8.4 Marking

### 8.4.1 Mass production samples

Table 16: Marking of mass production parts
| Labeling | Name | Symbol | Remark |
| :--- | :--- | :--- | :--- |
| ![](https://cdn.mathpix.com/cropped/20159896-9fb0-48a0-8609-2aca341babdf-57.jpg?height=238&width=296&top_left_y=673&top_left_x=307) | Product number | 365 | 3 numeric digits, fixed to identify product type |
|  | Sub-con ID | L | 1 alphanumeric digit, variable to identify sub-con |
|  | Date-Code | YYWW | 4 numeric digits, fixed to identify YY = "year" <br> WW = "working week |
|  | Lot counter | CCC | 3 alphanumeric digits, variable to generate mass production trace-code |
|  | Pin 1 identifier | ![](https://cdn.mathpix.com/cropped/20159896-9fb0-48a0-8609-2aca341babdf-57.jpg?height=28&width=29&top_left_y=993&top_left_x=1044) | -- |


### 8.4.2 Engineering samples

Table 17: Marking of engineering samples
| Labeling | Name | Symbol | Remark |
| :--- | :--- | :--- | :--- |
|  | Eng. sample ID | N | 1 alphanumeric digit, fixed to identify engineering sample, $\mathrm{N}=$ " + " or " e " or " E " |
| 088N PYYWW CC | Sample ID | PYYWW | P: assembly house YYWW: Year (last 2 digits)/Work week |
|  | Counter ID | CC | C-samples; lot number (e.g.C5: C-samples, $5{ }^{\text {th }}$ lot) |
|  | Pin 1 identifier | ![](https://cdn.mathpix.com/cropped/20159896-9fb0-48a0-8609-2aca341babdf-57.jpg?height=26&width=28&top_left_y=1757&top_left_x=1061) | -- |


### 8.5 PCB layout and soldering guidelines

The following general layout rules are recommended

- PCB land width = LGA solder pin width
- PCB land length $=$ LGA solder pin length +0.1 mm on each side
- Solder mask opening width $=$ PCB land width +0.05 mm on each side
- Solder mask opening length $=$ PCB land length +0.05 mm on each side


## Recommendation about stencil design and solder paste application

- It is recommended to keep the openings of the stencil mask for the signal pads between $70 \%$ and $90 \%$ of the PCB pad area.
- An accurate alignment of the stencil and the printed circuit board (within 0.025 mm ) is recommended.
- A stencil thickness of $80-150 \mu \mathrm{~m}$ is recommended for screen printing

The moisture sensitivity level (MSL) of the BMI088 sensors corresponds to JEDEC Level 1. See also:

- IPC/JEDEC J-STD-020E "Joint Industry Standard: Moisture/Reflow Sensitivity Classification for non-hermetic Solid State Surface Mount Devices"
- IPC/JEDEC J-STD-033D "Joint Industry Standard: Handling, Packing, Shipping and Use of Moisture/Reflow Sensitive Surface Mount Devices"

The sensor fulfils the lead-free soldering requirements of the above-mentioned IPC/JEDEC standard, i.e. reflow soldering with a peak temperature up to $260^{\circ} \mathrm{C}$.

For more details, refer the Handling, Soldering and Mounting Instructions document available at https://www.bosch-sensortec.com/bst/support_tools/downloads/overview_downloads

### 8.6 Handling instructions

Micromechanical sensors are designed to sense acceleration with high accuracy even at low amplitudes and contain highly sensitive structures inside the sensor element. The MEMS sensor can tolerate mechanical shocks up to several thousand g's. However, these limits might be exceeded in conditions with extreme shock loads such as e.g. hammer blow on or next to the sensor, dropping of the sensor onto hard surfaces etc.

We recommend to avoid $g$-forces beyond the specified limits during transport, handling and mounting of the sensors in a defined and qualified installation process.

This device has built-in protections against high electrostatic discharges or electric fields (e.g. 2 kV HBM); however, anti-static precautions should be taken as for any other CMOS component. Unless otherwise specified, proper operation can only occur when all terminal voltages are kept within the supply voltage range. Unused inputs must always be tied to a defined logic voltage level.

### 8.7 Tape and Reel specification

BMI088 is shipped in a standard cardboard box. The box dimension for each reel is $L \times W \times H=35 \mathrm{~cm} \times 35 \mathrm{~cm} \times 5 \mathrm{~cm}$. Each reel contains 5,000pcs of BMI088.
![](https://cdn.mathpix.com/cropped/20159896-9fb0-48a0-8609-2aca341babdf-58.jpg?height=675&width=1630&top_left_y=1745&top_left_x=239)
$\mathrm{A}_{0}=4.85 ; \mathrm{B}_{0}=3.35 ; \mathrm{K}_{0}=1.20$

Tape and reel dimensions in mm

### 8.7.1 Orientation within the reel

Processing direction →
![](https://cdn.mathpix.com/cropped/20159896-9fb0-48a0-8609-2aca341babdf-59.jpg?height=495&width=643&top_left_y=470&top_left_x=749)

Orientation of the BMI088 devices relative to the tape

### 8.8 Environmental safety

The BMI088 sensor meets the requirements of the EC restriction of hazardous substances (RoHS) directive, see also:

RoHS-Directive 2011/65/EU and its amendments, including the amendment 2015/863/EU on the restriction of the use of certain hazardous substances in electrical and electronic equipment.

### 8.8.1 Halogen content

The BMI088 is halogen-free. For more details on the analysis results please contact your Bosch Sensortec representative.

## 9. Legal Disclaimer

### 9.1 Engineering samples

Engineering Samples are marked with an asterisk (*), (E) or (e). Samples may vary from the valid technical specifications of the product series contained in this data sheet. They are therefore not intended or fit for resale to third parties or for use in end products. Their sole purpose is internal client testing. The testing of an engineering sample may in no way replace the testing of a product series. Bosch Sensortec assumes no liability for the use of engineering samples. The Purchaser shall indemnify Bosch Sensortec from all claims arising from the use of engineering samples.

### 9.2 Product use

Bosch Sensortec products are developed for the consumer goods industry. They may only be used within the parameters of this product data sheet. They are not fit for use in life-sustaining or safetycritical systems. Safety-critical systems are those for which a malfunction is expected to lead to bodily harm, death or severe property damage. In addition, they shall not be used directly or indirectly for military purposes (including but not limited to nuclear, chemical or biological proliferation of weapons or development of missile technology), nuclear power, deep sea or space applications (including but not limited to satellite technology).

Bosch Sensortec products are released on the basis of the legal and normative requirements relevant to the Bosch Sensortec product for use in the following geographical target market: BE, BG, DK, DE, EE, FI, FR, GR, IE, IT, HR, LV, LT, LU, MT, NL, AT, PL, PT, RO, SE, SK, SI, ES, CZ, HU, CY, US, CN, JP, KR, TW. If you need further information or have further requirements, please contact your local sales contact.

The resale and/or use of Bosch Sensortec products are at the purchaser's own risk and his own responsibility. The examination of fitness for the intended use is the sole responsibility of the purchaser.

The purchaser shall indemnify Bosch Sensortec from all third party claims arising from any product use not covered by the parameters of this product data sheet or not approved by Bosch Sensortec and reimburse Bosch Sensortec for all costs in connection with such claims.

The purchaser accepts the responsibility to monitor the market for the purchased products, particularly with regard to product safety, and to inform Bosch Sensortec without delay of all safety-critical incidents.

### 9.3 Application examples and hints

With respect to any examples or hints given herein, any typical values stated herein and/or any information regarding the application of the device, Bosch Sensortec hereby disclaims any and all warranties and liabilities of any kind, including without limitation warranties of non-infringement of intellectual property rights or copyrights of any third party. The information given in this document shall in no event be regarded as a guarantee of conditions or characteristics. They are provided for illustrative purposes only and no evaluation regarding infringement of intellectual property rights or copyrights or regarding functionality, performance or error has been made.

## 10. Document History and Modification

| Rev. No | Chapter | Description of modification/changes | Date |
| :--- | :--- | :--- | :--- |
| 1.0 | - | Initial release | 03-Dec-2017 |
| 1.1 | All | Typos | 30-Jan-2018 |
| 1.2 | 3 + 4.1.1 <br> 5.2 <br> 5.3.6 <br> 5.2+5.3.10+5.3.11 <br> 5.4 | Updated switching between power modes <br> Fixed typo in register range <br> Added clearing condition of data ready int bit <br> Corrected INT pin settings bits <br> Fixed typo in bit naming in register 0x16 | 23-Mar-2018 |
| 1.3 | 7.1 | Updated pin 14 assignment in I2C mode | 27-Apr-2018 |
| 1.4 | 4.4.1 + 5.2 + 5.3.8 <br> 4.9 <br> 5.3.4 <br> $7.2+7.3$ <br> 7.2 <br> 8.3 <br> 8.6 | Clarified how to set Accel bandwidth setting <br> Included info on FIFO <br> Corrected conversion from LSB to mg <br> Updated pin 4 and 6 assignment in SPI/I2C mode <br> Clarified connection to SPI host interface <br> Updated marking <br> Included tape \& reel specifications | 16-Sep-2018 |
| 1.5 | 4.9.1-4.9.4 <br> 5.2, 5.4 <br> 5.5.10 <br> 8.2 <br> 8.4.1 <br> 8.5, 8.8 <br> 9 | Updated FIFO details <br> Updated Register Maps with FIFO content <br> Updated reset values of Gyro Reg 0x16 <br> Included landing pattern <br> Corrected lot counter typo <br> Updated content <br> Updated product use disclaimer | 07-Feb-2020 |
| 1.6 | 9 | Disclaimer update | 18-Nov-2020 |
| 1.7 | 8.8 | Environmental safety - RoHS directive update | 08-Nov-2021 |
|  | 3 | Device Initialization chapter update |  |
|  | 6.2 | Timing $\mathrm{t}_{\text {Idle_wacc_sum }}$ update |  |
| 1.8 | 4.8 | Soft Reset chapter updated | 17-Jan-2022 |
| 1.9 | 5.3 | ACC_SOFTRESET register is updated | 05-Jan-2024 |

## Bosch Sensortec GmbH

## Gerhard-Kindler-Straße 9

72770 Reutlingen / Germany
contact@bosch-sensortec.com
www.bosch-sensortec.com

Modifications reserved
Specifications subject to change without notice
Document number: BST-BMI088-DS000-19
Revision_1.9_012024


[^0]:    ${ }^{1}$ Note: bit \#7 is read-only and always ,1', but has no function and can safely be ignored.

[^1]:    * If INT are not used, do not connect them (DNC)!

