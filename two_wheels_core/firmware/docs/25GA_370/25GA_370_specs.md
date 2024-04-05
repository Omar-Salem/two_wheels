https://mikroelectron.com/Product/25GA-370-12V-400RPM-DC-Reducer-Gear-Motor-with-Encoder

| Power Source   | Board   | ESP32   | Motors                                                    | TB6612FNG   |
|----------------|---------|---------|-----------------------------------------------------------|-------------|
| +              |         |         |                                                           | VM          |
| GND            | GND     | GND     | M1 Black  - negative of encoder power supply(-)(3.3-5V)   | GND         |
|                | +       | 5V      | M1 BLue   - positive of encoder power supply(+)(3.3-5V)   | STBY,VCC    |
|                |         |         | M1 Red  - positive power supply (+)                       | A0          |
|                |         |         | M1 White  - negative power supply (-)                     | A1          |
|                |         | 16      |                                                           | PWMA        |
|                |         | 4       |                                                           | AI1         |
|                |         | 0       |                                                           | AI2         |
|                |         | 23      | M1 Yellow                                                 |             |
|                |         | 22      | M1 Green                                                  |             |
| -------------- | ------- | ------- | --------------------------------------------------------- | ----------- |
|                | GND     |         | M2 Black  - negative of encoder power supply(-)(3.3-5V)   |             |
|                | +       |         | M2 BLue   - positive of encoder power supply(+)(3.3-5V)   |             |
|                |         |         | M2 White  -      negative power supply  (-)               | B0          |
|                |         |         | M2 Red  - positive power supply (+)                       | B1          |
|                |         | 27      |                                                           | PWMB        |
|                |         | 12      |                                                           | BI1         |
|                |         | 14      |                                                           | BI2         |
|                |         | 36      | M2 Yellow  - M2                                           |             |
|                |         | 39      | M2 Green  - M2                                            |             |

https://www.openimpulse.com/blog/products-page/25d-gearmotors/jga25-370-dc-gearmotor-77-rpm-6-v/

Specifications
Operating voltage: between 3 V and 9 V
Nominal voltage: 6 V
Free-run speed at 6 V: 77 RPM
Free-run current at 6 V: 80 mA
Stall current at 6V: 900 mA
Stall torque at 6V: 10 kg·cm
Gear ratio: 1:78
Reductor size: 23 mm
Weight: 88 g
