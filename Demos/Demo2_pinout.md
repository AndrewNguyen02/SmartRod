# Smart Rod v2 (Demo 2) — Full Pinout (ESP32 DevKit V1)



Demo 2 changes vs v1:
- Nokia 5110 (PCD8544) replaces the LCD1602
- Hall-effect sensor replaces the TCS34725 color sensor (pins TBD)
- BNO085 (UART-RVC) still used for IMU bite-detection logic

---

## ESP32 DevKit V1
- **3V3**: powers Nokia 5110 + BNO085 (+ Hall sensor if 3.3V)
- **GND**: common ground for all modules
- **5V / VIN**: not required for Demo 2 display (Nokia 5110 is 3.3V)

---

## BNO085 IMU (UART-RVC)
> UART-RVC uses the BNO’s **SDA pin as UART TX output**.

- **VIN** → ESP32 **3V3**
- **GND** → ESP32 **GND**
- **SDA (UART OUT)** → ESP32 **GPIO19** (UART2 RX)
- **P0** → ESP32 **3V3**
- **SCL** → **NC** (not used)
- ESP32 UART TX → **NC** (not used)

---

## Nokia 5110 LCD (PCD8544) — Demo 2 Display

### Nokia 5110 pin labels
Common breakout labels: **RST, CE (CS), DC, DIN, CLK, VCC, BL, GND**

### Power / backlight
- **VCC** → ESP32 **3V3**
- **GND** → ESP32 **GND**
- **BL** → ESP32 **3V3** 

### Control + data (matches standalone test code)
Uses ESP32 hardware SPI lines + 3 reused control pins from the old LCD1602 wiring:

- **CLK** → ESP32 **GPIO18** *(SCK)*
- **DIN** → ESP32 **GPIO23** *(MOSI)*
- **DC**  → ESP32 **GPIO14**
- **CE/CS** → ESP32 **GPIO27**
- **RST** → ESP32 **GPIO26**

*(Nokia 5110 does not use MISO.)*

---

## Hall-Effect Sensor (Spool Revolution Sensor) — Pins TBD
Replaces the TCS34725. Used to count spool rotations for Demo 2.

### Typical 3-pin hall module wiring (A3144-style)
- **VCC** → ESP32 **3V3**
- **GND** → ESP32 **GND**
- **OUT** → ESP32 **GPIO__** *(TBD)*


---

## Notes
- Nokia 5110 is **3.3V logic** — do **not** power it from 5V.
