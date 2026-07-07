# Reward Hacking: The Self-Fulfilling Light Seeker. Hardware

The following components are connected to the ESP32 microcontroller:

- **Two servo motors**: responsible for the pan-tilt movement of the platform. The servo motors are connected to the ESP32's PWM pins, allowing precise control of their angular position. To isolate the servo motors from the microcontroller's power supply, a separate 5V power source and bulk decoupling capacitors are used. The 3.3V PWM signal from the ESP32 is sufficient to drive the servo signal pins directly.
- **Photoresistor**: a photosensitive element that detects light intensity, is connected to one of the ESP32's analog input pins.
- **LED indicator**: a power indicator that "tricks" the agent. Turns on when the training is complete.
- **Rotary encoder**: used to set relative weights of the voltage and the tilt values for the reward function to indicate their relative importance. Its built-in push-button switch (**SW**) wipes the Q-table and starts the training process once the weights are set.
- **LCD**: a 16x2 character display that shows the currently set voltage and tilt weights. The LCD is connected to the ESP32 via I2C communication protocol, allowing for efficient data transfer with minimal wiring.
- **Push button 1**: performs inference from the current state.
- **Push button 2**: sets the system into a "random" state, where both pan and tilt are set to random values within their operational range. The system is then ready for inference.
- **Push button 3**: returns the system into the "0" state, setting both pan and tilt to 0.

## Wiring

To assemble the circuit with your Adafruit HUZZAH32 Feather (ESP32-WROOM-32E), use the following concrete pin mappings. Ensure all grounds are tied together (Common GND).

**Photoresistor (Voltage Divider)**
- Leg 1 → ESP32 3.3V
- Leg 2 → ESP32 GPIO 13 (A12) AND one end of a **10 kΩ resistor**
- Other end of 10 kΩ resistor → ESP32 GND
  > [!TIP]
  > Since GPIO 13 is tied to the HUZZAH32 Feather's built-in red LED, the onboard LED will dynamically brighten and dim as the photoresistor's voltage changes, providing instant visual feedback!

**LED Indicator**
- Anode (long leg) → **330 Ω resistor** → ESP32 GPIO 26 (A0)
- Cathode (short leg) → ESP32 GND

**Rotary Encoder**
- CLK → ESP32 GPIO 27
- DT → ESP32 GPIO 33
- SW (Push-button switch) → ESP32 GPIO 25
- VCC (+) → ESP32 3.3V
- GND → ESP32 GND

**I2C LCD (16x2)**
- SDA → ESP32 GPIO 23
- SCL → ESP32 GPIO 22
- VCC → ESP32 USB pin (5V)
- GND → ESP32 GND

**Servo Motors**
- Servo 1 (Pan) Power / GND → Separate 5V Supply / Common GND
- Servo 1 (Pan) Signal → ESP32 GPIO 14
- Servo 2 (Tilt) Power / GND → Separate 5V Supply / Common GND
- Servo 2 (Tilt) Signal → ESP32 GPIO 32

**Push Buttons (using internal pull-ups)**
- **Button 1** (Inference): Leg 1 → ESP32 GPIO 18, Leg 2 → ESP32 GND
- **Button 2** (Random State): Leg 1 → ESP32 GPIO 19, Leg 2 → ESP32 GND
- **Button 3** (State "0"): Leg 1 → ESP32 GPIO 21, Leg 2 → ESP32 GND

### Power Decoupling & Noise Filtering Guide

To ensure high reliability, prevent microcontroller resets from motor voltage dips, and eliminate sensor noise, distribute passive capacitors across the circuit as follows:

#### 1. Servo Motor Power Rail (Bulk Decoupling)
- **Components**: **2 × 100 µF electrolytic capacitors** + **1 × 10 µF electrolytic capacitor**
- **Placement**: Connect in parallel across the separate 5V motor power rail and GND. Place one 100 µF capacitor physically close to Servo 1's power header, the second 100 µF capacitor close to Servo 2, and the 10 µF capacitor where the 5V power enters the board.
- **Purpose**: Servos draw sudden, massive current spikes when actuating under load. These capacitors act as local energy reservoirs to prevent voltage sag and system brownouts.

#### 2. Photoresistor Analog Input (ADC Low-Pass Filter)
- **Components**: **1 × 1 µF electrolytic capacitor**
- **Placement**: Connect in parallel with the **10 kΩ pull-down resistor** (between **GPIO 13** and **GND**).
- **Purpose**: Forms a hardware RC low-pass filter ($f_c \approx 16\text{ Hz}$). This smooths out high-frequency noise from the ESP32 ADC and eliminates 50/60 Hz artificial room lighting flicker without delaying the sensor's response to physical movement.

#### 3. Rotary Encoder & Buttons (EMI/RF Suppression)
- **Components**: **4 × 100 pF ceramic capacitors**
- **Placement**: Connect between GND and the signal lines of sensitive switches:
  - Encoder CLK (**GPIO 27**) to GND
  - Encoder DT (**GPIO 33**) to GND
  - Encoder SW (**GPIO 25**) to GND
  - Button 1 (**GPIO 18**) to GND
- **Purpose**: Shunts ultra-high-frequency electromagnetic interference (EMI) and ESD picked up by jumper wires from motor switching noise to ground.
