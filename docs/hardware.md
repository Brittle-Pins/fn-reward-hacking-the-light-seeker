# Reward Hacking: The Self-Fulfilling Light Seeker. Hardware

The following components are connected to the ESP32 microcontroller:

- **Two servo motors**: responsible for the pan-tilt movement of the platform. The servo motors are connected to the ESP32's PWM pins, allowing precise control of their angular position. To isolate the servo motors from the microcontroller's power supply, a separate 5V power source and a level shifter (**L293DNE**) are used.
- **Photoresistor**: a photosensitive element that detects light intensity= is connected to one of the ESP32's analog input pins.
- **LED indicator**: a power indicator that "tricks" the agent. Turns on when the training is complete.
- **Rotary encoder**: used to set relative weights of the voltage and the tilt values for the reward function to indicate their relative importance.
- **LCD**: a 16x2 character display that shows the currently set voltage and tilt weights. The LCD is connected to the ESP32 via I2C communication protocol, allowing for efficient data transfer with minimal wiring.
- **Push button 1**: wipes the table and starts the training process.
- **Push button 2**: performs inference from the current state.
- **Push button 3**: sets the system into a "random" state, where both pan and tilt are set to random values within their operational range. The system is then ready for inference.
- **Push button 4**: returns the system into the "0" state, setting both pan and tilt to 0.
