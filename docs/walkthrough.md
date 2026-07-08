# The Self-Fulfilling Light Seeker

This walkthrough details the full implementation of the ESP32 firmware, from hardware control to the final Reinforcement Learning Inference engine.

## What was changed

### 1. Hardware & Core UI
- **Smooth Servo Control**: Abstracted the PWM hardware into a background FreeRTOS task that uses cosine easing to gracefully glide the motors.
- **Interrupt UI**: Installed a GPIO ISR for the rotary encoder to flawlessly track fast turns and update the Q-Learning weights ($w_v$ and $w_t$).
- **NVS Memory**: Preserves weights and the entire Q-Table across reboots.

### 2. The Training Phase
Triggered by pressing the Encoder Knob.
1. Sweeps the physical world (144 grid positions).
2. Uses the ADC on Pin 13 to record the real-world light levels ($V_{norm}$).
3. Calculates the true reward matrix in memory.
4. Performs 1,000 iterations of the Bellman equation to instantly solve the Q-Table.
5. Saves the resulting Q-Table to flash memory and turns on the LED!

### 3. The Inference Phase
Triggered by pressing Button 1 (GPIO 18).
The robot will now actively use its knowledge to find the light!
- The `inference_task` looks at the robot's current position and consults the Q-table to find the action with the absolute highest expected reward.
- It steps the servo motors exactly one grid square in that direction.
- **Oscillation Prevention**: It maintains a temporary "visited" map. If it determines that the next best move is to step onto a square it has already visited during this run (meaning it has reached the peak reward and is about to bounce back and forth forever), it halts and declares the destination found!

## How to Run the Experiment
1. Boot the robot. If a Q-table is already saved, the LED will be on.
2. Dial your weights using the knob (e.g. `wV: 1.0` and `wT: 0.0`).
3. Press the Encoder Knob to wipe the memory and start Training. 
4. Once training finishes and the LED turns on, press **Button 2** to randomize the robot's starting position.
5. Press **Button 1** to start Inference! The robot will smoothly step its way directly toward the highest reward.
6. Try dialing the weights the other way around (`wV: 0.0` and `wT: 1.0`), re-train, and watch how its final destination completely changes!
