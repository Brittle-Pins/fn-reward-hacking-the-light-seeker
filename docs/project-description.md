# Reward Hacking: The Self-Fulfilling Light Seeker. Project Description

**Reward hacking** (or **specification gaming**) is a phenomenon in which an agent exploits loopholes in the reward function to achieve high rewards without actually accomplishing the intended task. This project aims to demonstrate reward hacking by a robotic device imitating a solar panel, which is designed to turn towards the light source to maximize its energy intake. However, instead of genuinely seeking the ambient light, the device will learn to find an power indicator on the stationary base of the device. The brightness of the LED indicator exceeds that of the ambient light thanks to its proximity to the sensor, prompting the agent to ignore the environment and lock itself in sub-optimal state.

## Hardware

The light seeker consists of a stationary base and a platform that can rotate around two axes. The device is equipped with a photosensitive element (a photoresistor) that is used to locate the light source. Two servo motors are responsible for the movement of the platform. The LED indicator is placed on the base. The interaction of components is coordinated by an ESP32 microcontroller.

The detailed list of required components and their connections can be found in the [hardware documentation](hardware.md).

To ensure the reliability and reproducitility of the experiment, the photoresistor is deeply recessed within an opaque cylindrical blinder that narrows the sensor's field of view. This forces the system to rely on the mechanical pan-tilt rotation to seek out light sources, rather than relying on diffuse, off-axis ambient light.

## Theoretical foundations

### Markov Decision Processes (MDP)

At its core, a Markov Decision Process is a mathematical framework used to describe an environment in reinforcement learning. It relies on the Markov Property, which states that the future depends only on the current state and the action taken, not on the historical sequence of events that preceded it.

Formally, an MDP is defined by a tuple $(S, A, P, R, \gamma)$:
- $S$: A set of states (the state space).
- $A$: A set of actions (the action space).
- $P$: The transition probability function (how likely it is to move from one state to another given an action).
- $R$: The reward function (the immediate return for taking a specific action in a specific state).
- $\gamma$: The discount factor (which determines the importance of future rewards versus immediate ones).

Framed as a manifestation of Goodhart's Law, the interaction between the agent and the environment is described in terms of Markov Decision Processes. To achieve that, the continuous physical space is reduced into a computationally manageable matrix. Under the MCU memory limitations, the *state space* is discretised into a grid of 12 by 12 cells (non-overlapping states), where the functional range of each servo motor of 0 to 165 degrees is divided into uniform increments of 15 degrees.

The *action space* is comprised of four mutually exclusive movements: `PAN_LEFT`, `PAN_RIGHT`, `TILT_UP`, and `TILT_DOWN`.

> Note: To protect the motors from burnout, boundary conditions are enforced algoritmically, so that if an action attemts to drive a servo beyond its operation range, the state remains unchanged.

The *transition function*, while mathematically defined as a probatility of the system transitioning to state `s'` given the current state `s` and action `a`, is deterministic in this setup. Its purpose is to demonstrate the reward hacking phenomenon via tabular Q-learning, therefore it is assumed that `P(s'|s,a) = 1` for all valid, non-boundary transitions.

The *reward function* is designed to maximise the voltage output of the photoresistor, which is directly proportional to the intensity of light it receives. This definition encapsulates the **proxy alignment problem**, where the agent optimises for a proxy (the identification of the state yielding peak voltage) rather than the true objective (the identification of the state yielding the maximum energy from the ambient light source).

Under training, the agent iteratively learns a policy that maximises the expected cumulative discounted reward over time.

### Q-Learning

#### The Origins of Q-Learning

Q-Learning was introduced by Chris Watkins in 1989 as part of his PhD thesis. The "Q" stands for "Quality" — specifically, the quality of a given action in a given state.

Originally, it was developed to solve Markov Decision Processes (MDPs) where the agent does not have a complete model of the environment (model-free). Prior to Q-learning, algorithms often required the agent to know the exact transition probabilities (e.g., "If I move left, there is an 80% chance I succeed and a 20% chance I slip"). Watkins proved that an agent could learn optimal behavior purely through trial, error, and a clever mathematical update rule, without needing a predefined map of the world's physics.

#### Common applications

Historically, it was applied to discrete grid-world problems like maze navigation, simple games, and basic robotic pathfinding. Today, it remains a foundational technique for sequential decision-making problems, including autonomous navigation, dynamic resource allocation in telecommunications, and industrial control systems where an agent must optimize a long-term goal.

#### Tabular vs. Deep Q-Learning

In modern machine learning, a common implementation of the algorithm is Deep Q-Learning (DQN). At the core of Q-learning is its mathematical engine, the Bellman equation:

$Q(s,a) \leftarrow Q(s,a) + \alpha [r + \gamma \max_{a'} Q(s',a') - Q(s,a)]$

In Tabular Q-Learning, we are maintaining a matrix (a lookup table) of every possible state-action pair (the 12 by 12 by 4 grid in case of the Light Seeker). Occupying only 2.25 KB of memory, this is highly efficient for an MCU. However, if we wanted to increase the granularity and track 1,000 pan positions, 1,000 tilt positions, and add a third motor (the third rotation axis), the table would explode into millions of values. This is known as the "curse of dimensionality."

In Deep Q-Learning, the lookup table is entirely replaced by an Artificial Neural Network. Instead of looking up a specific row and column, the continuous state data (like the exact raw voltage and precise servo angles) is fed into the neural network, and it outputs the expected Q-values for the four actions.

Implementing Deep Q-Learning on an MCU being computationally expensive and memory-prohibitive, this solution would cause a fundamental architectural shift in the design of the system. The computation could be offleaded to a powerful PC over Wi-Fi, and the microcontroller would transition from being the "brain"of the system to acting as an edge client. The benefints of DQL over TQL, however, would include continuous state space, generalisation, and advanced state definitions, including, for example, the momentum of the system. On the other hand, network latency and occasional packet loss would need to be accounted for.

### Hysteresis loops

Running tabular Q-learing in physical hardware requires a large number of iterative physical motor movements to achieve convergence. The discretised state space somewhat limits the micro-movements, but the hadware wear and mechanical jitter are further prevented by implementing **hysteresis loops** that ensure smooth physical transitions.

In physical systems, hysteresis is the dependence of a system's state on its history. In the context of control systems and robotics, it translates to creating a deadband or a buffer zone around a threshold. Instead of a single, razor-thin line dictating when a system should change states, hysteresis uses two different thresholds: one for moving forward and a slightly different one for moving backward. Until the input signal completely crosses out of this buffer zone, the system ignores the change and maintains its current state.

To express this mathematically, we define a threshold $\epsilon$ (the hysteresis margin or deadband width). Let $\theta_{current}$ be the currently applied physical position of the servo, and $\theta_{target}$ be the newly calculated ideal position from the Q-learning policy.The physical position applied at time $t$, denoted as $\theta_{applied}(t)$, is only updated if the absolute difference between the target and the current position strictly exceeds the deadband $\epsilon$:

$$\theta_{applied}(t) =
\begin{cases}
\theta_{target}(t) & \text{if } |\theta_{target}(t) - \theta_{applied}(t-1)| > \epsilon \\
\theta_{applied}(t-1) & \text{otherwise}
\end{cases}$$

Algorithmically, this translates into a lightweight filter function. The implementation simply stores the last known valid state:

```C++
#include <math.h>

// Define the deadband threshold (e.g., 2.0 degrees)
#define HYSTERESIS_MARGIN 2.0f 

float current_servo_angle = 90.0f; // Track the physical state

/**
 * Calculates the next physical position, applying a hysteresis deadband.
 */
float apply_hysteresis(float target_angle) {
    // Calculate the absolute delta between target and current state
    float delta = fabs(target_angle - current_servo_angle);

    // Only update if the target has moved outside the deadband
    if (delta > HYSTERESIS_MARGIN) {
        current_servo_angle = target_angle;
    }
    
    // If the delta is within the margin, return the old, stable angle
    return current_servo_angle;
}

void control_loop(float q_learning_target_angle) {
    // 1. Filter the raw algorithm target through the hysteresis function
    float stable_angle = apply_hysteresis(q_learning_target_angle);
    
    // 2. Command the physical hardware (pseudo-code)
    // set_servo_pwm(stable_angle);
}
```

## Physics of the exploit

The light intensity is inversely proportional to the square of the distance from the source. As the emitted radiation travels from the origin, it spreads out over an expanding spherical area. The surface area of a sphere increases proportionally to the square of its radius, resulting in the dilution of the energy. This means that the source that is 100 times closer to the sensor will appear 10,000 times brighter.

## Experiment workflow

The observed reward hacking occurs during the training phase, and it is entirely a symptom of a misspecified reward function. The agent is not "breaking the rules"; it is perfectly optimizing for a flawed proxy metric (voltage) rather than the true objective (ambient light). During inference, the Light Seeker is simply executing the optimal, yet factually incorrect, policy it learned.

The demonstration aims to show how the improved specification of the reward function can mitigate the reward hacking phenomenon. By adjusting the reward function to account both for the voltage and the tilt angle, the agent is incentivized to seek out the ambient light source rather than the LED indicator. The relative weights of these two components can be adjusted using a rotary encoder. The experiment workflow is as follows:

- State 0 (Idle): The Q-table array is zeroed out via a simple memset. The platform is stationary.
  > Filling the table with random values is sometimes used in deep learning to break symmetry, but in tabular Q-learning for a deterministic physical environment, random initial values can introduce unwanted noise and misguide early exploration. By initializing the 2.25 KB memory grid entirely to zero, every state-action pair starts on equal footing.
- State 1 (Training): The user sets $w_v = 1.0$ and $w_t = 0.0$ using the rotary encoder and presses the encoder knob (SW) to start training. The device steps through the grid, reads the normalized sensor data, applies the reward function, and fills the Q-table.
- State 2 (Positioning): Training completes, as indicated by the LED turning on. The platform deliberately drives to a randomized, neutral pan-tilt position so that the inference starts from a fresh location every time.
- State 3 (Inference): Triggered by a Button 1 press. The agent strictly exploits the Q-table (exploration rate is 0). Because $w_v = 1.0$, it confidently locks onto the LED indicator on the base.
- State 3a (Iterative inference): A Button 2 press resets the system to a random pan-tilt position. After a Button 1 press, the agent again exploits the Q-table. The LED trap is consistently found, and the agent repeatedly returns to the sub-optimal state.
- State 4 (Iteration): By turning the rotary encoder, the user dials new weights (e.g., $w_v = 0.6, w_t = 0.4$) and presses the encoder knob to reset the system, wipe the table, and restart the training cycle. As $w_t$ increases, the algorithm mathematically overcomes the localized LED trap and successfully finds the ambient light above.

### Weighted reward function

The use of a convex combination of weights turns the problem into multi-objective reinforcement learning. The reward function is defined as:

$$R = w_v \cdot V_{norm} + w_t \cdot T_{norm}$$

However, using raw values for position and voltage can lead to a severe scale mismatch. The raw voltage from the photoresistor is read as a 12-bit ADC value ($0$ to $4095$), while the tilt state is represented by a 0-based grid index ($0$ to $11$). If these raw values were multiplied by weights directly, the massive ADC numbers would completely overpower the tilt index, rendering any weight adjustments useless. 

Before applying the weights, it is necessary to mathematically normalize both the sensor data and the spatial data to a common scale of $0.0$ to $1.0$.

#### Voltage Normalization ($V_{norm}$)
The photoresistor voltage reading is simply divided by the maximum 12-bit ADC resolution:
$$V_{norm} = \frac{\text{adc\_reading}}{4095.0}$$

#### Inverted Tilt Normalization ($T_{norm}$)
The tilt normalization requires special physical consideration. The mechanical assembly is constructed such that an angle of $0^\circ$ (tilt index $0$) points the sensor straight up towards the ceiling (the ambient light), while $165^\circ$ (tilt index $11$) points it straight down at the glowing LED indicator on the base.

Because the goal of adjusting the weights is to incentivize the agent to look *up* and ignore the LED trap below, we must design the reward such that looking up yields the highest reward ($1.0$), and looking down yields the lowest reward ($0.0$). This requires an inverted calculation:
$$T_{norm} = \frac{11.0 - \text{tilt\_index}}{11.0}$$

By normalizing the components in this way, the two values can be safely multiplied by their respective weights.

Strictly speaking, the weights do not have to sum to 1 for the Q-learning algorithm to execute. However, doing so—creating a convex combination—is critically important for both mathematical stability and pedagogical clarity.

1. The Attention Budget (Pedagogy): Constraining the sum to 1 frames the weights as percentages of an "attention budget." It visually demonstrates a clear trade-off: the agent cannot care 100% about the light proxy and 100% about its physical orientation. It forces a compromise (e.g., 60% proxy / 40% orientation). This framing is vastly easier to explain in a technical field journal and makes the narrative of "correcting the objective" intuitive.
2. Reward Bounding (Mathematics): Assuming both voltage and tilt values are normalized to a $0.0$ to $1.0$ scale ($V_{norm}$ and $T_{norm}$), constraining the weights so that $w_v + w_t = 1$ mathematically guarantees that the final calculated reward $R$ will also strictly fall between $0.0$ and $1.0$.$$R = w_v \cdot V_{norm} + w_t \cdot T_{norm}$$
3. Hyperparameter Stability (Engineering): The Q-learning Bellman update relies on a carefully tuned learning rate ($\alpha$) and discount factor ($\gamma$). If the weights are adjusted independently without bounding them (e.g., changing $w_v$ from $1.0$ to $5.0$ and $w_t$ to $10.0$), the scale of rewards inflates unpredictably. A sudden influx of massive reward values can lead to massive Q-value updates, causing the gradients to overshoot and destabilizing the convergence of the Q-table. By keeping the total maximum reward bounded to $1.0$, the $\alpha$ and $\gamma$ hyperparameters will behave consistently no matter how the two objectives are balanced.
