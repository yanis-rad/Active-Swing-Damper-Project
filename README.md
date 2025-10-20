# Active Swing Damper Control Software

This repository contains the source code for the **Active Swing Damper (ASD)**, a control system designed to actively reduce oscillations 
in a rocker subjected to external forces. 

## Overall view
The system uses a PID controller to command a DC motor based on feedback from an IMU and the motor encoder. More specifically, the angular velocity 
of the rocker is recorded and a proportional target speed is computed. The PID follows the target speed during the acceleration phase when the angular velocity keeps increasing.
Once the angular velocity reaches its maximum and begins to decrease, the motor enters the deceleration phase. During this phase, braking is applied to stop the flywheel before initiating rotation in the opposite direction.

The following graph shows the overall behaviour of ASD based on the data coming from the IMU:
<img width="815" height="564" alt="image" src="https://github.com/user-attachments/assets/38724eeb-c7a5-44a4-9d46-8a1b36f6aa16" />

---
## Hardware Components

- STM32F411-CEU6 Black Pill microcontroller  
- L298N Motor Driver  
- MPU6050 IMU  
- MicroSD Reader  
- Brushless 1000RPM DC Motor with encoder  
- 2 × LEDs (Green for normal operation, Red for fault indication)  
- 2 × 4.2V Batteries

First prototype image:

<img width="815" height="670" alt="image" src="https://github.com/user-attachments/assets/926a9a43-182c-4a17-9c55-503059662ef5" />

---
## Data Collection

The system measures the following variables during operation:
- **Angular Speed (Y-axis, IMU):** Used to set the target speed and detect acceleration and deceleration.
- **Linear Acceleration (X-axis, IMU):** Assists in identifying acceleration and deceleration phases.
- **Flywheel Speed (Motor Encoder):** Used as feedback in the speed control loop.
---
## State Machine

After initialization ASD operates in two main states, and reaches a third state in case of problems. 
The three states are: 
1. **Acceleration State:** accessed by default or when the rocker changes its direction of rotation. 
   In this state the PID is actively reacting proportionally to the angular velocity of the rocker.
2. **Deceleration State:** accessed once the rocker reaches a local peak of angular speed and starts to decelerate.
   In this state the motor enters braking mode to stop the flywheel before reversing direction.
3. **Fault State:** accessed if either the MicroSD reader is reporting problems or the IMU is not exchanging data correctly. 
   In this state the logic deactivates the motor for safety reasons. 

<img width="815" height="477" alt="image" src="https://github.com/user-attachments/assets/ddfea06a-b00f-430b-82ba-397fb97e6942" />


---
## Diagnostics
The connection statuses of both the IMU and the MicroSD reader are monitored. The state of the operations is shown to the user by using two LEDs:
- **Green LED** indicates normal operation.
- **Red LED** signals connection issues.

---

## Control Logic
The goal of the control system is to produce a torque in response to the oscillations of the rocker. To do this a speed control loop is implemented using a **PID controller**. The PID logic is active only during the acceleration phase and follows the target speed that is assigned as follows:
- During **acceleration**, the target speed is proportional to the angular velocity of the rocker.
- During **deceleration**, the target speed is reset to zero. 
Given the fast dynamics of the system, ASD implements the PID speed controller only with the **P and D terms** the coefficients of which have been tuned appropriately to give a fast and fluid response.


<img width="815" height="550" alt="image" src="https://github.com/user-attachments/assets/595cc69e-cb4c-4a04-94dd-265bb024bcc5" />





---
## Results

Multiple test were performed to asses the performance of ASD. The main test used for comparison consisted in positioning the rocker in the same unstable initial condition and left oscillating with and without the ASD. The collected data showed:
- Faster return to stability with the rocker arresting 30% faster.
- Peak angular speeds lower by 40% on average.
- Reduced oscillation frequency by 9% on average.
- Less overall oscillations before reaching stable position.

<img width="815" height="430" alt="image" src="https://github.com/user-attachments/assets/0d0a4546-e025-44c5-87e4-fe311e6d37ba" />

## Next Steps
- Improve prototype for better symmetry during rotation
- Improve braking logic for more fluid braking
- Rewrite sw in C++


