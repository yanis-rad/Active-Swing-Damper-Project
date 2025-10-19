# Active Swing Damper Control Software

This repository contains the source code for the **Active Swing Damper (ASD)**, a control system designed to actively reduce oscillations 
in a rocker subjected to external forces. 
The system uses a PID controller to command a DC motor based on feedback from an IMU and the motor encoder.
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

<img width="675" height="500" alt="image" src="https://github.com/user-attachments/assets/72faa524-e769-49ac-a1d4-7f0d7d16e54a" />



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

<img width="520" height="321" alt="Screenshot 2025-10-17 093359" src="https://github.com/user-attachments/assets/39c6467f-ac91-47f8-98a2-d7457c7ca51f" />

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


<img width="682" height="453" alt="image" src="https://github.com/user-attachments/assets/35cb1a34-933e-4441-97bc-707735823a41" />


---
## Results

Multiple test were performed to asses the performance of ASD. The main test used for comparison consisted in positioning the rocker in the same unstable initial condition and left oscillating with and without the ASD. The collected data showed:
- Faster return to stability with the rocker arresting 30% faster.
- Peak angular speeds lower by 40% on average.
- Reduced oscillation frequency by 9% on average.
- Less overall oscillations before reaching stable position.

  <img width="815" height="487" alt="Screenshot 2025-10-12 210849" src="https://github.com/user-attachments/assets/10b12386-d526-42e3-b766-7c5dea72b39e" />

