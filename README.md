# First Self-Balancing-Robot-PID-system-.

---
Introduction.

This repository contains the Arduino source code for a two-wheeled self-balancing robot. The robot uses an MPU-6050 Inertial Measurement Unit (IMU) to sense its orientation and a PID (Proportional-Integral-Derivative) controller to drive two DC motors, keeping the robot upright.

---
Hardware Needed:
- Arduino Uno
- MPU-6050 IMU Sensor
- L298N Motor Driver
- 2x DC Geared Motors with Wheels
- Robot Chassis
- 2x 12V batteries

---

Wiring: 
Arduino Pin	Connects to
- A4 (SDA)	MPU-6050 SDA
- A5 (SCL)	MPU-6050 SCL
- Pin 4 	  L298N ENA
- Pin 5	    L298N IN1
- Pin 6	    L298N IN2
- Pin 10	  L298N ENB
- Pin 8  	  L298N IN3
- Pin 9 	  L298N IN4
- Connect 5V and GND from Arduino to the MPU-6050 and L298N
- Power the L298N's motor input with the battery

---

Software Uses:
- I2Cdev:                      It makes the connection easier to use by simplifying how data is sent and received.                  

- MPU6050 (by Jeff Rowberg):   It knows the specific "language" of the MPU-6050, turning complex commands into simple functions like mpu.getAcceleration().

- Wire.h:                      It creates the basic I2C communication link between the Arduino and the sensor.

---

Calibration and Tuning Process:
Step 1: Find the Balance Angle (pitch_trim_offset)

1.1 - Upload the code.

1.2 - Hold the robot perfectly upright so it's balanced.

1.3 - Open the Serial Monitor at 115200 baud.

1.4 - Look at the Pitch, value being printed.

1.5 - Set pitch_trim_offset in the code to the value and re-upload.

---

// === IMPORTANT: TUNE THIS VALUE FIRST ===
float pitch_trim_offset = 0.0f; // Set this to the value that show in pitch found
(To make the robot center)

---

Step 2: Tune the PID Controller

2.1 - Place the robot on the ground and gently hold it. Adjust the values in the code and re-upload until it balances on its own.

2.2 - Tune Kp (Proportional): Set Ki and Kd to 0. Slowly increase Kp until the robot starts to react and oscillate quickly. A good Kp is usually just below the point of violent shaking.

2.3 - Tune Kd (Derivative): Slowly increase Kd to reduce the oscillations. This will "dampen" the robot's movement and make it stable.

2.4 - Tune Ki (Integral): If the robot drifts slightly, add a very small Ki (e.g., 0.1 or 0.2), too much Ki will make it unstable.

---

Key Control Variables:
- Kp, Ki, Kd                           : The PID gains for tuning.
- maxPWM                               : Sets the maximum motor speed (0, 255)(Goes Forward),
                                         (0, -255) (Goes Backward)
- deadZoneForward, deadZoneBackward    : Prevents motor jitter when balanced. Increase if motors hum.
- motorA_correction, motorB_correction : Adjust if one motor is faster than the other.

---

Important Code Notes and Tips:
-  If the robot runs away instead of balancing, find the line float pitch = -pitch_gyro_angle in the readPitch() function and adjust the negative and positive sign to make the robot steady not leaning forward       and backward. This is because the sensor might be mounted differently.

-  The code currently relies only on the gyroscope, which can cause the robot to slowly drift and fall. For better stability, find constexpr float alpha = 1.0f; and change it to 0.98 (Changing it to 0.98 blends     the gyroscope's fast reactions with the accelerometer's stable sense of gravity to stop the robot from slowlydrifting and falling over).

-  The robot will wait 2 seconds after starting before it tries to balance. This is normal and spend time to set it up.

-  To make the robot move, change the speedControlInput variable from 0.0. A positive value will make it lean and move forward; a negative value will make it move backward.

---

Extra Information: (Issue, Improvement)
- Issue and Improvement (Code) :  The robot initially exhibited significant oscillations, which were eliminated by tuning the Kp, Ki, and Kd parameters of the PID controller to achieve stable self-balancing.

- Issue and Improvement (Wiring): The connection instability was caused by corrosion on the cable's terminal, and the issue was resolved by replacing the wire.

- Issue and Improvement (Fabric): The robot's all-metal construction resulted in excessive weight and a high center of gravity, which hindered its ability to self-balance. To improve stability, the upper portion                                    of the chassis was replaced with a lighter plastic material, effectively lowering the center of gravity.

---
  

