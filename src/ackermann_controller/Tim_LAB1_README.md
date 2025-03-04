# Ackermann Steering: Inverse and Forward Kinematics

This document provides a simplified explanation of inverse and forward kinematics models used in Ackermann steering. It includes key equations, pros, cons, and suitable applications.

---

## 1. Inverse Kinematics (IK) Models



### 1.1 Bicycle Model

**Description**  
- A simplified model representing a four-wheel Ackermann vehicle using one front steering wheel and one rear driving wheel.  
- The front wheel turns at an angle $\delta$, and the rear wheel drives the vehicle forward.

**Key Equations**  

$$
\dot{x} = v \cos(\theta)
$$

$$
\dot{y} = v \sin(\theta)
$$

$$
\dot{\theta} = \frac{v}{L} \tan(\delta)
$$

To find the required steering angle $\delta$:

$$
\delta = \arctan\left(\frac{L \dot{\theta}}{v}\right)
$$

**Pros**  
- Simple and easy to use.  
- Requires fewer parameters, making it fast for real-time control.  

**Cons**  
- Less accurate at high speeds due to missing slip and weight effects.  
- Assumes only one steering wheel, ignoring left-right differences.  

**Suitable Applications**  
- Low-speed robots and path planning.  
- Teaching basic vehicle kinematics.  

---

### 1.2 No-Slip Condition Model

**Description**  
- Assumes that wheels roll without slipping, meaning all wheels must share the same turning center.  
- The left and right front wheels turn at different angles to follow Ackermann steering.

**Key Equations**  

To calculate the ideal wheel angles, the block uses these equations:

$$
\cot(\delta_L) - \cot(\delta_R) = \frac{TW}{WB}
$$

$$
\delta_{Ack} = \frac{\delta_{in}}{\gamma}
$$

$$
\delta_L = \tan^{-1}\left( \frac{WB \cdot \tan(\delta_{Ack})}{WB + 0.5 \cdot TW \cdot \tan(\delta_{Ack})} \right)
$$

$$
\delta_R = \tan^{-1}\left( \frac{WB \cdot \tan(\delta_{Ack})}{WB - 0.5 \cdot TW \cdot \tan(\delta_{Ack})} \right)
$$


Where:
- $\delta_L$ and $\delta_R$ are the left and right wheel angles, respectively.
- $\delta_{Ack}$ is the Ackermann angle.
- $\delta_{in} $ is the input steering angle.
- $ \gamma$ is a scaling factor.
- $TW$ is the track width (distance between the left and right wheels).
- $ WB $ $is the wheelbase (distance between the front and rear axles).
- $P_{Ack}$ is the Ackermann percentage.


#### In case, the code for the Bicycle model and the No-Slip Condition Constraints model shares similar equations, the implementation will be mostly the same, with some differences specific to each model. The code will look like this.
```python 
    def cmd_vel_callback(self, msg: Twist):
            self.linear_velocity = msg.linear.x  # Forward velocity (m/s)
            self.angular_velocity = msg.angular.z  # Rotation velocity (rad/s)

            if self.linear_velocity != 0.0:
                # Bicycle Model
                delta = math.atan((self.WB * self.angular_velocity) / self.linear_velocity)

                # Ackerman Steering Type: No-Slip
                delta_Ack = delta / self.gamma
                delta_L = math.atan((self.WB * math.tan(delta_Ack)) / (self.WB - 0.5 * self.TW * math.tan(delta_Ack)))
                delta_R = math.atan((self.WB * math.tan(delta_Ack)) / (self.WB + 0.5 * self.TW * math.tan(delta_Ack)))
            
            else:
                delta = 0.0
                delta_Ack = 0.0
                delta_L = 0.0
                delta_R = 0.0

            
            if self.model == 'bicycle':
                steering_angle_left_wheel = delta
                steering_angle_right_wheel = delta
            elif self.model == 'ackermann':
                steering_angle_left_wheel = delta_L
                steering_angle_right_wheel = delta_R

            speed_rear_wheel = self.linear_velocity / self.wheel_radius
```
**Pros**  
- More accurate at low speeds since it enforces pure rolling motion.  
- Ensures correct Ackermann geometry.  

**Cons**  
- More complex to implement.  
- Becomes inaccurate at high speeds due to tire slip.  

**Suitable Applications**  
- Precise low-speed maneuvers (e.g., parking, AGVs).  
- Vehicles where minimizing slip is important.  

---

## 2. Forward Kinematics (FK) Models

# Variable Definitions

- **\( x_k \)** - The x-coordinate of the vehicle at time step \( k \).
- **\( y_k \)** - The y-coordinate of the vehicle at time step \( k \).
- **\( \theta_k \)** - The orientation (yaw angle) of the vehicle at time step \( k \).
- **\( \beta_k \)** - The sideslip angle (angle between the velocity vector and the longitudinal axis of the vehicle) at time step \( k \).
- **\( v_k \)** - The linear velocity of the vehicle at time step \( k \).
- **\( \omega_k \)** - The angular velocity (yaw rate) of the vehicle at time step \( k \).
- **\( \Delta t \)** - The discrete time step between \( k-1 \) and \( k \).
- **\( \beta_{k-1} \)** - The sideslip angle at the previous time step \( k-1 \).
- **\( \theta_{k-1} \)** - The orientation at the previous time step \( k-1 \).
- **\( v_{k-1} \)** - The linear velocity at the previous time step \( k-1 \).
- **\( \omega_{k-1} \)** - The angular velocity at the previous time step \( k-1 \).
- **\( \beta_R^* \)** - The effective rear slip angle of the vehicle.
- **\( v_{R,L,k} \)** - The velocity of the rear left wheel at time step \( k \).
- **\( v_{R,R,k} \)** - The velocity of the rear right wheel at time step \( k \).
- **\( r_b \)** - The distance from the center of the rear axle to the vehicle's center of rotation.
- **\( \beta_F^* \)** - The effective front slip angle of the vehicle.


### 2.1 Yaw-Rate Model

**Description**  
- Yaw Rate model estimates a vehicle’s rotational speed around its vertical axis, essential for dead reckoning-based odometry. By integrating the yaw rate over time, the robot or vehicle can estimate its heading. This is often combined with wheel encoder data and IMU (Inertial Measurement Unit) readings to improve accuracy in mobile robots and self-driving cars (Autonomous).

$$
\begin{align*}
\begin{bmatrix}
x_k \\
y_k \\
\theta_k \\
\beta_k \\
v_k \\
\omega_k
\end{bmatrix}
&=
\begin{bmatrix}
y_{k-1} + v_{k-1} \cdot \Delta t \cdot \cos\left(\beta_{k-1} + \theta_{k-1} + \frac{\omega_{k-1} \cdot \Delta t}{2}\right) \\
y_{k-1} + v_{k-1} \cdot \Delta t \cdot \sin\left(\beta_{k-1} + \theta_{k-1} + \frac{\omega_{k-1} \cdot \Delta t}{2}\right) \\
\theta_{k-1} + \omega_{k-1} \cdot \Delta t \\
\beta_{R,k}^{\ast} \\
\frac{v_{R,L,k}^{\ast} + v_{R,R,k}^{\ast}}{2} \\
\quad \frac{v_{k-1}}{r_b} \left(\cos(\beta_{R,k}^{\ast}) \cdot (\tan(\beta_{F,k}^{\ast}) - \tan(\beta_{R,k}^{\ast}))\right)
\end{bmatrix}
\end{align*}
$$

**Pros**  
- Simple and easy to compute.  
- Works well if an IMU is available.  

**Cons**  
- Errors in yaw rate measurements affect accuracy.  
- Does not consider individual wheel dynamics.  

**Suitable Applications**  
- IMU-based odometry.  
- Fast and simple pose estimation.  

---

### 2.2 Single-Track Model

**Description**  
- Single-Track model approximates a four-wheeled vehicle as a single-track system, treating both left and right wheels as one virtual wheel per axle. 
- It simplifies kinematic calculations for estimating the vehicle's position and orientation to determine velocity and heading changes..

$$\begin{align*}
\begin{bmatrix}
x_k \\
y_k \\
\theta_k \\
\beta_k \\
v_k \\
\omega_k
\end{bmatrix}
&=
\begin{bmatrix}
y_{k-1} + v_{k-1} \cdot \Delta t \cdot \cos\left(\beta_{k-1} + \theta_{k-1} + \frac{\omega_{k-1} \cdot \Delta t}{2}\right) \\
y_{k-1} + v_{k-1} \cdot \Delta t \cdot \sin\left(\beta_{k-1} + \theta_{k-1} + \frac{\omega_{k-1} \cdot \Delta t}{2}\right) \\
\theta_{k-1} + \omega_{k-1} \cdot \Delta t \\
\beta_{R,k}^{\ast} \\
\frac{v_{R,L,k}^{\ast} + v_{R,R,k}^{\ast}}{2} \\
\quad \frac{v_{k-1}}{r_b} \left(\cos(\beta_{R,k}^{\ast}) \cdot (\tan(\beta_{F,k}^{\ast}) - \tan(\beta_{R,k}^{\ast}))\right)
\end{bmatrix}
\end{align*}$$


**Pros**  
- Balances simplicity and accuracy.  
- Useful for medium-speed applications. 
- Fast Calculations – Suitable for real-time control applications. 

**Cons**  
- Still an approximation, does not model lateral slip or weight transfer.  
- Errors increase during sharp turns.  

**Suitable Applications**  
- General-purpose mobile robots.  
- Mid-speed autonomous vehicles.  

---

### 2.3 Double-Track Model

**Description**  
- Double-Track model considers each wheel independently to make it more precise for odometry. By accounts for lateral wheel slips, load transfers, and individual wheel speeds. This is useful in high-accuracy odometry for vehicles with independent drive wheels or complex dynamics like four-wheeled mobile robots (ackerman model like we did) . 


$$\begin{align*}
\begin{bmatrix}
x_k \\
y_k \\
\theta_k \\
\beta_k \\
v_k \\
\omega_k
\end{bmatrix}
&=
\begin{bmatrix}
x_{k-1} + v_{k-1} \cdot \Delta t \cdot \cos\left(\beta_{k-1} + \theta_{k-1} + \frac{\omega_{k-1} \cdot \Delta t}{2}\right) \\
y_{k-1} + v_{k-1} \cdot \Delta t \cdot \sin\left(\beta_{k-1} + \theta_{k-1} + \frac{\omega_{k-1} \cdot \Delta t}{2}\right) \\
\theta_{k-1} + \omega_{k-1} \cdot \Delta t \\
0 \\
\frac{\tilde{v}_{RL,k} + \tilde{v}_{RR,k}}{2} \\
\frac{\tilde{v}_{RR,k} - \tilde{v}_{RL,k}}{TW}
\end{bmatrix}
\end{align*}$$


**Pros**  
- Most accurate for real-world applications.  
- Works well at high speeds and in dynamic environments.  

**Cons**  
- Requires more sensors (wheel speeds, individual steering angles).  
- Computationally more expensive.  

**Suitable Applications**  
- High-speed vehicles (racing, advanced robotics).  
- Automotive research and testing.  

#### The code for all three models has a similar structure but differs in some equations, like those involving $\omega_k$, the implementation will mostly be the same, with some variations for each model. The code will look like this:

```python
    def state_space(self):
        self.update_state_space = [0, 0, 0, 0, 0, 0]
        self.update_state_space[0] = self.odom[0] + (self.odom[4] * self.dt * math.cos(self.odom[3] + self.odom[2] + (self.odom[5]* self.dt)/2))
        self.update_state_space[1] = self.odom[1] + (self.odom[4] * self.dt * math.sin(self.odom[3] + self.odom[2] + (self.odom[5]* self.dt)/2))
        self.update_state_space[2] = self.odom[2] + (self.odom[5] * self.dt)
        self.update_state_space[4] = (self.rear_vel[0] + self.rear_vel[1]) / 2

        if(self.kinematic_model == 'single_track'):
            self.update_state_space[5] = (self.odom[4]/self.wheelbase) * math.tan(self.delta)
            # print("4")
            # print(self.update_state_space)
        elif(self.kinematic_model == 'double_track'):
            self.update_state_space[5] = (self.rear_vel[0] - self.rear_vel[1]) / self.track_width
        elif(self.kinematic_model == 'yaw_rate'):
            self.update_state_space[5] = self.yaw
            # print("5")

        if self.kinematic_model != 'ground_truth':
            self.odom = self.update_state_space
            self.odom_pub(self.odom)
        else:
            self.odom = self.odom_ground_truth
            self.odom_pub(self.odom_ground_truth)
```
---

## 3. Model Selection Guide (เผื่อเอาไปใช้ตอนสรุป Validation)

- **Inverse Kinematics (IK)**  
  - **Bicycle Model**: Best for simple path-following and control.  
  - **No-Slip Model**: Best for precise low-speed maneuvers.  

- **Forward Kinematics (FK)**  
  - **Yaw-Rate Model**: Easy to implement; works well with IMU.  
  - **Single-Track Model**: More accurate for steering-based motion.  
  - **Double-Track Model**: Best for high-speed and complex dynamics.  

---

