# Movement
A suitable motor for movement that can handle the robot's weight, which is relatively moderate with a maximum of **50 kg** as per Robocon's requirements. At the same time, a motor with quick response time for both acceleration and deceleration is needed. The best option appears to be a **brushless DC motor**
# 1. [VG.ECI4240BK1.PP42.1.5](https://www.digikey.com/en/products/detail/ebm-papst-inc/VG-ECI4240BK1-PP42-1-5/9598034?_gl=1*1c5pi6s*_up*MQ..&gclid=CjwKCAjwufq2BhAmEiwAnZqw8neDLmT8JB8UBWAWx6kuFay3l1NYqvf_K7xgLWcWFx7U-LIXoyBmOBoCnNsQAvD_BwE)
![MFG_VG ECI4240BK1 PP42 1 5](https://github.com/user-attachments/assets/dbfd1743-0828-4a05-9431-8e4c91295b75)


* RPM = 800 RPM
* Torque - Rated = 1.1 Nm
* Size / Dimension = Dia (42.00mm)
* Gear Reduction Ratio = 5
  
**Its advantages are that it is relatively cheaper, smaller in size, and has a significantly higher speed.**

# 2. [MOT-I-81491-L](https://www.digikey.com/en/products/detail/isl-products-international/MOT-I-81491-L/16553975?_gl=1*15thki5*_up*MQ..&gclid=CjwKCAjwufq2BhAmEiwAnZqw8neDLmT8JB8UBWAWx6kuFay3l1NYqvf_K7xgLWcWFx7U-LIXoyBmOBoCnNsQAvD_BwE)
![MFG_MOT-I-81491-L](https://github.com/user-attachments/assets/38b59154-de00-47fb-8ae5-e956a3838dd7)



* RPM = 58 RPM
* Torque - Rated = 8.53 Nm
* Size / Dimension = Dia (57.00mm)
* Gear Reduction Ratio = 77

**What sets this motor apart is its higher torque compared to the first one, but its downsides are the lower speed and larger size.**
# **Dribbling Motors**
The dribbling mechanism needs to ensure smooth, continuous motion, high torque, and precise control over the ball’s movement.
## Characteristics for Dribbling
 1. **Speed Control:**
    * capable of running at **moderate speeds** to control how fast the ball is bounced.
    * Dribbling requires precise speed adjustments.
    * speed controllers (PWM).
 2. **Torque:**
    * High torque to handle the ball's weight.
 3. **Precision Control:**
    * **encoders** to ensure accurate control and feedback during the dribbling process.
## Gearbox and Power Considerations
* A gearbox can adjust the motor's torque-to-speed ratio
* Power requirements should match the overall robot's system to ensure smooth operation without overloading the power supply.
##  Type of Motors Suitable for Dribbling
 1. [**37D Metal Gearmotors with encoder**](https://www.digikey.com/en/products/detail/pololu-corporation/4756/10450210)
  ![MFG_2183_4756](https://github.com/user-attachments/assets/c30b584a-024a-4c2f-9b00-bdc2af1ff60e)

  
  * High torque with gearbox (gear ratio 131.25:1).
  * PWM for precise control. 
  * encoder for real-time feedback.
  * **Advantages:**
     * Continuous Motion for dribbling.
     * maintaining ball control during dribbling due to **High Torque at Low Speeds**.
  * **Disadvantages:**
     * More Complex Control: requires motor drivers.
  * **Specs:**
     * Speed at max efficiency:    66 rpm.
     * Torque at max efficiency:    6.0 kg·cm.
     * Stall torque at 24V:    47 kg·cm3
 2. [**Power HD 1235MG**](https://ar.aliexpress.com/i/1005003538785235.html?gatewayAdapt=glo2ara)
  ![0J4979 600x480](https://github.com/user-attachments/assets/6bbea9cd-0b99-4e3d-abdf-ef50fdb0ce55)

  
  * **Advantages:**
     * **High torque** maintaining control during dribbling
     * **metal gears** ensure durability during continuous action.
     * Continuous Motion for dribbling.
  * **Disadvantages:**
     * Speed Limitation.
     * Cost: higher than standard servos.
  * **Specs:**
     * Speed at 7.4 V:    0.18 sec/60°.
     * Stall Torque at 7.4 v:    40 kg·cm.

### Mechanism
The dribbling mechanism involves two motors. The first motor, a servo motor, will control the movement of the robot’s arm in the horizontal plane to track the ball's position. The servo is suitable for this task due to its feedback capabilities, allowing precise angle adjustments based on the ball’s location. The second motor, either a DC motor or another servo motor, will handle the dribbling action itself. This motor will rotate continuously around the Z-axis, to make sure the ball is bouncing

# Shooting and Passing

-   **Shooting:** Needs **high torque and speed** to propel the ball with force, requiring a powerful motor that operates at high RPMs.
    
-   **Passing:** Requires **precision and controlled torque** for accurate ball handling, with less emphasis on high speed.
    

Using separate motors is ideal: a **Brushless motor** for shooting and a **DC motor with a gearbox** for passing. This ensures each motor is optimized for its task, simplifies the design, and enhances reliability.

### 1.   **Shooting:** [**Hobbywing EZRUN 4268**](https://www.hobbywingdirect.com/products/ezrun-4268-4278-motor-g2?pr_prod_strat=pinned&pr_rec_id=01586b278&pr_rec_pid=6825959784563&pr_ref_pid=6825954902131&pr_seq=uniform&variant=40027944157299) – High torque and speed for powerful shooting.
![3068d77b282f79b591fcd5026216a4ff](https://github.com/user-attachments/assets/64d0a29d-acb1-41ca-bbb9-c4444ae8db76)

-   Power Output: With its high power rating (1500-2000W), the EZRUN 4268 provides the necessary force to propel a basketball effectively.
    
-   High Torque: This motor is capable of delivering significant torque, which is essential for shooting mechanisms that require strong, forceful movement.
    
-   Speed: The high KV rating enables the motor to achieve high speeds, which is beneficial for achieving the necessary launch velocity for the ball.
    
### 2.  **Passing:** [**Pololu 37D Gearmotor**](https://www.pololu.com/product/4750) with a gearbox – Provides the necessary torque and control for precise passing.
  ![0J10454 600x480](https://github.com/user-attachments/assets/ba6a11b3-ed6b-411e-943d-7a730d94e791)

  - Torque: The Pololu 37D Gearmotor can deliver up to 10 kg-cm of torque with certain gearbox configurations. This should be sufficient for passing a basketball over short to moderate distances.

- Speed: You can select different gear ratios for the Pololu 37D to balance speed and torque. For passing, you'd want enough speed to pass the ball effectively, but also precise control to avoid overshooting.

    
