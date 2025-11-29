# Pedro Pathing - Complete Setup & Tuning Guide


## Table of Contents
1. [Initial Setup](#initial-setup)
2. [Localizer Configuration](#localizer-configuration)
3. [Automatic Tuners](#automatic-tuners)
4. [Manual PIDF Tuning](#manual-pidf-tuning)
5. [Testing & Validation](#testing--validation)
6. [Troubleshooting](#troubleshooting)

---

## Initial Setup

### Prerequisites
- FTC Panels Dashboard installed
- Robot with mecanum drivetrain
- Odometry pods installed (for Pinpoint localizer)
- Battery fully charged

### Step 1: Set Robot Mass

Your robot's mass is used to compensate for centripetal force during curved paths.

**Finding Your Robot's Mass:**
1. Weigh yourself on a scale (write down weight)
2. Hold your robot and weigh yourself + robot
3. Subtract your weight: `Robot Mass = (You + Robot) - You`
4. Convert to kilograms: `Mass (kg) = Mass (lbs) × 0.453592`

**Add to PedroConstants.java:**


### Step 2: Configure Drivetrain

Add mecanum drivetrain constants to `PedroConstants.java`:



⚠️ **IMPORTANT:** Match motor names to your hardware configuration! You will likely need to reverse one side.

**Update createFollower method:**


---

## Localizer Configuration

### Using goBILDA Pinpoint Odometry Computer

#### Prerequisites
- Two odometry pods connected to Pinpoint:
  - **Forward pod** (parallel to chassis) → Port X
  - **Lateral pod** (perpendicular to chassis) → Port Y
- Pinpoint connected to REV Hub via I2C (NOT port 0)

#### Step 1: Add Pinpoint Constants

In `PedroConstants.java`:



#### Step 2: Measure Odometry Pod Offsets

Offsets are measured from your robot's **center of rotation** (usually center of wheelbase).



**Measuring:**
- **forwardPodY**: Distance from center to forward pod (+ if forward, - if backward)
- **strafePodX**: Distance from center to strafe pod (+ if right, - if left)

**Add to createFollower:**


#### Step 3: Verify Encoder Directions

1. Select **Tuning** OpMode on Driver Hub
2. Navigate to **Localization** folder → **Localization Test**
3. Run the OpMode
4. **Push robot forward** → X coordinate should **increase**
5. **Push robot left** → Y coordinate should **increase**

**If directions are wrong:**


---

## Automatic Tuners

These tuners measure your robot's physical characteristics. Run them in order!

### Setup for All Automatic Tuners

1. Open FTC Panels Dashboard
2. On Driver Hub, select **Tuning** OpMode
3. Navigate to **Automatic** folder
4. Ensure you have adequate space (minimum 4 tiles = 96 inches)
5. Place robot on flat, smooth surface

---

### 1. Forward Velocity Tuner

**Purpose:** Measures maximum forward velocity for accurate path following.

**Procedure:**
1. Select **Forward Velocity Tuner**
2. Ensure 48+ inches of clear space ahead
3. Run OpMode - robot will accelerate forward at full power
4. Robot will stop after reaching set distance
5. Note the **Forward Velocity** value from telemetry

**Add to PedroConstants.java:**


**Typical Values:** 40-60 inches/second

**Troubleshooting:**
- Robot doesn't reach expected speed → Check for mechanical binding
- Value seems too low → Ensure battery is fully charged
- Inconsistent results → Run multiple trials and average

---

### 2. Lateral Velocity Tuner

**Purpose:** Measures maximum strafe velocity for accurate path following.

**Procedure:**
1. Select **Lateral Velocity Tuner**
2. Ensure 48+ inches of clear space to the left
3. Run OpMode - robot will accelerate left at full power
4. Robot will stop after reaching set distance
5. Note the **Strafe Velocity** value from telemetry

**Add to PedroConstants.java:**


**Typical Values:** 35-55 inches/second (usually lower than forward)

---

### 3. Forward Zero Power Acceleration Tuner

**Purpose:** Measures deceleration when power is cut while moving forward. Critical for accurate stopping.

**Procedure:**
1. Select **Forward Zero Power Acceleration Tuner**
2. Ensure 48+ inches of clear space ahead
3. Run OpMode - robot accelerates to 30 in/s, then cuts power
4. Robot measures deceleration as it coasts to stop
5. Note the **Forward Zero Power Acceleration** value (will be negative)

**Add to PedroConstants.java:**


**Typical Values:** -30 to -60 (negative indicates deceleration)

---

### 4. Lateral Zero Power Acceleration Tuner

**Purpose:** Measures deceleration when power is cut while strafing. Critical for accurate stopping.

**Procedure:**
1. Select **Lateral Zero Power Acceleration Tuner**
2. Ensure 48+ inches of clear space to the left
3. Run OpMode - robot accelerates to 30 in/s left, then cuts power
4. Robot measures deceleration as it coasts to stop
5. Note the **Lateral Zero Power Acceleration** value (will be negative)

**Add to PedroConstants.java:**


**Typical Values:** -25 to -55 (usually higher magnitude than forward due to mecanum wheel friction)

---

## Manual PIDF Tuning

Use FTC Panels Dashboard to tune PIDFs in real-time. Tuning order: **Translational → Heading → Drive → Centripetal**

### Prerequisites

1. **Open Panels Dashboard:**
   - Connect to Driver Hub WiFi
   - Navigate to: `http://192.168.43.1:8080` (Panels URL)
   
2. **Connect Gamepad:**
   - Plug gamepad into Driver Hub
   - Press Start + A to initialize

3. **Disable Auto-Stop:**
   - Settings → Disable autonomous 30-second timer

---

### 1. Translational PIDF Tuning

**Purpose:** Ensures robot stays on path without lateral drift.

**Setup:**
1. Select **Tuning** OpMode → **Manual** folder → **Translational Tuner**
2. Run OpMode (robot will stay in place - this is correct!)

**Tuning Process:**
1. In Panels, navigate to: **Tuning → Follower → Constants**
2. Find `coefficientsTranslationalPIDF`
3. Push robot left/right and observe correction
4. Adjust values:
   - **Too many oscillations** → Decrease P
   - **Corrects too slowly** → Increase P
   - **Persistent error** → Add small I (0.001-0.01)
   - **Overshoots** → Add small D (0.001-0.01)

**Starting Values:**


**After tuning, update PedroConstants.java:**


⚠️ **Remember:** Press Enter in Panels to save changes during tuning!

---

### 2. Heading PIDF Tuning

**Purpose:** Keeps robot pointed in correct direction while following path.

**Setup:**
1. Select **Tuning** OpMode → **Manual** folder → **Heading Tuner**
2. Run OpMode (robot will stay in place)

**Tuning Process:**
1. In Panels: **Tuning → Follower → Constants** → `coefficientsHeadingPIDF`
2. Rotate robot left/right and observe correction
3. Adjust values:
   - **Slow to correct heading** → Increase P
   - **Overshoots/oscillates** → Decrease P or add D
   - **Small steady error** → Add I

**Starting Values:**


**After tuning, update PedroConstants.java:**


---

### 3. Drive PIDF Tuning

**Purpose:** Controls acceleration and braking along path, prevents overshoot.

**Setup:**
1. Select **Tuning** OpMode → **Manual** folder → **Drive Tuner**
2. **⚠️ WARNING:** Robot will immediately move forward 40 inches then back!
3. Ensure adequate space

**Before Tuning - Set Braking Strength:**

Braking strength controls deceleration at path end.

In `PedroConstants.java`:


**Tuning Braking Strength:**
- **Robot overshoots end** → Increase braking strength (try 2.0-3.0)
- **Robot stops too early** → Decrease braking strength (try 1.0-1.5)
- **Abrupt stop** → Decrease braking strength

**Tuning Drive PIDF:**
1. In Panels: **Tuning → Follower → Constants** → `coefficientsDrivePIDF`
2. Observe robot moving back and forth
3. Adjust values:
   - **Moves too slowly** → Increase P
   - **Overshoots at end** → Decrease P or increase braking strength
   - **Oscillates** → Add D

**Starting Values:**


**After tuning, update PedroConstants.java:**


**Advanced - Kalman Filter (Optional):**

- Increase first parameter → Smoother but slower response
- Increase second parameter → Faster but noisier response

---

### 4. Centripetal Force Correction

**Purpose:** Compensates for centrifugal force during curved paths.

**Setup:**
1. Select **Tuning** OpMode → **Manual** folder → **Centripetal Tuner**
2. **⚠️ WARNING:** Robot will move 20 inches forward and left in curve!
3. Ensure adequate space

**Tuning Process:**
1. In Panels: **Tuning → Follower → Constants** → `centripetalScaling`
2. Observe robot following curved path
3. Adjust value:
   - **Robot cuts inside curve** → Decrease centripetal scaling
   - **Robot drifts outside curve** → Increase centripetal scaling

**Starting Value:**


**After tuning, update PedroConstants.java:**


**Typical Values:** 0.003 - 0.008

---

## Testing & Validation

After tuning, validate with these test OpModes (found in **Tests** folder):

### Line Test
- **Purpose:** Tests all PIDFs together on straight path
- **What it does:** Drives 48 inches forward and back continuously
- **Success criteria:** Robot stays on line, no lateral drift or heading errors

### Triangle Test
- **Purpose:** Tests straight-line interpolation with heading changes
- **What it does:** Drives in triangular path with 90° and 45° turns
- **Success criteria:** Sharp corners, maintains path shape after multiple loops

### Circle Test
- **Purpose:** Tests curved path following and centripetal correction
- **What it does:** Drives in circle while always facing center
- **Success criteria:** Maintains circular shape, faces center consistently

**Running Tests:**
1. Select **Tuning** OpMode → **Tests** folder
2. Choose test (Line, Triangle, or Circle)
3. Run and observe performance
4. If issues appear, return to respective PIDF tuner

---

## Complete PedroConstants.java Example

Here's what your fully tuned constants should look like:



---

## Troubleshooting

### Robot Doesn't Move During Tuning

**Possible Causes:**
- Motor names don't match hardware config
- Motors need to be reversed
- Battery low/disconnected
- Emergency stop engaged

**Solutions:**
- Verify motor names in hardware configuration
- Test motor directions in Localization Test
- Charge battery
- Check Driver Hub status

### Robot Drifts Off Path

**Possible Causes:**
- Translational PIDF not tuned
- Heading PIDF not tuned
- Localizer encoder directions wrong

**Solutions:**
- Re-tune translational PIDF
- Re-tune heading PIDF
- Verify encoder directions in Localization Test

### Robot Overshoots at End of Path

**Possible Causes:**
- Drive PIDF too aggressive
- Braking strength too low
- Forward zero power acceleration inaccurate

**Solutions:**
- Decrease drive P gain
- Increase braking strength (1.5 → 2.5)
- Re-run Forward Zero Power Acceleration Tuner

### Robot Cuts Corners on Curves

**Possible Causes:**
- Centripetal scaling too low
- Path constraints too aggressive

**Solutions:**
- Increase centripetal scaling
- Decrease max velocity in path constraints

### Jerky/Oscillating Movement

**Possible Causes:**
- PID gains too high
- Derivative gain needed

**Solutions:**
- Decrease P gain
- Add small D gain (0.01-0.02)
- Increase time constant T in drive PIDF

---

## Quick Reference - Tuning Workflow



---

## Additional Resources

- **Pedro Pathing Documentation:** https://pedropathing.com
- **FTC Panels Dashboard:** https://panels.ftc
- **NextFTC Documentation:** https://docs.nextftc.com

**Congratulations!** Your robot is now fully tuned for accurate autonomous path following with Pedro Pathing!

