# Shooter PID Velocity Control - Setup & Tuning Guide

## Overview
The Shooter subsystem uses PID velocity control with feedforward to maintain consistent RPM even as the battery drains during matches. This ensures reliable shooting performance throughout the competition.

## Why PID Velocity Control?

**Problem:** As the battery voltage drops during a match (from ~13.5V to ~11V), motor speeds decrease, causing inconsistent shot distances and accuracy.

**Solution:** PID velocity control continuously monitors the shooter wheel RPM and automatically adjusts motor power to compensate for voltage drops.

---

## Initial Setup

### 1. Prerequisites
- FTC Dashboard installed and connected to your Driver Hub
- Shooter motors configured in hardware map as "ShooterRight" and "ShooterLeft"
- Robot battery fully charged for initial tuning

### 2. Understanding the Constants

The shooter uses these tunable parameters in `ShooterConstants.java`:



---

## Tuning Process

### Step 1: Connect FTC Dashboard

1. Connect your laptop to the same WiFi network as your Driver Hub
2. Open a web browser and navigate to: `http://192.168.43.1:8080/dash`
3. You should see the FTC Dashboard interface

### Step 2: Set Target RPM

1. In FTC Dashboard, navigate to **Configuration** → **ShooterConstants**
2. Set `targetRPM` to your desired shooting speed (typical range: 2500-4000 RPM)
   - Lower RPM = shorter shot distance, more consistent
   - Higher RPM = longer shot distance, less consistent

**Finding Your Ideal RPM:**
- Start with 3000 RPM
- Test shoot into your target
- Adjust up/down by 200 RPM increments until shots consistently reach target
- Note: You can fine-tune this later during practice

### Step 3: Tune Feedforward (Kf)

Feedforward provides the baseline power needed to spin the shooter wheels.

**Procedure:**
1. In FTC Dashboard, set all PID gains to zero:
   
2. Set `velocityKf = 0.00008`
3. Run your shooter using `Shooter.INSTANCE.runAtTargetRPM().schedule()`
4. Observe the actual RPM in telemetry:
   - **If RPM is too low:** Increase Kf by 0.00002
   - **If RPM is too high:** Decrease Kf by 0.00002
5. Repeat until actual RPM is within ~200 RPM of target
6. **Goal:** Kf alone should get you to ~90-95% of target RPM

**Typical Kf values:** 0.00010 - 0.00015

### Step 4: Tune Proportional (Kp)

Proportional gain makes the system respond to RPM errors.

**Procedure:**
1. Keep Kf at the value you found in Step 3
2. Set `velocityKp = 0.00005`
3. Run the shooter and observe:
   - **Slow to reach target:** Increase Kp by 0.00002
   - **Oscillates/bounces around target:** Decrease Kp by 0.00001
4. **Goal:** Quick response to target with minimal overshoot

**Typical Kp values:** 0.00008 - 0.00015

### Step 5: Add Integral (Ki) - Optional

Integral gain eliminates persistent errors (useful for very consistent shooting).

**Procedure:**
1. Keep Kf and Kp at tuned values
2. Set `velocityKi = 0.000005`
3. Run the shooter for 10+ seconds:
   - **Steady-state error remains:** Increase Ki by 0.000002
   - **System becomes unstable:** Decrease Ki by 0.000002
4. **Goal:** Zero steady-state error without oscillations

**Warning:** Too much Ki can cause wind-up and instability. Start small!

**Typical Ki values:** 0.000005 - 0.00002

### Step 6: Add Derivative (Kd) - Optional

Derivative gain reduces overshoot and oscillations.

**Procedure:**
1. If you see oscillations even with tuned Kp:
2. Set `velocityKd = 0.00001`
3. Gradually increase if oscillations persist
4. **Goal:** Smooth approach to target without overshoot

**Typical Kd values:** 0.0 - 0.00003 (often not needed)

---

## Testing Your Tuning

### Battery Drain Test

This verifies that PID maintains consistent RPM as battery drains:

1. **Full Battery Test:**
   - Charge battery to full (13.2V+)
   - Run shooter at target RPM for 30 seconds
   - Record average RPM from telemetry

2. **Partial Battery Test:**
   - Drain battery to ~12V (run some practice matches)
   - Run shooter at same target RPM
   - Record average RPM from telemetry

3. **Low Battery Test:**
   - Drain battery to ~11V
   - Run shooter at same target RPM
   - Record average RPM from telemetry

**Success Criteria:** All three tests should show RPM within ±50 of target RPM

### Shot Consistency Test

1. Take 10 shots with full battery
2. Take 10 shots with low battery
3. Measure shot distance variance

**Success Criteria:** Low battery shots should land within same area as full battery shots

---

## Using the Shooter in Code

### Basic Usage (Recommended)



### Advanced Usage



---

## Telemetry Monitoring

Add this to your OpMode to monitor shooter performance:



---

## Troubleshooting

### Shooter Never Reaches Target RPM

**Possible Causes:**
- Kf too low → Increase feedforward
- Mechanical friction → Check for binding
- Battery voltage too low → Charge battery
- Target RPM too high → Lower target or check motor specs

### Shooter Oscillates/Vibrates

**Possible Causes:**
- Kp too high → Decrease proportional gain
- Kd needed → Add small derivative gain
- Ki too high → Decrease integral gain

### Inconsistent Shots Despite PID

**Possible Causes:**
- RPM tolerance too wide → Decrease `rpmTolerance` to 25-30
- Shooting before at speed → Always check `isAtTargetRPM()` first
- Hood servo position changing → Ensure servo position is consistent

### Shooter Overshoots Then Settles

**Possible Causes:**
- Need derivative gain → Add Kd starting at 0.00001
- Kp too aggressive → Decrease slightly

---

## FTC Dashboard Quick Reference

### Accessing Variables
1. Connect to Dashboard: `http://192.168.43.1:8080/dash`
2. Click **Configuration** tab
3. Navigate to **ShooterConstants**
4. Edit values in real-time

### Saving Tuned Values
⚠️ **IMPORTANT:** Dashboard changes are **NOT** saved to code automatically!

After tuning:
1. Note down your final values from Dashboard
2. Open `ShooterConstants.java`
3. Update the static values manually
4. Rebuild and deploy your code

---

## Recommended Starting Values



---

## Advanced Tips

### Multiple Shooting Positions

Create preset RPMs for different shot distances:



Then use in autonomous:



### Battery Voltage Compensation

Monitor voltage to predict performance:



---

## Summary Checklist

- [ ] FTC Dashboard installed and accessible
- [ ] Feedforward (Kf) tuned to reach ~90% of target
- [ ] Proportional (Kp) tuned for quick response
- [ ] Integral (Ki) added if needed for precision
- [ ] Derivative (Kd) added if needed for stability
- [ ] Battery drain test passed (consistent RPM at all voltages)
- [ ] Shot consistency test passed
- [ ] Values saved to ShooterConstants.java
- [ ] Code rebuilt and deployed

**Congratulations!** Your shooter now has battery-compensated velocity control for consistent performance throughout matches!

