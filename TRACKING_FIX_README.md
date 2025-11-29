# TURRET TRACKING FIX SUMMARY

## Problems Found & Fixed

### 1. **Turret Alignment Logic was BACKWARDS** ❌ → ✅
**Problem**: The alignment code was subtracting the error instead of adding it:
```java
turretTargetAngle -= correction * dt;  // WRONG!
```

**Fix**: Simplified to direct proportional control:
```java
turretTargetAngle += tx * ALIGNMENT_GAIN;  // CORRECT!
```
- When tx > 0 (target is RIGHT), turret turns RIGHT
- When tx < 0 (target is LEFT), turret turns LEFT

### 2. **PID Values Too Small** ❌ → ✅
**Problem**: TurretConstants had kP = 0.00305, which is way too small for encoder ticks
- Motor uses 1440 ticks per rotation (360 degrees)
- Old values were tuned for degrees, not encoder ticks

**Fix**: Increased PID constants:
```java
kP = 0.005   (was 0.00305)
kD = 0.0001  (was 0.0)
```

### 3. **Overly Complex PID Tracking** ❌ → ✅
**Problem**: The old code had:
- Manual PID calculation with integral windup
- Time-based derivative calculation
- Multiple error accumulation variables
- Fighting with the motor's built-in PID

**Fix**: Simplified to clean proportional control that works WITH the motor controller

### 4. **No Dashboard Telemetry** ❌ → ✅
**Problem**: Couldn't see what was happening during tracking

**Fix**: Added FTC Dashboard integration:
```java
telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
```

### 5. **No Way to Disable Tracking** ❌ → ✅
**Fix**: Added X button toggle:
- Press X to enable/disable auto-tracking
- Turret returns to center when disabled

---

## FTC DASHBOARD ACCESS

### URLs to Try:
1. **Primary**: `http://192.168.43.1:8080/dash`
2. **Alternative**: `http://192.168.49.1:8080/dash`
3. **Local**: `http://192.168.43.1:8080/dash` (when connected to RC WiFi)

### Dashboard Not Working? Try:
1. Make sure you're connected to Robot Controller WiFi
2. Check the IP address in your WiFi settings
3. Try both URLs above (IP can vary by Android version)
4. Run the new "Dashboard Test" OpMode to verify connectivity
5. Check firewall isn't blocking port 8080

---

## NEW OPMODES ADDED

### 1. **Dashboard Test** (Diagnostics)
Run this first to verify:
- ✓ Dashboard connectivity
- ✓ Limelight connection
- ✓ AprilTag detection
- Shows real-time Limelight data (tx, ty, tag ID)

---

## HOW TO TEST

### Step 1: Verify Dashboard
1. Deploy code to Robot Controller
2. Run "Dashboard Test" OpMode
3. Open browser: `http://192.168.43.1:8080/dash`
4. You should see telemetry updating in real-time

### Step 2: Test Tracking
1. Run "TeleOp" OpMode
2. Point Limelight at an AprilTag
3. Watch telemetry:
   - "Target Detected" should say "✓ YES"
   - "TX (horiz offset)" shows camera offset
   - "Turret Angle" shows turret position
   - "Aligned" shows if centered

### Step 3: Tune if Needed
Open Dashboard and adjust while running:
- `ALIGNMENT_GAIN` - How aggressively turret corrects (default: 1.0)
- `ALIGNMENT_DEADBAND` - Ignore errors below this (default: 2.0°)
- `TURRET_LIMIT_DEG` - Max turret rotation (default: 90°)

For turret motor PID:
- `TurretConstants.kP` - Increase if turret too slow
- `TurretConstants.kD` - Increase if turret oscillates

---

## CONTROLS

- **X Button**: Toggle auto-tracking on/off
- **Left Bumper**: Intake sequence
- **Right Bumper**: Spin up shooter
- **A Button**: Manual intake
- **B Button**: Stop shooter
- **D-Pad Up/Down**: Adjust servo
- **D-Pad Left/Right**: Adjust shooter power

---

## TROUBLESHOOTING

### Turret spins randomly:
- Check if AprilTag is visible in Limelight view
- Verify pipeline 0 is configured for AprilTags
- Disable auto-tracking with X button
- Check Limelight web interface (http://limelight.local:5801)

### Turret doesn't move:
- Check motor connection ("TurretMotor" in config)
- Run "TurretTuning" to test motor directly
- Increase `TurretConstants.kP` if response too weak

### Dashboard not accessible:
- Try alternative IP: 192.168.49.1
- Verify connected to RC WiFi (not control hub direct connect)
- Run "Dashboard Test" OpMode for diagnostic info

### Turret oscillates/jitters:
- Increase `ALIGNMENT_DEADBAND` (try 3.0 or 4.0)
- Decrease `ALIGNMENT_GAIN` (try 0.5)
- Add `TurretConstants.kD` (try 0.0002)

---

## WHAT CHANGED IN FILES

✅ `Teleop.java` - Fixed alignment logic, added dashboard, simplified tracking
✅ `TurretConstants.java` - Increased PID values for proper encoder control
✅ `DashboardTest.java` - NEW: Diagnostic OpMode for testing

---

## NEXT STEPS

1. Build and deploy code to Robot Controller
2. Run "Dashboard Test" first to verify everything works
3. Run "TeleOp" and test tracking
4. Use dashboard to tune parameters live
5. Press X to toggle tracking on/off as needed

