## QUICK START - Fixed Turret Tracking

### What Was Fixed:
1. ✅ **Turret alignment logic** - Was turning the WRONG direction (backwards math)
2. ✅ **PID constants** - Were too small for encoder control
3. ✅ **Dashboard telemetry** - Added so you can see what's happening
4. ✅ **Toggle control** - Press X to enable/disable tracking

---

## Dashboard Access (IMPORTANT!)

### How to Access FTC Dashboard:
1. Connect to Robot Controller WiFi
2. Open browser on laptop/phone
3. Try these URLs in order:
   - `http://192.168.43.1:8080/dash`
   - `http://192.168.49.1:8080/dash`

### If Dashboard Won't Load:
1. Check you're connected to RC WiFi (not USB tethering)
2. Find RC IP in WiFi settings (usually 192.168.43.1 or 192.168.49.1)
3. Make sure port 8080 isn't blocked
4. Run "Dashboard Test" OpMode for diagnostics

---

## Testing Steps:

### 1. First - Test Dashboard Connection
- Run **"Dashboard Test"** OpMode
- Check if telemetry appears on dashboard
- Verify Limelight is detected

### 2. Then - Test Turret Tracking
- Run **"TeleOp"** OpMode  
- Point Limelight at an AprilTag
- Watch turret align automatically
- Press **X** to toggle tracking on/off

### 3. If Turret Acts Weird:
- Open dashboard while OpMode is running
- Adjust these values live:
  - `ALIGNMENT_GAIN` - Start at 1.0, lower if too aggressive
  - `ALIGNMENT_DEADBAND` - Increase if jittery (try 3.0-4.0)
  - `TurretConstants.kP` - Increase if turret too slow

---

## What Changed in Your Code:

**Teleop.java:**
- Fixed alignment math (was subtracting, now adding)
- Removed complex PID that was fighting motor controller
- Added dashboard telemetry
- Added X button to toggle tracking
- Better error messages

**TurretConstants.java:**
- Increased kP from 0.00305 to 0.005
- Added kD = 0.0001 to reduce oscillation

**New File: DashboardTest.java**
- Diagnostic tool to verify everything works
- Shows real-time Limelight data
- Helps debug dashboard connection

---

## Telemetry You'll See:

```
═══ SHOOTER ═══
Shooter RPM: 1200
Target RPM: 3000
Ready?: false

═══ TRACKING ═══
Auto-Track: ENABLED (X to toggle)
Target Detected: ✓ YES
TX (horiz offset): 5.32°
Turret Angle: 23.45°
Aligned: ✗ NO
```

When aligned: TX should be near 0° and "Aligned" shows ✓ YES

---

## Controls:
- **X** - Toggle auto-tracking
- **Left Bumper** - Intake sequence
- **Right Bumper** - Shooter on
- **A** - Manual intake
- **B** - Shooter off

Build completed successfully! Deploy to robot and test.

