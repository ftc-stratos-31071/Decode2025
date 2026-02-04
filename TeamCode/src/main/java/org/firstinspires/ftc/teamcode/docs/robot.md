# Robot Documentation - FTC Decode 2025

**Last Updated:** February 3, 2026  
**Framework:** NextFTC Command-Based + Pedro Pathing  
**Game:** FTC 2024-2025 Season

---

## Table of Contents
1. [Robot Overview](#robot-overview)
2. [Subsystems](#subsystems)
3. [Hardware Configuration](#hardware-configuration)
4. [Commands](#commands)
5. [Autonomous Modes](#autonomous-modes)
6. [TeleOp Modes](#teleop-modes)
7. [Control Bindings](#control-bindings)
8. [Constants & Tuning](#constants--tuning)

---

## Robot Overview

This robot is designed for the FTC 2024-2025 season and features a **shooting and intake system** for game elements (balls). The robot uses a **mecanum drivetrain** for omnidirectional movement and incorporates vision-based targeting using a **Limelight 3A camera** for AprilTag detection.

### Key Features:
- **Mecanum Drive**: 4-wheel omnidirectional movement for precise field navigation
- **Shooter System**: Dual-motor flywheel shooter with PIDF velocity control
- **Adjustable Hood**: Servo-controlled hood for trajectory adjustment
- **Turret**: Dual-servo turret system for aiming (±180° rotation)
- **Intake System**: Motor-driven intake with transfer mechanism
- **Vision Tracking**: Limelight 3A for automatic turret targeting
- **Odometry**: GoBILDA Pinpoint localizer for autonomous path following
- **Ball Counting**: Laser rangefinder sensor for detecting intake balls

---

## Subsystems

### 1. Shooter Subsystem
**File:** `subsystems/Shooter.java`

The shooter is a **dual-motor flywheel system** with closed-loop velocity control (PIDF).

#### Hardware:
- **Right Motor**: `ShooterRight` (reversed)
- **Left Motor**: `ShooterLeft`
- **Hood Servo**: `HoodServo` (adjusts launch angle)

#### Key Features:
- **PIDF Velocity Control**: Maintains precise flywheel RPM for consistent shooting
  - Feed-forward (kF): Scales with target RPM
  - Proportional (kP): Corrects velocity error
  - Integral (kI): Eliminates steady-state error
  - Derivative (kD): Dampens oscillations
- **RPM Targets**:
  - Close shots: 3700 RPM
  - Far shots: 5000 RPM
- **Hood Positions**:
  - Close: 0.4 (higher angle)
  - Far: 0.1 (flatter trajectory)

#### Methods:
```java
setTargetRPM(double rpm)        // Set desired flywheel speed
stop()                          // Stop motors and disable PID
atSpeed(double toleranceRPM)    // Check if within tolerance
getRPM()                        // Get current average RPM
setHood(double position)        // Command to adjust hood angle
```

#### How It Works:
1. **Velocity Control Loop** (runs in `periodic()`):
   - Reads encoder velocities from both motors
   - Converts ticks/sec → RPM
   - Calculates PID output based on error
   - Applies power to motors (clamped 0.0-1.0)
   - Sends telemetry to FTC Dashboard

2. **Shooting Sequence**:
   - Spin up flywheel to target RPM
   - Wait until `atSpeed()` returns true (within tolerance)
   - Feed balls through intake/transfer system
   - Maintain constant RPM during firing

---

### 2. Intake Subsystem
**File:** `subsystems/Intake.java`

Handles ball collection and feeding to the shooter.

#### Hardware:
- **Intake Motor**: `IntakeMotor` (reversed, brake mode)
- **Transfer Motor**: `TransferMotor` (brake mode)
- **Door Servo**: `DoorServo` (opens/closes intake door)

#### Servo Positions:
- **Closed (collecting)**: 0.0 - Keeps balls in during intake
- **Open (shooting)**: 0.7 - Allows balls to feed to shooter

#### Key Commands:
```java
moveIntake(double power)        // Set intake roller power
moveTransfer(double power)      // Set transfer roller power
moveServoPos()                  // Close door (0.0)
defaultPos()                    // Open door (0.7)
transfer(double timeSec)        // Run transfer for duration
```

#### Intake Sequence (IntakeSeqCmd):
```
1. Start intake motor (spin in)
2. Start transfer motor (move balls up)
3. Continue until button released
```

---

### 3. Turret Subsystem
**File:** `subsystems/Turret.java`

A **dual-servo pan mechanism** for aiming the shooter at targets.

#### Hardware:
- **Left Servo**: `TurretLeft`
- **Right Servo**: `TurretRight`

#### Specifications:
- **Range**: 355° servo rotation
- **Center Offset**: 80° (calibration offset)
- **Default Position**: 0° (forward)
- **Blue Far Position**: 310° (for far-side autonomous)

#### How It Works:
```java
setTurretAngleDeg(double turretDeg)
```
1. Adds 80° center offset to input angle
2. Converts angle to servo position (0.0-1.0)
3. Clamps to valid range
4. Commands both servos to same position (synchronized)

**Example**: `setTurretAngleDeg(310)` → servo position = (310+80)/355 = 1.099 → clamped to 1.0

---

### 4. TeleopMecanumDrive Subsystem
**File:** `subsystems/TeleopMecanumDrive.java`

Direct motor control for mecanum drive (used in some teleop modes).

#### Hardware:
- **Front Left**: `frontLeftMotor` (forward)
- **Front Right**: `frontRightMotor` (reversed)
- **Back Left**: `backLeftMotor` (forward)
- **Back Right**: `backRightMotor` (reversed)
- **IMU**: Control Hub IMU (for heading)

#### Methods:
```java
setPowerMecanum(double forward, double strafe, double rotate)
setPowerRotation(double power)  // Pure rotation
stop()                          // Zero all motors
getHeading()                    // IMU yaw angle
resetHeading()                  // Zero IMU
```

#### Mecanum Drive Math:
```
frontLeft  = forward + strafe + rotate
frontRight = forward - strafe - rotate
backLeft   = forward - strafe + rotate
backRight  = forward + strafe - rotate
```
Powers are normalized if any exceed 1.0 to maintain drive direction.

---

### 5. LaserRangefinder
**File:** `subsystems/LaserRangefinder.java`

A custom driver for the **Rev Color Sensor V3** configured as a laser distance sensor.

#### Purpose:
- **Ball Detection**: Detects when balls enter the intake (< 40mm)
- **Ball Counting**: Tracks number of collected balls (up to 3)
- **Driver Feedback**: Rumbles gamepad when 3 balls collected

#### Configuration:
```java
rangefinder.setDistanceMode(DistanceMode.SHORT)  // Max 1.3m range
rangefinder.setTiming(20, 0)                      // 20ms budget, immediate
```

#### Usage in Teleop:
```java
double distance = rangefinder.getDistance(DistanceUnit.MM);
if (distance < 40mm && !ballPresent) {
    ballCount++;  // New ball detected
}
```

---

## Hardware Configuration

### Motor Configuration:
| Device | Hardware Name | Direction | Mode | Purpose |
|--------|---------------|-----------|------|---------|
| Front Left | `frontLeftMotor` | FORWARD | BRAKE | Drive |
| Front Right | `frontRightMotor` | REVERSE | BRAKE | Drive |
| Back Left | `backLeftMotor` | FORWARD | BRAKE | Drive |
| Back Right | `backRightMotor` | REVERSE | BRAKE | Drive |
| Intake | `IntakeMotor` | REVERSE | BRAKE | Collect balls |
| Transfer | `TransferMotor` | FORWARD | BRAKE | Feed shooter |
| Shooter Right | `ShooterRight` | REVERSE | BRAKE | Flywheel |
| Shooter Left | `ShooterLeft` | FORWARD | BRAKE | Flywheel |

### Servo Configuration:
| Device | Hardware Name | Range | Default | Purpose |
|--------|---------------|-------|---------|---------|
| Door Servo | `DoorServo` | 0.0-0.7 | 0.7 | Intake door |
| Hood Servo | `HoodServo` | 0.1-1.0 | 1.0 | Shooter angle |
| Turret Left | `TurretLeft` | 0.0-1.0 | 0.225 | Turret aim |
| Turret Right | `TurretRight` | 0.0-1.0 | 0.225 | Turret aim |

### Sensors:
| Device | Hardware Name | Type | Purpose |
|--------|---------------|------|---------|
| Range Sensor | `range` | Rev Color Sensor V3 | Ball detection |
| Limelight | `limelight` | Limelight 3A | AprilTag vision |
| Pinpoint | `pinpoint` | GoBILDA Pinpoint | Odometry |
| IMU | Built-in | Control Hub IMU | Heading |

---

## Commands

### Shooting Commands

#### ShootBallSteadyCmd
**File:** `commands/ShootBallSteadyCmd.java`

**Purpose**: Feeds balls to shooter ONLY when flywheel is at speed.

**How It Works**:
```java
if (Shooter.atSpeed(toleranceRPM)) {
    Intake.setIntakePower(feedPower);
    Intake.setTransferPower(feedPower);
} else {
    // Wait for shooter to spin up
    Intake.setIntakePower(0);
    Intake.setTransferPower(0);
}
```

**Usage**: Prevents jamming by ensuring shooter is ready before feeding.

#### RapidFireCmd
**File:** `commands/RapidFireCmd.java`

**Purpose**: Quick shooting sequence for teleop.

**Sequence**:
1. Wait 0.25 seconds (settle time)
2. ShootBallSteadyCmd with 100 RPM tolerance
3. Runs until button released

#### RapidFireTimeoutCmd
**File:** `commands/RapidFireTimeoutCmd.java`

**Purpose**: Autonomous shooting with timeout protection.

**Features**:
- Wraps RapidFireCmd with maximum time limit
- Prevents hanging if shooter fails to reach speed
- Finishes when either:
  - All balls shot (normal completion)
  - Timeout expires (safety)

**Default Timeout**: 1000ms (1 second)

---

### Intake Commands

#### IntakeSeqCmd
**File:** `commands/IntakeSeqCmd.java`

**Purpose**: Start intake and transfer motors simultaneously.

**Sequence**:
```
ParallelGroup:
  - moveIntake(intakePower)
  - moveTransfer(intakePower)
```

**Usage**: Runs continuously while left bumper held.

---

### Utility Commands

#### WaitCmd
**File:** `commands/WaitCmd.java`

**Purpose**: Time-based delay for command sequences.

**Example**: `WaitCmd.create(0.5)` → 500ms pause

#### AutoDriveTimeoutCmd
**File:** `commands/AutoDriveTimeoutCmd.java`

**Purpose**: Wraps path following with timeout protection.

**Usage**:
```java
AutoDriveTimeoutCmd.create(new FollowPath(path), 2.0)  // 2 second max
```

---

## Autonomous Modes

### AutoBlue & AutoRed
**Files:** `opmodes/autos/AutoBlue.java`, `AutoRed.java`

**Simple Movement Autos**:
- AutoBlue: Strafe LEFT 0.5 seconds at 50% power
- AutoRed: Strafe RIGHT 0.5 seconds at 50% power
- Used for basic positioning/testing

---

### CloseBlueAuto
**File:** `opmodes/autos/CloseBlueAuto.java`

**Starting Position**: Near alliance scoring zone (19.5", 123.5")

**Strategy**: Score preloaded balls, collect ground balls, score again.

**Path Sequence**:
```
Path 1: Reverse to scoring position (60.5", 82.5")
  → Shoot preloaded balls

Path 2: Curve to collection zone (40.0", 61.5")
  → Orient to 180°

Path 3: Drive to far collection point (10.0", 61.5")
  → Run intake to collect balls

Path 4: Return to scoring position (60.5", 82.5")
  → Turn to 210° for shooting angle

Path 5: Curve back to near collection (15.5", 67.0")
  → Run intake again

Path 6: Final adjustment (12.0", 59.5")
  → Orient to 150°
```

**Key Features**:
- Uses **Bézier curves** for smooth paths
- Heading interpolation for continuous orientation control
- Parallel commands (drive while intaking)
- Sequential shooting → driving → intaking cycles

---

### FarBlueAuto
**File:** `opmodes/autos/FarBlueAuto.java`

**Starting Position**: Far from alliance wall (56.0", 8.0")

**Configuration**:
- Hood: **Far position (0.1)** - flatter angle
- Target RPM: **5000** - higher speed for distance
- Turret: **310°** - angled toward goal

**Strategy**: Shoot preloaded from far position.

**Current Implementation**:
```
1. Spin up shooter to 5000 RPM
2. Open intake door
3. RapidFireTimeoutCmd (3 second max)
4. (Commented out: cycling paths for more balls)
```

**Note**: Additional cycles commented out for simpler autonomous.

---

## TeleOp Modes

### Main Teleop
**File:** `opmodes/teleops/Teleop.java`

The primary driver-controlled mode with **automatic turret tracking**.

#### Driver Controls (Gamepad 1):

**Driving**:
- Left Stick Y: Forward/Backward
- Left Stick X: Strafe Left/Right
- Right Stick X: Rotate
- Left Stick Button: Toggle **Slow Mode** (25% speed)

**Intake**:
- Left Bumper (hold): Run intake sequence
- B Button (hold): Reverse intake/transfer (eject balls)
- A Button (hold): Rapid fire shooting

**Shooter**:
- Right Bumper: Toggle shooter on/off
- D-Pad Up: Decrease hood position (-0.1)
- D-Pad Down: Increase hood position (+0.1)
- D-Pad Right: Increase target RPM (+100)
- D-Pad Left: Decrease target RPM (-100)

#### Operator Controls (Gamepad 2):

**Manual Turret**:
- D-Pad Right: Rotate turret +10°
- D-Pad Left: Rotate turret -10°

---

#### Automatic Turret Tracking

**How It Works**:
1. **Limelight 3A** detects AprilTags on field
2. System finds **closest tag** (largest pixel area)
3. Extracts horizontal offset (**TX** in degrees)
4. **Exponential smoothing** prevents jitter:
   ```java
   smoothedTx = 0.5 * smoothedTx + 0.5 * rawTx
   ```
5. **Deadband** prevents micro-adjustments (±3°)
6. **Proportional adjustment** to turret angle:
   ```java
   adjustment = smoothedTx * 0.08  // Tracking gain
   motorTargetX += adjustment
   ```
7. Turret continuously tracks target while driver moves

**Safety Features**:
- **Timeout**: Returns to center if no target for 0.5 seconds
- **Smoothing**: Prevents oscillation and jitter
- **Deadband**: Stabilizes when near alignment

**Configuration** (tunable via FTC Dashboard):
```java
TRACKING_GAIN = 0.08        // Adjustment speed
SMOOTHING = 0.5             // Filter strength (0-1)
DEADBAND = 3.0              // Ignore offsets < 3°
NO_TARGET_TIMEOUT_SEC = 0.5 // Return to center delay
AUTO_TRACK_ENABLED = true   // Enable/disable tracking
```

---

#### Ball Counting System

**Purpose**: Track collected balls and alert driver when full.

**Detection**:
```java
if (distance < 40mm && !ballPresent && timeSinceLastBall > 300ms) {
    ballCount++;
}
```

**Feedback**:
- When 3 balls collected: **Gamepad rumbles** (500ms)
- Count resets when intake stopped

**Debounce**: 300ms between detections prevents double-counting.

---

### TeleOpTurretTracking
**File:** `opmodes/teleops/TeleOpTurretTracking.java`

**Advanced field-centric turret tracking** using Pinpoint odometry.

**How It Differs from Main Teleop**:
- Uses **global field coordinates** instead of robot-relative
- Turret maintains aim on target **even as robot rotates**
- Requires Pinpoint odometry for robot heading

**Algorithm**:
1. Read robot heading from Pinpoint
2. When AprilTag first detected:
   - Record initial heading offset
   - Calculate target's **field-relative angle**
3. As robot rotates:
   - Adjust turret to compensate
   - Turret angle = fieldTarget - robotHeading + txCorrection
4. Turret stays locked on target regardless of robot orientation

**Use Case**: More advanced teleop for experienced drivers who want field-centric aiming.

---

## Control Bindings

### TeleOp Button Summary

| Control | Action | Subsystem | Command |
|---------|--------|-----------|---------|
| **DRIVER (GP1)** |
| Left Stick | Drive (strafe/forward) | Drivetrain | MecanumDriverControlled |
| Right Stick X | Rotate | Drivetrain | MecanumDriverControlled |
| Left Stick Btn | Toggle slow mode (0.25x) | - | Toggle variable |
| Left Bumper | Intake balls | Intake | IntakeSeqCmd |
| B Button | Reverse intake | Intake | moveIntake(-power) |
| A Button | Rapid fire | Intake + Shooter | RapidFireCmd |
| Right Bumper | Shooter on/off | Shooter | runRPM() / stop() |
| D-Pad Up | Hood down (-0.1) | Shooter | setHood() |
| D-Pad Down | Hood up (+0.1) | Shooter | setHood() |
| D-Pad Right | RPM +100 | Shooter | setTargetRPM() |
| D-Pad Left | RPM -100 | Shooter | setTargetRPM() |
| **OPERATOR (GP2)** |
| D-Pad Right | Turret +10° | Turret | goToAngle() |
| D-Pad Left | Turret -10° | Turret | goToAngle() |

---

## Constants & Tuning

### ShooterConstants.java

**PIDF Coefficients**:
```java
kF = 0.000172   // Feed-forward (power per RPM)
kP = 0.0019     // Proportional gain
kI = 0.0        // Integral gain (not used)
kD = 0.0        // Derivative gain (not used)
```

**Power Limits**:
```java
MIN_POWER = 0.0
MAX_POWER = 1.0
```

**RPM Targets**:
```java
closeTargetRPM = 3700.0    // Close shots
farTargetRPM = 5000.0      // Far shots
tolRpm = 50.0              // Tight tolerance
tolRpm2 = 100.0            // Loose tolerance
```

**Hood Positions**:
```java
servoPos = 0.4         // Close position (higher angle)
defaultPos = 1.0       // Resting position
farHoodPos = 0.1       // Far position (flatter)
```

**Hardware**:
```java
TICKS_PER_REV = 28.0   // Encoder resolution
```

---

### IntakeConstants.java

**Motor Powers**:
```java
intakePower = 1.0          // Full speed intake
intakePowerSlow = 0.75     // Slow speed
shootPower = 1.0           // Transfer feed speed
zeroPower = 0.0            // Stop
```

**Servo Positions**:
```java
defaultPos = 0.7   // Open (shooting)
servoPos = 0.0     // Closed (intaking)
```

**Timing**:
```java
reverseTime = 0.15        // Quick reverse
shootTimeFirst = 0.4      // First ball delay
shootTime = 0.15          // Between balls
shootTimeCont = 1.2       // Continuous shooting
shootTimeEnd = 1.0        // Final shot delay
```

---

### TurretConstants.java

```java
DEFAULT_TURRET_DEG = 0.0    // Forward position
BLUE_FAR_ANGLE = 310.0      // Far blue auto position
```

---

### PedroConstants.java

**Localization** (Pinpoint):
```java
forwardPodY = 93.5 mm      // Forward encoder Y offset
strafePodX = -118.7 mm     // Strafe encoder X offset
```

**Drive Characteristics**:
```java
xVelocity = 88.17 in/s     // Forward max speed
yVelocity = 73.63 in/s     // Strafe max speed
mass = 7.5 kg              // Robot mass
```

**Control Coefficients**:
- **Translational PID**: P=0.1, I=0, D=0.01, F=0.04
- **Secondary Translation**: P=0.5, I=0, D=0.03, F=0.03
- **Heading PID**: P=1.0, I=0, D=0.02, F=0.03
- **Drive PIDF**: P=0.007, I=0, D=0.001, F=0.6

---

### ShooterBallistics.java

**Distance-based RPM Model**:
```java
RPM(distance) = RPM_A + RPM_B*d + RPM_C*d²

RPM_A = 2600   // Base RPM at 0 meters
RPM_B = 700    // RPM increase per meter
RPM_C = 0      // Quadratic term (unused)
```

**Distance-based Hood Model**:
```java
Hood(distance) = HOOD_A + HOOD_B*d

HOOD_A = 0.60   // Hood at 0 meters (steep)
HOOD_B = -0.08  // Flattens per meter
```

**Limits**:
```java
MIN_RPM = 2000, MAX_RPM = 6000
MIN_HOOD = 0.20, MAX_HOOD = 0.60
MIN_DIST = 0.5m, MAX_DIST = 4.0m
```

---

### LimelightConstants.java

**Physical Measurements**:
```java
CAM_HEIGHT_M = 0.30         // Camera height above floor
TAG_HEIGHT_M = 1.20         // AprilTag center height
CAM_PITCH_DEG = 20.0        // Camera upward angle
```

**Distance Calculation**:
```java
distance = (TAG_HEIGHT - CAM_HEIGHT) / tan(CAM_PITCH + ty)
```

---

## Architecture Overview

### Framework: NextFTC Command-Based

**Subsystems**: Hardware abstraction (motors, servos, sensors)
- Shooter, Intake, Turret, Drivetrain

**Commands**: Atomic robot actions
- Shooting, Intaking, Aiming, Driving

**Scheduler**: Executes commands, prevents conflicts
- Runs in `periodic()` every loop
- Ensures one command per subsystem
- Handles interrupts and timeouts

**Components**: Lifecycle hooks
- SubsystemComponent: Initializes subsystems
- PedroComponent: Updates path follower
- BulkReadComponent: Optimizes sensor reads
- BindingsComponent: Handles gamepad bindings

---

### Autonomous: Pedro Pathing

**Path Following**:
1. Define paths using Bézier curves/lines
2. Follower uses odometry to track position
3. PIDF controllers compute motor powers
4. Corrects for pushes/drift in real-time

**FollowPath Command**:
- Schedules path on Follower
- Blocks until path complete
- Can be interrupted or timeout-protected

**Localization**:
- GoBILDA Pinpoint 2-wheel odometry
- Updates 100+ Hz for smooth tracking
- Integrated IMU for heading correction

---

## System Workflow

### Initialization (INIT Phase):
```
1. Hardware devices initialized via hardwareMap
2. Subsystems initialize() called:
   - Reset encoders
   - Zero servos
   - Calibrate IMU/sensors
3. Follower created (autonomous only)
4. Paths built (autonomous only)
5. Shooter/turret moved to default positions
```

### START Phase:
```
TeleOp:
  1. Schedule MecanumDriverControlled (continuous)
  2. Set up button bindings
  3. Initialize Limelight streaming
  4. Reset ball counter

Autonomous:
  1. Schedule pre-built command sequence
  2. Follower begins first path
  3. Shooter spins up to target RPM
```

### UPDATE Loop (every ~10ms):
```
1. BulkReadComponent reads all sensors
2. PedroComponent updates Follower (if used)
3. Subsystem periodic() methods run:
   - Shooter: Update PIDF control
   - Intake: Monitor sensors
   - Turret: Apply tracking adjustments
4. CommandScheduler runs active commands:
   - Check isDone conditions
   - Call update() on each command
   - Handle interrupts/conflicts
5. Telemetry sent to Driver Station
```

### STOP Phase:
```
1. CommandScheduler cancels all commands
2. Commands call stop(interrupted=true)
3. Motors set to zero power
4. Subsystems cleanup (if needed)
5. Limelight stopped
```

---

## Tuning Guide

### Shooter PIDF Tuning:

1. **Feed-forward (kF)**:
   - Set kP/kI/kD to 0
   - Increase kF until RPM reaches ~70-80% of target
   - Formula: `kF ≈ 1 / max_RPM`

2. **Proportional (kP)**:
   - Increase until oscillation appears
   - Reduce by 50%

3. **Derivative (kD)**:
   - Add small amount to dampen overshoot
   - Too much causes sluggish response

4. **Integral (kI)**:
   - Usually unnecessary (leave at 0)
   - Only add if persistent steady-state error

**Use FTC Dashboard** to tune live during test!

---

### Turret Tracking Tuning:

**Too Jittery**:
- Increase `SMOOTHING` (0.5 → 0.7)
- Increase `DEADBAND` (3.0 → 5.0)
- Decrease `TRACKING_GAIN` (0.08 → 0.05)

**Too Slow to Track**:
- Decrease `SMOOTHING` (0.5 → 0.3)
- Increase `TRACKING_GAIN` (0.08 → 0.10)
- Decrease `DEADBAND` (3.0 → 2.0)

---

### Path Following Tuning:

See `PedroConstants.java` - adjust:
- Translational PIDF for driving accuracy
- Heading PIDF for turning precision
- Path constraints for speed limits

---

## Troubleshooting

### Shooter Issues:

**RPM not reaching target**:
- Check kF is not too low
- Verify motors spinning correct direction
- Check battery voltage (low voltage = low max speed)

**RPM oscillating**:
- Reduce kP
- Add small kD to dampen

**Balls jamming**:
- Ensure ShootBallSteadyCmd tolerance is reasonable (50-100 RPM)
- Check transfer motor direction

---

### Intake Issues:

**Not collecting balls**:
- Verify intake motor direction (should be REVERSED)
- Check IntakeSeqCmd starts both motors
- Test rangefinder detection threshold (40mm)

**Ball count incorrect**:
- Adjust DEBOUNCE_MS if double-counting
- Check rangefinder distance in telemetry

---

### Turret Issues:

**Not tracking targets**:
- Verify Limelight connected (check telemetry)
- Check AprilTag pipeline (should be pipeline 0)
- Ensure AUTO_TRACK_ENABLED = true

**Tracking wrong target**:
- Algorithm picks closest (largest pixel area)
- May need to filter by tag ID if needed

---

### Autonomous Issues:

**Robot not moving**:
- Verify PedroComponent added to OpMode
- Check Follower updates every loop
- Verify paths built correctly (telemetry pose)

**Robot drives wrong path**:
- Check starting pose matches field position
- Verify motor directions in PedroConstants
- Check odometry encoder directions

**Path never finishes**:
- Check `isBusy()` condition
- Verify holdEnd setting
- Add timeout with AutoDriveTimeoutCmd

---

## Future Improvements

### Potential Enhancements:
1. **Distance-based shooting**: Use Limelight distance + ShooterBallistics to auto-adjust RPM/hood
2. **Multi-target tracking**: Track multiple AprilTags and pick optimal
3. **Predictive aiming**: Lead moving targets
4. **Auto-alignment**: Use Pedro Pathing to auto-drive to shooting position
5. **State machine**: Automated game element cycling (intake → drive → shoot)
6. **Vision-based intaking**: Use camera to locate balls on field
7. **Telemetry logging**: Record match data for post-game analysis

---

## Code Organization

```
teamcode/
├── subsystems/           # Hardware abstractions
│   ├── Shooter.java
│   ├── Intake.java
│   ├── Turret.java
│   ├── TeleopMecanumDrive.java
│   └── LaserRangefinder.java
├── commands/             # Robot actions
│   ├── IntakeSeqCmd.java
│   ├── RapidFireCmd.java
│   ├── ShootBallSteadyCmd.java
│   └── WaitCmd.java
├── constants/            # Tuning parameters
│   ├── ShooterConstants.java
│   ├── IntakeConstants.java
│   ├── TurretConstants.java
│   ├── PedroConstants.java
│   ├── ShooterBallistics.java
│   └── LimelightConstants.java
├── opmodes/
│   ├── teleops/         # Driver control modes
│   │   ├── Teleop.java
│   │   └── TeleOpTurretTracking.java
│   └── autos/           # Autonomous routines
│       ├── AutoBlue.java
│       ├── AutoRed.java
│       ├── CloseBlueAuto.java
│       └── FarBlueAuto.java
└── docs/
    ├── AutoContext.md   # Framework deep-dive
    └── robot.md         # This file
```

---

## Credits

- **Framework**: NextFTC (Command-based architecture)
- **Path Following**: Pedro Pathing by Team 10158
- **Vision**: Limelight 3A
- **Odometry**: GoBILDA Pinpoint

**Team**: FTC Decode 2025  
**Season**: 2024-2025

---

**End of Documentation**

