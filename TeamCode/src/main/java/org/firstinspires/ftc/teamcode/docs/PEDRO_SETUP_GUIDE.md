# Pedro Pathing Setup and Tuning Guide

## Overview
Pedro Pathing is a powerful motion planning library for FTC robots. This guide explains how to set it up and tune it for your robot.

## What I've Done For You

### 1. Refactored Auto2.java
- **Removed raw sleeps** - Now uses NextFTCOpMode's command-based system
- **Added Pedro Pathing integration** - Uses PedroPathingCommand to drive forward 5 inches
- **Command groups** - Turret movements are sequenced using SequentialGroup
- **Clean structure** - The autonomous routine is defined declaratively

### 2. Created PedroPathingCommand.java
A reusable command wrapper that:
- Takes a Follower and Path as parameters
- Executes the path using Pedro's update loop
- Finishes when the path is complete
- Can be interrupted safely

### 3. Created PedroConstants.java
Configuration file with:
- Starting pose for autonomous
- Tuning parameters (PID coefficients, accelerations)
- `createFollower()` method to initialize Pedro
- `createForwardPath()` helper to generate simple paths

## Setup Steps

### 1. Fix Java Version (Current Issue)
The Gradle error indicates a Java version mismatch. You need:
- **Java 17** for FTC SDK 11.0.0
- Check your Java version: `java -version`
- If needed, update JAVA_HOME environment variable

### 2. Verify Pedro Pathing Dependency
The dependency is already added to `TeamCode/build.gradle`:
```groovy
implementation 'com.pedropathing:pedro:1.0.0'
```

### 3. Hardware Configuration
In your Robot Controller app, configure these motor names:
- `leftFront`, `leftBack`, `rightFront`, `rightBack`
- These are the default names Pedro expects

If your motors have different names (like `frontLeftMotor`), you'll need to either:
- Rename them in the hardware config, OR
- Pass custom motor names to the Follower constructor

## Tuning Pedro Pathing

### Phase 1: Localization Tuning

#### Step 1: Forward Pusher Test
1. Open `PedroConstants.java` in FTC Dashboard
2. Place robot in a known position
3. Command it to drive forward a known distance (e.g., 48 inches)
4. Measure actual distance traveled
5. Adjust `FORWARD_ZERO_POWER_ACCELERATION` if needed

#### Step 2: Lateral Pusher Test
1. Command robot to strafe sideways a known distance
2. Measure actual distance
3. Adjust `LATERAL_MULTIPLIER` and `LATERAL_ZERO_POWER_ACCELERATION`

### Phase 2: PID Tuning

#### Translational PID (Movement to Target)
1. Start with current values in `PedroConstants.java`:
   - `TRANSLATIONAL_PID_P = 0.1`
   - `TRANSLATIONAL_PID_I = 0.0`
   - `TRANSLATIONAL_PID_D = 0.01`

2. Tuning process:
   - **Increase P** until robot oscillates around target
   - **Add D** to dampen oscillations (typically 10% of P)
   - **Add I** only if robot doesn't reach target (rare)

3. Test with different path lengths (5", 24", 48")

#### Heading PID (Rotation Control)
1. Start with:
   - `HEADING_PID_P = 2.0`
   - `HEADING_PID_I = 0.0`
   - `HEADING_PID_D = 0.1`

2. Test rotation commands
3. Adjust if robot oscillates or is too sluggish

### Phase 3: Path Following

#### Creating Paths
Simple forward path:
```
Path forward = new Path(new BezierCurve(
    new Point(0, 0, Point.CARTESIAN),
    new Point(24, 0, Point.CARTESIAN)
));
```

Path with curve:
```
Path curved = new Path(new BezierCurve(
    new Point(0, 0, Point.CARTESIAN),
    new Point(12, 12, Point.CARTESIAN),
    new Point(24, 0, Point.CARTESIAN)
));
```

Path with heading control:
```
Path withHeading = new Path(new BezierLine(
    new Point(0, 0, Point.CARTESIAN),
    new Point(24, 0, Point.CARTESIAN)
));
withHeading.setConstantHeadingInterpolation(Math.toRadians(45));
```

#### Testing Paths
1. Start with simple straight paths
2. Gradually increase complexity
3. Use FTC Dashboard to visualize paths
4. Monitor telemetry for position error

### Phase 4: Advanced Tuning

#### Max Power
```
follower.setMaxPower(0.8);  // Adjust in PedroConstants.createFollower()
```
- Lower for more precise movements
- Higher for faster but less accurate paths

#### Centripetal Force Correction
```
// Add to createFollower() method
follower.setEnableCentripetalCorrection(true);
follower.setCentripetalScaling(0.0001);
```
- Helps with curved paths
- Tune scaling factor based on robot behavior

## Using in Your Auto

Your refactored `Auto2.java` shows the proper pattern:

```
private Command autonomousRoutine() {
    // Create path
    Path forwardPath = PedroConstants.createForwardPath(5.0);
    
    // Compose with other commands
    return new SequentialGroup(
        // Drive using Pedro
        new PedroPathingCommand(follower(), forwardPath),
        
        // Other subsystem commands
        Turret.INSTANCE.runTurret(-45.0),
        Turret.INSTANCE.runTurret(45.0),
        Turret.INSTANCE.runTurret(0.0)
    );
}
```

### Command Composition Examples

#### Parallel Movement
```
new ParallelGroup(
    new PedroPathingCommand(follower(), path),
    Intake.INSTANCE.turnOn  // Run intake while driving
)
```

#### Sequential with Delays
```
new SequentialGroup(
    new PedroPathingCommand(follower(), driveToBasket),
    Lift.INSTANCE.toHigh,
    new Delay(0.5),
    Claw.INSTANCE.open,
    new PedroPathingCommand(follower(), driveBack)
)
```

## Troubleshooting

### Robot Doesn't Move
- Check motor directions in hardware config
- Verify IMU is initialized correctly
- Check that PedroComponent is added to OpMode

### Robot Oscillates
- Reduce P gain
- Increase D gain
- Lower max power

### Robot Overshoots
- Reduce max power
- Increase D gain
- Check deceleration constraints

### Position Drift
- Recalibrate localization
- Check for wheel slippage
- Verify encoder counts are accurate

## Resources

- **Pedro Pathing Docs**: https://pedropathing.com/
- **NextFTC Docs**: Check `/docs` folder in this repo
- **FTC Dashboard**: Essential for real-time tuning
  - Connect to `192.168.43.1:8080` (or your robot's IP)
  - Adjust PedroConstants values live
  - Visualize robot position and paths

## Quick Start Checklist

- [ ] Fix Java version issue (use Java 17)
- [ ] Sync Gradle dependencies
- [ ] Configure motor names in Robot Controller
- [ ] Run Auto2 to test basic movement
- [ ] Tune localization (forward/lateral)
- [ ] Tune PID gains
- [ ] Create and test your autonomous paths
- [ ] Compose paths with subsystem commands

## Next Steps

1. **Fix the Java version** - This is blocking the build
2. **Deploy to robot** - Test the basic forward movement
3. **Tune incrementally** - Start with localization, then PID
4. **Build complex autos** - Use the command composition pattern shown in Auto2

Your autonomous is now using proper command groups instead of sleeps, making it more maintainable and allowing for better timing control!

