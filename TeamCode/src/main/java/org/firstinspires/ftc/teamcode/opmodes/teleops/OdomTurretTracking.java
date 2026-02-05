package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.constants.TurretConstants;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * OpMode that uses odometry to maintain a fixed global turret heading.
 * The turret counter-rotates as the robot rotates to keep pointing at the same global direction.
 * When turret constraints are hit, it wraps around to continue tracking from the opposite side.
 */
@Config
@TeleOp(name = "OdomTurretTracking")
public class OdomTurretTracking extends NextFTCOpMode {

    private GoBildaPinpointDriver pinpoint;

    // Tunable parameters via FTC Dashboard
    public static double INITIAL_GLOBAL_HEADING = 135.0; // Global direction turret should face (degrees) - 45° left of init
    public static boolean ENABLE_TRACKING = true; // Toggle tracking on/off
    public static double PREDICTION_GAIN = 0.1; // Gain for predicting future position (tuneable)
    public static double MAX_PREDICTION_DEG = 5.0; // Max prediction angle (degrees) to prevent overshoot

    private final MotorEx frontLeftMotor = new MotorEx("frontLeftMotor").brakeMode().reversed();
    private final MotorEx frontRightMotor = new MotorEx("frontRightMotor").brakeMode().reversed();
    private final MotorEx backLeftMotor = new MotorEx("backLeftMotor").brakeMode();
    private final MotorEx backRightMotor = new MotorEx("backRightMotor").brakeMode();

    private double targetGlobalHeading = 135.0; // The global direction we want turret to face
    private boolean initialized = false;
    private double lastRobotHeading = 135.0;
    private long lastUpdateTime = 0;

    // Smoothing parameters
    public static double VELOCITY_SMOOTHING = 0.7; // Smoothing factor for velocity (0 to 1)
    public static boolean USE_PREDICTION = true; // Enable/disable prediction

    private double smoothedVelocity = 0.0;

    public OdomTurretTracking() {
        addComponents(
                new SubsystemComponent(Turret.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();

        // Set initial robot heading to 90 degrees (robot's starting orientation)
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 90));

        // Initialize turret to center position
        Turret.INSTANCE.setTurretAngleDeg(TurretConstants.DEFAULT_TURRET_DEG);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Robot Initial Heading", "90°");
        telemetry.addData("Will Lock To", "135° global (45° left of robot start)");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        // Set up mecanum drive control
        var forward = Gamepads.gamepad1().leftStickY().negate();
        var strafe = Gamepads.gamepad1().leftStickX();
        var rotate = Gamepads.gamepad1().rightStickX();

        Command drive = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                forward,
                strafe,
                rotate
        );

        drive.schedule();
    }

    @Override
    public void onUpdate() {
        // Update odometry
        pinpoint.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);
        double currentRobotHeading = pinpoint.getHeading(AngleUnit.DEGREES);

        long currentTime = System.currentTimeMillis();

        // Initialize on first update - lock to target global heading
        if (!initialized) {
            targetGlobalHeading = INITIAL_GLOBAL_HEADING;
            lastRobotHeading = currentRobotHeading;
            lastUpdateTime = currentTime;
            initialized = true;
        }

        // Calculate required turret angle to maintain global heading
        if (ENABLE_TRACKING) {
            // Calculate robot's rotation velocity with smoothing
            double deltaTime = (currentTime - lastUpdateTime) / 1000.0; // Convert to seconds
            double headingChange = normalizeAngleSigned(currentRobotHeading - lastRobotHeading);

            // Avoid division by zero and handle first frame
            double instantVelocity = 0.0;
            if (deltaTime > 0.001) { // More than 1ms
                instantVelocity = headingChange / deltaTime; // degrees per second
            }

            // Smooth the velocity to reduce jitter
            smoothedVelocity = VELOCITY_SMOOTHING * smoothedVelocity + (1.0 - VELOCITY_SMOOTHING) * instantVelocity;

            // Predict where the robot will be based on current rotation velocity
            double prediction = 0.0;
            if (USE_PREDICTION) {
                prediction = smoothedVelocity * PREDICTION_GAIN;

                // Clamp prediction to prevent wild overshoots
                if (prediction > MAX_PREDICTION_DEG) prediction = MAX_PREDICTION_DEG;
                if (prediction < -MAX_PREDICTION_DEG) prediction = -MAX_PREDICTION_DEG;
            }

            // Calculate turret angle with predictive compensation
            double predictedRobotHeading = currentRobotHeading + prediction;
            double requiredTurretAngle = calculateTurretAngle(predictedRobotHeading, targetGlobalHeading);

            // Debug: Calculate servo positions
            double servoAngleDeg = requiredTurretAngle;
            double rawServoPos = servoAngleDeg / 355.0;
            double clampedServoPos = Math.max(0, Math.min(1, rawServoPos));
            boolean isClamping = (rawServoPos != clampedServoPos);

            Turret.INSTANCE.setTurretAngleDeg(requiredTurretAngle);

            // Store for next iteration
            lastRobotHeading = currentRobotHeading;
            lastUpdateTime = currentTime;

            // COMPREHENSIVE TELEMETRY
            telemetry.addLine("=== ODOMETRY TRACKING ===");
            telemetry.addData("Robot Heading", "%.1f°", currentRobotHeading);
            telemetry.addData("Target Global", "%.1f°", targetGlobalHeading);
            telemetry.addData("Error", "%.1f°", normalizeAngleSigned(targetGlobalHeading - (currentRobotHeading + requiredTurretAngle)));
            telemetry.addLine();

            telemetry.addLine("=== VELOCITY & PREDICTION ===");
            telemetry.addData("Instant Velocity", "%.1f °/s", instantVelocity);
            telemetry.addData("Smoothed Velocity", "%.1f °/s", smoothedVelocity);
            telemetry.addData("Prediction", USE_PREDICTION ? String.format("%.1f°", prediction) : "DISABLED");
            telemetry.addData("Predicted Heading", "%.1f°", predictedRobotHeading);
            telemetry.addLine();

            telemetry.addLine("=== TURRET OUTPUT ===");
            telemetry.addData("Required Turret", "%.1f°", requiredTurretAngle);
            telemetry.addData("Actual Turret", "%.1f°", Turret.INSTANCE.getTargetTurretDeg());
            telemetry.addData("Actual Global", "%.1f°", normalizeAngle(currentRobotHeading + Turret.INSTANCE.getTargetTurretDeg()));
            telemetry.addLine();

            telemetry.addLine("=== SERVO DEBUG ===");
            telemetry.addData("Servo Angle", "%.1f°", servoAngleDeg);
            telemetry.addData("Raw Servo Pos", "%.3f", rawServoPos);
            telemetry.addData("Clamped Pos", "%.3f", clampedServoPos);
            telemetry.addData("IS CLAMPING?", isClamping ? "⚠️ YES!" : "✓ No");

            if (isClamping) {
                telemetry.addLine();
                telemetry.addData("WARNING", "Turret at physical limit!");
            }
        } else {
            telemetry.addData("Tracking", "❌ DISABLED");
        }

        telemetry.update();
    }

    /**
     * Calculate the turret angle needed to point at the target global heading.
     * Handles wrapping around constraints when necessary.
     *
     * @param robotHeading Current robot heading (0-360)
     * @param globalTarget Target global heading (0-360)
     * @return Turret angle in degrees (constrained to MIN/MAX)
     */
    private double calculateTurretAngle(double robotHeading, double globalTarget) {
        // Calculate the naive turret angle: global_target - robot_heading
        double naiveTurretAngle = globalTarget - robotHeading;

        // Normalize to [-180, 180] range for shortest path
        naiveTurretAngle = normalizeAngleSigned(naiveTurretAngle);

        // Check if naive angle is within turret constraints
        if (naiveTurretAngle >= TurretConstants.MIN_TURRET_DEG &&
            naiveTurretAngle <= TurretConstants.MAX_TURRET_DEG) {
            return naiveTurretAngle;
        }

        // Naive angle is out of bounds, try wrapping around
        double wrappedAngle = naiveTurretAngle > 0 ? naiveTurretAngle - 360 : naiveTurretAngle + 360;

        // Check if wrapped angle is within constraints
        if (wrappedAngle >= TurretConstants.MIN_TURRET_DEG &&
            wrappedAngle <= TurretConstants.MAX_TURRET_DEG) {
            return wrappedAngle;
        }

        // Neither works perfectly, clamp to nearest constraint
        if (Math.abs(naiveTurretAngle - TurretConstants.MIN_TURRET_DEG) <
            Math.abs(naiveTurretAngle - TurretConstants.MAX_TURRET_DEG)) {
            return TurretConstants.MIN_TURRET_DEG;
        } else {
            return TurretConstants.MAX_TURRET_DEG;
        }
    }

    /**
     * Normalize angle to [0, 360) range
     */
    private double normalizeAngle(double degrees) {
        degrees = degrees % 360.0;
        if (degrees < 0) {
            degrees += 360.0;
        }
        return degrees;
    }

    /**
     * Normalize angle to [-180, 180] range (shortest path)
     */
    private double normalizeAngleSigned(double degrees) {
        degrees = degrees % 360.0;
        if (degrees > 180.0) {
            degrees -= 360.0;
        } else if (degrees < -180.0) {
            degrees += 360.0;
        }
        return degrees;
    }

    @Override
    public void onStop() {
        // Return turret to center position
        Turret.INSTANCE.setTurretAngleDeg(TurretConstants.DEFAULT_TURRET_DEG);
    }
}

