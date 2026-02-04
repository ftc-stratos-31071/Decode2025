package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "TurretOdomAlign")
public class TurretOdomAlign extends NextFTCOpMode {

    // Drivetrain motors
    private final MotorEx frontLeftMotor = new MotorEx("frontLeftMotor").brakeMode().reversed();
    private final MotorEx frontRightMotor = new MotorEx("frontRightMotor").brakeMode().reversed();
    private final MotorEx backLeftMotor = new MotorEx("backLeftMotor").brakeMode();
    private final MotorEx backRightMotor = new MotorEx("backRightMotor").brakeMode();

    // Pinpoint odometry sensor
    private GoBildaPinpointDriver pinpoint;

    // Target field angle for turret (45 degrees left of center = 135 degrees)
    private static final double TARGET_FIELD_ANGLE_DEG = 135.0;

    // Turret physical limits (90 to 270 degrees range)
    private static final double TURRET_MIN_DEG = 90.0;
    private static final double TURRET_MAX_DEG = 270.0;

    // Robot starting heading (90 degrees as specified)
    private double robotStartHeading = 90.0;
    private boolean hasInitializedHeading = false;

    // CRITICAL: Track turret position like Teleop.java does with motorTargetX
    private double turretTargetAngle = 180.0;

    // Smoothing and deadband (matching Teleop.java values)
    private double smoothedTurretAngle = 180.0;
    private static final double TURRET_SMOOTHING = 0.5;  // Matching Teleop.java SMOOTHING value
    private static final double TURRET_DEADBAND = 3.0;   // Matching Teleop.java DEADBAND value

    public TurretOdomAlign() {
        addComponents(
                new SubsystemComponent(Turret.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        // Initialize Pinpoint sensor
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        configurePinpoint();

        // Initialize turret to center (180 degrees)
        turretTargetAngle = 180.0;
        Turret.INSTANCE.setTurretAngleDeg(turretTargetAngle);
        Turret.INSTANCE.goToAngle(turretTargetAngle).schedule();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Target Field Angle", TARGET_FIELD_ANGLE_DEG + "°");
        telemetry.addData("Info", "Turret will maintain 45° left of field forward");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        // Set up driver control
        var forward = Gamepads.gamepad1().leftStickY().negate();
        var strafe = Gamepads.gamepad1().leftStickX();
        var rotate = Gamepads.gamepad1().rightStickX();

        Command driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                forward,
                strafe,
                rotate
        );
        driverControlled.schedule();

        // Optional: Add button to reset heading calibration
        Gamepads.gamepad1().y().whenBecomesTrue(() -> {
            pinpoint.resetPosAndIMU();
            hasInitializedHeading = false;
            telemetry.addData("Reset", "Heading and position reset");
        });
    }

    @Override
    public void onUpdate() {
        // Update pinpoint sensor to get latest position
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();

        // Get current robot heading in degrees
        double robotHeadingDeg = pose.getHeading(AngleUnit.DEGREES);

        // On first update, record the starting heading
        if (!hasInitializedHeading) {
            robotStartHeading = robotHeadingDeg;
            hasInitializedHeading = true;
        }

        // Calculate robot's current heading relative to start (normalized to 0-360)
        double robotCurrentHeading = normalizeAngle(robotHeadingDeg - robotStartHeading + 90.0);

        // Calculate required turret angle to point at target field angle
        // Turret angle (robot-relative) = target field angle - robot heading
        double desiredTurretAngle = normalizeAngle(TARGET_FIELD_ANGLE_DEG - robotCurrentHeading);

        // Convert from 0-360 range to -180 to +180 range for turret control
        if (desiredTurretAngle > 180.0) {
            desiredTurretAngle -= 360.0;
        }

        // Handle turret limits and wrapping
        double finalTurretAngle = constrainTurretAngle(desiredTurretAngle);

        // Update target angle based on odometry calculations
        turretTargetAngle = finalTurretAngle;

        // Apply smoothing using the EXACT formula from Teleop.java (line 306)
        // This provides gradual changes for even tiny movements
        // Formula: smoothed = SMOOTHING * oldSmoothed + (1.0 - SMOOTHING) * newTarget
        smoothedTurretAngle = TURRET_SMOOTHING * smoothedTurretAngle + (1.0 - TURRET_SMOOTHING) * turretTargetAngle;

        // ALWAYS schedule the turret command (like Teleop.java line 365)
        Turret.INSTANCE.goToAngle(smoothedTurretAngle).schedule();

        // Telemetry
        telemetry.addData("═══ ODOMETRY TURRET ALIGN ═══", "");
        telemetry.addData("Status", hasInitializedHeading ? "TRACKING" : "INITIALIZING");
        telemetry.addData("Raw Robot Heading", String.format("%.1f°", robotHeadingDeg));
        telemetry.addData("Start Heading Offset", String.format("%.1f°", robotStartHeading));
        telemetry.addData("Robot Heading (field)", String.format("%.1f°", robotCurrentHeading));
        telemetry.addData("Target Field Angle", String.format("%.1f°", TARGET_FIELD_ANGLE_DEG));
        telemetry.addData("Desired Turret Angle", String.format("%.1f°", desiredTurretAngle));
        telemetry.addData("Final Turret Angle", String.format("%.1f°", finalTurretAngle));
        telemetry.addData("Turret Target Variable", String.format("%.1f°", turretTargetAngle));
        telemetry.addData("Smoothed Turret Angle", String.format("%.1f°", smoothedTurretAngle));
        telemetry.addData("Actual Turret Position", String.format("%.1f°", Turret.INSTANCE.getTargetTurretDeg()));
        telemetry.addData("", "");
        telemetry.addData("Robot X (inches)", String.format("%.1f", pose.getX(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH)));
        telemetry.addData("Robot Y (inches)", String.format("%.1f", pose.getY(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH)));
        telemetry.addData("", "");
        telemetry.addData("Press Y", "Reset heading calibration");
        telemetry.update();
    }

    /**
     * Normalize angle to 0-360 degree range
     */
    private double normalizeAngle(double degrees) {
        degrees = degrees % 360.0;
        if (degrees < 0) {
            degrees += 360.0;
        }
        return degrees;
    }

    /**
     * Constrain turret angle to physical limits, with wrapping logic
     * If the target angle is outside limits, try wrapping (adding/subtracting 360)
     * If still outside, clamp to nearest limit
     */
    private double constrainTurretAngle(double angleDeg) {
        // If within limits, use as-is
        if (angleDeg >= TURRET_MIN_DEG && angleDeg <= TURRET_MAX_DEG) {
            return angleDeg;
        }

        // Try wrapping by 360 degrees
        double wrappedAngle = angleDeg;
        if (angleDeg > TURRET_MAX_DEG) {
            wrappedAngle = angleDeg - 360.0;
        } else if (angleDeg < TURRET_MIN_DEG) {
            wrappedAngle = angleDeg + 360.0;
        }

        // If wrapped angle is within limits, use it
        if (wrappedAngle >= TURRET_MIN_DEG && wrappedAngle <= TURRET_MAX_DEG) {
            return wrappedAngle;
        }

        // If still out of bounds, clamp to nearest limit
        if (angleDeg > TURRET_MAX_DEG) {
            return TURRET_MAX_DEG;
        } else {
            return TURRET_MIN_DEG;
        }
    }

    /**
     * Configure the Pinpoint sensor with appropriate settings
     */
    private void configurePinpoint() {
        // Set odometry pod offsets (using values from PedroConstants)
        // forwardPodY = 93.5mm, strafePodX = -118.7mm
        pinpoint.setOffsets(-118.7, 93.5, org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM);

        // Set encoder type (4-bar pods as used in PedroConstants)
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Set encoder directions (from PedroConstants: both REVERSED)
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        // Reset position and recalibrate IMU
        // Start at (0, 0) with 90 degree heading as specified
        pinpoint.setPosition(new Pose2D(
                org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH,
                0, 0,
                AngleUnit.DEGREES,
                90.0
        ));
    }
}

