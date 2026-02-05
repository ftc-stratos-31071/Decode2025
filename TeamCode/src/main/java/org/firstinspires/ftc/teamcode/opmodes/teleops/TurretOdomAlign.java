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

    // Turret physical limits (updated to match TurretConstants)
    private static final double TURRET_MIN_DEG = -30.0;
    private static final double TURRET_MAX_DEG = 200.0;

    // Robot starting heading (90 degrees as specified)
    private double robotStartHeading = 90.0;
    private boolean hasInitializedHeading = false;

    // CRITICAL: Track turret position like Teleop.java does with motorTargetX
    private double turretTargetAngle = 90.0;

    // Smoothing and deadband (matching Teleop.java values)
    private double smoothedTurretAngle = 90.0;
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

        // Initialize turret to center (90 degrees)
        turretTargetAngle = 90.0;
        smoothedTurretAngle = 90.0;
        Turret.INSTANCE.setTurretAngleDeg(turretTargetAngle);

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

        // Get current robot heading in degrees from odometry
        double robotHeadingDeg = pose.getHeading(AngleUnit.DEGREES);

        // On first update, record the starting heading
        if (!hasInitializedHeading) {
            robotStartHeading = robotHeadingDeg;
            hasInitializedHeading = true;
        }

        // Simple field-centric turret calculation:
        // Turret angle = TARGET_FIELD_ANGLE - current_robot_heading
        // When robot rotates RIGHT (heading increases), turret must rotate LEFT (angle decreases)
        double desiredTurretAngle = TARGET_FIELD_ANGLE_DEG - robotHeadingDeg;

        // Handle turret limits and wrapping - this also normalizes the angle
        double finalTurretAngle = constrainTurretAngle(desiredTurretAngle);

        // Update target angle based on odometry calculations
        turretTargetAngle = finalTurretAngle;

        // Apply smoothing using the EXACT formula from Teleop.java
        smoothedTurretAngle = TURRET_SMOOTHING * smoothedTurretAngle + (1.0 - TURRET_SMOOTHING) * turretTargetAngle;

        // ALWAYS schedule the turret command
        Turret.INSTANCE.goToAngle(smoothedTurretAngle).schedule();

        // Telemetry
        telemetry.addData("═══ ODOMETRY TURRET ALIGN ═══", "");
        telemetry.addData("Status", hasInitializedHeading ? "TRACKING" : "INITIALIZING");
        telemetry.addData("Robot Heading (Pinpoint)", String.format("%.1f°", robotHeadingDeg));
        telemetry.addData("Start Heading", String.format("%.1f°", robotStartHeading));
        telemetry.addData("Target Field Angle", String.format("%.1f°", TARGET_FIELD_ANGLE_DEG));
        telemetry.addData("Desired Turret Angle", String.format("%.1f°", desiredTurretAngle));
        telemetry.addData("Final Turret Angle", String.format("%.1f°", finalTurretAngle));
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
     * Constrain turret angle to physical limits, with improved wrapping logic
     * for the -30° to 200° range (centered at 90°)
     */
    private double constrainTurretAngle(double angleDeg) {
        // If within limits, use as-is
        if (angleDeg >= TURRET_MIN_DEG && angleDeg <= TURRET_MAX_DEG) {
            return angleDeg;
        }

        // Try wrapping by ±360 degrees to find the equivalent angle within range
        double wrappedPlus = angleDeg + 360.0;
        double wrappedMinus = angleDeg - 360.0;

        // Check if either wrapped version is within limits
        if (wrappedPlus >= TURRET_MIN_DEG && wrappedPlus <= TURRET_MAX_DEG) {
            return wrappedPlus;
        }
        if (wrappedMinus >= TURRET_MIN_DEG && wrappedMinus <= TURRET_MAX_DEG) {
            return wrappedMinus;
        }

        // If no wrapped version works, find the closest reachable angle
        // This handles edge cases where the target is unreachable

        // Calculate distance to each limit (accounting for wrapping)
        double distToMin = Math.abs(angleDistance(angleDeg, TURRET_MIN_DEG));
        double distToMax = Math.abs(angleDistance(angleDeg, TURRET_MAX_DEG));

        // Return the closest limit
        return (distToMin < distToMax) ? TURRET_MIN_DEG : TURRET_MAX_DEG;
    }

    /**
     * Calculate the shortest angular distance between two angles
     * Accounts for wrapping (e.g., -10° is only 20° away from 10°, not 20°)
     */
    private double angleDistance(double angle1, double angle2) {
        double diff = angle2 - angle1;
        // Normalize to -180 to +180
        while (diff > 180.0) diff -= 360.0;
        while (diff < -180.0) diff += 360.0;
        return diff;
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

