package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.constants.TurretConstants;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * Tuning OpMode to find the correct ratio between robot rotation and turret counter-rotation.
 *
 * Controls:
 * - A Button: Rotate robot 90° clockwise AND rotate turret to compensate
 * - B Button: Reset turret to center (0°)
 * - Adjust TURRET_TO_ROBOT_RATIO via FTC Dashboard until turret maintains same global direction
 */
@Config
@TeleOp(name = "TurretRotationTuner")
public class TurretRotationTuner extends NextFTCOpMode {

    private GoBildaPinpointDriver pinpoint;

    // TUNABLE: This is the ratio you need to find!
    // If robot rotates 90°, turret should rotate (90° * ratio) in opposite direction
    // Start at 1.0 (1:1 ratio) and adjust until turret maintains global direction
    public static double TURRET_TO_ROBOT_RATIO = 1.0;

    // Amount robot rotates when you press A
    public static double TEST_ROTATION_DEG = 90.0;

    private double initialRobotHeading = 90.0;
    private double currentRobotHeading = 90.0;
    private double initialTurretAngle = 0.0;
    private boolean testActive = false;

    public TurretRotationTuner() {
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

        // Set initial robot heading to 90 degrees
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 90));

        // Initialize turret to center
        Turret.INSTANCE.setTurretAngleDeg(0.0);

        telemetry.addData("Status", "Initialized");
        telemetry.addLine();
        telemetry.addData("Instructions", "");
        telemetry.addData("Quick Start", "Press Y to test");
        telemetry.addData("1.", "Y = Rotate robot + apply compensation");
        telemetry.addData("2.", "Check Error on screen");
        telemetry.addData("3.", "Adjust TURRET_TO_ROBOT_RATIO");
        telemetry.addData("4.", "Press B to reset and try again");
        telemetry.addLine();
        telemetry.addData("OR Manual", "");
        telemetry.addData("A", "Rotate robot 90° only");
        telemetry.addData("X", "Apply compensation manually");
        telemetry.addLine();
        telemetry.addData("Current Ratio", TURRET_TO_ROBOT_RATIO);
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        // Button A: Simulate robot rotating 90° by updating Pinpoint heading
        Gamepads.gamepad1().a().whenBecomesTrue(() -> {
            // Record starting positions
            initialRobotHeading = currentRobotHeading;
            initialTurretAngle = Turret.INSTANCE.getTargetTurretDeg();

            // SIMULATE robot rotating 90° clockwise by setting new heading
            double newHeading = normalizeAngle(currentRobotHeading - TEST_ROTATION_DEG);
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, newHeading));

            testActive = true;

            telemetry.addData("Simulated Rotation", "Robot heading changed by %.0f°", -TEST_ROTATION_DEG);
            telemetry.addData("Old Heading", "%.1f°", initialRobotHeading);
            telemetry.addData("New Heading", "%.1f°", newHeading);
        });

        // Button B: Reset turret to center
        Gamepads.gamepad1().b().whenBecomesTrue(() -> {
            Turret.INSTANCE.setTurretAngleDeg(0.0);
            testActive = false;

            // Also reset robot heading back to 90°
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 90));
            initialRobotHeading = 90.0;
            currentRobotHeading = 90.0;

            telemetry.addData("Reset", "Turret and robot heading reset");
        });

        // Button X: Apply turret compensation based on current robot heading change
        Gamepads.gamepad1().x().whenBecomesTrue(() -> {
            double robotRotation = normalizeAngleSigned(currentRobotHeading - initialRobotHeading);
            double turretCompensation = -robotRotation * TURRET_TO_ROBOT_RATIO;

            // Clamp to turret limits
            if (turretCompensation < TurretConstants.MIN_TURRET_DEG) {
                turretCompensation = TurretConstants.MIN_TURRET_DEG;
            }
            if (turretCompensation > TurretConstants.MAX_TURRET_DEG) {
                turretCompensation = TurretConstants.MAX_TURRET_DEG;
            }

            Turret.INSTANCE.setTurretAngleDeg(turretCompensation);

            telemetry.addData("Applied Compensation", "Turret moved to %.1f°", turretCompensation);
        });

        // Button Y: Combined - rotate robot AND apply turret compensation in one press
        Gamepads.gamepad1().y().whenBecomesTrue(() -> {
            // Record starting positions
            initialRobotHeading = currentRobotHeading;
            initialTurretAngle = Turret.INSTANCE.getTargetTurretDeg();

            // Simulate robot rotating
            double newHeading = normalizeAngle(currentRobotHeading - TEST_ROTATION_DEG);
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, newHeading));

            // Immediately calculate and apply turret compensation
            double robotRotation = -TEST_ROTATION_DEG; // We know exactly how much we rotated
            double turretCompensation = -robotRotation * TURRET_TO_ROBOT_RATIO;

            // Clamp to turret limits
            if (turretCompensation < TurretConstants.MIN_TURRET_DEG) {
                turretCompensation = TurretConstants.MIN_TURRET_DEG;
            }
            if (turretCompensation > TurretConstants.MAX_TURRET_DEG) {
                turretCompensation = TurretConstants.MAX_TURRET_DEG;
            }

            Turret.INSTANCE.setTurretAngleDeg(turretCompensation);

            testActive = true;

            telemetry.addData("ONE-STEP TEST", "Robot rotated %.0f°, Turret compensated %.1f°",
                -TEST_ROTATION_DEG, turretCompensation);
        });
    }

    @Override
    public void onUpdate() {
        // Update odometry
        pinpoint.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);
        currentRobotHeading = pinpoint.getHeading(AngleUnit.DEGREES);

        // Calculate how much robot has rotated since test started
        double robotRotation = normalizeAngleSigned(currentRobotHeading - initialRobotHeading);

        // Calculate what turret compensation should be
        double idealTurretCompensation = -robotRotation * TURRET_TO_ROBOT_RATIO;

        // Clamp to turret physical limits
        double clampedTurretCompensation = idealTurretCompensation;
        if (clampedTurretCompensation < TurretConstants.MIN_TURRET_DEG) {
            clampedTurretCompensation = TurretConstants.MIN_TURRET_DEG;
        }
        if (clampedTurretCompensation > TurretConstants.MAX_TURRET_DEG) {
            clampedTurretCompensation = TurretConstants.MAX_TURRET_DEG;
        }

        // Calculate what the global direction should be
        double currentTurretAngle = Turret.INSTANCE.getTargetTurretDeg();
        double actualGlobalDirection = normalizeAngle(currentRobotHeading + currentTurretAngle);

        // Telemetry
        telemetry.addLine("=== TURRET ROTATION TUNER ===");
        telemetry.addData("Current Ratio", "%.3f", TURRET_TO_ROBOT_RATIO);
        telemetry.addData("Test Rotation", "%.0f°", TEST_ROTATION_DEG);
        telemetry.addLine();

        telemetry.addLine("=== ROBOT ===");
        telemetry.addData("Initial Heading", "%.1f°", initialRobotHeading);
        telemetry.addData("Current Heading", "%.1f°", currentRobotHeading);
        telemetry.addData("Robot Rotated", "%.1f°", robotRotation);
        telemetry.addLine();

        telemetry.addLine("=== TURRET ===");
        telemetry.addData("Current Turret Angle", "%.1f°", currentTurretAngle);
        telemetry.addData("Ideal Compensation", "%.1f°", idealTurretCompensation);
        telemetry.addData("Clamped Compensation", "%.1f°", clampedTurretCompensation);
        telemetry.addData("Is Clamped?", (idealTurretCompensation != clampedTurretCompensation) ? "⚠️ YES" : "No");
        telemetry.addLine();

        telemetry.addLine("=== RESULT ===");
        telemetry.addData("Actual Global Direction", "%.1f°", actualGlobalDirection);
        telemetry.addData("Expected Global", "%.1f°", normalizeAngle(initialRobotHeading + initialTurretAngle));
        telemetry.addData("Error", "%.1f°", normalizeAngleSigned(
                (initialRobotHeading + initialTurretAngle) - actualGlobalDirection));
        telemetry.addLine();

        telemetry.addLine("=== CONTROLS ===");
        telemetry.addData("A", "Mark start (then rotate robot)");
        telemetry.addData("X", "Apply compensation now");
        telemetry.addData("B", "Reset turret to 0°");
        telemetry.addLine();

        telemetry.addLine("=== TUNING GUIDE ===");
        telemetry.addData("If turret lags behind", "INCREASE ratio");
        telemetry.addData("If turret overshoots", "DECREASE ratio");
        telemetry.addData("Goal", "Error should be ~0°");

        telemetry.update();
    }

    private double normalizeAngle(double degrees) {
        degrees = degrees % 360.0;
        if (degrees < 0) {
            degrees += 360.0;
        }
        return degrees;
    }

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
        Turret.INSTANCE.setTurretAngleDeg(TurretConstants.DEFAULT_TURRET_DEG);
    }
}

