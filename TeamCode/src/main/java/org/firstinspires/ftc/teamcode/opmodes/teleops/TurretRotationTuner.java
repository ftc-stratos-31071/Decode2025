package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
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
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.Turret2;

/**
 * OpMode to test turret counter-rotation using Turret2 with odometry-based tracking.
 *
 * TRACKING MODE:
 * - Turret continuously calculates and tracks the angle to a target position
 * - Uses atan2(goalY - currentY, goalX - currentX) to find bearing to goal
 * - As robot moves/rotates, turret adjusts to keep pointing at the goal
 *
 * Controls:
 * - Left Stick: Drive forward/backward and strafe
 * - Right Stick X: Rotate robot (turret compensates if tracking enabled)
 * - X Button: Manual mode - Turret to +45° (left)
 * - Y Button: Manual mode - Turret to 0° (center)
 * - B Button: Manual mode - Turret to -45° (right)
 * - A Button: Resume automatic tracking to goal position
 */
@Config
@TeleOp(name = "TurretRotationTuner")
public class TurretRotationTuner extends NextFTCOpMode {

    private GoBildaPinpointDriver pinpoint;

    // Motor declarations for mecanum drive
    private final MotorEx frontLeftMotor = new MotorEx("frontLeftMotor").brakeMode().reversed();
    private final MotorEx frontRightMotor = new MotorEx("frontRightMotor").brakeMode().reversed();
    private final MotorEx backLeftMotor = new MotorEx("backLeftMotor").brakeMode();
    private final MotorEx backRightMotor = new MotorEx("backRightMotor").brakeMode();

    // TUNABLE: Goal position to track (inches) - CHANGE THESE ON DASHBOARD
    // FTC Coordinate System: (0, 0) = Center of field
    // X: -72 (left) to +72 (right)
    // Y: -72 (bottom) to +72 (top)
    // Red goal position at top-right corner
    public static double GOAL_X = -72.0;
    public static double GOAL_Y = 72.0;

    // TUNABLE: Starting robot position (inches) - CENTER OF FIELD
    public static double START_X = 0.0;
    public static double START_Y = 0.0;
    public static double START_HEADING = 90.0;  // 0=East, 90=North, 180=West, 270=South

    // TUNABLE: Enable/disable automatic tracking
    public static boolean ENABLE_TRACKING = true;

    // TUNABLE: Manual button positions (logical degrees from center)
    public static double X_BUTTON_ANGLE = 45.0;   // Left
    public static double Y_BUTTON_ANGLE = 0.0;    // Center
    public static double B_BUTTON_ANGLE = -45.0;  // Right

    private double targetGlobalHeading = 0.0;
    private boolean trackingEnabled = true;

    public TurretRotationTuner() {
        addComponents(
                new SubsystemComponent(Turret2.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();

        // Set initial robot position and heading (configurable via dashboard)
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, START_X, START_Y, AngleUnit.DEGREES, START_HEADING));

        // Initialize turret to center position (logical 0° = facing forward)
        Turret2.INSTANCE.setAngle(0.0);

        telemetry.addData("Status", "✓ Initialized");
        telemetry.addLine();
        telemetry.addData("🔄 Turret Angle", "%.1f° (logical)", Turret2.INSTANCE.getTargetLogicalDeg());
        telemetry.addLine();
        telemetry.addLine("🎯 TRACKING SETUP:");
        telemetry.addData("  Goal Position", "(%.1f, %.1f)", GOAL_X, GOAL_Y);
        telemetry.addData("  Robot Start", "(%.1f, %.1f)", START_X, START_Y);
        telemetry.addData("  Robot Start Heading", "%.1f°", START_HEADING);
        telemetry.addData("  Method", "atan2(goalY-y, goalX-x)");
        telemetry.addLine();
        telemetry.addLine("💡 QUICK PRESETS:");
        telemetry.addData("  Red Goal (top-right)", "(72, 72)");
        telemetry.addData("  Blue Goal (bottom-left)", "(-72, -72)");
        telemetry.addData("  Field Center", "(0, 0)");
        telemetry.addLine();
        telemetry.addLine("🎮 CONTROLS:");
        telemetry.addData("Left Stick", "Drive + Strafe");
        telemetry.addData("Right Stick X", "Rotate");
        telemetry.addData("X Button", "Manual: +45° (left)");
        telemetry.addData("Y Button", "Manual: 0° (center)");
        telemetry.addData("B Button", "Manual: -45° (right)");
        telemetry.addData("A Button", "Resume auto tracking");
        telemetry.addLine();
        telemetry.addLine("📝 TO CHANGE SETTINGS:");
        telemetry.addLine("Open FTC Dashboard and adjust:");
        telemetry.addLine("- GOAL_X, GOAL_Y (target position)");
        telemetry.addLine("- START_X, START_Y, START_HEADING");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        // Set up mecanum drive control
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

        // X Button: Manual mode - turret to X_BUTTON_ANGLE (default +45° left)
        Gamepads.gamepad1().x().whenBecomesTrue(() -> {
            trackingEnabled = false;
            Turret2.INSTANCE.setAngle(X_BUTTON_ANGLE);
        });

        // Y Button: Manual mode - turret to Y_BUTTON_ANGLE (default 0° center)
        Gamepads.gamepad1().y().whenBecomesTrue(() -> {
            trackingEnabled = false;
            Turret2.INSTANCE.setAngle(Y_BUTTON_ANGLE);
        });

        // B Button: Manual mode - turret to B_BUTTON_ANGLE (default -45° right)
        Gamepads.gamepad1().b().whenBecomesTrue(() -> {
            trackingEnabled = false;
            Turret2.INSTANCE.setAngle(B_BUTTON_ANGLE);
        });

        // A Button: Resume automatic tracking
        Gamepads.gamepad1().a().whenBecomesTrue(() -> {
            trackingEnabled = true;
        });
    }

    @Override
    public void onUpdate() {
        // Update odometry to get current position and heading
        pinpoint.update();
        Pose2D currentPose = pinpoint.getPosition();

        double currentX = currentPose.getX(DistanceUnit.INCH);
        double currentY = currentPose.getY(DistanceUnit.INCH);
        double currentRobotHeading = currentPose.getHeading(AngleUnit.DEGREES);

        // Create telemetry packet for FTC Dashboard field view
        TelemetryPacket packet = new TelemetryPacket();
        Canvas fieldOverlay = packet.fieldOverlay();

        // Draw goal position (red X marker)
        fieldOverlay.setStroke("#FF0000");  // Red
        fieldOverlay.setStrokeWidth(2);
        double goalSize = 4;
        fieldOverlay.strokeLine(GOAL_X - goalSize, GOAL_Y - goalSize, GOAL_X + goalSize, GOAL_Y + goalSize);
        fieldOverlay.strokeLine(GOAL_X - goalSize, GOAL_Y + goalSize, GOAL_X + goalSize, GOAL_Y - goalSize);
        fieldOverlay.strokeCircle(GOAL_X, GOAL_Y, 6);

        // Draw robot position (blue circle with heading indicator)
        fieldOverlay.setStroke("#0000FF");  // Blue
        fieldOverlay.setFill("#0000FF");
        fieldOverlay.fillCircle(currentX, currentY, 6);  // Robot body

        // Draw robot heading arrow
        double headingRadians = Math.toRadians(currentRobotHeading);
        double arrowLength = 12;
        double arrowEndX = currentX + arrowLength * Math.cos(headingRadians);
        double arrowEndY = currentY + arrowLength * Math.sin(headingRadians);
        fieldOverlay.strokeLine(currentX, currentY, arrowEndX, arrowEndY);

        // Draw line from robot to goal (green line)
        fieldOverlay.setStroke("#00FF00");  // Green
        fieldOverlay.setStrokeWidth(1);
        fieldOverlay.strokeLine(currentX, currentY, GOAL_X, GOAL_Y);

        // Draw turret direction (yellow line showing where turret is pointing)
        if (trackingEnabled) {
            // Flip the visualization: subtract instead of add to match physical turret direction
            double turretGlobalHeading = normalizeAngle(currentRobotHeading - Turret2.INSTANCE.getTargetLogicalDeg());
            double turretRadians = Math.toRadians(turretGlobalHeading);
            double turretLength = 18;
            double turretEndX = currentX + turretLength * Math.cos(turretRadians);
            double turretEndY = currentY + turretLength * Math.sin(turretRadians);
            fieldOverlay.setStroke("#FFFF00");  // Yellow
            fieldOverlay.setStrokeWidth(3);
            fieldOverlay.strokeLine(currentX, currentY, turretEndX, turretEndY);
        }

        // Send packet to dashboard
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        // Calculate required turret angle to maintain tracking to goal
        if (trackingEnabled && ENABLE_TRACKING) {
            // Calculate the angle from current position to goal using atan2
            // This gives us the global bearing we need to face
            targetGlobalHeading = calculateAngleToGoal(currentX, currentY, GOAL_X, GOAL_Y);

            // Calculate logical turret angle: global_target - robot_heading
            double logicalTurretAngle = calculateTurretAngle(currentRobotHeading, targetGlobalHeading);

            // Send to turret
            Turret2.INSTANCE.setAngle(logicalTurretAngle);

            // Calculate actual global heading achieved
            // The turret angle is negated in calculateTurretAngle, so we add it back
            double actualGlobalHeading = normalizeAngle(currentRobotHeading + Turret2.INSTANCE.getTargetLogicalDeg());
            double error = normalizeAngleSigned(targetGlobalHeading - actualGlobalHeading);

            // Calculate distance to goal
            double distanceToGoal = Math.sqrt(
                Math.pow(GOAL_X - currentX, 2) +
                Math.pow(GOAL_Y - currentY, 2)
            );

            // Telemetry
            telemetry.addLine("═══════════════════════════════════");
            telemetry.addLine("    AUTO TRACKING ENABLED");
            telemetry.addLine("═══════════════════════════════════");
            telemetry.addLine();

            telemetry.addLine("🎯 TARGET");
            telemetry.addData("  Goal Position", "(%.1f, %.1f)", GOAL_X, GOAL_Y);
            telemetry.addData("  Distance to Goal", "%.1f in", distanceToGoal);
            telemetry.addData("  Bearing to Goal", "%.1f° (%s)", targetGlobalHeading, getDirectionName(targetGlobalHeading));
            telemetry.addLine();

            telemetry.addLine("🤖 ROBOT");
            telemetry.addData("  Position", "(%.1f, %.1f)", currentX, currentY);
            telemetry.addData("  Heading", "%.1f° (%s)", currentRobotHeading, getDirectionName(currentRobotHeading));
            telemetry.addLine();

            telemetry.addLine("🔄 TURRET");
            telemetry.addData("  Logical Angle", "%.1f°", Turret2.INSTANCE.getTargetLogicalDeg());
            telemetry.addData("  Raw Angle", "%.1f°", Turret2.INSTANCE.getCurrentRawDeg());
            telemetry.addData("  Servo Position", "%.3f", Turret2.INSTANCE.getServoPosition());
            telemetry.addData("  Global Result", "%.1f° (%s)", actualGlobalHeading, getDirectionName(actualGlobalHeading));
            telemetry.addData("  Constraints", "±%.0f°", Turret2.MAX_ROTATION);
            telemetry.addLine();

            // DEBUG: Show calculation details
            telemetry.addLine("🔧 DEBUG INFO:");
            telemetry.addData("  Target Heading Calc", "atan2(%.1f, %.1f) = %.1f°",
                GOAL_Y - currentY, GOAL_X - currentX, targetGlobalHeading);
            telemetry.addData("  Turret Offset Needed", "%.1f° - %.1f° = %.1f°",
                targetGlobalHeading, currentRobotHeading, logicalTurretAngle);
            telemetry.addLine();

            telemetry.addLine("📊 ACCURACY");
            telemetry.addData("  Error", "%.1f°", error);
            telemetry.addData("  Status", Math.abs(error) < 5.0 ? "✓ ON TARGET" : "⚠ ADJUSTING");
            telemetry.addLine();

            // Check if turret is at limits
            boolean atMinLimit = Math.abs(Turret2.INSTANCE.getTargetLogicalDeg() + Turret2.MAX_ROTATION) < 1.0;
            boolean atMaxLimit = Math.abs(Turret2.INSTANCE.getTargetLogicalDeg() - Turret2.MAX_ROTATION) < 1.0;

            if (atMinLimit || atMaxLimit) {
                telemetry.addLine("⚠️ WARNING: TURRET AT LIMIT");
                telemetry.addData("  Limit", atMinLimit ? "MIN (-" + Turret2.MAX_ROTATION + "°)" : "MAX (+" + Turret2.MAX_ROTATION + "°)");
                telemetry.addLine();
            }

        } else {
            telemetry.addLine("═══════════════════════════════��═══");
            telemetry.addLine("     MANUAL CONTROL MODE");
            telemetry.addLine("═══════════════════════════════════");
            telemetry.addLine();
            telemetry.addData("Tracking", "❌ DISABLED");
            telemetry.addLine();
            telemetry.addData("Robot Position", "(%.1f, %.1f)", currentX, currentY);
            telemetry.addData("Robot Heading", "%.1f° (%s)", currentRobotHeading, getDirectionName(currentRobotHeading));
            telemetry.addData("Turret Logical", "%.1f°", Turret2.INSTANCE.getTargetLogicalDeg());
            telemetry.addData("Turret Raw", "%.1f°", Turret2.INSTANCE.getCurrentRawDeg());
            telemetry.addLine();
            telemetry.addData("Press A", "to resume auto tracking");
            telemetry.addLine();
        }

        telemetry.addLine("─── CONTROLS ───");
        telemetry.addData("  X", "Manual → %.1f° (left)", X_BUTTON_ANGLE);
        telemetry.addData("  Y", "Manual → %.1f° (center)", Y_BUTTON_ANGLE);
        telemetry.addData("  B", "Manual → %.1f° (right)", B_BUTTON_ANGLE);
        telemetry.addData("  A", "Resume Tracking");
        telemetry.update();
    }

    /**
     * Calculate the angle from current position to goal position using atan2.
     * This is the global bearing/heading needed to face the goal.
     * Returns angle in degrees (0-360, where 0° = East, 90° = North, 180° = West, 270° = South)
     */
    private double calculateAngleToGoal(double currentX, double currentY, double goalX, double goalY) {
        double deltaX = goalX - currentX;
        double deltaY = goalY - currentY;

        // atan2 returns radians in range [-PI, PI]
        // Convert to degrees and normalize to [0, 360]
        double angleRad = Math.atan2(deltaY, deltaX);
        double angleDeg = Math.toDegrees(angleRad);

        return normalizeAngle(angleDeg);
    }

    /**
     * Calculate the turret angle needed to point at the target global heading.
     * Returns the LOGICAL turret angle (relative to center at 0°).
     */
    private double calculateTurretAngle(double robotHeading, double globalTarget) {
        // Calculate the turret angle: global_target - robot_heading
        // INVERTED: Negate the result because turret rotates opposite to expected
        double logicalTurretAngle = -(globalTarget - robotHeading);

        // Normalize to [-180, 180] range for shortest path
        logicalTurretAngle = normalizeAngleSigned(logicalTurretAngle);

        // Clamp to turret's physical limits (handled by Turret2.setAngle internally)
        // But we still check here to handle wrapping cases

        double minLimit = -Turret2.MAX_ROTATION;
        double maxLimit = Turret2.MAX_ROTATION;

        if (logicalTurretAngle >= minLimit && logicalTurretAngle <= maxLimit) {
            return logicalTurretAngle;
        }

        // Try wrapping by ±360°
        double wrappedPositive = logicalTurretAngle - 360;
        double wrappedNegative = logicalTurretAngle + 360;

        boolean positiveValid = (wrappedPositive >= minLimit && wrappedPositive <= maxLimit);
        boolean negativeValid = (wrappedNegative >= minLimit && wrappedNegative <= maxLimit);

        // If one wrapped direction is valid, use it
        if (positiveValid && !negativeValid) return wrappedPositive;
        if (negativeValid && !positiveValid) return wrappedNegative;

        // Both valid - choose closer to center (0°)
        if (positiveValid && negativeValid) {
            return (Math.abs(wrappedPositive) < Math.abs(wrappedNegative)) ? wrappedPositive : wrappedNegative;
        }

        // None valid - clamp to nearest constraint
        double distToMin = Math.abs(logicalTurretAngle - minLimit);
        double distToMax = Math.abs(logicalTurretAngle - maxLimit);

        return (distToMin < distToMax) ? minLimit : maxLimit;
    }

    private double normalizeAngle(double degrees) {
        degrees = degrees % 360.0;
        if (degrees < 0) degrees += 360.0;
        return degrees;
    }

    private double normalizeAngleSigned(double degrees) {
        degrees = degrees % 360.0;
        if (degrees > 180.0) degrees -= 360.0;
        else if (degrees < -180.0) degrees += 360.0;
        return degrees;
    }

    /**
     * Get a human-readable direction name for a heading
     */
    private String getDirectionName(double heading) {
        heading = normalizeAngle(heading);
        if (heading >= 337.5 || heading < 22.5) return "E →";
        if (heading >= 22.5 && heading < 67.5) return "NE ↗";
        if (heading >= 67.5 && heading < 112.5) return "N ↑";
        if (heading >= 112.5 && heading < 157.5) return "NW ↖";
        if (heading >= 157.5 && heading < 202.5) return "W ←";
        if (heading >= 202.5 && heading < 247.5) return "SW ↙";
        if (heading >= 247.5 && heading < 292.5) return "S ↓";
        return "SE ↘";
    }
}

