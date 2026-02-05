package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import org.firstinspires.ftc.teamcode.constants.TurretConstants;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * Manual turret tuning OpMode.
 *
 * This OpMode lets you manually control the turret to find the correct angles
 * and understand how the turret moves.
 *
 * Controls:
 * - Left Bumper: Move turret left (increment angle)
 * - Right Bumper: Move turret right (decrement angle)
 * - X Button: Set turret to X_BUTTON_ANGLE (configurable via dashboard)
 * - Y Button: Set turret to Y_BUTTON_ANGLE (configurable via dashboard)
 * - B Button: Set turret to B_BUTTON_ANGLE (configurable via dashboard)
 * - A Button: Set turret to A_BUTTON_ANGLE (configurable via dashboard)
 * - DPad Up: Reset turret to center (0°)
 * - DPad Down: Go to current MANUAL_ANGLE setting
 */
@Config
@TeleOp(name = "TurretRobotTuner", group = "Tuning")
public class TurretRobotTuner extends NextFTCOpMode {

    // TUNABLE: Bumper increment step (degrees per press)
    public static double BUMPER_STEP = 5.0;

    // TUNABLE: Manual angle control (use DPad Down to test this angle)
    public static double MANUAL_ANGLE = 0.0;

    // TUNABLE: Button preset angles (adjust these via dashboard)
    public static double X_BUTTON_ANGLE = 90.0;   // Default: left
    public static double Y_BUTTON_ANGLE = 0.0;    // Default: center
    public static double B_BUTTON_ANGLE = -90.0;  // Default: right
    public static double A_BUTTON_ANGLE = 45.0;   // Default: 45° right

    private double currentAngle = 0.0;

    public TurretRobotTuner() {
        addComponents(
                new SubsystemComponent(Turret.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        // Initialize turret to center position
        currentAngle = 0.0;
        Turret.INSTANCE.setTurretAngleDeg(currentAngle);

        telemetry.addData("Status", "✓ Initialized");
        telemetry.addLine();
        telemetry.addData("🔄 Turret Angle", "%.1f°", currentAngle);
        telemetry.addData("🔄 Physical Angle", "%.1f°", currentAngle + TurretConstants.PHYSICAL_CENTER);
        telemetry.addLine();
        telemetry.addLine("🎮 CONTROLS:");
        telemetry.addData("Left Bumper", "Move Left (+ angle)");
        telemetry.addData("Right Bumper", "Move Right (- angle)");
        telemetry.addData("X Button", "Go to %.1f°", X_BUTTON_ANGLE);
        telemetry.addData("Y Button", "Go to %.1f°", Y_BUTTON_ANGLE);
        telemetry.addData("B Button", "Go to %.1f°", B_BUTTON_ANGLE);
        telemetry.addData("A Button", "Go to %.1f°", A_BUTTON_ANGLE);
        telemetry.addData("DPad Up", "Reset to 0° (center)");
        telemetry.addData("DPad Down", "Go to MANUAL_ANGLE");
        telemetry.addLine();
        telemetry.addLine("📊 TUNING INFO:");
        telemetry.addData("Bumper Step", "%.1f°", BUMPER_STEP);
        telemetry.addData("Physical Center Offset", "%.1f°", TurretConstants.PHYSICAL_CENTER);
        telemetry.addData("Range", "%.0f° to %.0f°", TurretConstants.MIN_TURRET_DEG, TurretConstants.MAX_TURRET_DEG);
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        // Left Bumper: Increment angle (move turret left)
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(() -> {
            currentAngle += BUMPER_STEP;
            // No clamping - allow full range testing
            Turret.INSTANCE.setTurretAngleDeg(currentAngle);
        });

        // Right Bumper: Decrement angle (move turret right)
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(() -> {
            currentAngle -= BUMPER_STEP;
            // No clamping - allow full range testing
            Turret.INSTANCE.setTurretAngleDeg(currentAngle);
        });

        // X Button: Set to X_BUTTON_ANGLE
        Gamepads.gamepad1().x().whenBecomesTrue(() -> {
            currentAngle = X_BUTTON_ANGLE;
            Turret.INSTANCE.setTurretAngleDeg(currentAngle);
        });

        // Y Button: Set to Y_BUTTON_ANGLE
        Gamepads.gamepad1().y().whenBecomesTrue(() -> {
            currentAngle = Y_BUTTON_ANGLE;
            Turret.INSTANCE.setTurretAngleDeg(currentAngle);
        });

        // B Button: Set to B_BUTTON_ANGLE
        Gamepads.gamepad1().b().whenBecomesTrue(() -> {
            currentAngle = B_BUTTON_ANGLE;
            Turret.INSTANCE.setTurretAngleDeg(currentAngle);
        });

        // A Button: Set to A_BUTTON_ANGLE
        Gamepads.gamepad1().a().whenBecomesTrue(() -> {
            currentAngle = A_BUTTON_ANGLE;
            Turret.INSTANCE.setTurretAngleDeg(currentAngle);
        });

        // DPad Up: Reset to center
        Gamepads.gamepad1().dpadUp().whenBecomesTrue(() -> {
            currentAngle = 0.0;
            Turret.INSTANCE.setTurretAngleDeg(currentAngle);
        });

        // DPad Down: Go to MANUAL_ANGLE
        Gamepads.gamepad1().dpadDown().whenBecomesTrue(() -> {
            currentAngle = MANUAL_ANGLE;
            Turret.INSTANCE.setTurretAngleDeg(currentAngle);
        });
    }

    @Override
    public void onUpdate() {
        // Display current turret state
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("      TURRET ROBOT TUNER");
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine();

        // Current position
        telemetry.addLine("📍 CURRENT POSITION:");
        telemetry.addData("  Logical Angle", "%.1f°", currentAngle);
        telemetry.addData("  Physical Angle", "%.1f°", Turret.INSTANCE.getTargetTurretDeg());
        telemetry.addData("  Servo Target", "%.1f°", Turret.INSTANCE.getTargetTurretDeg());
        telemetry.addData("  Servo Current", "%.1f°", Turret.INSTANCE.getCurrentTurretDeg());
        telemetry.addLine();

        // Range info
        telemetry.addLine("📊 RANGE INFO:");
        telemetry.addData("  Min Logical", "%.0f°", TurretConstants.MIN_TURRET_DEG);
        telemetry.addData("  Max Logical", "%.0f°", TurretConstants.MAX_TURRET_DEG);
        telemetry.addData("  Physical Offset", "%.1f°", TurretConstants.PHYSICAL_CENTER);
        telemetry.addLine();

        // Button presets
        telemetry.addLine("🎮 BUTTON PRESETS:");
        telemetry.addData("  X Button", "%.1f° → Physical: %.1f°",
            X_BUTTON_ANGLE, X_BUTTON_ANGLE + TurretConstants.PHYSICAL_CENTER);
        telemetry.addData("  Y Button", "%.1f° → Physical: %.1f°",
            Y_BUTTON_ANGLE, Y_BUTTON_ANGLE + TurretConstants.PHYSICAL_CENTER);
        telemetry.addData("  B Button", "%.1f° → Physical: %.1f°",
            B_BUTTON_ANGLE, B_BUTTON_ANGLE + TurretConstants.PHYSICAL_CENTER);
        telemetry.addData("  A Button", "%.1f° → Physical: %.1f°",
            A_BUTTON_ANGLE, A_BUTTON_ANGLE + TurretConstants.PHYSICAL_CENTER);
        telemetry.addLine();

        // Manual control
        telemetry.addLine("🕹️ MANUAL CONTROL:");
        telemetry.addData("  Bumper Step", "%.1f°", BUMPER_STEP);
        telemetry.addData("  Manual Test Angle", "%.1f°", MANUAL_ANGLE);
        telemetry.addLine();

        // Controls reminder
        telemetry.addLine("═══════════════════════════════");
        telemetry.addLine("CONTROLS:");
        telemetry.addData("  Left Bumper", "Increase angle (left)");
        telemetry.addData("  Right Bumper", "Decrease angle (right)");
        telemetry.addData("  X, Y, B, A", "Go to preset angles");
        telemetry.addData("  DPad Up", "Reset to 0° center");
        telemetry.addData("  DPad Down", "Test MANUAL_ANGLE");
        telemetry.addLine();

        // Tips
        telemetry.addLine("💡 TUNING TIPS:");
        telemetry.addLine("1. Use bumpers to find center");
        telemetry.addLine("2. Note the logical angle when centered");
        telemetry.addLine("3. Update button presets on dashboard");
        telemetry.addLine("4. Physical angle = Logical + Offset");

        telemetry.update();
    }

    @Override
    public void onStop() {
        // Return turret to safe center position
        Turret.INSTANCE.setTurretAngleDeg(0.0);
    }
}
