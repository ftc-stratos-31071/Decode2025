package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import org.firstinspires.ftc.teamcode.subsystems.Turret2;

/**
 * Turret2 Tuning OpMode
 *
 * CONTROLS:
 * - Y Button: Go to 0° (center/forward)
 * - X Button: Go to -45° (LEFT)
 * - B Button: Go to +45° (RIGHT)
 * - A Button: Go to CONFIGURABLE_ANGLE (default 120°)
 * - Left Bumper: Decrease angle by STEP_SIZE
 * - Right Bumper: Increase angle by STEP_SIZE
 */
@Config
@TeleOp(name = "Turret2 Tuner", group = "Tuning")
public class Turret2Tuner extends NextFTCOpMode {

    // Dashboard tunable values
    public static double STEP_SIZE = 5.0;

    /** Configurable test angle for A button - change this on dashboard */
    public static double CONFIGURABLE_ANGLE = 120.0;

    private String lastAction = "Initialized";
    private String limitWarning = "";

    // Servo selection mode: 0 = Both, 1 = Left Only, 2 = Right Only
    private int servoMode = 0;
    private String[] servoModeNames = {"BOTH", "LEFT ONLY", "RIGHT ONLY"};

    public Turret2Tuner() {
        addComponents(
                new SubsystemComponent(Turret2.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        Turret2.INSTANCE.setAngle(0.0);
        updateTelemetry();
    }

    @Override
    public void onStartButtonPressed() {
        // Y BUTTON: Go to center (0°)
        Gamepads.gamepad1().y().whenBecomesTrue(() -> {
            Turret2.INSTANCE.setAngle(0.0);
            lastAction = "Y → Center 0°";
            limitWarning = "";
        });

        // X BUTTON: Go to -45° (LEFT)
        Gamepads.gamepad1().x().whenBecomesTrue(() -> {
            Turret2.INSTANCE.setAngle(-45.0);
            lastAction = "X → Left -45°";
            limitWarning = "";
        });

        // B BUTTON: Go to +45° (RIGHT)
        Gamepads.gamepad1().b().whenBecomesTrue(() -> {
            Turret2.INSTANCE.setAngle(45.0);
            lastAction = "B → Right +45°";
            limitWarning = "";
        });

        // A BUTTON: Go to configurable angle
        Gamepads.gamepad1().a().whenBecomesTrue(() -> {
            Turret2.INSTANCE.setAngle(CONFIGURABLE_ANGLE);
            lastAction = "A → Configurable " + CONFIGURABLE_ANGLE + "°";
            limitWarning = "";
        });

        // DPAD LEFT: Set servo position to 0.0
        Gamepads.gamepad1().dpadLeft().whenBecomesTrue(() -> {
            if (servoMode == 0 || servoMode == 1) {
                Turret2.INSTANCE.setServoPositionLeft(0.0);
                lastAction = "DPAD LEFT → Left Servo Position 0.0";
            }
            if (servoMode == 0 || servoMode == 2) {
                Turret2.INSTANCE.setServoPositionRight(0.0);
                lastAction = "DPAD LEFT → Right Servo Position 0.0";
            }
            limitWarning = "";
        });

        // DPAD UP: Set servo position to 0.5
        Gamepads.gamepad1().dpadUp().whenBecomesTrue(() -> {
            if (servoMode == 0 || servoMode == 1) {
                Turret2.INSTANCE.setServoPositionLeft(0.5);
                lastAction = "DPAD UP → Left Servo Position 0.5";
            }
            if (servoMode == 0 || servoMode == 2) {
                Turret2.INSTANCE.setServoPositionRight(0.5);
                lastAction = "DPAD UP → Right Servo Position 0.5";
            }
            limitWarning = "";
        });

        // DPAD RIGHT: Set servo position to 1.0
        Gamepads.gamepad1().dpadRight().whenBecomesTrue(() -> {
            if (servoMode == 0 || servoMode == 1) {
                Turret2.INSTANCE.setServoPositionLeft(1.0);
                lastAction = "DPAD RIGHT → Left Servo Position 1.0";
            }
            if (servoMode == 0 || servoMode == 2) {
                Turret2.INSTANCE.setServoPositionRight(1.0);
                lastAction = "DPAD RIGHT → Right Servo Position 1.0";
            }
            limitWarning = "";
        });

        // DPAD DOWN: Toggle servo selection mode
        Gamepads.gamepad1().dpadDown().whenBecomesTrue(() -> {
            servoMode = (servoMode + 1) % 3;
            lastAction = "DPAD DOWN → Servo Mode: " + servoModeNames[servoMode];
            limitWarning = "";
        });

        // LEFT BUMPER: Decrease angle (turn more LEFT / negative)
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(() -> {
            // Read current turret position
            double currentAngle = Turret2.INSTANCE.getTargetLogicalDeg();

            // Check if already at minimum limit
            if (currentAngle <= -Turret2.MAX_ROTATION + 0.1) {
                lastAction = "LB → Already at LEFT limit!";
                limitWarning = "⚠️ AT MINIMUM LIMIT (-" + Turret2.MAX_ROTATION + "°)";
                return;
            }

            // Calculate new angle
            double newAngle = currentAngle - STEP_SIZE;

            // Clamp to turret limits
            if (newAngle < -Turret2.MAX_ROTATION) {
                newAngle = -Turret2.MAX_ROTATION;
                limitWarning = "⚠️ CLAMPED TO MINIMUM LIMIT";
            } else {
                limitWarning = "";
            }

            Turret2.INSTANCE.setAngle(newAngle);
            lastAction = "LB → " + String.format("%.1f", newAngle) + "°";
        });

        // RIGHT BUMPER: Increase angle (turn more RIGHT / positive)
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(() -> {
            // Read current turret position
            double currentAngle = Turret2.INSTANCE.getTargetLogicalDeg();

            // Check if already at maximum limit
            if (currentAngle >= Turret2.MAX_ROTATION - 0.1) {
                lastAction = "RB → Already at RIGHT limit!";
                limitWarning = "⚠️ AT MAXIMUM LIMIT (+" + Turret2.MAX_ROTATION + "°)";
                return;
            }

            // Calculate new angle
            double newAngle = currentAngle + STEP_SIZE;

            // Clamp to turret limits
            if (newAngle > Turret2.MAX_ROTATION) {
                newAngle = Turret2.MAX_ROTATION;
                limitWarning = "⚠️ CLAMPED TO MAXIMUM LIMIT";
            } else {
                limitWarning = "";
            }

            Turret2.INSTANCE.setAngle(newAngle);
            lastAction = "RB → " + String.format("%.1f", newAngle) + "°";
        });
    }

    @Override
    public void onUpdate() {
        updateTelemetry();
    }

    private void updateTelemetry() {
        // Read actual turret values
        double currentLogical = Turret2.INSTANCE.getTargetLogicalDeg();
        double currentRaw = Turret2.INSTANCE.getCurrentRawDeg();
        double servoPosLeft = Turret2.INSTANCE.getServoPositionLeft();
        double servoPosRight = Turret2.INSTANCE.getServoPositionRight();

        // Check if at limits
        boolean atMinLimit = currentLogical <= -Turret2.MAX_ROTATION + 0.1;
        boolean atMaxLimit = currentLogical >= Turret2.MAX_ROTATION - 0.1;

        telemetry.addLine("══════════════════════════════════");
        telemetry.addLine("        TURRET2 TUNER");
        telemetry.addLine("══════════════════════════════════");
        telemetry.addLine();

        telemetry.addLine("─── CURRENT STATE ───");
        telemetry.addData("Servo Mode", servoModeNames[servoMode]);
        telemetry.addData("Logical Angle", "%.1f°", currentLogical);
        telemetry.addData("Raw Angle", "%.1f°", currentRaw);
        telemetry.addData("Left Servo Position", "%.3f", servoPosLeft);
        telemetry.addData("Right Servo Position", "%.3f", servoPosRight);
        telemetry.addLine();

        // Show limit status
        if (atMinLimit) {
            telemetry.addLine("⚠️ AT LEFT LIMIT (LB disabled)");
        } else if (atMaxLimit) {
            telemetry.addLine("⚠️ AT RIGHT LIMIT (RB disabled)");
        } else {
            telemetry.addLine("✓ Within range");
        }
        telemetry.addLine();

        telemetry.addLine("─── CONFIGURATION ───");
        telemetry.addData("PHYSICAL_CENTER_RAW", "%.1f°", Turret2.PHYSICAL_CENTER_RAW);
        telemetry.addData("MAX_ROTATION", "±%.1f°", Turret2.MAX_ROTATION);
        telemetry.addData("Step Size", "%.1f°", STEP_SIZE);
        telemetry.addData("Configurable Angle (A)", "%.1f°", CONFIGURABLE_ANGLE);
        telemetry.addLine();

        telemetry.addData("Last Action", lastAction);
        if (!limitWarning.isEmpty()) {
            telemetry.addLine(limitWarning);
        }
        telemetry.addLine();

        telemetry.addLine("─── CONTROLS ───");
        telemetry.addLine("Y = Center (0°)");
        telemetry.addLine("X = Left (-45°)");
        telemetry.addLine("B = Right (+45°)");
        telemetry.addLine("A = Configurable (" + CONFIGURABLE_ANGLE + "°)");
        telemetry.addLine("LB/RB = Adjust angle ±" + STEP_SIZE + "°");
        telemetry.addLine("DPAD LEFT/UP/RIGHT = Set Servo Position (0.0/0.5/1.0)");
        telemetry.addLine("DPAD DOWN = Toggle Servo Mode (" + servoModeNames[servoMode] + ")");
        telemetry.addLine();

        telemetry.addLine("─── TURRET CONVENTION ───");
        telemetry.addLine("0° = Forward");
        telemetry.addLine("POSITIVE = RIGHT");
        telemetry.addLine("NEGATIVE = LEFT");

        telemetry.update();
    }

    @Override
    public void onStop() {
        Turret2.INSTANCE.setAngle(0.0);
    }
}
