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
 * TUNING PROCESS:
 * 1. Use LEFT/RIGHT BUMPERS to move turret in raw degrees
 * 2. Find the position where turret faces STRAIGHT FORWARD
 * 3. Note the "Raw Angle" shown in telemetry
 * 4. Set PHYSICAL_CENTER_RAW to that value
 * 5. Test with Y button (should go to center)
 *
 * CONTROLS:
 * - Left Bumper: Increase raw angle
 * - Right Bumper: Decrease raw angle
 * - Y Button: Go to 0° (center)
 * - X Button: Go to +45° (left)
 * - B Button: Go to -45° (right)
 * - A Button: Go to A_ANGLE
 * - DPad Up: Go to PHYSICAL_CENTER_RAW
 * - DPad Down: Go to TEST_RAW_ANGLE
 * - DPad Left/Right: Adjust step size
 */
@Config
@TeleOp(name = "Turret2 Tuner", group = "Tuning")
public class Turret2Tuner extends NextFTCOpMode {

    // Dashboard tunable values
    public static double STEP_SIZE = 5.0;
    public static double TEST_RAW_ANGLE = 177.5;

    /** Test angle in LOGICAL degrees (relative to center) - change this on dashboard */
    public static double CONFIGURABLE_TEST_ANGLE = 45.0;

    // Button preset angles (logical degrees)
    public static double X_ANGLE = 45.0;
    public static double B_ANGLE = -45.0;

    private double currentRawAngle = 177.5;
    private boolean usingRawMode = true;
    private String lastAction = "Initialized";

    public Turret2Tuner() {
        addComponents(
                new SubsystemComponent(Turret2.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        currentRawAngle = Turret2.PHYSICAL_CENTER_RAW;
        Turret2.INSTANCE.setRawAngleNoLimits(currentRawAngle);
        updateTelemetry();
    }

    @Override
    public void onStartButtonPressed() {
        // RAW CONTROL (for finding physical center)
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(() -> {
            currentRawAngle += STEP_SIZE;
            Turret2.INSTANCE.setRawAngleNoLimits(currentRawAngle);
            usingRawMode = true;
            lastAction = "Raw +" + STEP_SIZE + "° → " + currentRawAngle + "°";
        });

        Gamepads.gamepad1().rightBumper().whenBecomesTrue(() -> {
            currentRawAngle -= STEP_SIZE;
            Turret2.INSTANCE.setRawAngleNoLimits(currentRawAngle);
            usingRawMode = true;
            lastAction = "Raw -" + STEP_SIZE + "° → " + currentRawAngle + "°";
        });

        // Y BUTTON: Go to center (raw 225°)
        Gamepads.gamepad1().y().whenBecomesTrue(() -> {
            currentRawAngle = Turret2.PHYSICAL_CENTER_RAW;
            Turret2.INSTANCE.setRawAngleNoLimits(currentRawAngle);
            usingRawMode = true;
            lastAction = "Y → Raw Center " + currentRawAngle + "°";
        });

        // X BUTTON: Go to logical +45° (left)
        Gamepads.gamepad1().x().whenBecomesTrue(() -> {
            Turret2.INSTANCE.setAngle(X_ANGLE);
            currentRawAngle = Turret2.INSTANCE.getCurrentRawDeg();
            usingRawMode = false;
            lastAction = "X → Logical " + X_ANGLE + "° (Left)";
        });

        // B BUTTON: Go to logical -45° (right)
        Gamepads.gamepad1().b().whenBecomesTrue(() -> {
            Turret2.INSTANCE.setAngle(B_ANGLE);
            currentRawAngle = Turret2.INSTANCE.getCurrentRawDeg();
            usingRawMode = false;
            lastAction = "B → Logical " + B_ANGLE + "° (Right)";
        });

        // A BUTTON: Go to configurable test angle (LOGICAL)
        Gamepads.gamepad1().a().whenBecomesTrue(() -> {
            Turret2.INSTANCE.setAngle(CONFIGURABLE_TEST_ANGLE);
            currentRawAngle = Turret2.INSTANCE.getCurrentRawDeg();
            usingRawMode = false;
            lastAction = "A → Logical " + CONFIGURABLE_TEST_ANGLE + "°";
        });

        // DPAD CONTROLS
        Gamepads.gamepad1().dpadUp().whenBecomesTrue(() -> {
            currentRawAngle = Turret2.PHYSICAL_CENTER_RAW;
            Turret2.INSTANCE.setRawAngleNoLimits(currentRawAngle);
            usingRawMode = true;
            lastAction = "DPad Up → Raw Center " + currentRawAngle + "°";
        });

        Gamepads.gamepad1().dpadDown().whenBecomesTrue(() -> {
            currentRawAngle = TEST_RAW_ANGLE;
            Turret2.INSTANCE.setRawAngleNoLimits(currentRawAngle);
            usingRawMode = true;
            lastAction = "DPad Down → Test Raw " + currentRawAngle + "°";
        });

        Gamepads.gamepad1().dpadLeft().whenBecomesTrue(() -> {
            STEP_SIZE = Math.max(1.0, STEP_SIZE - 1.0);
            lastAction = "Step size → " + STEP_SIZE + "°";
        });

        Gamepads.gamepad1().dpadRight().whenBecomesTrue(() -> {
            STEP_SIZE = Math.min(30.0, STEP_SIZE + 1.0);
            lastAction = "Step size → " + STEP_SIZE + "°";
        });
    }

    @Override
    public void onUpdate() {
        updateTelemetry();
    }

    private void updateTelemetry() {
        telemetry.addLine("���═════════════════════════════════");
        telemetry.addLine("        TURRET2 TUNER");
        telemetry.addLine("══════════════════════════════════");
        telemetry.addLine();

        telemetry.addLine("─── CURRENT STATE ───");
        telemetry.addData("Mode", usingRawMode ? "RAW (tuning)" : "LOGICAL (testing)");
        telemetry.addData("Raw Angle", "%.1f°", Turret2.INSTANCE.getCurrentRawDeg());
        telemetry.addData("Logical Angle", "%.1f°", Turret2.INSTANCE.getTargetLogicalDeg());
        telemetry.addData("Servo Position", "%.3f", Turret2.INSTANCE.getServoPosition());
        telemetry.addLine();

        telemetry.addLine("─── CONFIGURATION ───");
        telemetry.addData("PHYSICAL_CENTER_RAW", "%.1f° (logical 0°)", Turret2.PHYSICAL_CENTER_RAW);
        telemetry.addData("MAX_ROTATION", "±%.1f°", Turret2.MAX_ROTATION);
        telemetry.addData("REVERSE_DIRECTION", Turret2.REVERSE_DIRECTION);
        telemetry.addData("Step Size", "%.1f°", STEP_SIZE);
        telemetry.addLine();

        // Show configurable test angle info
        telemetry.addLine("─── TEST ANGLE (A Button) ───");
        telemetry.addData("Configurable Logical", "%.1f°", CONFIGURABLE_TEST_ANGLE);
        telemetry.addData("  = Raw", "%.1f°", CONFIGURABLE_TEST_ANGLE + Turret2.PHYSICAL_CENTER_RAW);
        telemetry.addLine();

        telemetry.addData("Last Action", lastAction);
        telemetry.addLine();

        telemetry.addLine("─── CONTROLS ───");
        telemetry.addLine("LB/RB = Raw angle ±step");
        telemetry.addLine("Y=Center(225°) X=Left(+45°)");
        telemetry.addLine("B=Right(-45°) A=TestAngle");
        telemetry.addLine("DPad ↑=Center ↓=TestRaw ←/→=Step");
        telemetry.addLine();

        telemetry.addLine("─── REFERENCE ───");
        telemetry.addLine("Raw 225° = Logical 0° = CENTER");
        telemetry.addLine("Raw > 225 = Logical + = LEFT");
        telemetry.addLine("Raw < 225 = Logical - = RIGHT");

        telemetry.update();
    }

    @Override
    public void onStop() {
        Turret2.INSTANCE.setAngle(0.0);
    }
}
