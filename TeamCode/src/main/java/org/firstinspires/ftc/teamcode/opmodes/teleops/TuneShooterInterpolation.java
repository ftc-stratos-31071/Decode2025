package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.ShooterInterpolation;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

/**
 * TuneShooterInterpolation - Test and calibrate distance-based shooting
 *
 * This OpMode helps you:
 * 1. Test interpolated RPM/hood values at different distances
 * 2. Find what RPM/hood combinations work at each distance
 * 3. Update calibration data in ShooterInterpolation.java
 *
 * CONTROLS:
 * - D-Pad Up/Down: Adjust test distance (+/- 5 inches)
 * - D-Pad Right/Left: Fine adjust distance (+/- 1 inch)
 * - Right Bumper: Toggle shooter on/off
 * - A: Use interpolated values for current distance
 * - B: Manual mode (adjust with bumpers)
 * - X: Reinitialize interpolation (after updating constants)
 * - Y: Print current settings (for adding to calibration table)
 */
@Config
@TeleOp(name = "TuneShooterInterpolation", group = "Tuning")
public class TuneShooterInterpolation extends NextFTCOpMode {

    // Test distance (tunable via FTC Dashboard)
    public static double TEST_DISTANCE = 60.0;  // inches

    // Manual override mode
    public static double MANUAL_RPM = 3500.0;
    public static double MANUAL_HOOD = 0.45;

    private boolean shooterOn = false;
    private boolean useInterpolation = true;

    public TuneShooterInterpolation() {
        addComponents(
                new SubsystemComponent(Shooter.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        // Setup telemetry to show on both Driver Station and FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Initialize shooter
        Shooter.INSTANCE.stop();

        telemetry.addData("Status", "Initialized - Ready to tune");
        telemetry.addData("", "");
        telemetry.addData("Calibration Status", ShooterInterpolation.getCalibrationStatus());
        telemetry.addData("", "");
        telemetry.addData("Controls", "See OpMode description");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        // Distance adjustment
        Gamepads.gamepad1().dpadUp().whenBecomesTrue(() -> {
            TEST_DISTANCE += 5.0;
        });

        Gamepads.gamepad1().dpadDown().whenBecomesTrue(() -> {
            TEST_DISTANCE -= 5.0;
        });

        Gamepads.gamepad1().dpadRight().whenBecomesTrue(() -> {
            TEST_DISTANCE += 1.0;
        });

        Gamepads.gamepad1().dpadLeft().whenBecomesTrue(() -> {
            TEST_DISTANCE -= 1.0;
        });

        // Shooter control
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(() -> {
            shooterOn = !shooterOn;
            if (!shooterOn) {
                Shooter.INSTANCE.stop();
            }
        });

        // Mode switching
        Gamepads.gamepad1().a().whenBecomesTrue(() -> {
            useInterpolation = true;
            telemetry.addData("Mode", "✓ Interpolation");
        });

        Gamepads.gamepad1().b().whenBecomesTrue(() -> {
            useInterpolation = false;
            telemetry.addData("Mode", "Manual Override");
        });

        // Reinitialize splines (after updating constants via Dashboard)
        Gamepads.gamepad1().x().whenBecomesTrue(() -> {
            ShooterInterpolation.forceReinitialize();
            telemetry.addData("Splines", "Reinitialized!");
        });

        // Print current settings for calibration table
        Gamepads.gamepad1().y().whenBecomesTrue(() -> {
            double rpm = useInterpolation ?
                ShooterInterpolation.getRPMForDistance(TEST_DISTANCE) : MANUAL_RPM;
            double hood = useInterpolation ?
                ShooterInterpolation.getHoodForDistance(TEST_DISTANCE) : MANUAL_HOOD;

            String calibrationLine = String.format(
                "Distance: %.1f\" | RPM: %.0f | Hood: %.3f",
                TEST_DISTANCE, rpm, hood
            );

            telemetry.addLine("═══════════════════════════════");
            telemetry.addLine("COPY THIS TO CALIBRATION TABLE:");
            telemetry.addLine(calibrationLine);
            telemetry.addLine("═══════════════════════════════");
        });

        // Manual RPM adjustment (when in manual mode)
        Gamepads.gamepad2().dpadUp().whenBecomesTrue(() -> {
            MANUAL_RPM += 100;
        });

        Gamepads.gamepad2().dpadDown().whenBecomesTrue(() -> {
            MANUAL_RPM -= 100;
        });

        Gamepads.gamepad2().dpadRight().whenBecomesTrue(() -> {
            MANUAL_HOOD += 0.05;
        });

        Gamepads.gamepad2().dpadLeft().whenBecomesTrue(() -> {
            MANUAL_HOOD -= 0.05;
        });
    }

    @Override
    public void onUpdate() {
        // Calculate interpolated values
        double targetRPM = useInterpolation ?
            ShooterInterpolation.getRPMForDistance(TEST_DISTANCE) : MANUAL_RPM;
        double targetHood = useInterpolation ?
            ShooterInterpolation.getHoodForDistance(TEST_DISTANCE) : MANUAL_HOOD;

        // Apply to shooter
        if (shooterOn) {
            Shooter.INSTANCE.setTargetRPM(targetRPM);
            Shooter.INSTANCE.setHood(targetHood).schedule();

            // Start shooter if not already running
            if (Shooter.INSTANCE.getRPM() < 100) {
                Shooter.INSTANCE.runRPM(targetRPM).schedule();
            }
        }

        // Get actual shooter state
        double currentRPM = Shooter.INSTANCE.getRPM();
        boolean atSpeed = Shooter.INSTANCE.atSpeed(50.0);

        // Telemetry
        telemetry.addData("═══ SHOOTER INTERPOLATION TUNING ═══", "");
        telemetry.addData("", "");

        telemetry.addData("Mode", useInterpolation ? "✓ INTERPOLATION" : "⚠ MANUAL");
        telemetry.addData("Shooter", shooterOn ? "✓ ON" : "✗ OFF");
        telemetry.addData("", "");

        telemetry.addData("Test Distance", String.format("%.1f inches", TEST_DISTANCE));
        telemetry.addData("Target RPM", String.format("%.0f", targetRPM));
        telemetry.addData("Current RPM", String.format("%.0f", currentRPM));
        telemetry.addData("At Speed?", atSpeed ? "✓ YES" : "✗ NO");
        telemetry.addData("Target Hood", String.format("%.3f", targetHood));
        telemetry.addData("", "");

        telemetry.addData("Interpolation Status", ShooterInterpolation.getCalibrationStatus());
        telemetry.addData("Splines OK?", ShooterInterpolation.isInitialized() ? "✓" : "✗ ERROR");
        telemetry.addData("", "");

        telemetry.addLine("═══ CONTROLS ═══");
        telemetry.addLine("D-Pad Up/Down: Distance ±5\"");
        telemetry.addLine("D-Pad Right/Left: Distance ±1\"");
        telemetry.addLine("Right Bumper: Toggle shooter");
        telemetry.addLine("A: Interpolation mode");
        telemetry.addLine("B: Manual mode");
        telemetry.addLine("X: Reinitialize splines");
        telemetry.addLine("Y: Print calibration line");
        telemetry.addData("", "");

        if (!useInterpolation) {
            telemetry.addLine("═══ MANUAL MODE ═══");
            telemetry.addLine("GP2 D-Pad Up/Down: RPM ±100");
            telemetry.addLine("GP2 D-Pad Right/Left: Hood ±0.05");
        }

        telemetry.update();
    }

    @Override
    public void onStop() {
        Shooter.INSTANCE.stop();
    }
}

