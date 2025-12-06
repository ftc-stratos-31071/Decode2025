package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

/**
 * Shooter Velocity PID Tuning OpMode
 *
 * This OpMode helps you tune the shooter's velocity PID controller.
 *
 * CONTROLS:
 * - A Button: Start shooter at target RPM
 * - B Button: Stop shooter
 * - D-Pad Up: Increase target power by 10%
 * - D-Pad Down: Decrease target power by 10%
 * - D-Pad Right: Increase target RPM by 500
 * - D-Pad Left: Decrease target RPM by 500
 *
 * DASHBOARD TUNING:
 * - Open FTC Dashboard: http://192.168.43.1:8080/dash
 * - Go to "Configuration" tab
 * - Adjust ShooterConstants values live:
 *   - velocityKp, velocityKi, velocityKd, velocityKf
 *   - targetRPM, rpmTolerance
 * - Watch the "Shooter RPM" graph in real-time
 *
 * TUNING TIPS:
 * 1. Start with Kf (feedforward) - calculate as: power / velocity
 * 2. Tune Kp until slight oscillation, then reduce by 30%
 * 3. Add Kd if oscillations persist (start with Kp/10)
 * 4. Add Ki only if steady-state error exists (start with Kp/100)
 */
@Config
@TeleOp(name = "PIDTest - Shooter Tuning", group = "Tuning")
public class PIDTest extends NextFTCOpMode {

    // Tunable test parameters
    public static double TEST_POWER = 0.7;  // Direct power mode
    public static double TEST_RPM = 3000;   // Target RPM for velocity mode
    public static boolean USE_VELOCITY_CONTROL = true;  // Toggle between power/velocity mode

    private FtcDashboard dashboard;
    private boolean shooterRunning = false;
    private long testStartTime = 0;
    private double measuredVelocity = 0;
    private double elapsedSeconds = 0.0;  // Time variable for graphing

    public PIDTest() {
        addComponents(
                new SubsystemComponent(Shooter.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        // Setup dashboard with telemetry multiplexing
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Mode", USE_VELOCITY_CONTROL ? "Velocity PID" : "Direct Power");
        telemetry.addData("", "");
        telemetry.addData("CONTROLS", "");
        telemetry.addData("A", "Start Shooter");
        telemetry.addData("B", "Stop Shooter");
        telemetry.addData("D-Pad Up/Down", "Adjust Power ±10%");
        telemetry.addData("D-Pad Right/Left", "Adjust RPM ±500");
        telemetry.addData("", "");
        telemetry.addData("Dashboard", "http://192.168.43.1:8080/dash");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        // A Button - Start shooter
        Gamepads.gamepad1().a().whenBecomesTrue(() -> {
            if (USE_VELOCITY_CONTROL) {
                Shooter.INSTANCE.runAtRPM(TEST_RPM).schedule();
            } else {
                Shooter.INSTANCE.moveShooter(TEST_POWER).schedule();
            }
            shooterRunning = true;
            testStartTime = System.currentTimeMillis();
        });

        // B Button - Stop shooter
        Gamepads.gamepad1().b().whenBecomesTrue(() -> {
            Shooter.INSTANCE.zeroPower.schedule();
            shooterRunning = false;
        });

        // D-Pad Up - Increase power
        Gamepads.gamepad1().dpadUp().whenBecomesTrue(() -> {
            TEST_POWER = Math.min(1.0, TEST_POWER + 0.1);
        });

        // D-Pad Down - Decrease power
        Gamepads.gamepad1().dpadDown().whenBecomesTrue(() -> {
            TEST_POWER = Math.max(0.0, TEST_POWER - 0.1);
        });

        // D-Pad Right - Increase RPM
        Gamepads.gamepad1().dpadRight().whenBecomesTrue(() -> {
            TEST_RPM = Math.min(6000, TEST_RPM + 500);
        });

        // D-Pad Left - Decrease RPM
        Gamepads.gamepad1().dpadLeft().whenBecomesTrue(() -> {
            TEST_RPM = Math.max(0, TEST_RPM - 500);
        });

        // X Button - Toggle control mode
        Gamepads.gamepad1().x().whenBecomesTrue(() -> {
            USE_VELOCITY_CONTROL = !USE_VELOCITY_CONTROL;
            if (shooterRunning) {
                Shooter.INSTANCE.zeroPower.schedule();
                shooterRunning = false;
            }
        });
    }

    @Override
    public void onUpdate() {
        // Get current shooter metrics
        double currentRPM = Shooter.INSTANCE.getRPM();
        measuredVelocity = Shooter.INSTANCE.getRPM() / 60.0;  // Convert RPM to RPS
        double targetRPM = USE_VELOCITY_CONTROL ? TEST_RPM : 0;
        double error = targetRPM - currentRPM;
        double errorPercent = targetRPM > 0 ? (error / targetRPM) * 100 : 0;
        boolean atTarget = Shooter.INSTANCE.isAtRPM(TEST_RPM);

        // Calculate Kf suggestion based on current measurements
        double suggestedKf = measuredVelocity > 0 ? TEST_POWER / measuredVelocity : 0;

        // Calculate elapsed time
        long elapsedMs = shooterRunning ? System.currentTimeMillis() - testStartTime : 0;
        elapsedSeconds = elapsedMs / 1000.0;  // Convert to seconds for graphing

        // ========================================================================
        // TELEMETRY - Displayed on Driver Station and Dashboard
        // ========================================================================
        telemetry.addData("═══ MODE ═══", "");
        telemetry.addData("Control Mode", USE_VELOCITY_CONTROL ? "✓ VELOCITY PID" : "✗ Direct Power");
        telemetry.addData("Status", shooterRunning ? "✓ RUNNING" : "✗ STOPPED");

        if (shooterRunning) {
            telemetry.addData("Running Time", String.format("%.1f sec", elapsedSeconds));
        }

        telemetry.addData("", "");
        telemetry.addData("═══ CURRENT PERFORMANCE ═══", "");
        telemetry.addData("Current RPM", String.format("%.0f", currentRPM));
        telemetry.addData("Target RPM", String.format("%.0f", targetRPM));
        telemetry.addData("Error", String.format("%.0f RPM (%.1f%%)", error, errorPercent));
        telemetry.addData("At Target", atTarget ? "✓ YES" : "✗ NO");

        telemetry.addData("", "");
        telemetry.addData("═══ TEST PARAMETERS ═══", "");
        telemetry.addData("Test Power", String.format("%.2f", TEST_POWER));
        telemetry.addData("Test RPM Target", String.format("%.0f", TEST_RPM));

        telemetry.addData("", "");
        telemetry.addData("═══ CURRENT PID VALUES ═══", "");
        telemetry.addData("Kp", ShooterConstants.velocityKp);
        telemetry.addData("Ki", ShooterConstants.velocityKi);
        telemetry.addData("Kd", ShooterConstants.velocityKd);
        telemetry.addData("Kf (Feedforward)", ShooterConstants.velocityKf);

        if (!USE_VELOCITY_CONTROL && measuredVelocity > 0) {
            telemetry.addData("", "");
            telemetry.addData("═══ Kf CALCULATION ═══", "");
            telemetry.addData("Suggested Kf", String.format("%.6f", suggestedKf));
            telemetry.addData("Formula", "Kf = Power / Velocity");
            telemetry.addData("Calculation", String.format("%.2f / %.2f", TEST_POWER, measuredVelocity));
        }

        // Debug info for velocity control mode
        if (USE_VELOCITY_CONTROL && shooterRunning) {
            telemetry.addData("", "");
            telemetry.addData("═══ DEBUG INFO ═══", "");
            telemetry.addData("Velocity Control Active", Shooter.INSTANCE.velocityControlActive);
            telemetry.addData("Current Velocity (ticks/s)", measuredVelocity * 60.0);  // Convert back to ticks/s
            double targetTicksPerSec = (TEST_RPM * 112.0) / 60.0;
            telemetry.addData("Target Velocity (ticks/s)", targetTicksPerSec);
            double calcFeedforward = targetTicksPerSec * ShooterConstants.velocityKf;
            telemetry.addData("Calculated Feedforward", String.format("%.3f", calcFeedforward));
            telemetry.addData("FF Clamped?", calcFeedforward > 1.0 ? "YES - TOO HIGH!" : "No");
        }

        telemetry.addData("", "");
        telemetry.addData("═══ CONTROLS ═══", "");
        telemetry.addData("A", "Start Shooter");
        telemetry.addData("B", "Stop Shooter");
        telemetry.addData("X", "Toggle Control Mode");
        telemetry.addData("D-Pad ↑↓", "Power ±10%");
        telemetry.addData("D-Pad ←→", "RPM ±500");

        telemetry.update();

        // ========================================================================
        // DASHBOARD DATA - Send all values for graphing
        // ========================================================================
        // IMPORTANT: All these values will be available in Dashboard's graph selector
        // You can plot any value against "Time" or against each other

        // Time variable (use this as X-axis for most graphs)
        dashboard.getTelemetry().addData("Time", elapsedSeconds);

        // Main performance metrics
        dashboard.getTelemetry().addData("Current RPM", currentRPM);
        dashboard.getTelemetry().addData("Target RPM", targetRPM);
        dashboard.getTelemetry().addData("RPM Error", error);
        dashboard.getTelemetry().addData("Error Percent", errorPercent);

        // Raw velocity for Kf calculation
        dashboard.getTelemetry().addData("Velocity RPS", measuredVelocity);

        // Binary indicator (0 or 1)
        dashboard.getTelemetry().addData("At Target", atTarget ? 1.0 : 0.0);

        // Test parameters (useful for multi-test comparison)
        dashboard.getTelemetry().addData("Test Power", TEST_POWER);
        dashboard.getTelemetry().addData("Test RPM", TEST_RPM);

        dashboard.getTelemetry().update();
    }
}
