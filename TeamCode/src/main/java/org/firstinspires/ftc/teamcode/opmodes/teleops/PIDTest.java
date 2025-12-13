package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

/**
 * Shooter Power Testing OpMode
 *
 * This OpMode helps you test the shooter at different power levels.
 *
 * CONTROLS:
 * - A Button: Start shooter at test power
 * - B Button: Stop shooter
 * - D-Pad Up: Increase power by 10%
 * - D-Pad Down: Decrease power by 10%
 *
 * DASHBOARD:
 * - Open FTC Dashboard: http://192.168.43.1:8080/dash
 * - Watch the "Shooter RPM" graph in real-time
 * - Adjust TEST_POWER to find optimal shooter speed
 */
@Config
//@TeleOp(name = "Shooter Power Test", group = "Tuning")
public class PIDTest extends NextFTCOpMode {

    // Tunable test parameter
    public static double TEST_POWER = 0.7;  // Direct power control

    private FtcDashboard dashboard;
    private boolean shooterRunning = false;
    private long testStartTime = 0;
    private double elapsedSeconds = 0.0;

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
        telemetry.addData("Mode", "Direct Power Control");
        telemetry.addData("", "");
        telemetry.addData("CONTROLS", "");
        telemetry.addData("A", "Start Shooter");
        telemetry.addData("B", "Stop Shooter");
        telemetry.addData("D-Pad Up/Down", "Adjust Power ±10%");
        telemetry.addData("", "");
        telemetry.addData("Dashboard", "http://192.168.43.1:8080/dash");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        // A Button - Start shooter
        Gamepads.gamepad1().a().whenBecomesTrue(() -> {
            Shooter.INSTANCE.moveShooter(TEST_POWER).schedule();
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
            if (shooterRunning) {
                Shooter.INSTANCE.moveShooter(TEST_POWER).schedule();
            }
        });

        // D-Pad Down - Decrease power
        Gamepads.gamepad1().dpadDown().whenBecomesTrue(() -> {
            TEST_POWER = Math.max(0.0, TEST_POWER - 0.1);
            if (shooterRunning) {
                Shooter.INSTANCE.moveShooter(TEST_POWER).schedule();
            }
        });
    }

    @Override
    public void onUpdate() {
        // Get current shooter metrics
        double currentRPM = Shooter.INSTANCE.getRPM();

        // Calculate elapsed time
        long elapsedMs = shooterRunning ? System.currentTimeMillis() - testStartTime : 0;
        elapsedSeconds = elapsedMs / 1000.0;

        // ========================================================================
        // TELEMETRY - Displayed on Driver Station and Dashboard
        // ========================================================================
        telemetry.addData("═══ STATUS ═══", "");
        telemetry.addData("Shooter", shooterRunning ? "✓ RUNNING" : "✗ STOPPED");

        if (shooterRunning) {
            telemetry.addData("Running Time", String.format("%.1f sec", elapsedSeconds));
        }

        telemetry.addData("", "");
        telemetry.addData("═══ PERFORMANCE ═══", "");
        telemetry.addData("Current RPM", String.format("%.0f", currentRPM));
        telemetry.addData("Test Power", String.format("%.2f (%.0f%%)", TEST_POWER, TEST_POWER * 100));

        telemetry.addData("", "");
        telemetry.addData("═══ CONTROLS ═══", "");
        telemetry.addData("A", "Start Shooter");
        telemetry.addData("B", "Stop Shooter");
        telemetry.addData("D-Pad ↑↓", "Power ±10%");

        telemetry.update();

        // ========================================================================
        // DASHBOARD DATA - Send values for graphing
        // ========================================================================
        dashboard.getTelemetry().addData("Time", elapsedSeconds);
        dashboard.getTelemetry().addData("Current RPM", currentRPM);
        dashboard.getTelemetry().addData("Test Power", TEST_POWER);
        dashboard.getTelemetry().addData("Running", shooterRunning ? 1.0 : 0.0);

        dashboard.getTelemetry().update();
    }
}
