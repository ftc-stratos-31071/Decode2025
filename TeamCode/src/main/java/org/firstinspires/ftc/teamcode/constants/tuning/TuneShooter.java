package org.firstinspires.ftc.teamcode.constants.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.ShooterPIDF;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;

/**
 * Simple tuning OpMode for ShooterPIDF.
 *
 * - Use FTC Dashboard to:
 *   - Adjust ShooterPIDF.kP, kI, kD, kF
 *   - Change targetRpm
 *   - Toggle closed-loop on/off
 *
 * ShooterPIDF.periodic() is called automatically by NextFTC via SubsystemComponent.
 */
@Config
@TeleOp(name = "TuneShooter", group = "Tuning")
public class TuneShooter extends NextFTCOpMode {

    // Tunable from FTC Dashboard
    public static double targetRpm = 3000.0;
    public static boolean closedLoopEnabled = true;

    public TuneShooter() {
        // Register ShooterPIDF as a subsystem so its periodic() runs every loop
        addComponents(
                new SubsystemComponent(ShooterPIDF.INSTANCE)
        );
    }

    @Override
    public void onInit() {
        // Hook telemetry to FTC Dashboard
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Optionally set hood to default once for safety
        ShooterPIDF.INSTANCE.defaultPos.schedule();

        telemetry.addLine("TuneShooter Initialized");
        telemetry.addLine("Use FTC Dashboard to tune ShooterPIDF gains & targetRpm.");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        // Nothing special; ShooterPIDF will be driven in onUpdate() + periodic()
    }

    @Override
    public void onUpdate() {
        ShooterPIDF shooter = ShooterPIDF.INSTANCE;

        if (closedLoopEnabled) {
            // Closed-loop: PIDF holds targetRpm
            shooter.setTargetRPM(targetRpm);
        } else {
            // Disabled: stop shooter
            shooter.stopShooter();
        }

        double currentRpm = shooter.getRPM();
        double error = targetRpm - currentRpm;

        telemetry.addLine("==== ShooterPIDF Tuning ====");
        telemetry.addData("Closed Loop Enabled", closedLoopEnabled);
        telemetry.addData("Target RPM", "%.0f", targetRpm);
        telemetry.addData("Current RPM", "%.0f", currentRpm);
        telemetry.addData("Error RPM", "%.0f", error);

        telemetry.addLine();
        telemetry.addData("kP", ShooterPIDF.kP);
        telemetry.addData("kI", ShooterPIDF.kI);
        telemetry.addData("kD", ShooterPIDF.kD);
        telemetry.addData("kF", ShooterPIDF.kF);

        telemetry.update();
    }
}