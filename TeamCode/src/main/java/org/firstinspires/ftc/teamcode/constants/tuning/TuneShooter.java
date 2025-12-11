package org.firstinspires.ftc.teamcode.constants.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.constants.ShooterConstants;

import java.util.List;

/**
 * TuneShooter - Simple PIDF Tuning OpMode
 *
 * This OpMode directly controls the shooter motors for PIDF tuning.
 * No command scheduling - just direct motor control in the loop.
 *
 * HOW TO USE:
 * 1. Run this OpMode
 * 2. Open FTC Dashboard (http://192.168.43.1:8080/dash)
 * 3. Go to "Configuration" and find "TuneShooter" and "ShooterConstants"
 * 4. Set shooterEnabled = true to start the shooter
 * 5. Adjust PIDF gains in ShooterConstants and watch the graph
 *
 * TUNING STEPS:
 * 1. Set kP, kI, kD to 0
 * 2. Increase kF until shooter reaches ~80% of target RPM
 * 3. Increase kP until it reaches target quickly (stop if oscillating)
 * 4. Add small kD if there's overshoot
 * 5. Usually keep kI at 0
 */
@Config
@TeleOp(name = "TuneShooter", group = "Tuning")
public class TuneShooter extends OpMode {

    // ===== DASHBOARD TUNABLE VALUES =====
    public static double targetRpm = 3000.0;
    public static boolean shooterEnabled = false;

    // Hardware
    private DcMotorEx shooterRight;
    private DcMotorEx shooterLeft;
    private List<LynxModule> allHubs;

    // PIDF state
    private double integral = 0.0;
    private double lastError = 0.0;

    // Dashboard
    private FtcDashboard dashboard;

    @Override
    public void init() {
        // Enable bulk reads for fresh sensor data every loop
        allHubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Get hardware directly
        shooterRight = hardwareMap.get(DcMotorEx.class, "ShooterRight");
        shooterLeft = hardwareMap.get(DcMotorEx.class, "ShooterLeft");

        // Configure motors
        shooterRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooterLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        shooterLeft.setDirection(DcMotorEx.Direction.REVERSE);

        // Use RUN_WITHOUT_ENCODER for raw power control
        // getVelocity() still works in this mode!
        shooterRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        shooterLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Setup dashboard
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        telemetry.addLine("=== TuneShooter Ready ===");
        telemetry.addLine("Set shooterEnabled = true in Dashboard to start");
        telemetry.addLine("Adjust PIDF gains in ShooterConstants");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Toggle with gamepad as backup
        if (gamepad1.b) {
            shooterEnabled = true;
        }
        if (gamepad1.x) {
            shooterEnabled = false;
            integral = 0;
            lastError = 0;
        }
        if (gamepad1.a) {
            // Reset PIDF state
            integral = 0;
            lastError = 0;
        }

        // Get current RPM - try reading from both motors
        double velocityRight = shooterRight.getVelocity();
        double velocityLeft = shooterLeft.getVelocity();

        // Use whichever motor has the encoder connected
        double ticksPerSecond = velocityRight != 0 ? velocityRight : velocityLeft;

        // Convert to RPM: (ticks/sec) / (ticks/rev) * 60 = RPM
        // 112 ticks per revolution for this motor, * 5 multiplier used in Teleop
        double currentRpm = (ticksPerSecond / 112.0) * 60.0 * 5;

        double output = 0;
        double error = 0;

        if (shooterEnabled && targetRpm > 0) {
            // Calculate PIDF
            error = targetRpm - currentRpm;
            integral += error;
            double derivative = error - lastError;
            lastError = error;

            // Compute output using ShooterConstants (live tunable!)
            output = ShooterConstants.kF * targetRpm
                   + ShooterConstants.kP * error
                   + ShooterConstants.kI * integral
                   + ShooterConstants.kD * derivative;

            // Clamp output
            output = Math.max(ShooterConstants.MIN_POWER, Math.min(ShooterConstants.MAX_POWER, output));

            // Apply power to motors
            shooterRight.setPower(output);
            shooterLeft.setPower(output);
        } else {
            // Shooter off
            shooterRight.setPower(0);
            shooterLeft.setPower(0);
            integral = 0;
            lastError = 0;
        }

        // Send data to dashboard graph
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target RPM", targetRpm);
        packet.put("Current RPM", currentRpm);
        packet.put("Error", error);
        packet.put("Output Power", output);
        dashboard.sendTelemetryPacket(packet);

        // Driver station telemetry
        telemetry.addLine("===== SHOOTER PIDF TUNING =====");
        telemetry.addData("Status", shooterEnabled ? "✓ RUNNING" : "✗ STOPPED");
        telemetry.addData("Target RPM", "%.0f", targetRpm);
        telemetry.addData("Current RPM", "%.0f", currentRpm);
        telemetry.addData("Error", "%.0f", error);
        telemetry.addData("Output Power", "%.3f", output);

        telemetry.addLine();
        telemetry.addLine("----- DEBUG: Raw Velocity -----");
        telemetry.addData("Right Motor Vel", "%.2f ticks/sec", velocityRight);
        telemetry.addData("Left Motor Vel", "%.2f ticks/sec", velocityLeft);
        telemetry.addData("Right Motor Pos", shooterRight.getCurrentPosition());
        telemetry.addData("Left Motor Pos", shooterLeft.getCurrentPosition());

        telemetry.addLine();
        telemetry.addLine("----- PIDF GAINS (ShooterConstants) -----");
        telemetry.addData("kP", "%.6f", ShooterConstants.kP);
        telemetry.addData("kI", "%.6f", ShooterConstants.kI);
        telemetry.addData("kD", "%.6f", ShooterConstants.kD);
        telemetry.addData("kF", "%.6f", ShooterConstants.kF);

        telemetry.addLine();
        telemetry.addLine("----- CONTROLS -----");
        telemetry.addData("B", "Start Shooter");
        telemetry.addData("X", "Stop Shooter");
        telemetry.addData("A", "Reset PIDF State");

        telemetry.update();
    }

    @Override
    public void stop() {
        shooterRight.setPower(0);
        shooterLeft.setPower(0);
    }
}
