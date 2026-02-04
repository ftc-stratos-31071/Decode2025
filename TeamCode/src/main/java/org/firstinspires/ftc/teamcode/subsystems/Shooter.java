package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.constants.ShooterConstants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

@Config
public class Shooter implements Subsystem {

    public static final Shooter INSTANCE = new Shooter();

    /* ================= HARDWARE ================= */

    public final MotorEx rightMotor = new MotorEx("ShooterRight").brakeMode().reversed();
    public final MotorEx leftMotor  = new MotorEx("ShooterLeft").brakeMode();

    private final ServoEx hoodServo = new ServoEx("HoodServo");

    private VoltageSensor voltageSensor = null;

    /* ================= PIDF STATE ================= */

    private double targetRPM = 0.0;
    private boolean pidEnabled = false;

    private double integral = 0.0;
    private double lastError = 0.0;
    private long lastTimeNs = 0;

    /* ================= DASHBOARD DEBUG ================= */

    public static double DEBUG_currentRPM;
    public static double DEBUG_error;
    public static double DEBUG_output;
    public static double DEBUG_rightRPM;
    public static double DEBUG_leftRPM;
    public static double DEBUG_voltage;
    public static double DEBUG_compensation;

    private Shooter() {
        lastTimeNs = System.nanoTime();
    }

    /* ================= PUBLIC API ================= */

    public void setTargetRPM(double rpm) {
        targetRPM = Math.max(0.0, rpm);
        integral = 0.0;
        lastError = 0.0;
        lastTimeNs = System.nanoTime();
        pidEnabled = true;
    }

    public void stop() {
        pidEnabled = false;
        targetRPM = 0.0;
        integral = 0.0;
        lastError = 0.0;
        rightMotor.setPower(0);
        leftMotor.setPower(0);
    }

    public boolean atSpeed(double toleranceRPM) {
        return Math.abs(getRPM() - targetRPM) <= toleranceRPM;
    }

    /* ================= VELOCITY ================= */

    public double ticksPerSecondToRPM(double ticksPerSecond) {
        return (ticksPerSecond / ShooterConstants.TICKS_PER_REV) * 60.0;
    }

    public double getRPM() {
        double rightRPM = ticksPerSecondToRPM(Math.abs(rightMotor.getVelocity()));
        double leftRPM  = ticksPerSecondToRPM(Math.abs(leftMotor.getVelocity()));
        return (rightRPM + leftRPM) / 2.0;
    }

    /* ================= SERVO ================= */

    public Command setHood(double position) {
        return new Command() {
            @Override
            public void start() {
                hoodServo.setPosition(position);
            }

            @Override
            public boolean isDone() {
                return true;
            }
        }.requires(this);
    }

    /* ================= COMMANDS ================= */

    public Command runRPM(double rpm) {
        return new Command() {
            @Override
            public void start() {
                setTargetRPM(rpm);
            }

            @Override
            public boolean isDone() {
                return false;
            }

            @Override
            public void stop(boolean interrupted) {
            }
        }.requires(this);
    }

    public Command openLoop(double power) {
        return new Command() {
            @Override
            public void start() {
                pidEnabled = false;
                rightMotor.setPower(power);
                leftMotor.setPower(power);
            }

            @Override
            public boolean isDone() {
                return true;
            }
        }.requires(this);
    }

    /* ================= MAIN CONTROL LOOP ================= */

    @Override
    public void periodic() {
        if (!pidEnabled || targetRPM <= 0) {
            return;
        }

        long now = System.nanoTime();
        double dt = (now - lastTimeNs) / 1e9;
        lastTimeNs = now;

        if (dt <= 0) return;

        double rightRPM = ticksPerSecondToRPM(Math.abs(rightMotor.getVelocity()));
        double leftRPM  = ticksPerSecondToRPM(Math.abs(leftMotor.getVelocity()));
        double currentRPM = (rightRPM + leftRPM) / 2.0;

        double error = targetRPM - currentRPM;
        integral += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double output =
                ShooterConstants.kF * targetRPM +
                        ShooterConstants.kP * error +
                        ShooterConstants.kI * integral +
                        ShooterConstants.kD * derivative;

        // ========== BATTERY COMPENSATION ==========
        double compensatedOutput = output;
        double voltageCompensation = 1.0;

        if (ShooterConstants.ENABLE_BATTERY_COMPENSATION) {
            // Lazy-load voltage sensor on first use
            if (voltageSensor == null) {
                try {
                    voltageSensor = ActiveOpMode.hardwareMap().voltageSensor.iterator().next();
                } catch (Exception e) {
                    // If sensor unavailable, disable compensation
                    voltageSensor = null;
                }
            }

            if (voltageSensor != null) {
                double currentVoltage = voltageSensor.getVoltage();
                voltageCompensation = ShooterConstants.NOMINAL_VOLTAGE / currentVoltage;
                compensatedOutput = output * voltageCompensation;

                DEBUG_voltage = currentVoltage;
                DEBUG_compensation = voltageCompensation;
            }
        }
        // ==========================================

        compensatedOutput = Math.max(
                ShooterConstants.MIN_POWER,
                Math.min(ShooterConstants.MAX_POWER, compensatedOutput)
        );

        rightMotor.setPower(compensatedOutput);
        leftMotor.setPower(compensatedOutput);

        DEBUG_currentRPM = currentRPM;
        DEBUG_error = error;
        DEBUG_output = compensatedOutput;
        DEBUG_rightRPM = rightRPM;
        DEBUG_leftRPM = leftRPM;

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Target RPM", targetRPM);
        packet.put("Current RPM", currentRPM);
        packet.put("Output", compensatedOutput);
        if (ShooterConstants.ENABLE_BATTERY_COMPENSATION && voltageSensor != null) {
            packet.put("Battery Voltage", DEBUG_voltage);
            packet.put("Compensation", String.format("%.2fx", DEBUG_compensation));
        }
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }
}

