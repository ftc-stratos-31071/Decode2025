package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.constants.ShooterConstants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

@Config
public class ShooterPIDF implements Subsystem {
    public static final ShooterPIDF INSTANCE = new ShooterPIDF();

    private final ServoEx servo = new ServoEx("HoodServo");
    private final MotorEx motor1 = new MotorEx("ShooterRight").brakeMode();
    private final MotorEx motor2 = new MotorEx("ShooterLeft").brakeMode().reversed();

    // ===== PIDF GAINS TUNABLE IN DASHBOARD =====
    public static double kP = 0.0005;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 1.0 / 6000.0;  // FF: adjusts based on your peak RPM

    public static double MIN_POWER = 0.0;
    public static double MAX_POWER = 1.0;

    private double targetRpm = 0.0;
    private double integral = 0.0;
    private double lastError = 0.0;

    private ShooterPIDF() {}

    // ========= PUBLIC API =========

    /** Set desired closed-loop RPM */
    public void setTargetRPM(double rpm) {
        targetRpm = Math.max(0, rpm);
    }

    public double getTargetRPM() {
        return targetRpm;
    }

    /** Stops shooter + resets controller */
    public void stopShooter() {
        targetRpm = 0.0;
        integral = 0.0;
        lastError = 0.0;
        motor1.setPower(0);
        motor2.setPower(0);
    }

    // ========= Servo Commands =========

    public final Command moveServoPos = new Command() {
        @Override
        public void start() {
            servo.setPosition(ShooterConstants.servoPos);
        }

        @Override
        public boolean isDone() {
            return true;
        }
    }.requires(this);

    public final Command defaultPos = new Command() {
        @Override
        public void start() {
            servo.setPosition(ShooterConstants.defaultPos);
        }

        @Override
        public boolean isDone() {
            return true;
        }
    }.requires(this);

    public Command moveServo(double servoPos) {
        return new SetPosition(servo, servoPos).requires(this);
    }

    // ========= OPTIONAL OPEN-LOOP POWER (FOR TESTING) =========
    public Command moveShooter(double power) {
        return new ParallelGroup(
                new SetPower(motor1, power),
                new SetPower(motor2, power)
        ).requires(this);
    }

    public final Command zeroPower = new Command() {
        @Override
        public void start() {
            stopShooter();
        }

        @Override
        public boolean isDone() {
            return true;
        }
    }.requires(this);

    // ========= SENSOR =========

    public double getRPM() {
        double ticksPerSecond = motor1.getVelocity();
        return (ticksPerSecond / 112.0) * 60.0;
    }

    // ========= MAIN PIDF LOOP (CALLED EVERY CYCLE) =========

    @Override
    public void periodic() {
        if (targetRpm <= 0) {
            motor1.setPower(0);
            motor2.setPower(0);
            integral = 0;
            lastError = 0;
            return;
        }

        double currentRpm = getRPM();
        double error = targetRpm - currentRpm;

        integral += error;
        double derivative = error - lastError;
        lastError = error;

        double output =
                kF * targetRpm +
                        kP * error +
                        kI * integral +
                        kD * derivative;

        output = Math.max(MIN_POWER, Math.min(MAX_POWER, output));

        motor1.setPower(output);
        motor2.setPower(output);
    }
}