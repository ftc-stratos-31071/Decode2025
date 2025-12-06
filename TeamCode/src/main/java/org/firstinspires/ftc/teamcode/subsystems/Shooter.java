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
public class Shooter implements Subsystem {
    public static final Shooter INSTANCE = new Shooter();

    // Expansion Hub Servo Port 1
    private final ServoEx servo = new ServoEx("HoodServo");
    // Expansion Hub Motor Port 0
    private final MotorEx motor1 = new MotorEx("ShooterRight").brakeMode();
    // Expansion Hub Motor Port 1
    private final MotorEx motor2 = new MotorEx("ShooterLeft").brakeMode().reversed();

    private Shooter() {
    }

    // Dynamic servo commands - read values from ShooterConstants each time
    public final Command moveServoPos = new Command() {
        @Override
        public void start() {
            servo.setPosition(ShooterConstants.servoPos);
        }

        @Override
        public boolean isDone() {
            return true;  // Completes immediately after setting position
        }
    }.requires(this);

    public final Command defaultPos = new Command() {
        @Override
        public void start() {
            servo.setPosition(ShooterConstants.defaultPos);
        }

        @Override
        public boolean isDone() {
            return true;  // Completes immediately after setting position
        }
    }.requires(this);

    /**
     * Run shooter at a specific power
     */
    public Command moveShooter(double shooterPower) {
        return new ParallelGroup(
                new SetPower(motor1, shooterPower),
                new SetPower(motor2, shooterPower)
        ).requires(this);
    }

    public Command moveServo(double servoPos) {
        return new SetPosition(servo, servoPos).requires(this);
    }

    // Dynamic commands that read from constants
    public final Command moveShooterReversed = new Command() {
        @Override
        public void update() {
            motor1.setPower(-ShooterConstants.motorPower);
            motor2.setPower(-ShooterConstants.motorPower);
        }

        @Override
        public boolean isDone() {
            return false;  // Runs continuously until interrupted
        }

        @Override
        public void stop(boolean interrupted) {
            // Don't stop motor here - let other commands control it
        }
    }.requires(this);

    public final Command zeroPower = new Command() {
        @Override
        public void start() {
            motor1.setPower(ShooterConstants.zeroPower);
            motor2.setPower(ShooterConstants.zeroPower);
        }

        @Override
        public boolean isDone() {
            return true;  // Completes immediately after setting power
        }
    }.requires(this);

    /**
     * Get current shooter RPM (for telemetry only)
     */
    public double getRPM() {
        double ticksPerSecond = motor1.getVelocity();
        return (ticksPerSecond / 112.0) * 60.0;
    }
}
