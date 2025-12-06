package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.constants.IntakeConstants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

@Config
public class Intake implements Subsystem {
    public static final Intake INSTANCE = new Intake();

    private Intake() {}

    // Control Hub Motor Port 0
    private final MotorEx intake = new MotorEx("IntakeMotor").brakeMode().reversed();
    // Expansion Hub Servo Port 0
    private final ServoEx servo = new ServoEx("IntakeServo");

    // Dynamic servo commands - read values from IntakeConstants each time
    public final Command moveServoPos = new Command() {
        @Override
        public void start() {
            servo.setPosition(IntakeConstants.servoPos);
        }

        @Override
        public boolean isDone() {
            return true;  // Completes immediately after setting position
        }
    }.requires(this);

    public final Command defaultPos = new Command() {
        @Override
        public void start() {
            servo.setPosition(IntakeConstants.defaultPos);
        }

        @Override
        public boolean isDone() {
            return true;  // Completes immediately after setting position
        }
    }.requires(this);

    // Dynamic motor commands - read values from IntakeConstants each time
    public final Command turnOn = new Command() {
        @Override
        public void update() {
            intake.setPower(IntakeConstants.intakePower);
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

    public final Command turnOnReverse = new Command() {
        @Override
        public void update() {
            intake.setPower(-IntakeConstants.intakePowerSlow);
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
            intake.setPower(IntakeConstants.zeroPower);
        }

        @Override
        public boolean isDone() {
            return true;  // Completes immediately after setting power
        }
    }.requires(this);

    public final Command shoot = new Command() {
        @Override
        public void update() {
            intake.setPower(IntakeConstants.shootPower);
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
}
