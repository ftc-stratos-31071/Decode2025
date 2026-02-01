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

    private final MotorEx intake = new MotorEx("IntakeMotor").brakeMode().reversed();
    private final MotorEx transfer = new MotorEx("TransferMotor").brakeMode();
    private final ServoEx servo = new ServoEx("DoorServo");


    public void setIntakePower(double power) {
        intake.setPower(power);
    }

    public void setTransferPower(double power) {
        transfer.setPower(power);
    }

    public Command moveIntake(double power) {
        return new Command() {
            @Override
            public void start() {
                intake.setPower(power);
            }
            @Override
            public boolean isDone() {
                return true;
            }
        }.requires(this);
    }

    public Command moveTransfer(double power) {
        return new Command() {
            @Override
            public void start() {
                transfer.setPower(power);
            }
            @Override
            public boolean isDone() {
                return true;
            }
        }.requires(this);
    }

    public Command zeroPowerIntake() {
        return new Command() {
            @Override
            public void start() {
                intake.setPower(IntakeConstants.zeroPower);
            }

            @Override
            public boolean isDone() {
                return true;
            }
        }.requires(this);
    }

    public Command zeroPowerTransfer() {
        return new Command() {
            @Override
            public void start() {
                transfer.setPower(IntakeConstants.zeroPower);
            }

            @Override
            public boolean isDone() {
                return true;
            }
        }.requires(this);
    }

    public Command moveServoPos() {
        return new Command() {
            @Override
            public void start() {
                servo.setPosition(IntakeConstants.servoPos);
            }

            @Override
            public boolean isDone() {
                return true;
            }
        }.requires(this);
    }

    public Command defaultPos() {
        return new Command() {
            @Override
            public void start() {
                servo.setPosition(IntakeConstants.defaultPos);
            }

            @Override
            public boolean isDone() {
                return true;
            }
        }.requires(this);
    }

    public Command turnOn() {
        return new Command() {
            @Override
            public void update() {
                intake.setPower(IntakeConstants.intakePower);
            }

            @Override
            public boolean isDone() {
                return false;
            }
        }.requires(this);
    }

    public Command transfer(double timeSec) {
        return new Command() {
            private double startTime;

            @Override
            public void start() {
                startTime = System.currentTimeMillis() / 1000.0;
                transfer.setPower(-IntakeConstants.shootPower);
            }

            @Override
            public void update() {
                transfer.setPower(-IntakeConstants.shootPower);
            }

            @Override
            public boolean isDone() {
                return System.currentTimeMillis() / 1000.0 - startTime >= timeSec;
            }

            @Override
            public void stop(boolean interrupted) {
                transfer.setPower(IntakeConstants.zeroPower);
            }
        }.requires(this);
    }
}
