package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.opmodes.autos.LaserRangefinder;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.powerable.SetPower;

@Config
public class Intake implements Subsystem {

    public static final Intake INSTANCE = new Intake();

    private Intake() {}

    private final MotorEx intake = new MotorEx("IntakeMotor").brakeMode().reversed();
    private final ServoEx servo = new ServoEx("DoorServo");

    /* =========================
       BASIC MOTOR COMMANDS
       ========================= */

    public Command moveIntake(double power) {
        return new SetPower(intake, power).requires(this);
    }

    public Command zeroPower() {
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

    /* =========================
       SERVO COMMANDS
       ========================= */

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

    /* =========================
       TIMED INTAKE ACTIONS
       ========================= */

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

    public Command turnOnReverse() {
        return new Command() {
            private double startTime;

            @Override
            public void start() {
                startTime = System.currentTimeMillis() / 1000.0;
                intake.setPower(-IntakeConstants.intakePowerSlow);
            }

            @Override
            public void update() {
                intake.setPower(-IntakeConstants.intakePowerSlow);
            }

            @Override
            public boolean isDone() {
                return System.currentTimeMillis() / 1000.0 - startTime
                        >= IntakeConstants.reverseTime;
            }

            @Override
            public void stop(boolean interrupted) {
                intake.setPower(IntakeConstants.zeroPower);
            }
        }.requires(this);
    }

    /* =========================
       SHOOT SEQUENCES
       ========================= */

    public Command shoot(double timeSec) {
        return new Command() {
            private double startTime;

            @Override
            public void start() {
                startTime = System.currentTimeMillis() / 1000.0;
                intake.setPower(IntakeConstants.shootPower);
            }

            @Override
            public void update() {
                intake.setPower(IntakeConstants.shootPower);
            }

            @Override
            public boolean isDone() {
                return System.currentTimeMillis() / 1000.0 - startTime >= timeSec;
            }

            @Override
            public void stop(boolean interrupted) {
                intake.setPower(IntakeConstants.zeroPower);
            }
        }.requires(this);
    }

    public Command shootCont(double timeSec) {
        return new Command() {
            private double startTime;

            @Override
            public void start() {
                startTime = System.currentTimeMillis() / 1000.0;
                intake.setPower(IntakeConstants.shootPower);
            }

            @Override
            public void update() {
                intake.setPower(IntakeConstants.shootPower);
            }

            @Override
            public boolean isDone() {
                return System.currentTimeMillis() / 1000.0 - startTime >= timeSec;
            }

            @Override
            public void stop(boolean interrupted) {
                intake.setPower(IntakeConstants.zeroPower);
            }
        }.requires(this);
    }

    /* =========================
       BALL DETECTION (SAFE)
       ========================= */

    public Command waitForBall(final LaserRangefinder sensor) {
        return new Command() {
            private double startTime;

            @Override
            public void start() {
                startTime = System.currentTimeMillis() / 1000.0;
                intake.setPower(IntakeConstants.intakePower);
            }

            @Override
            public void update() {
                intake.setPower(IntakeConstants.intakePower);
            }

            @Override
            public boolean isDone() {
                double d = sensor.getDistance(DistanceUnit.MM);

                boolean valid = !Double.isNaN(d) && d > 0 && d < 200;
                boolean detected = valid && d >= 20;
                boolean timedOut = System.currentTimeMillis() / 1000.0 - startTime >= 1.0;

                return detected || timedOut;
            }

            @Override
            public void stop(boolean interrupted) {
                intake.setPower(IntakeConstants.zeroPower);
            }
        }.requires(this);
    }

    public Command waitForBallLong(final LaserRangefinder sensor) {
        return new Command() {
            private double startTime;

            @Override
            public void start() {
                startTime = System.currentTimeMillis() / 1000.0;
                intake.setPower(IntakeConstants.intakePower);
            }

            @Override
            public void update() {
                intake.setPower(IntakeConstants.intakePower);
            }

            @Override
            public boolean isDone() {
                double d = sensor.getDistance(DistanceUnit.MM);

                boolean valid = !Double.isNaN(d) && d > 0 && d < 200;
                boolean detected = valid && d >= 20;
                boolean timedOut = System.currentTimeMillis() / 1000.0 - startTime >= 2.0;

                return detected || timedOut;
            }

            @Override
            public void stop(boolean interrupted) {
                intake.setPower(IntakeConstants.zeroPower);
            }
        }.requires(this);
    }
}
