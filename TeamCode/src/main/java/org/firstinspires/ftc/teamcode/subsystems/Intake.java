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

    public Command moveIntake(double power) {
        return new SetPower(intake, power);
    }

    public final Command moveServoPos = new Command() {
        @Override
        public void start() {
            servo.setPosition(IntakeConstants.servoPos);
        }

        @Override
        public boolean isDone() {
            return true;
        }
    }.requires(this);

    public final Command defaultPos = new Command() {
        @Override
        public void start() {
            servo.setPosition(IntakeConstants.defaultPos);
        }

        @Override
        public boolean isDone() {
            return true;
        }
    }.requires(this);

    public final Command turnOn = new Command() {
        @Override
        public void update() {
            intake.setPower(IntakeConstants.intakePower);
        }

        @Override
        public boolean isDone() {
            return false;
        }
    }.requires(this);

    private double shootStartTimeSec2 = 0.0;

    public final Command turnOnReverse = new Command() {
        @Override
        public void start() {
            shootStartTimeSec2 = System.currentTimeMillis() / 1000.0;
            intake.setPower(-IntakeConstants.intakePowerSlow);
        }

        @Override
        public void update() {
            intake.setPower(-IntakeConstants.intakePowerSlow);
        }

        @Override
        public boolean isDone() {
            return System.currentTimeMillis() / 1000.0 - shootStartTimeSec2
                    >= IntakeConstants.reverseTime;
        }

        @Override
        public void stop(boolean interrupted) {
            intake.setPower(IntakeConstants.zeroPower);
        }
    }.requires(this);

    public final Command zeroPower = new Command() {
        @Override
        public void start() {
            intake.setPower(IntakeConstants.zeroPower);
        }

        @Override
        public boolean isDone() {
            return true;
        }
    }.requires(this);

    private double shootStartTimeSec = 0.0;

    public final Command shoot = new Command() {
        @Override
        public void start() {
            shootStartTimeSec = System.currentTimeMillis() / 1000.0;
            intake.setPower(IntakeConstants.shootPower);
        }

        @Override
        public void update() {
            intake.setPower(IntakeConstants.shootPower);
        }

        @Override
        public boolean isDone() {
            return System.currentTimeMillis() / 1000.0 - shootStartTimeSec
                    >= IntakeConstants.shootTime;
        }

        @Override
        public void stop(boolean interrupted) {
            intake.setPower(IntakeConstants.zeroPower);
        }
    }.requires(this);

    public final Command shootEnd = new Command() {
        @Override
        public void start() {
            shootStartTimeSec = System.currentTimeMillis() / 1000.0;
            intake.setPower(IntakeConstants.shootPower);
        }

        @Override
        public void update() {
            intake.setPower(IntakeConstants.shootPower);
        }

        @Override
        public boolean isDone() {
            return System.currentTimeMillis() / 1000.0 - shootStartTimeSec
                    >= IntakeConstants.shootTimeEnd;
        }

        @Override
        public void stop(boolean interrupted) {
            intake.setPower(IntakeConstants.zeroPower);
        }
    }.requires(this);

    public final Command shootFirst = new Command() {
        @Override
        public void start() {
            shootStartTimeSec = System.currentTimeMillis() / 1000.0;
            intake.setPower(IntakeConstants.shootPower);
        }

        @Override
        public void update() {
            intake.setPower(IntakeConstants.shootPower);
        }

        @Override
        public boolean isDone() {
            return System.currentTimeMillis() / 1000.0 - shootStartTimeSec
                    >= IntakeConstants.shootTimeFirst;
        }

        @Override
        public void stop(boolean interrupted) {
            intake.setPower(IntakeConstants.zeroPower);
        }
    }.requires(this);

    public Command waitForBall(final LaserRangefinder cSensor) {
        return new Command() {
            private double startTimeSec = 0.0;

            @Override
            public void start() {
                startTimeSec = System.currentTimeMillis() / 1000.0;
                intake.setPower(IntakeConstants.intakePower);
            }

            @Override
            public void update() {
                intake.setPower(IntakeConstants.intakePower);
            }

            @Override
            public boolean isDone() {
                double d = cSensor.getDistance(DistanceUnit.MM);
                boolean detected = d >= 35 && d <= 45;
                boolean timedOut = System.currentTimeMillis() / 1000.0 - startTimeSec >= 1.0;
                return detected || timedOut;
            }

            @Override
            public void stop(boolean interrupted) {
                intake.setPower(IntakeConstants.zeroPower);
            }
        }.requires(this);
    }
}