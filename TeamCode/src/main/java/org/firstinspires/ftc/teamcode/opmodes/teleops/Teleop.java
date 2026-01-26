package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Teleop")
public class Teleop extends NextFTCOpMode {

    public Teleop() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    private final MotorEx frontLeftMotor = new MotorEx("frontLeftMotor").brakeMode().reversed();
    private final MotorEx frontRightMotor = new MotorEx("frontRightMotor").brakeMode();
    private final MotorEx backLeftMotor = new MotorEx("backLeftMotor").brakeMode();
    private final MotorEx backRightMotor = new MotorEx("backRightMotor").brakeMode().reversed();
    private boolean slowMode = false;
    private double driveScale = 1.0;

    private boolean shooterOn = false;

    private Servo shooterServo;

    @Override
    public void onStartButtonPressed() {
        var rotate = Gamepads.gamepad1().leftStickY().negate().map(v -> v * driveScale); //needs to be fixed
        var strafe  = Gamepads.gamepad1().leftStickX().map(v -> v * driveScale);
        var forward  = Gamepads.gamepad1().rightStickX().map(v -> v * driveScale); //needs to be fixed
        shooterServo = hardwareMap.get(Servo.class, "hoodServo");
        shooterServo.setPosition(ShooterConstants.servoPos);

        Command driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                forward,
                strafe,
                rotate
        );
        driverControlled.schedule();

        Gamepads.gamepad1().leftStickButton().whenBecomesTrue(() -> {
            slowMode = !slowMode;
            driveScale = slowMode ? 0.25 : 1.0;
        });

        Gamepads.gamepad1().leftBumper().whenBecomesTrue(() -> {
            Intake.INSTANCE.moveIntake(IntakeConstants.intakePower).schedule();
        });

        Gamepads.gamepad1().leftBumper().whenBecomesFalse(() -> {
            Intake.INSTANCE.zeroPowerIntake().schedule();
        });

        Gamepads.gamepad1().b().whenBecomesTrue(() -> {
            Intake.INSTANCE.moveIntake(-IntakeConstants.intakePowerSlow).schedule();
            Intake.INSTANCE.moveTransfer(-IntakeConstants.intakePowerSlow).schedule();
        });

        Gamepads.gamepad1().b().whenBecomesFalse(() -> {
            Intake.INSTANCE.zeroPowerIntake().schedule();
            Intake.INSTANCE.zeroPowerTransfer().schedule();
        });

        Gamepads.gamepad1().a().whenBecomesTrue(() -> {
            Intake.INSTANCE.moveTransfer(IntakeConstants.shootPower).schedule();
        });

        Gamepads.gamepad1().a().whenBecomesFalse(() -> {
            Intake.INSTANCE.zeroPowerTransfer().schedule();
        });

        Gamepads.gamepad1().y().whenBecomesTrue(() -> {
            Intake.INSTANCE.moveIntake(IntakeConstants.intakePower).schedule();
            Intake.INSTANCE.moveTransfer(IntakeConstants.shootPower).schedule();
        });

        Gamepads.gamepad1().y().whenBecomesFalse(() -> {
            Intake.INSTANCE.zeroPowerIntake().schedule();
            Intake.INSTANCE.zeroPowerTransfer().schedule();
        });

        Gamepads.gamepad1().rightBumper().whenBecomesTrue(() -> {
            shooterOn = !shooterOn;

            if (shooterOn) {
                Shooter.INSTANCE.runRPM(ShooterConstants.closeTargetRPM).schedule();
            } else {
                Shooter.INSTANCE.stop();
            }
        });
        Gamepads.gamepad1().dpadUp().whenBecomesTrue(() -> {
            ShooterConstants.servoPos = ShooterConstants.servoPos + 0.1;
            shooterServo.setPosition(ShooterConstants.servoPos);
        });


        Gamepads.gamepad1().dpadDown().whenBecomesTrue(() -> {
            ShooterConstants.servoPos = ShooterConstants.servoPos - 0.1;
            shooterServo.setPosition(ShooterConstants.servoPos);
        });
    }
}