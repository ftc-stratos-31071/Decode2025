package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "Teleop")
public class Teleop extends NextFTCOpMode {

    public Teleop() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    private final MotorEx frontLeftMotor = new MotorEx("frontLeftMotor").brakeMode();
    private final MotorEx frontRightMotor = new MotorEx("frontRightMotor").brakeMode();
    private final MotorEx backLeftMotor = new MotorEx("backLeftMotor").brakeMode().reversed();
    private final MotorEx backRightMotor = new MotorEx("backRightMotor").brakeMode().reversed();
    private boolean slowMode = false;
    private double driveScale = 1.0;

    @Override
    public void onStartButtonPressed() {
        var forward = Gamepads.gamepad1().leftStickY().map(v -> v * driveScale);
        var strafe  = Gamepads.gamepad1().leftStickX().negate().map(v -> v * driveScale);
        var rotate  = Gamepads.gamepad1().rightStickX().negate().map(v -> v * driveScale);

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

        Gamepads.gamepad1().a().whenBecomesTrue(() -> {
            Intake.INSTANCE.moveIntake(IntakeConstants.intakePower).schedule();
            Intake.INSTANCE.moveTransfer(IntakeConstants.shootPower).schedule();
        });

        Gamepads.gamepad1().a().whenBecomesFalse(() -> {
            Intake.INSTANCE.zeroPowerIntake().schedule();
            Intake.INSTANCE.zeroPowerTransfer().schedule();
        });

        Gamepads.gamepad1().b().whenBecomesTrue(() -> {
            Intake.INSTANCE.moveIntake(-IntakeConstants.intakePowerSlow).schedule();
            Intake.INSTANCE.moveTransfer(-IntakeConstants.intakePowerSlow).schedule();
        });

        Gamepads.gamepad1().b().whenBecomesFalse(() -> {
            Intake.INSTANCE.zeroPowerIntake().schedule();
            Intake.INSTANCE.zeroPowerTransfer().schedule();
        });

        Gamepads.gamepad1().rightBumper().whenBecomesTrue(() -> {
            Intake.INSTANCE.moveTransfer(IntakeConstants.shootPower).schedule();
        });

        Gamepads.gamepad1().rightBumper().whenBecomesFalse(() -> {
            Intake.INSTANCE.zeroPowerTransfer().schedule();
        });
    }
}