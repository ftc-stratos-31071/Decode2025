package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.IntakeSeqCmd;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(name = "ReadDistanceI2C", group = "Testing")
public class ReadDistanceI2C extends NextFTCOpMode {

    private LaserRangefinder lrf;

    public ReadDistanceI2C() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        lrf = new LaserRangefinder(hardwareMap.get(RevColorSensorV3.class, "Color"));
        lrf.i2c.setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
    }

    @Override
    public void onStartButtonPressed() {
        Intake.INSTANCE.defaultPos.schedule();
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(() -> {
            IntakeSeqCmd.create().schedule();
        });
        Gamepads.gamepad1().leftBumper().whenBecomesFalse(() -> {
            Intake.INSTANCE.zeroPower.schedule();
        });
    }

    @Override
    public void onUpdate() {
        double distance = lrf.getDistance(DistanceUnit.MM);

        telemetry.addData("Distance", distance);
        telemetry.update();

        if (distance >= 30 && distance <= 40) {
            Intake.INSTANCE.zeroPower.schedule();
        }
    }
}