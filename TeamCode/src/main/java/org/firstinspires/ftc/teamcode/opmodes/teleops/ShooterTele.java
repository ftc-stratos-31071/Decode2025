package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp
public class ShooterTele extends NextFTCOpMode {
    public ShooterTele() {
        addComponents(
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onStartButtonPressed() {
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(Shooter.INSTANCE.movePos);
        Gamepads.gamepad1().rightBumper().whenBecomesTrue(Shooter.INSTANCE.moveShooter);
    }
}