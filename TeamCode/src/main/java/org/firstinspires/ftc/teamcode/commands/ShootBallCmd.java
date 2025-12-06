package org.firstinspires.ftc.teamcode.commands;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;

public class ShootBallCmd {

    public static Command create() {
        return new SequentialGroup(
                Intake.INSTANCE.shoot,
                new Delay(ShooterConstants.shootDelaySeconds),
                Intake.INSTANCE.zeroPower
        );
    }
}
