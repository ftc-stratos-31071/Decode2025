package org.firstinspires.ftc.teamcode.commands;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class ShooterOffCmd {

    public static Command create() {
        return new ParallelGroup(
                Intake.INSTANCE.zeroPower,
                Shooter.INSTANCE.zeroPower
        );
    }
}
