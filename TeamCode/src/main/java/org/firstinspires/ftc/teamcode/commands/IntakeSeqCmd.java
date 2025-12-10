package org.firstinspires.ftc.teamcode.commands;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeSeqCmd {

    public static Command create() {
        return new SequentialGroup(
                Intake.INSTANCE.moveServoPos,
                new ParallelGroup(
                        Intake.INSTANCE.turnOn
                )
        );
    }
}
