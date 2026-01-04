package org.firstinspires.ftc.teamcode.commands;


import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * Reset robot to known safe defaults.
 * Mirrors Teleop.onStartButtonPressed() initialization behavior.
 */
public class RobotResetCmd {

    public static Command create() {

        // 1) Do subsystem "direct calls" in a tiny command
        Command turretZero = new Command() {
            @Override
            public void start() {
                // Matches TeleOp start behavior
                Turret.INSTANCE.turret.zeroed();

                // If you have these methods, they’re good to include:
                // Turret.INSTANCE.disableManualControl();
                // Turret.INSTANCE.setTargetDegrees(0.0);
            }

            @Override
            public boolean isDone() {
                return true;
            }
        }.requires(Turret.INSTANCE);

        // 2) Use existing Commands you already have for the rest
        return new SequentialGroup(
                // Stop things first (safe)
                Intake.INSTANCE.zeroPower(),
                Shooter.INSTANCE.zeroPower,

                // Servos to known positions (matches TeleOp)
                Intake.INSTANCE.defaultPos(),
                Shooter.INSTANCE.moveServo(0.1),       // hood
                Shooter.INSTANCE.kickDefaultPos,       // kicker

                // Turret zero last (or first—either is fine)
                turretZero
        ).named("RobotReset");
    }
}

