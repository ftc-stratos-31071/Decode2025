package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.roadrunner.AutoMecanumDrive;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;

public class StopDriveCmd {
    public static Command create(AutoMecanumDrive drive) {
        return new InstantCommand("StopDrive", () ->
                drive.setDrivePowers(
                        new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0)
                )
        );
    }
}