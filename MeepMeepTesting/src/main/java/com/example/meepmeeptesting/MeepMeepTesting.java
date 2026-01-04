package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // maxVel, maxAccel, maxAngVel, maxAngAccel, trackWidth
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        // Start pose = Path1 start, heading tangent along Path1 (approx -45°)
        Pose2d startPose = new Pose2d(
                62.5,                 // x
                13.0,                  // y
                Math.toRadians(-180.0)  // heading
        );

        myBot.runAction(
                myBot.getDrive()
                        .actionBuilder(startPose)
                        .splineToLinearHeading(new Pose2d(36.0, 40.0, Math.toRadians(-270.0)), Math.toRadians(-260.0))
                        .strafeTo(new Vector2d(36.0, 56.0))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(62.5, 13.0, Math.toRadians(-180.0)), Math.toRadians(-30.0))
                        .setReversed(false)
                        .splineToLinearHeading(new Pose2d(12.0, 40.0, Math.toRadians(-270.0)), Math.toRadians(-260.0))
                        .strafeTo(new Vector2d(12.0, 56.0))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(62.5, 13.0, Math.toRadians(-180.0)), Math.toRadians(-30.0))
                        .strafeTo(new Vector2d(62.5, 35.0))
                        .build()
        );

        meepMeep
                .setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}