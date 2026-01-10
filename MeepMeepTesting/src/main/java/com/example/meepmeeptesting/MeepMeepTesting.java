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

        Pose2d startPose = new Pose2d(
                62.5,                  // x
                13.0,                  // y
                Math.toRadians(-180.0) // heading
        );

        myBot.runAction(
                myBot.getDrive()
                        .actionBuilder(startPose)
                        .strafeTo(new Vector2d(62.0, 13.0))
//                        .splineTo(new Vector2d(60, 70.0), Math.toRadians(180))
                        .strafeToSplineHeading(new Vector2d(56.5, 71.0), Math.toRadians(-260.0))
                        .turnTo(Math.toRadians(-300.0))
//                        .strafeToLinearHeading(new Vector2d(62.5, 12.0), Math.toRadians(-180.0))
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