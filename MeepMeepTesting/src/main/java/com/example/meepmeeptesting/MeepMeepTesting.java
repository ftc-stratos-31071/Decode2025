package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // maxVel, maxAccel, maxAngVel, maxAngAccel, trackWidth
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        // Start pose = Path1 start, heading tangent along Path1 (approx -45°)
        Pose2d startPose = new Pose2d(
                -52.5,                 // x
                -51.5,                  // y
                Math.toRadians(-124)  // heading
        );

        myBot.runAction(
                myBot.getDrive()
                        .actionBuilder(startPose)

                        // --- score preload ---
                        .setReversed(true)
                        .strafeTo(new Vector2d(-14.0, -13.0))

//                        // --- pick up artifacts line 1 ---
                        .setReversed(false)
                        .splineTo(
                                new Vector2d(-12.0, -54.0),
                                Math.toRadians(270)  // radians
                        )
//
//                        // --- back to shoot position ---
                        .setReversed(true)
//                        .splineTo(new Vector2d(-14.0, -13),
//                                Math.toRadians(45) )
                        .splineToLinearHeading(new Pose2d(-14.0, -13.0, Math.toRadians(225)),
                                Math.toRadians(90) )
//
//                        // --- pick up line 2---
                        .setReversed(false)
                        .splineTo(
                                new Vector2d(12, -48.0),
                                Math.toRadians(270)  // radians
                        )
//
                        // --- return to score ---
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(-14.0, -13.0, Math.toRadians(225)),
                                Math.toRadians(90) )
//                        )


//
//
//                        // --- Path7: Pedro BezierLine, reversed ---
//                        .setReversed(true)
//                        .strafeTo(new Vector2d(-13.5, 13.0))

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