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

//        Pose2d startPose = new Pose2d(
//                -52.5,                  // x
//                -51.5,                  // y
//                Math.toRadians(230.0) // heading
//        );

//        Pose2d startPose = new Pose2d(
//                -52.5,                  // x
//                51.5,                  // y
//                Math.toRadians(-230.0) // heading
//        );

                Pose2d startPose = new Pose2d(
                62.5,                  // x
                -13.0,                  // y
                Math.toRadians(230.0) // heading
        );



        myBot.runAction(
                myBot.getDrive()
                        .actionBuilder(startPose)
//                        .setReversed(true)
//                        .strafeTo(new Vector2d(-20.0, -16.0))
//                        .setReversed(false)
//                        .splineToLinearHeading(
//                                new Pose2d(-16.0, -30.0, Math.toRadians(270.0)),
//                                -Math.toRadians(360.0))
//                        .strafeTo(new Vector2d(-16.0, -60.0))
//                        .strafeToSplineHeading(new Vector2d(-3.0, -57.0), Math.toRadians(180.0))
//                        .setReversed(true)
//                        .strafeToLinearHeading(new Vector2d(-20.0, -16.0), Math.toRadians(225.0))
//                        .setReversed(false)
//                        .splineToSplineHeading(
//                                new Pose2d(8.0, -30.0, Math.toRadians(270.0)),
//                                -Math.toRadians(360.0))
//                        .strafeTo(new Vector2d(8.0, -60.0))
//                        .setReversed(true)
//                        .splineToSplineHeading(
//                                new Pose2d(-20.0, -16.0, Math.toRadians(225.0)),
//                                -Math.toRadians(240.0))
//                        .setReversed(false)
//                        .splineToSplineHeading(
//                                new Pose2d(32.0, -30.0, Math.toRadians(270.0)),
//                                -Math.toRadians(360.0))
//                        .strafeTo(new Vector2d(32.0, -60.0))
//                        .setReversed(true)
//                        .splineToSplineHeading(
//                                new Pose2d(-20.0, -16.0, Math.toRadians(225.0)),
//                                -Math.toRadians(225.0))
//                        .setReversed(false)
//                        .strafeTo(new Vector2d(-12.0, -40.0))

//                        .setReversed(true)
//                        .strafeTo(new Vector2d(-20.0, 16.0))
//                        .setReversed(false)
//                        .splineToLinearHeading(new Pose2d(-16.0, 30.0, Math.toRadians(-270.0)), Math.toRadians(-300.0))
//                        .strafeTo(new Vector2d(-16.0, 60.0))
//                        .strafeToSplineHeading(new Vector2d(-3.0, 57.0), Math.toRadians(-180.0))
//                        .setReversed(true)
//                        .strafeToLinearHeading(new Vector2d(-20.0, 16.0), Math.toRadians(-225.0))
//                        .setReversed(false)
//                        .splineToSplineHeading(new Pose2d(8.0, 30.0, Math.toRadians(-270.0)), Math.toRadians(-360.0))
//                        .strafeTo(new Vector2d(8.0, 60.0))
//                        .setReversed(true)
//                        .splineToSplineHeading(new Pose2d(-20.0, 16.0, Math.toRadians(-225.0)), -Math.toRadians(-240.0))
//                        .setReversed(false)
//                        .splineToSplineHeading(new Pose2d(32.0, 30.0, Math.toRadians(-270.0)), Math.toRadians(-360.0))
//                        .strafeTo(new Vector2d(32.0, 60.0))
//                        .setReversed(true)
//                        .splineToSplineHeading(new Pose2d(-20.0, 16.0, Math.toRadians(-225.0)), -Math.toRadians(-225.0))
//                        .setReversed(false)
//                        .strafeTo(new Vector2d(-12.0, 40.0))

                        .strafeTo(new Vector2d(62.0, -13.0))
                        .strafeToSplineHeading(
                                new Vector2d(51.0, -70.5),
                                Math.toRadians(270.0))
                        .turnTo(Math.toRadians(295.0))
                        .strafeToLinearHeading(
                                new Vector2d(61.5, -10.5),
                                Math.toRadians(180.0))
                        .strafeTo(new Vector2d(61.5, -45.0))
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