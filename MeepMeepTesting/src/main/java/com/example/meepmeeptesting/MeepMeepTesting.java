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
                51.5,                  // y
                Math.toRadians(-45.0)  // heading
        );

        myBot.runAction(
                myBot.getDrive()
                        .actionBuilder(startPose)

                        // --- Path1: Pedro BezierLine, reversed ---
                        .setReversed(true)
                        .strafeTo(new Vector2d(-14.0, 13.0))

                        // --- Path2: Pedro BezierCurve, tangent heading ---
                        // Use splineTo with tangent heading (RR's default).
                        // End tangent ≈ 178.98° (almost straight left).
                        .setReversed(false)
                        .splineTo(
                                new Vector2d(-39.0, -12.0),
                                3.124  // radians
                        )

                        // --- Path3: Pedro BezierLine, reversed ---
                        .setReversed(true)
                        .strafeTo(new Vector2d(-14.0, 13.5))

                        // --- Path4: Pedro BezierCurve, tangent heading ---
                        // End tangent from cp→end ≈ 141.77°.
                        .setReversed(false)
                        .splineTo(
                                new Vector2d(-59.0, -13.0),
                                2.474  // radians
                        )

                        // --- Path5: Pedro BezierCurve, reversed, tangent heading ---
                        // End tangent from last cp→end ≈ 21.63°.
                        .setReversed(true)
                        .splineTo(
                                new Vector2d(-14.0, 13.0),
                                0.378  // radians
                        )

                        // --- Path6: Pedro BezierLine ---
                        .setReversed(false)
                        .strafeTo(new Vector2d(-39.0, 12.0))

                        // --- Path7: Pedro BezierLine, reversed ---
                        .setReversed(true)
                        .strafeTo(new Vector2d(-13.5, 13.0))

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