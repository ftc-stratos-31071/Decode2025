package org.firstinspires.ftc.teamcode.constants;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.localization.Localizer;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.Drivetrain;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class PedroConstants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(6.9)
            .forwardZeroPowerAcceleration(-37.50473470344061)
            .lateralZeroPowerAcceleration(-69.9432918300914)
            .useSecondaryDrivePIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryTranslationalPIDF(false)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.05,
                    0,
                    0.0001,
                    0.03
            ));
//            .translationalPIDFSwitch(4)
//            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(
//                    0.4,
//                    0,
//                    0.005,
//                    0.0006
//            ))
//            .headingPIDFCoefficients(new PIDFCoefficients(
//                    0.8,
//                    0,
//                    0,
//                    0.01
//            ))
//            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(
//                    2.5,
//                    0,
//                    0.1,
//                    0.0005
//            ))
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
//                    0.1,
//                    0,
//                    0.00035,
//                    0.6,
//                    0.015
//            ))
//            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(
//                    0.02,
//                    0,
//                    0.000005,
//                    0.6,
//                    0.01
//            ))
//            .drivePIDFSwitch(15)
//            .centripetalScaling(0.0005);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRightMotor")
            .rightRearMotorName("backRightMotor")
            .leftRearMotorName("backLeftMotor")
            .leftFrontMotorName("frontLeftMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(80.48367285540724)
            .yVelocity(49.252003346841164);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-3.4252)
            .strafePodX(-6.18996063)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);
    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}