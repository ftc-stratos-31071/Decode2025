package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.opmodes.autos.LaserRangefinder;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret2;
import org.firstinspires.ftc.teamcode.roadrunner.AutoMecanumDrive;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@Config
@TeleOp(name = "TestOp")
public class TestOp extends NextFTCOpMode {

    private static final double APRIL_TAG_HEADING = 225.0;
    private static final double STICK_DEADBAND = 0.05;

    private LaserRangefinder lrf;
    private AutoMecanumDrive drive;
    private Limelight3A limelight;

    private final MotorEx frontLeftMotor = new MotorEx("frontLeftMotor").brakeMode();
    private final MotorEx frontRightMotor = new MotorEx("frontRightMotor").brakeMode();
    private final MotorEx backLeftMotor = new MotorEx("backLeftMotor").brakeMode();
    private final MotorEx backRightMotor = new MotorEx("backRightMotor").brakeMode();

    private double fieldHeadingOffset = 0.0;
    private boolean offsetInitialized = false;
    private double motorTargetX = 0.0;

    public TestOp() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, Turret2.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        lrf = new LaserRangefinder(hardwareMap.get(RevColorSensorV3.class, "Color"));
        lrf.i2c.setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);

        drive = new AutoMecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.start();
            limelight.pipelineSwitch(0);
            FtcDashboard.getInstance().startCameraStream(limelight, 0);
        } catch (Exception e) {
            limelight = null;
        }

        Turret2.INSTANCE.setTargetDegrees(0.0);
    }

    @Override
    public void onStartButtonPressed() {
        new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
        ).schedule();
    }

    @Override
    public void onUpdate() {
        drive.updatePoseEstimate();

        double robotHeading =
                Math.toDegrees(drive.localizer.getPose().heading.toDouble());

        double lx = Gamepads.gamepad1().getGamepad().invoke().left_stick_x;
        double ly = Gamepads.gamepad1().getGamepad().invoke().left_stick_y;
        boolean leftStickActive =
                Math.abs(lx) > STICK_DEADBAND || Math.abs(ly) > STICK_DEADBAND;

        if (leftStickActive && limelight != null) {
            try {
                var result = limelight.getLatestResult();
                if (result != null && result.isValid()
                        && result.getFiducialResults() != null
                        && !result.getFiducialResults().isEmpty()) {

                    double tx = result.getTx();

                    fieldHeadingOffset =
                            APRIL_TAG_HEADING
                                    - robotHeading
                                    - tx;

                    offsetInitialized = true;
                }
            } catch (Exception ignored) {}
        }

        if (offsetInitialized) {
            motorTargetX =
                    APRIL_TAG_HEADING
                            - (robotHeading + fieldHeadingOffset);
        } else {
            motorTargetX = 0.0;
        }

        motorTargetX = Math.max(-90.0, Math.min(90.0, motorTargetX));
        Turret2.INSTANCE.setTargetDegrees(motorTargetX);

        telemetry.addData("Robot Heading", robotHeading);
        telemetry.addData("Field Offset", fieldHeadingOffset);
        telemetry.addData("Stick Active", leftStickActive);
        telemetry.addData("Turret Target", motorTargetX);
        telemetry.update();
    }
}