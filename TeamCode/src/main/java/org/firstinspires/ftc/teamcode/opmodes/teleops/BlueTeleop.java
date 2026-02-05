package org.firstinspires.ftc.teamcode.opmodes.teleops;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.IntakeSeqCmd;
import org.firstinspires.ftc.teamcode.commands.RapidFireCmd;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LaserRangefinder;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@Config
@TeleOp(name = "BlueTeleop")
public class BlueTeleop extends NextFTCOpMode {

    public BlueTeleop() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    public static double TRACKING_GAIN = 0.08;
    public static double SMOOTHING = 0.2;
    public static double DEADBAND = 3.0;
    public static boolean AUTO_TRACK_ENABLED = true;
    public static double NO_TARGET_TIMEOUT_SEC = 0.5;

    private final MotorEx frontLeftMotor  = new MotorEx("frontLeftMotor").brakeMode().reversed();
    private final MotorEx frontRightMotor = new MotorEx("frontRightMotor").brakeMode().reversed();
    private final MotorEx backLeftMotor   = new MotorEx("backLeftMotor").brakeMode();
    private final MotorEx backRightMotor  = new MotorEx("backRightMotor").brakeMode();

    private LaserRangefinder rangefinder;
    private int ballCount = 0;
    private boolean ballPresent = false;
    private long lastBallTime = 0;

    private static final double BALL_DISTANCE_MM = 30;
    private static final long DEBOUNCE_MS = 400;

    private boolean slowMode = false;
    private double driveScale = 1.0;

    private boolean shooterOn = false;
    private double hoodPos = ShooterConstants.servoPos;
    private double turretPos = 90.0;
    private double targetRpm = ShooterConstants.closeTargetRPM;

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private double motorTargetX = 90.0;
    private double smoothedTx = 0.0;
    private boolean hasSeenTarget = false;
    private long lastTargetSeenTime = 0;
    private boolean ballCountingEnabled = true;


    @Override
    public void onInit() {

        Shooter.INSTANCE.setHood(hoodPos).schedule();
        Shooter.INSTANCE.setTargetRPM(0.0);
        Shooter.INSTANCE.runRPM(0.0).schedule();

        Turret.INSTANCE.setTurretAngleDeg(90.0);
        Intake.INSTANCE.moveServoPos().schedule();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        RevColorSensorV3 sensor = hardwareMap.get(RevColorSensorV3.class, "range");
        rangefinder = new LaserRangefinder(sensor);
        rangefinder.setDistanceMode(LaserRangefinder.DistanceMode.SHORT);
        rangefinder.setTiming(20, 0);

        motorTargetX = 90.0;
        smoothedTx = 0.0;
        hasSeenTarget = false;
        lastTargetSeenTime = System.currentTimeMillis();

        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(530.423, 530.423, 447.938, 335.553)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "arducam"))
                .setCameraResolution(new Size(800, 600))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .build();

        dashboard.startCameraStream(visionPortal, 30);
    }

    @Override
    public void onStartButtonPressed() {
        Intake.INSTANCE.moveServoPos().schedule();

        var forward = Gamepads.gamepad1().leftStickY().negate().map(v -> v * driveScale);
        var strafe  = Gamepads.gamepad1().leftStickX().map(v -> v * driveScale);
        var rotate  = Gamepads.gamepad1().rightStickX().map(v -> v * driveScale);

        Command driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                forward,
                strafe,
                rotate
        );
        driverControlled.schedule();

        Gamepads.gamepad1().leftStickButton().whenBecomesTrue(() -> {
            slowMode = !slowMode;
            driveScale = slowMode ? 0.25 : 1.0;
        });

        Gamepads.gamepad1().leftBumper().whenBecomesTrue(() -> {
            Intake.INSTANCE.moveServoPos().schedule();
            IntakeSeqCmd.create().schedule();
        });

        Gamepads.gamepad1().leftBumper().whenBecomesFalse(() -> {
            Intake.INSTANCE.zeroPowerIntake().schedule();
            Intake.INSTANCE.zeroPowerTransfer().schedule();
        });

        Gamepads.gamepad1().b().whenBecomesTrue(() -> {
            ballCountingEnabled = false;

            Intake.INSTANCE.moveIntake(-IntakeConstants.intakePowerSlow).schedule();
            Intake.INSTANCE.moveTransfer(-IntakeConstants.intakePowerSlow).schedule();
            Intake.INSTANCE.defaultPos().schedule();
        });

        Gamepads.gamepad1().b().whenBecomesFalse(() -> {
            ballCountingEnabled = true;

            Intake.INSTANCE.zeroPowerIntake().schedule();
            Intake.INSTANCE.zeroPowerTransfer().schedule();
        });


        Gamepads.gamepad1().a().whenBecomesTrue(() -> {
            ballCount = 0;
            Intake.INSTANCE.defaultPos().schedule();
            RapidFireCmd.create().schedule();
        });

        Gamepads.gamepad1().a().whenBecomesFalse(() -> {
            Intake.INSTANCE.zeroPowerIntake().schedule();
            Intake.INSTANCE.zeroPowerTransfer().schedule();
        });

        Gamepads.gamepad1().rightBumper().whenBecomesTrue(() -> {
            shooterOn = !shooterOn;

            if (shooterOn) {
                Shooter.INSTANCE.runRPM(ShooterConstants.closeTargetRPM).schedule();
            } else {
                Shooter.INSTANCE.stop();
            }
        });

        Gamepads.gamepad1().dpadUp().whenBecomesTrue(() -> {
            hoodPos = hoodPos - 0.1;
            Shooter.INSTANCE.setHood(hoodPos).schedule();
        });

        Gamepads.gamepad1().dpadDown().whenBecomesTrue(() -> {
            hoodPos = hoodPos + 0.1;
            Shooter.INSTANCE.setHood(hoodPos).schedule();
        });

        Gamepads.gamepad2().dpadRight().whenBecomesTrue(() -> {
            turretPos = turretPos + 10;
            Turret.INSTANCE.goToAngle(turretPos).schedule();
        });

        Gamepads.gamepad2().dpadLeft().whenBecomesTrue(() -> {
            turretPos = turretPos - 10;
            Turret.INSTANCE.goToAngle(turretPos).schedule();
        });

        Gamepads.gamepad1().dpadRight().whenBecomesTrue(() -> {
            targetRpm = targetRpm + 100;
            if (Shooter.INSTANCE.getRPM() > 0) {
                Shooter.INSTANCE.setTargetRPM(targetRpm);
            }
        });

        Gamepads.gamepad1().dpadLeft().whenBecomesTrue(() -> {
            targetRpm = targetRpm - 100;
            if (Shooter.INSTANCE.getRPM() > 0) {
                Shooter.INSTANCE.setTargetRPM(targetRpm);
            }
        });
    }

    @Override
    public void onUpdate() {

        double distance = rangefinder.getDistance(DistanceUnit.MM);
        long now = System.currentTimeMillis();
        boolean detected = distance < BALL_DISTANCE_MM;

        if (ballCountingEnabled) {
            if (detected && !ballPresent && now - lastBallTime > DEBOUNCE_MS) {
                ballCount++;

                if (ballCount > 3) {
                    ballCount = 1;
                }

                if (ballCount == 3) {
                    Gamepads.gamepad1().getGamepad().invoke().rumble(500);
                }

                lastBallTime = now;
            }

            ballPresent = detected;
        }

        List<AprilTagDetection> detections = aprilTag.getDetections();
        boolean sawTargetThisLoop = false;

        if (!detections.isEmpty() && AUTO_TRACK_ENABLED) {
            for (AprilTagDetection tag : detections) {
                if (tag.id == 20) {
                    sawTargetThisLoop = true;
                    lastTargetSeenTime = System.currentTimeMillis();

                    double rawTx = 90.0 - tag.ftcPose.bearing;

                    if (!hasSeenTarget) {
                        smoothedTx = rawTx;
                    }

                    double error = rawTx - smoothedTx;
                    smoothedTx += error * SMOOTHING;

                    motorTargetX = smoothedTx;
                    break;
                }
            }
        }

        hasSeenTarget = sawTargetThisLoop;

        if (!hasSeenTarget &&
                System.currentTimeMillis() - lastTargetSeenTime > NO_TARGET_TIMEOUT_SEC * 1000) {

            motorTargetX = 90.0;
            smoothedTx = 90.0;
        }

        Turret.INSTANCE.setTurretAngleDeg(motorTargetX);

        telemetry.addData("Ball Count", ballCount);
        telemetry.addData("Range (mm)", distance);
        telemetry.addData("Turret Target", motorTargetX);
        telemetry.addData("Tag 20 Seen", hasSeenTarget);

        double rightRPM = Shooter.INSTANCE.ticksPerSecondToRPM(Math.abs(Shooter.INSTANCE.rightMotor.getVelocity()));
        double leftRPM = Shooter.INSTANCE.ticksPerSecondToRPM(Math.abs(Shooter.INSTANCE.leftMotor.getVelocity()));
        double currentRPM = (rightRPM + leftRPM) / 2.0;
        telemetry.addData("Current RPM", currentRPM);
        telemetry.addData("Hood Position", hoodPos);
        telemetry.addData("Current Turret Deg", Turret.INSTANCE.getTargetTurretDeg());

        telemetry.update();
    }
}
