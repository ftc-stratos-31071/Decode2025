package org.firstinspires.ftc.teamcode.opmodes.teleops;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
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

@TeleOp(name = "BlueTeleop")
public class BlueTeleop extends NextFTCOpMode {

    /* ====================== COMPONENT SETUP ====================== */

    public BlueTeleop() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    /* ====================== TRACKING TUNABLES ====================== */

    public static double TRACKING_GAIN = 0.08;
    public static double SMOOTHING = 0.5;
    public static double DEADBAND = 3.0;
    public static boolean AUTO_TRACK_ENABLED = true;
    public static double NO_TARGET_TIMEOUT_SEC = 0.5;
    public static boolean SHOW_VISION_TELEMETRY = false;

    /* ====================== DRIVE ====================== */

    private final MotorEx frontLeftMotor  = new MotorEx("frontLeftMotor").brakeMode().reversed();
    private final MotorEx frontRightMotor = new MotorEx("frontRightMotor").brakeMode().reversed();
    private final MotorEx backLeftMotor   = new MotorEx("backLeftMotor").brakeMode();
    private final MotorEx backRightMotor  = new MotorEx("backRightMotor").brakeMode();

    /* ====================== SUBSYSTEM STATE ====================== */

    private LaserRangefinder rangefinder;
    private int ballCount = 0;
    private boolean ballPresent = false;
    private long lastBallTime = 0;

    private static final double BALL_DISTANCE_MM = 40;
    private static final long DEBOUNCE_MS = 300;

    private boolean intakeActive = false;
    private boolean hasRumbled = false;

    private boolean slowMode = false;
    private double driveScale = 1.0;

    private boolean shooterOn = false;
    private double hoodPos = ShooterConstants.servoPos;
    private double turretPos = 180.0;
    private double targetRpm = ShooterConstants.closeTargetRPM;

    /* ====================== APRILTAG VISION ====================== */

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    private double motorTargetX = 180.0;
    private double smoothedTx = 0.0;
    private boolean hasSeenTarget = false;
    private long lastTargetSeenTime = 0;

    /* ====================== INIT ====================== */

    @Override
    public void onInit() {

        Shooter.INSTANCE.setHood(hoodPos).schedule();
        Shooter.INSTANCE.setTargetRPM(0.0);
        Shooter.INSTANCE.runRPM(0.0).schedule();

        Turret.INSTANCE.goToAngle(180.0).schedule();
        Intake.INSTANCE.moveServoPos().schedule();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        RevColorSensorV3 sensor = hardwareMap.get(RevColorSensorV3.class, "range");
        rangefinder = new LaserRangefinder(sensor);
        rangefinder.setDistanceMode(LaserRangefinder.DistanceMode.SHORT);
        rangefinder.setTiming(20, 0);

        motorTargetX = 180.0;
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

        telemetry.addData("Vision", "✓ AprilTag Ready (ONLY ID 20)");
        telemetry.update();
    }

    /* ====================== START ====================== */

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
            intakeActive = true;
            hasRumbled = false;
            ballCount = 0;
            Intake.INSTANCE.moveServoPos().schedule();
            IntakeSeqCmd.create().schedule();
        });

        Gamepads.gamepad1().leftBumper().whenBecomesFalse(() -> {
            intakeActive = false;
            ballPresent = false;
            Intake.INSTANCE.zeroPowerIntake().schedule();
            Intake.INSTANCE.zeroPowerTransfer().schedule();
        });

        Gamepads.gamepad1().b().whenBecomesTrue(() -> {
            Intake.INSTANCE.moveIntake(-IntakeConstants.intakePowerSlow).schedule();
            Intake.INSTANCE.moveTransfer(-IntakeConstants.intakePowerSlow).schedule();
            Intake.INSTANCE.defaultPos().schedule();
        });

        Gamepads.gamepad1().b().whenBecomesFalse(() -> {
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

    /* ====================== UPDATE LOOP ====================== */

    @Override
    public void onUpdate() {

        /* ---------- Ball Counting ---------- */

        double distance = rangefinder.getDistance(DistanceUnit.MM);
        long now = System.currentTimeMillis();
        boolean detected = distance < BALL_DISTANCE_MM;

        if (intakeActive && detected && !ballPresent && now - lastBallTime > DEBOUNCE_MS) {
            ballCount++;
            lastBallTime = now;
        }

        ballPresent = detected;

        if (ballCount >= 3 && !hasRumbled) {
            Gamepads.gamepad1().getGamepad().invoke().rumble(500);
            hasRumbled = true;
        }

        /* ---------- AprilTag Tracking (ID 20 ONLY) ---------- */
        List<AprilTagDetection> detections = aprilTag.getDetections();
        hasSeenTarget = false;

        if (!detections.isEmpty() && AUTO_TRACK_ENABLED) {
            for (AprilTagDetection tag : detections) {
                if (tag.id == 20) {
                    hasSeenTarget = true;
                    lastTargetSeenTime = System.currentTimeMillis(); // update last seen time

                    // Compute target turret angle relative to center (180°)
                    double rawTx = 180.0 + tag.ftcPose.x;

                    // Apply deadband and smoothing
                    if (Math.abs(rawTx - motorTargetX) > DEADBAND) {
                        smoothedTx = smoothedTx + (rawTx - smoothedTx) * SMOOTHING;
                        motorTargetX = smoothedTx;
                        Turret.INSTANCE.goToAngle(motorTargetX).schedule();
                    }

                    break; // only track tag 20
                }
            }
        } else {
            // If no tag seen recently, optionally return to default after timeout
            if (System.currentTimeMillis() - lastTargetSeenTime > NO_TARGET_TIMEOUT_SEC * 1000) {
                motorTargetX = 180.0;  // middle
                Turret.INSTANCE.goToAngle(motorTargetX).schedule();
            }
        }
        /* ---------- Telemetry ---------- */

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
