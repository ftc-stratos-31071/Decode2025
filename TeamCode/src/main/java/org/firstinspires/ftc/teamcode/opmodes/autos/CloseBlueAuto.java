package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.commands.ShootBallSteadyAutoCmd;
import org.firstinspires.ftc.teamcode.commands.StopDriveCmd;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants;
import org.firstinspires.ftc.teamcode.roadrunner.AutoMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.List;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Disabled
@Autonomous(name = "CloseBlueAuto", preselectTeleOp = "BlueTeleop")
public class CloseBlueAuto extends NextFTCOpMode {

    // =============================
    // Trajectory points (from MeepMeep)
    // =============================
    private static final Pose2d START_POSE = new Pose2d(
            -52.5,                 // x
            -51.5,                 // y (MIRRORED)
            Math.toRadians(230.0)  // heading (MIRRORED)
    );

    // =============================
    // Auto behavior config
    // =============================
    public static double AUTO_TARGET_RPM = 2750;
    public static double AUTO_HOOD_POS = 0.225;
    public static boolean STREAM_LIMELIGHT_TO_DASH = true;

    // Turret auto-tracking
    public static double TRACKING_GAIN = 0.08;
    public static double SMOOTHING = 0.7;
    public static double TURRET_LIMIT_DEG = 45.0;
    public static double DEADBAND = 3.0;
    public static boolean AUTO_TRACK_ENABLED = true;
    public static double NO_TARGET_TIMEOUT_SEC = 0.5;

    private AutoMecanumDrive drive;
    private Command autoCommand;

    private Limelight3A limelight;
    private double motorTargetX = 0.0;
    private double smoothedTx = 0.0;
    private boolean hasSeenTarget = false;
    private long lastTargetSeenTimeMs = 0;

    public CloseBlueAuto() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, Turret.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new AutoMecanumDrive(hardwareMap, START_POSE);
        hardStopAll();

        Shooter.INSTANCE.moveServo(AUTO_HOOD_POS).schedule();
        Shooter.INSTANCE.kickDefaultPos.schedule();

        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);
            limelight.start();
            limelight.pipelineSwitch(0);

            if (STREAM_LIMELIGHT_TO_DASH) {
                FtcDashboard.getInstance().startCameraStream(limelight, 0);
            }

            telemetry.addData("Limelight", "✓ Connected");
        } catch (Exception e) {
            limelight = null;
            telemetry.addData("Limelight", "✗ ERROR: " + e.getMessage());
        }

        motorTargetX = 0.0;
        smoothedTx = 0.0;
        hasSeenTarget = false;
        lastTargetSeenTimeMs = System.currentTimeMillis();

        autoCommand = drive.commandBuilder(START_POSE)
                .setReversed(true)
                .strafeTo(new Vector2d(-20.0, -16.0))

                .stopAndAdd(StopDriveCmd.create(drive))
                .stopAndAdd(Intake.INSTANCE.defaultPos())
                .stopAndAdd(ShootBallSteadyAutoCmd.create(IntakeConstants.shootPower, ShooterConstants.tolRpm2))
                .stopAndAdd(Intake.INSTANCE.moveServoPos())
                .stopAndAdd(Intake.INSTANCE.moveIntake(IntakeConstants.intakePower))

                .setReversed(false)
                .splineToLinearHeading(
                        new Pose2d(-16.0, -24.0, Math.toRadians(270.0)),
                        -Math.toRadians(360.0))

                .strafeTo(new Vector2d(-16.0, -56.0))
                .stopAndAdd(Intake.INSTANCE.zeroPowerIntake())
//                .strafeToSplineHeading(new Vector2d(-10.0, -66.0), Math.toRadians(180.0))

                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(-20.0, -16.0), Math.toRadians(225.0))

                .stopAndAdd(StopDriveCmd.create(drive))
                .stopAndAdd(Intake.INSTANCE.defaultPos())
                .stopAndAdd(ShootBallSteadyAutoCmd.create(IntakeConstants.shootPower, ShooterConstants.tolRpm2))
                .stopAndAdd(Intake.INSTANCE.moveServoPos())
                .stopAndAdd(Intake.INSTANCE.moveIntake(IntakeConstants.intakePower))

                .setReversed(false)
                .splineToSplineHeading(
                        new Pose2d(10.0, -20.0, Math.toRadians(270.0)),
                        -Math.toRadians(360.0))

                .strafeTo(new Vector2d(10.0, -56.0))
                .stopAndAdd(Intake.INSTANCE.zeroPowerIntake())

                .setReversed(true)
                .splineToSplineHeading(
                        new Pose2d(-20.0, -16.0, Math.toRadians(225.0)),
                        -Math.toRadians(240.0))

                .stopAndAdd(StopDriveCmd.create(drive))
                .stopAndAdd(Intake.INSTANCE.defaultPos())
                .stopAndAdd(ShootBallSteadyAutoCmd.create(IntakeConstants.shootPower, ShooterConstants.tolRpm2))
                .stopAndAdd(Intake.INSTANCE.moveServoPos())
                .stopAndAdd(Intake.INSTANCE.moveIntake(IntakeConstants.intakePower))

                .setReversed(false)
                .splineToSplineHeading(
                        new Pose2d(34.0, -24.0, Math.toRadians(270.0)),
                        -Math.toRadians(360.0))

                .strafeTo(new Vector2d(34.0, -56.0))
                .stopAndAdd(Intake.INSTANCE.zeroPowerIntake())

                .setReversed(true)
                .splineToSplineHeading(
                        new Pose2d(-22.0, -18.0, Math.toRadians(225.0)),
                        -Math.toRadians(225.0))

                .stopAndAdd(StopDriveCmd.create(drive))
                .stopAndAdd(Intake.INSTANCE.defaultPos())
                .stopAndAdd(ShootBallSteadyAutoCmd.create(IntakeConstants.shootPower, ShooterConstants.tolRpm2))

                .setReversed(false)
                .strafeTo(new Vector2d(-12.0, -40.0))
                .stopAndAdd(StopDriveCmd.create(drive))
                .build();

        telemetry.addData("Status", "Initialized (HARD STOP applied)");
        telemetry.addData("Auto", "Ready - press START");
        telemetry.update();
    }

    @Override
    public void onWaitForStart() {
        hardStopMotorsOnly();
        telemetry.addData("Safety", "Holding all motors at 0 during INIT");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        Intake.INSTANCE.moveServoPos().schedule();

        Turret.INSTANCE.turret.zeroed();
        Turret.INSTANCE.setTargetDegrees(0.0);

        Shooter.INSTANCE.setTargetRPM(AUTO_TARGET_RPM);

        motorTargetX = 0.0;
        smoothedTx = 0.0;
        hasSeenTarget = false;
        lastTargetSeenTimeMs = System.currentTimeMillis();

        autoCommand.schedule();
        Turret.INSTANCE.setTargetDegrees(0.0);
    }

    @Override
    public void onUpdate() {
        updateTurretTracking();

        telemetry.addData("Shooter Target RPM", Shooter.INSTANCE.getTargetRPM());
        telemetry.addData("Shooter RPM", String.format("%.0f", Shooter.INSTANCE.getRPM()));
        telemetry.addData("Turret Target (deg)", String.format("%.2f", motorTargetX));
        telemetry.update();
    }

    @Override
    public void onStop() {
        hardStopAll();
        try {
            if (limelight != null) limelight.stop();
        } catch (Exception ignored) { }
    }

    private void hardStopAll() {
        Shooter.INSTANCE.stopShooter();
        Intake.INSTANCE.zeroPowerIntake().schedule();

        Turret.INSTANCE.disableManualControl();
        Turret.INSTANCE.turret.setPower(0.0);
        motorTargetX = 0.0;
        smoothedTx = 0.0;

        if (drive != null) {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
        }
    }

    private void hardStopMotorsOnly() {
        Shooter.INSTANCE.stopShooter();
        Intake.INSTANCE.zeroPowerIntake().schedule();

        Turret.INSTANCE.disableManualControl();
        Turret.INSTANCE.turret.setPower(0.0);

        if (drive != null) {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
        }
    }

    private void updateTurretTracking() {
        LLResult result = null;
        if (limelight != null) {
            try {
                result = limelight.getLatestResult();
            } catch (Exception e) {
                telemetry.addData("Limelight Error", e.getMessage());
            }
        }

        if (AUTO_TRACK_ENABLED && result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            if (fiducials != null && !fiducials.isEmpty()) {
                LLResultTypes.FiducialResult closestTag = null;
                double largestArea = 0.0;

                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    double targetArea = fiducial.getTargetXPixels() * fiducial.getTargetYPixels();
                    if (closestTag == null || targetArea > largestArea) {
                        closestTag = fiducial;
                        largestArea = targetArea;
                    }
                }

                if (closestTag != null) {
                    hasSeenTarget = true;
                    lastTargetSeenTimeMs = System.currentTimeMillis();

                    double tx = result.getTx();
                    smoothedTx = SMOOTHING * smoothedTx + (1.0 - SMOOTHING) * tx;

                    if (Math.abs(smoothedTx) > DEADBAND) {
                        double adjustment = smoothedTx * TRACKING_GAIN;
                        if (Math.abs(smoothedTx) < 10.0) adjustment *= 0.5;

                        motorTargetX += adjustment;
                        motorTargetX = Math.max(-TURRET_LIMIT_DEG, Math.min(TURRET_LIMIT_DEG, motorTargetX));
                    }

                    telemetry.addData("Tag", closestTag.getFiducialId());
                    telemetry.addData("TX raw", String.format("%.2f", tx));
                    telemetry.addData("TX smooth", String.format("%.2f", smoothedTx));
                }
            } else {
                telemetry.addData("Tag", "None");
                if (!hasSeenTarget) {
                    motorTargetX = 0.0;
                    smoothedTx = 0.0;
                }
            }
        } else {
            telemetry.addData("Tag", "No valid target");
            if (!hasSeenTarget) {
                motorTargetX = 0.0;
                smoothedTx = 0.0;
            }
        }

        if (System.currentTimeMillis() - lastTargetSeenTimeMs >
                (long) (NO_TARGET_TIMEOUT_SEC * 1000.0)) {
            motorTargetX = 0.0;
            smoothedTx = 0.0;
            hasSeenTarget = false;
        }

        Turret.INSTANCE.setTargetDegrees(motorTargetX);
    }
}
