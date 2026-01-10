package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.IntakeSeqCmd;
import org.firstinspires.ftc.teamcode.commands.ShootBallCmd;
import org.firstinspires.ftc.teamcode.commands.ShootBallCont;
import org.firstinspires.ftc.teamcode.commands.StopDriveCmd;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
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

@Autonomous(name = "FarRedAuto")
public class FarRedAuto extends NextFTCOpMode {

    // =============================
    // Trajectory points (from MeepMeep)
    // =============================
    private static final Pose2d START_POSE = new Pose2d(
            62.5,                  // x
            13.0,                  // y
            Math.toRadians(-180.0) // heading
    );

    // =============================
    // Auto behavior config
    // =============================
    public static double AUTO_TARGET_RPM = 4000.0;     // shooter runs ALL the time (after START)
    public static double AUTO_HOOD_POS = 0.2;          // hood position set on START
    public static boolean STREAM_LIMELIGHT_TO_DASH = true;

    // Turret auto-tracking
    public static double TRACKING_GAIN = 0.08;
    public static double SMOOTHING = 0.7;
    public static double TURRET_LIMIT_DEG = 90.0;
    public static double DEADBAND = 3.0;
    public static boolean AUTO_TRACK_ENABLED = true;
    public static double NO_TARGET_TIMEOUT_SEC = 0.5;

    // =============================
    // NextFTC + RR objects
    // =============================
    private AutoMecanumDrive drive;
    private Command autoCommand;

    private LaserRangefinder lrf;

    // =============================
    // Limelight + turret tracking state
    // =============================
    private Limelight3A limelight;
    private double motorTargetX = 0.0;
    private double smoothedTx = 0.0;
    private boolean hasSeenTarget = false;
    private long lastTargetSeenTimeMs = 0;

    public FarRedAuto() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, Turret.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        lrf = new LaserRangefinder(hardwareMap.get(RevColorSensorV3.class, "Color"));
        lrf.i2c.setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);

        // Build drive + command on init, but DO NOT move mechanisms yet.
        drive = new AutoMecanumDrive(hardwareMap, START_POSE);

        // HARD STOP EVERYTHING immediately on init (prevents leftover state from a prior OpMode)
        hardStopAll();

        Intake.INSTANCE.moveServoPos().schedule();
        Shooter.INSTANCE.moveServo(AUTO_HOOD_POS).schedule();
        Shooter.INSTANCE.kickDefaultPos.schedule();

        Turret.INSTANCE.turret.zeroed();
        Turret.INSTANCE.setTargetDegrees(30);

        // ===== init Limelight (camera can run on init without moving hardware) =====
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);
            limelight.start();
            limelight.pipelineSwitch(1);

            if (STREAM_LIMELIGHT_TO_DASH) {
                FtcDashboard.getInstance().startCameraStream(limelight, 0);
            }

            telemetry.addData("Limelight", "✓ Connected");
        } catch (Exception e) {
            limelight = null;
            telemetry.addData("Limelight", "✗ ERROR: " + e.getMessage());
        }

        // Reset tracking state (we will start tracking AFTER START)
        motorTargetX = 0.0;
        smoothedTx = 0.0;
        hasSeenTarget = false;
        lastTargetSeenTimeMs = System.currentTimeMillis();

        // Build autonomous command (same path + stopAndAdd actions)
        autoCommand = drive.commandBuilder(START_POSE)
                .stopAndAdd(ShootBallCont.create())
                .splineToLinearHeading(new Pose2d(36.0, 40.0, Math.toRadians(-270.0)), Math.toRadians(-260.0))
//                .strafeTo(new Vector2d(36.0, 56.0))
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(62.5, 13.0, Math.toRadians(-180.0)), Math.toRadians(-30.0))
//                .waitSeconds(0.5)
//                .setReversed(false)
//                .splineToLinearHeading(new Pose2d(12.0, 40.0, Math.toRadians(-270.0)), Math.toRadians(-260.0))
//                .strafeTo(new Vector2d(12.0, 56.0))
//                .waitSeconds(0.5)
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(62.5, 13.0, Math.toRadians(-180.0)), Math.toRadians(-30.0))
//                .waitSeconds(0.5)
//                .strafeTo(new Vector2d(62.5, 35.0))
//                .stopAndAdd(StopDriveCmd.create(drive))
                .build();

        telemetry.addData("Status", "Initialized (HARD STOP applied)");
        telemetry.addData("Auto", "Ready - press START");
        telemetry.update();
    }

    /**
     * Runs every loop while sitting on INIT (before START).
     * This is the best place to prevent “glitch running” from old commands/state.
     */
    @Override
    public void onWaitForStart() {
        hardStopMotorsOnly(); // keep everything OFF while waiting
        telemetry.addData("Safety", "Holding all motors at 0 during INIT");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        // Start shooter immediately and keep running all auto
        Shooter.INSTANCE.setTargetRPM(AUTO_TARGET_RPM);

        // Reset tracking state and begin tracking after start
        motorTargetX = 0.0;
        smoothedTx = 0.0;
        hasSeenTarget = false;
        lastTargetSeenTimeMs = System.currentTimeMillis();

        // Run auto
        autoCommand.schedule();
    }

    @Override
    public void onUpdate() {
        // Always track during the match
        updateTurretTracking();

        telemetry.addData("Shooter Target RPM", Shooter.INSTANCE.getTargetRPM());
        telemetry.addData("Shooter RPM", String.format("%.0f", Shooter.INSTANCE.getRPM()));
        telemetry.addData("Turret Target (deg)", String.format("%.2f", motorTargetX));
        telemetry.update();
    }

    @Override
    public void onStop() {
        // HARD STOP at end of OpMode too
        hardStopAll();

        try {
            if (limelight != null) limelight.stop();
        } catch (Exception ignored) { }
    }

    // ==========================================================
    // SAFETY STOP HELPERS
    // ==========================================================

    /** Stop motors AND reset control loops. Safe to call repeatedly. */
    private void hardStopAll() {
        // Shooter: hard stop (sets power 0 directly + disables PIDF)
        Shooter.INSTANCE.stopShooter();

        // Intake: stop
        Intake.INSTANCE.zeroPower().schedule();

        // Turret: stop and disable manual tracking
        Turret.INSTANCE.disableManualControl();
        Turret.INSTANCE.turret.setPower(0.0);
        motorTargetX = 0.0;
        smoothedTx = 0.0;

        // Drive: stop
        if (drive != null) {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
        }
    }

    /** Keep motors off during INIT without constantly resetting servo positions. */
    private void hardStopMotorsOnly() {
        // Shooter motors off + PIDF disabled
        Shooter.INSTANCE.stopShooter();

        // Intake motor off
        Intake.INSTANCE.zeroPower().schedule();

        // Turret motor off (no tracking while in INIT)
        Turret.INSTANCE.disableManualControl();
        Turret.INSTANCE.turret.setPower(0.0);

        // Drive motors off
        if (drive != null) {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
        }
    }

    // ==========================================================
    // Limelight -> turret tracking (active AFTER START)
    // ==========================================================
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

        if (System.currentTimeMillis() - lastTargetSeenTimeMs > (long) (NO_TARGET_TIMEOUT_SEC * 1000.0)) {
            motorTargetX = 0.0;
            smoothedTx = 0.0;
            hasSeenTarget = false;
        }

        Turret.INSTANCE.setTargetDegrees(motorTargetX);
    }
}

