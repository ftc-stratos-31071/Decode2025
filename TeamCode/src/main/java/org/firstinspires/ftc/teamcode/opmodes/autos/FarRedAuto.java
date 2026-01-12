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

import org.firstinspires.ftc.teamcode.commands.AutoSlowDriveForwardTimeoutCmd;
import org.firstinspires.ftc.teamcode.commands.ShootBallSteadyAutoCmd;
import org.firstinspires.ftc.teamcode.commands.ShootBallSteadyAutoTimeoutCmd;
import org.firstinspires.ftc.teamcode.commands.StopDriveCmd;
import org.firstinspires.ftc.teamcode.commands.WaitCmd;
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

@Autonomous(name = "FarRedAuto", preselectTeleOp = "RedTeleop")
public class FarRedAuto extends NextFTCOpMode {

    // =============================
    // Trajectory points (FROM FarRedAuto)
    // =============================
    private static final Pose2d START_POSE = new Pose2d(
            62.5,                  // x
            13.0,                  // y
            Math.toRadians(-180.0) // heading
    );

    // =============================
    // Auto behavior config
    // =============================
    public static double AUTO_TARGET_RPM = ShooterConstants.farTargetRPM;
    public static double AUTO_HOOD_POS = 0.2;
    public static double AUTO_TURRET_DEG = 21.5;
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

    // Limelight + turret tracking state
    private Limelight3A limelight;
    private double motorTargetX = AUTO_TURRET_DEG;
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

        drive = new AutoMecanumDrive(hardwareMap, START_POSE);

        // HARD STOP EVERYTHING immediately on init
//        hardStopAll();
        hardStopMotorsOnly();

        // Keep your existing mechanism init (you can tweak these)
        Intake.INSTANCE.moveServoPos().schedule();
        Shooter.INSTANCE.moveServo(AUTO_HOOD_POS).schedule();
        Shooter.INSTANCE.kickDefaultPos.schedule();

        // ===== init Limelight =====
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

        // Reset tracking state
        motorTargetX = AUTO_TURRET_DEG;
        smoothedTx = 0.0;
        hasSeenTarget = false;
        lastTargetSeenTimeMs = System.currentTimeMillis();

        // =============================
        // PATH (FROM FarRedAuto)
        // =============================
        autoCommand = drive.commandBuilder(START_POSE)
                .strafeTo(new Vector2d(62.0, 13.0))
                .stopAndAdd(StopDriveCmd.create(drive))
                .stopAndAdd(Turret.INSTANCE.setTargetDegreesCmd(30.0))
                .stopAndAdd(WaitCmd.create(2.0))
                .stopAndAdd(ShootBallSteadyAutoTimeoutCmd.create(IntakeConstants.shootPower, ShooterConstants.tolRpm, 2.5))
                .stopAndAdd(Intake.INSTANCE.moveIntake(IntakeConstants.intakePower))
                .stopAndAdd(Intake.INSTANCE.moveServoPos())
                .strafeToSplineHeading(new Vector2d(51.0, 70.5), Math.toRadians(-270.0))
                .stopAndAdd(WaitCmd.create(0.5))
                .turnTo(Math.toRadians(-295.0))
                .stopAndAdd(AutoSlowDriveForwardTimeoutCmd.create(drive, 0.2, 3))
                .stopAndAdd(WaitCmd.create(1))
                .stopAndAdd(StopDriveCmd.create(drive))
                .strafeToLinearHeading(new Vector2d(61.5, 10.5), Math.toRadians(-180.0))
                .stopAndAdd(StopDriveCmd.create(drive))
                .stopAndAdd(Intake.INSTANCE.moveIntake(IntakeConstants.zeroPower))
                .stopAndAdd(Intake.INSTANCE.defaultPos())
                .stopAndAdd(ShootBallSteadyAutoTimeoutCmd.create(IntakeConstants.shootPower, ShooterConstants.tolRpm, 2.5))
                .stopAndAdd(StopDriveCmd.create(drive))
                .strafeTo(new Vector2d(61.5, 45.0))
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
        Intake.INSTANCE.defaultPos().schedule();
        // Start shooter immediately and keep running all auto
        Shooter.INSTANCE.setTargetRPM(AUTO_TARGET_RPM);

        motorTargetX = AUTO_TURRET_DEG;              // IMPORTANT: set the tracking target state too

        // Tracking state (do NOT reset motorTargetX to 0)
        smoothedTx = 0.0;
        hasSeenTarget = false;
        lastTargetSeenTimeMs = System.currentTimeMillis();

        // Run auto path if desired
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

    // ==========================================================
    // SAFETY STOP HELPERS
    // ==========================================================
    private void hardStopAll() {
        Shooter.INSTANCE.stopShooter();
        Intake.INSTANCE.zeroPower().schedule();

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
        Intake.INSTANCE.zeroPower().schedule();

//        Turret.INSTANCE.disableManualControl();
//        Turret.INSTANCE.turret.setPower(0.0);

        if (drive != null) {
            drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0));
        }
    }

    // ==========================================================
    // Limelight -> turret tracking
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

        boolean sawTagThisLoop = false;

        if (AUTO_TRACK_ENABLED && result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

            if (fiducials != null && !fiducials.isEmpty()) {
                // We have at least one tag -> track
                sawTagThisLoop = true;
                hasSeenTarget = true;
                lastTargetSeenTimeMs = System.currentTimeMillis();

                double tx = result.getTx();

                // Exponential smoothing
                smoothedTx = SMOOTHING * smoothedTx + (1.0 - SMOOTHING) * tx;

                // Adjust only if outside deadband
                if (Math.abs(smoothedTx) > DEADBAND) {
                    double adjustment = smoothedTx * TRACKING_GAIN;
                    if (Math.abs(smoothedTx) < 10.0) adjustment *= 0.5;

                    motorTargetX += adjustment;
                    motorTargetX = Math.max(-TURRET_LIMIT_DEG, Math.min(TURRET_LIMIT_DEG, motorTargetX));
                }

                telemetry.addData("Tag", "YES");
                telemetry.addData("TX raw", String.format("%.2f", tx));
                telemetry.addData("TX smooth", String.format("%.2f", smoothedTx));
            }
        }

        // If we did NOT see a tag, HOLD the current motorTargetX (e.g., 30 degrees).
        if (!sawTagThisLoop) {
            telemetry.addData("Tag", "NO (holding)");

            // Optional timeout behavior: stop considering ourselves "tracking" after a while,
            // but DO NOT snap back to 0 degrees.
            if (System.currentTimeMillis() - lastTargetSeenTimeMs > (long) (NO_TARGET_TIMEOUT_SEC * 1000.0)) {
                hasSeenTarget = false;
                smoothedTx = 0.0;
            }
        }

        // Always apply the current target
        Turret.INSTANCE.setTargetDegrees(motorTargetX);
    }
}