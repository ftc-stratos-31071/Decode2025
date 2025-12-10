package org.firstinspires.ftc.teamcode.constants.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.LimelightConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterBallistics;
import org.firstinspires.ftc.teamcode.subsystems.ShooterPIDF;

import java.util.List;

import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;

/**
 * TuneShooterBallistics:
 *  - Uses Limelight AprilTag distance
 *  - Computes target RPM & hood from ShooterBallistics (continuous model)
 *  - Uses ShooterPIDF to hold that RPM
 *  - Right bumper: enable shooter (uses current ballistics output)
 *
 * Use FTC Dashboard to tune:
 *  - ShooterBallistics.RPM_A / RPM_B / RPM_C
 *  - ShooterBallistics.HOOD_A / HOOD_B
 *  - LimelightConstants geometry for good distance readings
 */
@Config
@TeleOp(name = "TuneShooterBallistics", group = "Tuning")
public class TuneShooterBallistics extends NextFTCOpMode {

    private Limelight3A limelight;
    private double currentDistanceMeters = Double.NaN;
    private boolean shooterEnabled = false;

    public TuneShooterBallistics() {
        addComponents(
                new SubsystemComponent(ShooterPIDF.INSTANCE)
        );
    }

    @Override
    public void onInit() {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Initialize Limelight
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);
            limelight.pipelineSwitch(0);
            limelight.start();
            dashboard.startCameraStream(limelight, 0);

            telemetry.addData("Limelight", "✓ Connected");
        } catch (Exception e) {
            telemetry.addData("Limelight", "✗ ERROR: %s", e.getMessage());
        }

        // Put hood in a known safe/default position
        ShooterPIDF.INSTANCE.defaultPos.schedule();

        telemetry.addLine("TuneShooterBallistics Initialized");
        telemetry.addLine("Hold right bumper to spin up shooter using distance-based ballistics.");
        telemetry.addLine("Tune ShooterBallistics & LimelightConstants in FTC Dashboard.");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        // Simple control: right bumper toggles shooter enabled/disabled
        dev.nextftc.ftc.Gamepads.gamepad1().rightBumper().whenBecomesTrue(() -> shooterEnabled = true);
        dev.nextftc.ftc.Gamepads.gamepad1().rightBumper().whenBecomesFalse(() -> {
            shooterEnabled = false;
            ShooterPIDF.INSTANCE.stopShooter();
        });
    }

    public void onStopButtonPressed() {
        ShooterPIDF.INSTANCE.stopShooter();
        if (limelight != null) {
            limelight.stop();
        }
    }

    @Override
    public void onUpdate() {
        // ===== 1. Read Limelight result & compute distance =====
        LLResult result = null;
        if (limelight != null) {
            try {
                result = limelight.getLatestResult();
            } catch (Exception e) {
                telemetry.addData("Limelight Error", e.getMessage());
            }
        }

        currentDistanceMeters = Double.NaN;
        int visibleTags = 0;
        int chosenId = -1;

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                visibleTags = fiducials.size();

                // For tuning, just use the FIRST tag (or you could pick closest, etc.)
                LLResultTypes.FiducialResult tag = fiducials.get(0);
                chosenId = tag.getFiducialId();

                double ty = result.getTy(); // vertical offset in degrees

                double camHeight = LimelightConstants.CAM_HEIGHT_M;
                double tagHeight = LimelightConstants.TAG_HEIGHT_M;
                double camPitchRad = Math.toRadians(LimelightConstants.CAM_PITCH_DEG);

                double denom = Math.tan(camPitchRad + Math.toRadians(ty));

                if (Math.abs(denom) > 1e-3) {
                    double dist = (tagHeight - camHeight) / denom;
                    dist *= LimelightConstants.DISTANCE_SCALE;
                    dist = Math.max(
                            LimelightConstants.MIN_DISTANCE_M,
                            Math.min(LimelightConstants.MAX_DISTANCE_M, dist)
                    );
                    currentDistanceMeters = dist;
                }
            }
        }

        // ===== 2. Compute target RPM & hood from ShooterBallistics =====
        double targetRpm = 0.0;
        double targetHood = ShooterPIDF.INSTANCE.getTargetRPM(); // just to have something; will override if valid distance
        boolean hasValidDistance = !Double.isNaN(currentDistanceMeters);

        if (hasValidDistance) {
            targetRpm = ShooterBallistics.rpmFromDistance(currentDistanceMeters);
            targetHood = ShooterBallistics.hoodFromDistance(currentDistanceMeters);
        }

        // Clamp hood against physical limit if you have one
        targetHood = Math.max(ShooterBallistics.MIN_HOOD, Math.min(ShooterBallistics.MAX_HOOD, targetHood));

        // ===== 3. Apply to ShooterPIDF if enabled =====
        if (shooterEnabled && hasValidDistance) {
            ShooterPIDF.INSTANCE.setTargetRPM(targetRpm);
        } else if (!shooterEnabled) {
            ShooterPIDF.INSTANCE.setTargetRPM(0.0);
        }

        // Always move hood to target (even if shooter disabled) so you can see arc effect
        ShooterPIDF.INSTANCE.moveServo(targetHood).schedule();

        double currentRpm = ShooterPIDF.INSTANCE.getRPM();
        double rpmError = targetRpm - currentRpm;

        // ===== 4. Telemetry for tuning =====
        telemetry.addLine("==== Tune Shooter Ballistics ====");
        telemetry.addData("Shooter Enabled (RB)", shooterEnabled);

        telemetry.addLine();
        telemetry.addData("Tags Visible", visibleTags);
        telemetry.addData("Chosen Tag ID", chosenId);
        telemetry.addData("Distance (m)", hasValidDistance ? String.format("%.2f", currentDistanceMeters) : "N/A");

        telemetry.addLine();
        telemetry.addData("Target RPM", "%.0f", targetRpm);
        telemetry.addData("Current RPM", "%.0f", currentRpm);
        telemetry.addData("RPM Error", "%.0f", rpmError);

        telemetry.addData("Target Hood", "%.3f", targetHood);

        telemetry.addLine();
        telemetry.addData("Ballistics RPM_A", ShooterBallistics.RPM_A);
        telemetry.addData("Ballistics RPM_B", ShooterBallistics.RPM_B);
        telemetry.addData("Ballistics RPM_C", ShooterBallistics.RPM_C);
        telemetry.addData("Ballistics HOOD_A", ShooterBallistics.HOOD_A);
        telemetry.addData("Ballistics HOOD_B", ShooterBallistics.HOOD_B);

        telemetry.addLine();
        telemetry.addData("CamHeight (m)", LimelightConstants.CAM_HEIGHT_M);
        telemetry.addData("TagHeight (m)", LimelightConstants.TAG_HEIGHT_M);
        telemetry.addData("CamPitch (deg)", LimelightConstants.CAM_PITCH_DEG);
        telemetry.addData("Distance Scale", LimelightConstants.DISTANCE_SCALE);

        telemetry.update();
    }
}