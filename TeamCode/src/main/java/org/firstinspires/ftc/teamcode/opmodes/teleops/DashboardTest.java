package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

/**
 * Dashboard and Limelight diagnostic OpMode with REAL camera stream viewing.
 *
 * This OpMode helps diagnose:
 * 1. FTC Dashboard connectivity (telemetry should appear on dashboard)
 * 2. Limelight connection and ACTUAL data
 * 3. AprilTag detection with real values
 * 4. Camera stream viewing from Limelight on dashboard
 *
 * TO ACCESS DASHBOARD:
 * 1. Connect to Robot Controller WiFi
 * 2. Open browser and go to: http://192.168.43.1:8080/dash
 *    (or http://192.168.49.1:8080/dash if using newer Android)
 * 3. You should see telemetry data streaming in real-time
 * 4. Camera stream should show in the Camera section
 */
//@TeleOp(name = "Dashboard Test", group = "Diagnostics")
public class DashboardTest extends OpMode {
    private Limelight3A limelight;
    private FtcDashboard dashboard;
    private int updateCount = 0;

    @Override
    public void init() {
        // Setup dashboard telemetry
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Initialize Limelight
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);
            limelight.start();
            limelight.pipelineSwitch(0);

            // Stream Limelight camera to dashboard
            dashboard.startCameraStream(limelight, 0);

            telemetry.addData("Limelight", "✓ Connected");
            telemetry.addData("Camera Stream", "✓ Started from Limelight");
        } catch (Exception e) {
            telemetry.addData("Limelight", "✗ ERROR: " + e.getMessage());
            telemetry.addData("", "Check hardware config for 'limelight'");
        }

        telemetry.addData("Dashboard", "✓ Initialized");
        telemetry.addData("", "");
        telemetry.addData("Dashboard URL", "http://192.168.43.1:8080/dash");
        telemetry.addData("Alt Dashboard URL", "http://192.168.49.1:8080/dash");
        telemetry.update();
    }

    @Override
    public void loop() {
        updateCount++;

        telemetry.addData("═══ DASHBOARD STATUS ═══", "");
        telemetry.addData("Update Count", updateCount);
        telemetry.addData("Loop Time (ms)", String.format("%.2f", getRuntime() * 1000));
        telemetry.addData("Dashboard Active", dashboard != null ? "✓ YES" : "✗ NO");

        telemetry.addData("", "");
        telemetry.addData("═══ LIMELIGHT STATUS ═══", "");

        if (limelight != null) {
            try {
                LLResult result = limelight.getLatestResult();

                if (result != null) {
                    boolean hasTarget = result.isValid();

                    telemetry.addData("Result Valid", hasTarget ? "✓ YES" : "✗ NO");

                    if (hasTarget) {
                        // Get REAL values from Limelight
                        double tx = result.getTx();  // Horizontal offset
                        double ty = result.getTy();  // Vertical offset
                        double ta = result.getTa();  // Target area

                        telemetry.addData("TX (horizontal)", String.format("%.2f°", tx));
                        telemetry.addData("TY (vertical)", String.format("%.2f°", ty));
                        telemetry.addData("TA (area)", String.format("%.2f%%", ta));

                        // Get AprilTag ID from fiducial results
                        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                        if (fiducials != null && !fiducials.isEmpty()) {
                            LLResultTypes.FiducialResult fiducial = fiducials.get(0);
                            long tagId = fiducial.getFiducialId();

                            telemetry.addData("AprilTag ID", tagId);
                            telemetry.addData("Family", fiducial.getFamily());
                        } else {
                            telemetry.addData("AprilTag ID", "No fiducial data");
                        }
                    } else {
                        telemetry.addData("Status", "No AprilTag visible");
                        telemetry.addData("", "Make sure:");
                        telemetry.addData("1", "AprilTag is in view");
                        telemetry.addData("2", "Pipeline 0 is for AprilTags");
                        telemetry.addData("3", "Limelight has power & ethernet");
                        telemetry.addData("4", "Tag is not covered/blocked");
                    }

                    // Capture latency info
                    telemetry.addData("Capture Latency", String.format("%.1f ms", result.getCaptureLatency()));
                    telemetry.addData("Target Latency", String.format("%.1f ms", result.getTargetingLatency()));
                    telemetry.addData("Pipeline Index", result.getPipelineIndex());

                } else {
                    telemetry.addData("Result", "NULL - No data from Limelight");
                    telemetry.addData("", "Limelight may be disconnected");
                }
            } catch (Exception e) {
                telemetry.addData("Error", e.getMessage());
                telemetry.addData("Stack", e.getClass().getSimpleName());
            }
        } else {
            telemetry.addData("Limelight", "Not initialized - check config");
            telemetry.addData("", "Hardware name should be 'limelight'");
        }

        telemetry.addData("", "");
        telemetry.addData("═══ INSTRUCTIONS ═══", "");
        telemetry.addData("Dashboard", "Open http://192.168.43.1:8080/dash");
        telemetry.addData("", "Data appears on both DS and Dashboard");
        telemetry.addData("Camera", "View in Camera section of dashboard");

        telemetry.update();
    }
}
