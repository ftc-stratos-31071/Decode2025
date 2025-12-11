package org.firstinspires.ftc.teamcode.opmodes.teleops;

import android.graphics.Color;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.nextftc.ftc.NextFTCOpMode;

/**
 * TestColor OpMode
 *
 * Tests the Color Rangefinder sensor to detect and track colored balls (Purple and Green).
 * Displays current detection and history of last 3 detected balls.
 */
@TeleOp(name = "TestColor", group = "Testing")
public class TestColor extends NextFTCOpMode {

    private RevColorSensorV3 colorSensor;
    private String[] detectionHistory; // Stores last 3 colors
    private int totalDetections = 0;
    private String currentColor = "NONE";
    private ElapsedTime detectionTimer;

    // Detection thresholds
    private static final double DISTANCE_THRESHOLD = 30.0; // mm - ball must be within this distance
    private static final double MIN_CONFIDENCE_TIME = 0.15; // seconds - must see color consistently

    // Color thresholds (tune these based on testing)
    private static final double PURPLE_HUE_MIN = 260.0;
    private static final double PURPLE_HUE_MAX = 310.0;
    private static final double GREEN_HUE_MIN = 80.0;
    private static final double GREEN_HUE_MAX = 160.0;
    private static final double MIN_SATURATION = 0.3; // Minimum color saturation to avoid false detections

    private String lastSeenColor = "NONE";
    private double confidenceStartTime = 0;
    private boolean isConfirmed = false;

    public TestColor() {
        // No components needed
    }

    @Override
    public void onInit() {
        // Initialize color sensor
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color");

        // Enable I2C Fast Mode (400kHz) for faster reads
        try {
            ((LynxI2cDeviceSynch) colorSensor.getDeviceClient())
                    .setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
            telemetry.addLine("✓ Color sensor initialized with Fast I2C mode");
        } catch (Exception e) {
            telemetry.addLine("⚠ Fast I2C mode not available, using standard mode");
        }

        // Initialize detection tracking - simple array for last 3
        detectionHistory = new String[3];
        detectionHistory[0] = "-";
        detectionHistory[1] = "-";
        detectionHistory[2] = "-";
        detectionTimer = new ElapsedTime();

        telemetry.addLine("TestColor OpMode Initialized");
        telemetry.addLine("Ready to detect Purple and Green balls");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        detectionTimer.reset();
    }

    @Override
    public void onUpdate() {
        // Read color and distance data - CORRECT API
        var colors = colorSensor.getNormalizedColors();
        double distance = colorSensor.getDistance(DistanceUnit.MM);

        // Convert normalized RGB (0-1) to 0-255 scale for HSV conversion
        int red = (int)(colors.red * 255);
        int green = (int)(colors.green * 255);
        int blue = (int)(colors.blue * 255);

        // Calculate HSV values for better color detection
        float[] hsv = new float[3];
        Color.RGBToHSV(red, green, blue, hsv);
        double hue = hsv[0];
        double saturation = hsv[1];

        // Determine detected color
        String detectedColor = detectColor(hue, saturation, distance);

        // State machine for ball detection
        if (distance < DISTANCE_THRESHOLD && !detectedColor.equals("NONE")) {
            // Ball is present and we can see a color

            if (detectedColor.equals(lastSeenColor)) {
                // Same color - check if we've seen it long enough
                if (!isConfirmed && detectionTimer.seconds() - confidenceStartTime >= MIN_CONFIDENCE_TIME) {
                    // Confirmed detection - add to history
                    isConfirmed = true;
                    currentColor = detectedColor;
                    totalDetections++;

                    // Add to history array (shift old values)
                    detectionHistory[0] = detectionHistory[1];
                    detectionHistory[1] = detectionHistory[2];
                    detectionHistory[2] = detectedColor;
                }
            } else {
                // Different color detected - restart confidence timer
                lastSeenColor = detectedColor;
                confidenceStartTime = detectionTimer.seconds();
                isConfirmed = false;
            }
        } else {
            // No ball present or no clear color - reset detection
            if (distance >= DISTANCE_THRESHOLD) {
                lastSeenColor = "NONE";
                isConfirmed = false;
                currentColor = "NONE";
            }
        }

        // Display telemetry
        displayTelemetry(detectedColor, distance, hue, saturation, red, green, blue);
    }

    /**
     * Detect color based on HSV values and distance
     */
    private String detectColor(double hue, double saturation, double distance) {
        // Must be close enough and have enough color saturation
        if (distance >= DISTANCE_THRESHOLD || saturation < MIN_SATURATION) {
            return "NONE";
        }

        // Check for purple
        if (hue >= PURPLE_HUE_MIN && hue <= PURPLE_HUE_MAX) {
            return "PURPLE";
        }

        // Check for green
        if (hue >= GREEN_HUE_MIN && hue <= GREEN_HUE_MAX) {
            return "GREEN";
        }

        return "UNKNOWN";
    }

    /**
     * Display comprehensive telemetry
     */
    private void displayTelemetry(String detectedColor, double distance, double hue,
                                   double saturation, int red, int green, int blue) {
        telemetry.addLine("==== Color Sensor Test ====");
        telemetry.addLine();

        // Current detection
        telemetry.addLine("--- CURRENT SEEING ---");
        if (!currentColor.equals("NONE")) {
            telemetry.addData("Color", currentColor);
        } else if (!detectedColor.equals("NONE")) {
            telemetry.addData("Detecting", "%s (%.0f%%)",
                    detectedColor,
                    Math.min(100, ((detectionTimer.seconds() - confidenceStartTime) / MIN_CONFIDENCE_TIME) * 100));
        } else {
            telemetry.addData("Status", "No ball detected");
        }
        telemetry.addData("Distance", "%.1f mm", distance);
        telemetry.addLine();

        // Detection history (last 3)
        telemetry.addLine("--- PREVIOUS (Last 3) ---");
        telemetry.addData("Most Recent", detectionHistory[2]);
        telemetry.addData("2nd", detectionHistory[1]);
        telemetry.addData("3rd", detectionHistory[0]);
        telemetry.addLine();

        // Raw sensor data
        telemetry.addLine("--- RAW SENSOR DATA ---");
        telemetry.addData("RGB", "R:%d G:%d B:%d", red, green, blue);
        telemetry.addData("HSV", "H:%.1f° S:%.2f", hue, saturation);
        telemetry.addData("Total Detected", totalDetections);

        telemetry.update();
    }
}
