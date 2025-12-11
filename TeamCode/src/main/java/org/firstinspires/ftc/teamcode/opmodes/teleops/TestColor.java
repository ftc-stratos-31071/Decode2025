package org.firstinspires.ftc.teamcode.opmodes.teleops;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.commands.IntakeSeqCmd;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;

/**
 * TestColor OpMode
 *
 * Detects balls passing through the intake using color sensor.
 * Counts purple and green balls, automatically stops intake when 3 balls detected.
 * Uses distance and color saturation to detect ball passes.
 *
 * Controls:
 * - Left Bumper: Toggle intake on/off (resets count and auto-stops at 3 balls)
 * - Y Button: Toggle diagnostic mode
 */
@Config  // Enables FTC Dashboard tuning
@TeleOp(name = "TestColor", group = "Testing")
public class TestColor extends NextFTCOpMode {

    private RevColorSensorV3 colorSensor;
    private String[] detectionHistory; // Stores last 3 colors
    private int ballsInside = 0;  // Count of balls currently inside
    private ElapsedTime detectionTimer;

    // Intake state
    private boolean intakeRunning = false;
    private boolean diagnosticMode = false;  // Y button toggles diagnostic mode

    // Detection thresholds - TUNABLE VIA FTC DASHBOARD
    public static double BALL_PRESENT_DISTANCE = 25.0; // mm - ball is close enough to trigger
    public static double BALL_GONE_DISTANCE = 40.0; // mm - ball has passed through
    public static double MIN_SATURATION = 0.35; // Minimum color saturation (0-1 scale)
    public static double MIN_COLOR_POWER = 0.15; // Minimum brightness/power to consider valid

    // Color thresholds in hue (0-360 degrees) - TUNABLE VIA FTC DASHBOARD
    public static double PURPLE_HUE_MIN = 260.0;
    public static double PURPLE_HUE_MAX = 310.0;
    public static double GREEN_HUE_MIN = 80.0;
    public static double GREEN_HUE_MAX = 160.0;

    // Ball passing state machine
    private enum BallState {
        WAITING_FOR_BALL,    // No ball detected, waiting
        BALL_DETECTED,       // Ball is in front of sensor
        BALL_PASSING         // Ball is moving away (distance increasing)
    }

    private BallState ballState = BallState.WAITING_FOR_BALL;
    private String detectedBallColor = "NONE";
    private double lastDistance = 999.0;

    public TestColor() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE),
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        // Initialize color sensor
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "color");

        // Amplify sensor readings for better color detection
        colorSensor.setGain(2);  // Amplify sensor readings (LED is automatically on in I2C mode)

        // Enable I2C Fast Mode (400kHz) for faster reads
        try {
            ((LynxI2cDeviceSynch) colorSensor.getDeviceClient())
                    .setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
            telemetry.addLine("✓ Color sensor initialized with Fast I2C + LED on");
        } catch (Exception e) {
            telemetry.addLine("⚠ Fast I2C mode not available, using standard mode");
        }

        // Initialize detection tracking
        detectionHistory = new String[3];
        detectionHistory[0] = "-";
        detectionHistory[1] = "-";
        detectionHistory[2] = "-";
        detectionTimer = new ElapsedTime();

        // Initialize intake to default position
        Intake.INSTANCE.defaultPos.schedule();

        telemetry.addLine("TestColor OpMode Initialized");
        telemetry.addLine("Ball Detection Ready");
        telemetry.addLine("Press Left Bumper to start intake");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        detectionTimer.reset();

        // Left Bumper - Toggle intake on/off
        Gamepads.gamepad1().leftBumper().whenBecomesTrue(() -> {
            if (intakeRunning) {
                // Stop intake
                Intake.INSTANCE.zeroPower.schedule();
                intakeRunning = false;
            } else {
                // Start intake - RESET EVERYTHING
                ballsInside = 0;
                ballState = BallState.WAITING_FOR_BALL;
                detectedBallColor = "NONE";
                detectionHistory[0] = "-";
                detectionHistory[1] = "-";
                detectionHistory[2] = "-";

                IntakeSeqCmd.create().schedule();
                intakeRunning = true;
            }
        });

        // Y Button - Toggle diagnostic mode
        Gamepads.gamepad1().y().whenBecomesTrue(() -> {
            diagnosticMode = !diagnosticMode;
            if (diagnosticMode) {
                telemetry.addLine("Diagnostic Mode: ✓ ON");
            } else {
                telemetry.addLine("Diagnostic Mode: ✗ OFF");
            }
            telemetry.update();
        });
    }

    @Override
    public void onUpdate() {
        // Read sensor data
        var colors = colorSensor.getNormalizedColors();
        double distance = colorSensor.getDistance(DistanceUnit.MM);

        // Convert normalized RGB (0-1) to 0-255 scale for HSV conversion
        int red = (int)(colors.red * 255);
        int green = (int)(colors.green * 255);
        int blue = (int)(colors.blue * 255);

        // Calculate HSV values and color power
        float[] hsv = new float[3];
        Color.RGBToHSV(red, green, blue, hsv);
        double hue = hsv[0];
        double saturation = hsv[1];
        double value = hsv[2]; // brightness/power

        // Calculate overall color power (max of RGB normalized values)
        double colorPower = Math.max(Math.max(colors.red, colors.green), colors.blue);

        // BALL PASSING STATE MACHINE - only active when intake is running
        if (intakeRunning) {
            switch (ballState) {
                case WAITING_FOR_BALL:
                    // Check if ball is approaching
                    if (distance < BALL_PRESENT_DISTANCE &&
                        saturation > MIN_SATURATION &&
                        colorPower > MIN_COLOR_POWER) {

                        // Ball detected! Identify the color
                        String color = identifyColor(hue);
                        if (!color.equals("UNKNOWN")) {
                            detectedBallColor = color;
                            ballState = BallState.BALL_DETECTED;
                            lastDistance = distance;
                        }
                    }
                    break;

                case BALL_DETECTED:
                    // Wait for ball to start moving away (distance increasing)
                    if (distance > lastDistance + 5.0) { // Ball moving away
                        ballState = BallState.BALL_PASSING;
                    }
                    lastDistance = distance;
                    break;

                case BALL_PASSING:
                    // Wait for ball to completely pass through
                    if (distance > BALL_GONE_DISTANCE) {
                        // Ball has passed! Log it
                        ballsInside++;

                        // Add to history array (shift old values)
                        detectionHistory[0] = detectionHistory[1];
                        detectionHistory[1] = detectionHistory[2];
                        detectionHistory[2] = detectedBallColor;

                        // Check if 3 balls detected - auto-stop intake
                        if (ballsInside >= 3) {
                            Intake.INSTANCE.zeroPower.schedule();
                            intakeRunning = false;

                            // Rumble controller to notify driver
                            try {
                                Gamepads.gamepad1().getGamepad().invoke().rumble(1000);
                            } catch (Exception ignored) { }
                        }

                        // Reset for next ball
                        ballState = BallState.WAITING_FOR_BALL;
                        detectedBallColor = "NONE";
                        lastDistance = 999.0;
                    }
                    break;
            }
        }

        // Display telemetry
        if (diagnosticMode) {
            displayDiagnosticTelemetry(distance, hue, saturation, colorPower, red, green, blue);
        } else {
            displayTelemetry(distance, hue, saturation, colorPower, red, green, blue);
        }
    }

    /**
     * Identify color based on hue value
     */
    private String identifyColor(double hue) {
        if (hue >= PURPLE_HUE_MIN && hue <= PURPLE_HUE_MAX) {
            return "PURPLE";
        }
        if (hue >= GREEN_HUE_MIN && hue <= GREEN_HUE_MAX) {
            return "GREEN";
        }
        return "UNKNOWN";
    }

    /**
     * Display comprehensive telemetry
     */
    private void displayTelemetry(double distance, double hue, double saturation,
                                   double colorPower, int red, int green, int blue) {
        telemetry.addLine("==== Ball Detection System ====");
        telemetry.addLine();

        // Intake status
        telemetry.addLine("--- INTAKE STATUS ---");
        telemetry.addData("Intake", intakeRunning ? "✓ RUNNING" : "✗ STOPPED");
        telemetry.addData("Balls Inside", "%d / 3", ballsInside);
        if (ballsInside >= 3) {
            telemetry.addData("Status", "✓ FULL - Auto-Stopped!");
        }
        telemetry.addData("Control", "Left Bumper to toggle");
        telemetry.addLine();

        // Current detection state
        telemetry.addLine("--- DETECTION STATE ---");
        telemetry.addData("State", ballState.toString());
        if (!detectedBallColor.equals("NONE")) {
            telemetry.addData("Detecting", detectedBallColor);
        }
        telemetry.addData("Distance", "%.1f mm", distance);
        telemetry.addLine();

        // Balls inside (last 3 detected) with clear order
        telemetry.addLine("--- BALLS INSIDE (In Order) ---");
        if (ballsInside > 0) {
            // Show most recent to oldest
            if (ballsInside >= 3) {
                telemetry.addData("3rd Ball (Oldest)", detectionHistory[0]);
            }
            if (ballsInside >= 2) {
                telemetry.addData("2nd Ball", detectionHistory[1]);
            }
            if (ballsInside >= 1) {
                telemetry.addData("1st Ball (Latest)", detectionHistory[2]);
            }
        } else {
            telemetry.addData("Status", "No balls detected yet");
        }
        telemetry.addLine();

        // Sensor diagnostics
        telemetry.addLine("--- SENSOR DATA ---");
        telemetry.addData("RGB", "R:%d G:%d B:%d", red, green, blue);
        telemetry.addData("Hue", "%.1f°", hue);
        telemetry.addData("Saturation", "%.2f", saturation);
        telemetry.addData("Color Power", "%.2f", colorPower);
        telemetry.addData("LED", "AUTO ON (I2C Mode)");

        telemetry.update();
    }

    /**
     * Display telemetry in diagnostic mode (raw sensor readings)
     */
    private void displayDiagnosticTelemetry(double distance, double hue, double saturation,
                                             double colorPower, int red, int green, int blue) {
        telemetry.addLine("==== DIAGNOSTIC MODE ====");
        telemetry.addData("Press Y", "Exit Diagnostic Mode");
        telemetry.addLine();

        // Sensor health check
        telemetry.addLine("--- SENSOR HEALTH ---");
        boolean sensorWorking = (red > 0 || green > 0 || blue > 0);
        telemetry.addData("Sensor Status", sensorWorking ? "✓ WORKING" : "✗ NO DATA");
        
        if (!sensorWorking) {
            telemetry.addData("⚠ WARNING", "Sensor not reading data!");
            telemetry.addData("Check:", "1. Sensor plugged in?");
            telemetry.addData("Check:", "2. Configured as 'Rev Color Sensor V3'?");
            telemetry.addData("Check:", "3. Named 'color' in config?");
        }
        telemetry.addLine();

        // Raw sensor data with real-time feedback
        telemetry.addLine("--- RAW SENSOR READINGS ---");
        telemetry.addData("Distance", "%.1f mm %s", distance, 
            distance < BALL_PRESENT_DISTANCE ? "✓ CLOSE ENOUGH" : "✗ TOO FAR");
        telemetry.addData("RGB", "R:%d G:%d B:%d", red, green, blue);
        telemetry.addData("Color Power", "%.2f %s", colorPower, 
            colorPower > MIN_COLOR_POWER ? "✓ STRONG" : "✗ TOO WEAK");
        telemetry.addData("Saturation", "%.2f %s", saturation, 
            saturation > MIN_SATURATION ? "✓ GOOD" : "✗ TOO LOW");
        telemetry.addLine();

        // Color identification
        telemetry.addLine("--- COLOR DETECTION ---");
        telemetry.addData("Hue", "%.1f°", hue);
        String detectedColor = identifyColor(hue);
        telemetry.addData("Identified As", detectedColor);
        
        if (detectedColor.equals("PURPLE")) {
            telemetry.addData("Match", "✓ PURPLE detected");
        } else if (detectedColor.equals("GREEN")) {
            telemetry.addData("Match", "✓ GREEN detected");
        } else if (detectedColor.equals("UNKNOWN")) {
            telemetry.addData("Match", "Color not purple or green");
            telemetry.addData("Hue Range", "Purple: 260-310°, Green: 80-160°");
        }
        telemetry.addLine();

        // Threshold checks
        telemetry.addLine("--- DETECTION REQUIREMENTS ---");
        boolean distanceOK = distance < BALL_PRESENT_DISTANCE;
        boolean saturationOK = saturation > MIN_SATURATION;
        boolean powerOK = colorPower > MIN_COLOR_POWER;
        boolean colorOK = !detectedColor.equals("UNKNOWN");
        
        telemetry.addData("Distance < 25mm", distanceOK ? "✓ PASS" : "✗ FAIL");
        telemetry.addData("Saturation > 0.35", saturationOK ? "✓ PASS" : "✗ FAIL");
        telemetry.addData("Color Power > 0.15", powerOK ? "✓ PASS" : "✗ FAIL");
        telemetry.addData("Color Match", colorOK ? "✓ PASS" : "✗ FAIL");
        telemetry.addLine();
        
        boolean wouldDetect = distanceOK && saturationOK && powerOK && colorOK;
        if (wouldDetect) {
            telemetry.addData("RESULT", "✓✓✓ BALL WOULD BE DETECTED ✓✓✓");
        } else {
            telemetry.addData("RESULT", "✗ Ball would NOT be detected");
            telemetry.addData("Why?", "Check failed requirements above ↑");
        }
        telemetry.addLine();

        // Troubleshooting tips
        telemetry.addLine("--- TROUBLESHOOTING TIPS ---");
        if (distance > 100) {
            telemetry.addData("⚠", "Distance > 100mm - Hold ball closer!");
        }
        if (colorPower < 0.05) {
            telemetry.addData("⚠", "Very low light - LED may be off!");
            telemetry.addData("Fix:", "Check sensor power/wiring");
        }
        if (saturation < 0.2) {
            telemetry.addData("⚠", "Low saturation - Ball too faded?");
            telemetry.addData("Fix:", "Use brighter colored balls");
        }
        telemetry.addData("Control", "Left Bumper = Toggle intake");

        telemetry.update();
    }
}
