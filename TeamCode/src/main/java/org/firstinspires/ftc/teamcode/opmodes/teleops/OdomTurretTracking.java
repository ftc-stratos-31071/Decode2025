package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.constants.TurretConstants;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

/**
 * OpMode that uses odometry to maintain a fixed global turret heading.
 * The turret counter-rotates as the robot rotates to keep pointing at the same global direction.
 * When turret constraints are hit, it wraps around to continue tracking from the opposite side.
 */
@Config
@TeleOp(name = "OdomTurretTracking")
public class OdomTurretTracking extends NextFTCOpMode {

    private GoBildaPinpointDriver pinpoint;

    // Tunable parameters via FTC Dashboard
    public static double INITIAL_GLOBAL_HEADING = 135.0; // Global direction turret should face (degrees) - 45° left of init
    public static boolean ENABLE_TRACKING = true; // Toggle tracking on/off

    private final MotorEx frontLeftMotor = new MotorEx("frontLeftMotor").brakeMode().reversed();
    private final MotorEx frontRightMotor = new MotorEx("frontRightMotor").brakeMode().reversed();
    private final MotorEx backLeftMotor = new MotorEx("backLeftMotor").brakeMode();
    private final MotorEx backRightMotor = new MotorEx("backRightMotor").brakeMode();

    private double targetGlobalHeading = 135.0; // The global direction we want turret to face
    private boolean initialized = false;

    public OdomTurretTracking() {
        addComponents(
                new SubsystemComponent(Turret.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.resetPosAndIMU();

        // Set initial robot heading to 90 degrees (robot's starting orientation)
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 90));

        // Initialize turret to center position
        Turret.INSTANCE.setTurretAngleDeg(TurretConstants.DEFAULT_TURRET_DEG);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Robot Initial Heading", "90°");
        telemetry.addData("Will Lock To", "135° global (45° left of robot start)");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        // Set up mecanum drive control
        var forward = Gamepads.gamepad1().leftStickY().negate();
        var strafe = Gamepads.gamepad1().leftStickX();
        var rotate = Gamepads.gamepad1().rightStickX();

        Command drive = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                forward,
                strafe,
                rotate
        );

        drive.schedule();
    }

    @Override
    public void onUpdate() {
        // Update odometry
        pinpoint.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);
        double currentRobotHeading = pinpoint.getHeading(AngleUnit.DEGREES);

        // Initialize on first update - lock to target global heading
        if (!initialized) {
            targetGlobalHeading = INITIAL_GLOBAL_HEADING;
            initialized = true;
        }

        // Calculate required turret angle to maintain global heading
        if (ENABLE_TRACKING) {
            double requiredTurretAngle = calculateTurretAngle(currentRobotHeading, targetGlobalHeading);
            Turret.INSTANCE.setTurretAngleDeg(requiredTurretAngle);
        }

        // Telemetry
        telemetry.addData("Robot Heading", "%.1f°", currentRobotHeading);
        telemetry.addData("Target Global", "%.1f°", targetGlobalHeading);
        telemetry.addData("Turret Angle", "%.1f°", Turret.INSTANCE.getTargetTurretDeg());
        telemetry.addData("Actual Global", "%.1f°",
                normalizeAngle(currentRobotHeading + Turret.INSTANCE.getTargetTurretDeg()));
        telemetry.addData("Tracking", ENABLE_TRACKING ? "ENABLED" : "DISABLED");
        telemetry.update();
    }

    /**
     * Calculate the turret angle needed to point at the target global heading.
     * Handles wrapping around constraints when necessary.
     *
     * @param robotHeading Current robot heading (0-360)
     * @param globalTarget Target global heading (0-360)
     * @return Turret angle in degrees (constrained to MIN/MAX)
     */
    private double calculateTurretAngle(double robotHeading, double globalTarget) {
        // Calculate the naive turret angle: global_target - robot_heading
        double naiveTurretAngle = globalTarget - robotHeading;

        // Normalize to [-180, 180] range for shortest path
        naiveTurretAngle = normalizeAngleSigned(naiveTurretAngle);

        // Check if naive angle is within turret constraints
        if (naiveTurretAngle >= TurretConstants.MIN_TURRET_DEG &&
            naiveTurretAngle <= TurretConstants.MAX_TURRET_DEG) {
            return naiveTurretAngle;
        }

        // Naive angle is out of bounds, try wrapping around
        double wrappedAngle = naiveTurretAngle > 0 ? naiveTurretAngle - 360 : naiveTurretAngle + 360;

        // Check if wrapped angle is within constraints
        if (wrappedAngle >= TurretConstants.MIN_TURRET_DEG &&
            wrappedAngle <= TurretConstants.MAX_TURRET_DEG) {
            return wrappedAngle;
        }

        // Neither works perfectly, clamp to nearest constraint
        if (Math.abs(naiveTurretAngle - TurretConstants.MIN_TURRET_DEG) <
            Math.abs(naiveTurretAngle - TurretConstants.MAX_TURRET_DEG)) {
            return TurretConstants.MIN_TURRET_DEG;
        } else {
            return TurretConstants.MAX_TURRET_DEG;
        }
    }

    /**
     * Normalize angle to [0, 360) range
     */
    private double normalizeAngle(double degrees) {
        degrees = degrees % 360.0;
        if (degrees < 0) {
            degrees += 360.0;
        }
        return degrees;
    }

    /**
     * Normalize angle to [-180, 180] range (shortest path)
     */
    private double normalizeAngleSigned(double degrees) {
        degrees = degrees % 360.0;
        if (degrees > 180.0) {
            degrees -= 360.0;
        } else if (degrees < -180.0) {
            degrees += 360.0;
        }
        return degrees;
    }

    @Override
    public void onStop() {
        // Return turret to center position
        Turret.INSTANCE.setTurretAngleDeg(TurretConstants.DEFAULT_TURRET_DEG);
    }
}

