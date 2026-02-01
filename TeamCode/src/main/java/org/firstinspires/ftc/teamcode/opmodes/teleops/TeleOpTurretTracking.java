package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
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
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name = "TeleOpTurretTracking")
public class TeleOpTurretTracking extends NextFTCOpMode {

    private GoBildaPinpointDriver pinpoint;
    private Limelight3A limelight;

    private static final double TX_SMOOTHING = 0.7;
    private static final double TX_DEADBAND = 1.5;

    private static final double TURRET_MIN_DEG = -90.0;
    private static final double TURRET_MAX_DEG = 90.0;

    private double smoothedTx = 0.0;

    private double fieldTargetDeg = 0.0;
    private boolean hasSeenTarget = false;

    private double initialPinpointOffset = 0.0;

    private final MotorEx frontLeftMotor = new MotorEx("frontLeftMotor").brakeMode().reversed();
    private final MotorEx frontRightMotor = new MotorEx("frontRightMotor").brakeMode().reversed();
    private final MotorEx backLeftMotor = new MotorEx("backLeftMotor").brakeMode();
    private final MotorEx backRightMotor = new MotorEx("backRightMotor").brakeMode();

    private double normalizeDeg(double deg) {
        deg %= 360.0;
        if (deg < 0) deg += 360.0;
        return deg;
    }

    private double clampDeg(double deg, double min, double max) {
        return Math.max(min, Math.min(max, deg));
    }

    public TeleOpTurretTracking() {
        addComponents(
                new SubsystemComponent(Turret.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();

        Turret.INSTANCE.setTurretAngleDeg(0.0);

        telemetry.addData("Status", "Initialized — waiting for AprilTag");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
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
        pinpoint.update(GoBildaPinpointDriver.ReadData.ONLY_UPDATE_HEADING);

        double rawHeadingDeg =
                normalizeDeg(pinpoint.getHeading(AngleUnit.DEGREES));

        double robotHeadingDeg = hasSeenTarget
                ? normalizeDeg(rawHeadingDeg - initialPinpointOffset)
                : 0.0;

        double txCorrection = 0.0;

        if (limelight.isConnected()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double tx = result.getTx();
                smoothedTx = TX_SMOOTHING * smoothedTx + (1.0 - TX_SMOOTHING) * tx;

                if (!hasSeenTarget) {
                    initialPinpointOffset = rawHeadingDeg;
                    fieldTargetDeg = normalizeDeg(robotHeadingDeg + tx);
                    hasSeenTarget = true;
                }

                if (Math.abs(smoothedTx) > TX_DEADBAND) {
                    txCorrection = smoothedTx;
                }
            }
        }

        double turretDeg = hasSeenTarget
                ? normalizeDeg(fieldTargetDeg - robotHeadingDeg + txCorrection)
                : 0.0;

        turretDeg = clampDeg(turretDeg, TURRET_MIN_DEG, TURRET_MAX_DEG);

        Turret.INSTANCE.setTurretAngleDeg(turretDeg);

        telemetry.addData("Raw Heading", rawHeadingDeg);
        telemetry.addData("Robot Heading", robotHeadingDeg);
        telemetry.addData("Field Target", fieldTargetDeg);
        telemetry.addData("Turret Cmd", turretDeg);
        telemetry.addData("Has Seen Target", hasSeenTarget);
        telemetry.addData("LL tx (smooth)", smoothedTx);
        telemetry.update();
    }

    @Override
    public void onStop() {
        limelight.stop();
        Turret.INSTANCE.setTurretAngleDeg(0.0);
    }
}
