package org.firstinspires.ftc.teamcode.roadrunner;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.ftc.*;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.*;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.roadrunner.FollowTrajectory;
import dev.nextftc.extensions.roadrunner.NextFTCMecanumDrive;
import dev.nextftc.extensions.roadrunner.TrajectoryCommandBuilder;
import dev.nextftc.extensions.roadrunner.Turn;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.jetbrains.annotations.NotNull;

import java.lang.Math;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

@Config
public class MecanumDrive extends NextFTCMecanumDrive implements Subsystem {

    public static final MecanumDrive INSTANCE = new MecanumDrive();

    public MecanumDrive() {}

    /** Constructor for tuning opmodes and standalone use */
    public MecanumDrive(HardwareMap hardwareMap, Pose2d startPose) {
        IMU imu = hardwareMap.get(IMU.class, "imu");
        init(hardwareMap, imu, startPose);
    }

    public static class Params {
        // IMU orientation
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        // drive model parameters
        public double inPerTick = 1;
        public double lateralInPerTick = inPerTick;
        public double trackWidthTicks = 0;

        // feedforward parameters (in tick units)
        public double kS = 0;
        public double kV = 0;
        public double kA = 0;

        // path profile parameters (in inches)
        public double maxWheelVel = 50;
        public double minProfileAccel = -30;
        public double maxProfileAccel = 50;

        // turn profile parameters (in radians)
        public double maxAngVel = Math.PI; // shared with path
        public double maxAngAccel = Math.PI;

        // path controller gains
        public double axialGain = 0.0;
        public double lateralGain = 0.0;
        public double headingGain = 0.0; // shared with turn

        public double axialVelGain = 0.0;
        public double lateralVelGain = 0.0;
        public double headingVelGain = 0.0; // shared with turn
    }

    public static Params PARAMS = new Params();

    // NOTE: these are (re)built from PARAMS; if you change PARAMS at runtime, rebuild these too.
    public MecanumKinematics kinematics = new MecanumKinematics(
            PARAMS.inPerTick * PARAMS.trackWidthTicks,
            PARAMS.inPerTick / PARAMS.lateralInPerTick
    );

    public TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel
    );

    public VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                    new AngularVelConstraint(PARAMS.maxAngVel)
            ));

    public AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

    // Hardware (matches YOUR config names)
    public DcMotorEx frontLeft;   // frontLeftMotor
    public DcMotorEx frontRight;  // frontRightMotor
    public DcMotorEx backLeft;    // backLeftMotor
    public DcMotorEx backRight;   // backRightMotor

    public VoltageSensor voltageSensor;

    // We wrap the IMU passed into init() so the RR localizer can use it.
    private IMU sdkImu;
    public LazyImu lazyImu;

    public Localizer localizer;

    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);

    public class DriveLocalizer implements Localizer {
        public final Encoder leftFrontEnc, leftBackEnc, rightBackEnc, rightFrontEnc;
        public final IMU imu;

        private int lastLeftFrontPos, lastLeftBackPos, lastRightBackPos, lastRightFrontPos;
        private Rotation2d lastHeading;
        private boolean initialized;
        private Pose2d pose;

        public DriveLocalizer(Pose2d pose) {
            leftFrontEnc = new OverflowEncoder(new RawEncoder(MecanumDrive.this.frontLeft));
            leftBackEnc = new OverflowEncoder(new RawEncoder(MecanumDrive.this.backLeft));
            rightBackEnc = new OverflowEncoder(new RawEncoder(MecanumDrive.this.backRight));
            rightFrontEnc = new OverflowEncoder(new RawEncoder(MecanumDrive.this.frontRight));

            imu = lazyImu.get();

            // If your pose drifts backwards/sideways, you may need to reverse encoder(s) here.
            // Example:
            // leftFrontEnc.setDirection(DcMotorSimple.Direction.REVERSE);

            this.pose = pose;
        }

        @Override
        public void setPose(Pose2d pose) {
            this.pose = pose;
        }

        @Override
        public Pose2d getPose() {
            return pose;
        }

        @Override
        public PoseVelocity2d update() {
            PositionVelocityPair leftFrontPosVel = leftFrontEnc.getPositionAndVelocity();
            PositionVelocityPair leftBackPosVel = leftBackEnc.getPositionAndVelocity();
            PositionVelocityPair rightBackPosVel = rightBackEnc.getPositionAndVelocity();
            PositionVelocityPair rightFrontPosVel = rightFrontEnc.getPositionAndVelocity();

            YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

            FlightRecorder.write("MECANUM_LOCALIZER_INPUTS", new MecanumLocalizerInputsMessage(
                    leftFrontPosVel, leftBackPosVel, rightBackPosVel, rightFrontPosVel, angles
            ));

            Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));

            if (!initialized) {
                initialized = true;

                lastLeftFrontPos = leftFrontPosVel.position;
                lastLeftBackPos = leftBackPosVel.position;
                lastRightBackPos = rightBackPosVel.position;
                lastRightFrontPos = rightFrontPosVel.position;

                lastHeading = heading;

                return new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
            }

            double headingDelta = heading.minus(lastHeading);

            Twist2dDual<Time> twist = kinematics.forward(new MecanumKinematics.WheelIncrements<>(
                    new DualNum<Time>(new double[]{
                            (leftFrontPosVel.position - lastLeftFrontPos),
                            leftFrontPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (leftBackPosVel.position - lastLeftBackPos),
                            leftBackPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightBackPosVel.position - lastRightBackPos),
                            rightBackPosVel.velocity,
                    }).times(PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{
                            (rightFrontPosVel.position - lastRightFrontPos),
                            rightFrontPosVel.velocity,
                    }).times(PARAMS.inPerTick)
            ));

            lastLeftFrontPos = leftFrontPosVel.position;
            lastLeftBackPos = leftBackPosVel.position;
            lastRightBackPos = rightBackPosVel.position;
            lastRightFrontPos = rightFrontPosVel.position;

            lastHeading = heading;

            pose = pose.plus(new Twist2d(
                    twist.line.value(),
                    headingDelta
            ));

            return twist.velocity().value();
        }
    }

    // --------- INIT (matches your old subsystem style) ---------

    /** Keeps your existing init signature; starts at (0,0,0). */
    public void init(HardwareMap hardwareMap, IMU imu) {
        init(hardwareMap, imu, new Pose2d(0, 0, 0));
    }

    /** Full init with a starting pose for Road Runner/NextFTC. */
    public void init(HardwareMap hardwareMap, IMU imu, Pose2d startPose) {
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        this.sdkImu = imu;

        // Motor names from YOUR working drive:
        //  frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRight = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        // Same directions as your current working code
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU init (same orientation as your current working code, configurable via PARAMS)
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(PARAMS.logoFacingDirection, PARAMS.usbFacingDirection)
        );
        sdkImu.initialize(imuParams);
        sdkImu.resetYaw();

        // Wrap SDK IMU for Road Runner localizer
        lazyImu = new LazyImu() {
            @Override public IMU get() {
                return MecanumDrive.this.sdkImu;
            }
        };

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        localizer = new DriveLocalizer(startPose);

        FlightRecorder.write("MECANUM_PARAMS", PARAMS);
    }

    // --------- Convenience methods you already use ---------

    public double getHeading() {
        if (sdkImu == null) return 0.0;
        return sdkImu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void resetHeading() {
        if (sdkImu != null) sdkImu.resetYaw();
    }

    public void stop() {
        if (frontLeft == null) return;
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    /** Same behavior as your old method: forward (+) forward, strafe (+) right, rotate (+) CCW. */
    public void setPowerMecanum(double forward, double strafe, double rotate) {
        if (frontLeft == null) return;

        // RR convention: +y is left. Your old code: +strafe is right. So we negate strafe.
        setDrivePowers(new PoseVelocity2d(new Vector2d(forward, -strafe), rotate));
    }

    public void setPowerRotation(double power) {
        power = Math.max(-1.0, Math.min(1.0, power));
        setPowerMecanum(0, 0, power);
    }

    // --------- Road Runner / NextFTC drive implementation ---------

    /** Converts robot "powers" into wheel powers and normalizes to [-1, 1]. */
    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1)
        );

        double maxPowerMag = 1.0;
        for (DualNum<Time> p : wheelVels.all()) {
            maxPowerMag = Math.max(maxPowerMag, Math.abs(p.value()));
        }

        frontLeft.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
        backLeft.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
        backRight.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
        frontRight.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
    }

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2))
            );
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= timeTrajectory.duration) {
                stop();
                return false;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            ).compute(txWorldTarget, localizer.getPose(), robotVelRobot);

            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            MotorFeedforward feedforward = new MotorFeedforward(
                    PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick,
                    PARAMS.kA / PARAMS.inPerTick
            );

            double lf = feedforward.compute(wheelVels.leftFront) / voltage;
            double lb = feedforward.compute(wheelVels.leftBack) / voltage;
            double rb = feedforward.compute(wheelVels.rightBack) / voltage;
            double rf = feedforward.compute(wheelVels.rightFront) / voltage;

            mecanumCommandWriter.write(new MecanumCommandMessage(voltage, lf, lb, rb, rf));

            frontLeft.setPower(lf);
            backLeft.setPower(lb);
            backRight.setPower(rb);
            frontRight.setPower(rf);

            p.put("x", localizer.getPose().position.x);
            p.put("y", localizer.getPose().position.y);
            p.put("heading (deg)", Math.toDegrees(localizer.getPose().heading.toDouble()));

            Pose2d error = txWorldTarget.value().minusExp(localizer.getPose());
            p.put("xError", error.position.x);
            p.put("yError", error.position.y);
            p.put("headingError (deg)", Math.toDegrees(error.heading.toDouble()));

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, localizer.getPose());

            c.setStroke("#4CAF50FF");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#4CAF507A");
            c.setStrokeWidth(1);
            c.strokePolyline(xPoints, yPoints);
        }
    }

    public final class TurnAction implements Action {
        private final TimeTurn turn;
        private double beginTs = -1;

        public TurnAction(TimeTurn turn) {
            this.turn = turn;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = Actions.now();
                t = 0;
            } else {
                t = Actions.now() - beginTs;
            }

            if (t >= turn.duration) {
                stop();
                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = new HolonomicController(
                    PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                    PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
            ).compute(txWorldTarget, localizer.getPose(), robotVelRobot);

            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            MotorFeedforward feedforward = new MotorFeedforward(
                    PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick,
                    PARAMS.kA / PARAMS.inPerTick
            );

            double lf = feedforward.compute(wheelVels.leftFront) / voltage;
            double lb = feedforward.compute(wheelVels.leftBack) / voltage;
            double rb = feedforward.compute(wheelVels.rightBack) / voltage;
            double rf = feedforward.compute(wheelVels.rightFront) / voltage;

            mecanumCommandWriter.write(new MecanumCommandMessage(voltage, lf, lb, rb, rf));

            frontLeft.setPower(lf);
            backLeft.setPower(lb);
            backRight.setPower(rb);
            frontRight.setPower(rf);

            Canvas c = p.fieldOverlay();
            drawPoseHistory(c);

            c.setStroke("#4CAF50");
            Drawing.drawRobot(c, txWorldTarget.value());

            c.setStroke("#3F51B5");
            Drawing.drawRobot(c, localizer.getPose());

            c.setStroke("#7C4DFFFF");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);

            return true;
        }

        @Override
        public void preview(Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    public PoseVelocity2d updatePoseEstimate() {
        if (localizer == null) return new PoseVelocity2d(new Vector2d(0, 0), 0);

        PoseVelocity2d vel = localizer.update();
        poseHistory.add(localizer.getPose());

        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(localizer.getPose()));
        return vel;
    }

    private void drawPoseHistory(Canvas c) {
        double[] xPoints = new double[poseHistory.size()];
        double[] yPoints = new double[poseHistory.size()];

        int i = 0;
        for (Pose2d t : poseHistory) {
            xPoints[i] = t.position.x;
            yPoints[i] = t.position.y;
            i++;
        }

        c.setStrokeWidth(1);
        c.setStroke("#3F51B5");
        c.strokePolyline(xPoints, yPoints);
    }

    public TrajectoryActionBuilder actionBuilder(Pose2d beginPose) {
        return new TrajectoryActionBuilder(
                TurnAction::new,
                FollowTrajectoryAction::new,
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(0.25, 0.1, 1e-2)
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }

    public HolonomicController controller = new HolonomicController(
            PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
            PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
    );

    @NotNull
    @Override
    public HolonomicController getController() {
        return controller;
    }

    @NotNull
    @Override
    public Pose2d getPose() {
        return (localizer == null) ? new Pose2d(0, 0, 0) : localizer.getPose();
    }

    @Override
    public void setDrivePowersFF(@NotNull PoseVelocity2dDual<Time> poseVelocity2dDual) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(poseVelocity2dDual);
        double voltage = voltageSensor.getVoltage();

        MotorFeedforward feedforward = new MotorFeedforward(
                PARAMS.kS,
                PARAMS.kV / PARAMS.inPerTick,
                PARAMS.kA / PARAMS.inPerTick
        );

        double lf = feedforward.compute(wheelVels.leftFront) / voltage;
        double lb = feedforward.compute(wheelVels.leftBack) / voltage;
        double rb = feedforward.compute(wheelVels.rightBack) / voltage;
        double rf = feedforward.compute(wheelVels.rightFront) / voltage;

        mecanumCommandWriter.write(new MecanumCommandMessage(voltage, lf, lb, rb, rf));

        frontLeft.setPower(lf);
        backLeft.setPower(lb);
        backRight.setPower(rb);
        frontRight.setPower(rf);
    }

    @Override
    @NotNull
    public TrajectoryCommandBuilder commandBuilder(@NotNull Pose2d beginPose) {
        return new TrajectoryCommandBuilder(
                turn -> new Turn(this, turn),
                traj -> new FollowTrajectory(this, traj),
                new TrajectoryBuilderParams(
                        1e-6,
                        new ProfileParams(0.25, 0.1, 1e-2)
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }

    // --------- Subsystem hook ---------

    @Override
    public void periodic() {
        // Keeps pose/velocity updated even during TeleOp manual driving.
        updatePoseEstimate();
    }
}
