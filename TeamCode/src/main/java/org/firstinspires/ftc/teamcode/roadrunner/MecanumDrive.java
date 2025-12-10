package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.ProfileParams;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.TimeTurn;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilderParams;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.DownsampledWriter;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.LynxFirmware;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.jetbrains.annotations.NotNull;

import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

@Config
public final class MecanumDrive {
    public static class Params {
        // IMU orientation
        public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        // drive model parameters
        public double inPerTick = 1.0;
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

    public final MecanumKinematics kinematics = new MecanumKinematics(
            PARAMS.inPerTick * PARAMS.trackWidthTicks, PARAMS.inPerTick / PARAMS.lateralInPerTick);

    public final TurnConstraints defaultTurnConstraints = new TurnConstraints(
            PARAMS.maxAngVel, -PARAMS.maxAngAccel, PARAMS.maxAngAccel);
    public final VelConstraint defaultVelConstraint =
            new MinVelConstraint(Arrays.asList(
                    kinematics.new WheelVelConstraint(PARAMS.maxWheelVel),
                    new AngularVelConstraint(PARAMS.maxAngVel)
            ));
    public final AccelConstraint defaultAccelConstraint =
            new ProfileAccelConstraint(PARAMS.minProfileAccel, PARAMS.maxProfileAccel);

    // Motor names matching your hardware config
    public final DcMotorEx frontLeft, frontRight, backLeft, backRight;

    public final VoltageSensor voltageSensor;

    public final IMU imu;

    public final Localizer localizer;

    private final LinkedList<Pose2d> poseHistory = new LinkedList<>();

    private final DownsampledWriter estimatedPoseWriter = new DownsampledWriter("ESTIMATED_POSE", 50_000_000);
    private final DownsampledWriter poseHistoryWriter = new DownsampledWriter("POSE_HISTORY", 50_000_000);
    private final DownsampledWriter mecanumCommandWriter = new DownsampledWriter("MECANUM_COMMAND", 50_000_000);
    private final DownsampledWriter targetPoseWriter = new DownsampledWriter("TARGET_POSE", 50_000_000);
    private final DownsampledWriter driveCommandWriter = new DownsampledWriter("DRIVE_COMMAND", 50_000_000);

    public final HolonomicController controller;

    public class DriveLocalizer implements Localizer {
        public final DcMotorEx frontLeft, frontRight, backLeft, backRight;

        private double lastFrontLeftPos, lastFrontRightPos, lastBackLeftPos, lastBackRightPos;
        private Pose2d txWorldRobot;
        private boolean initialized;
        private double lastHeadingRad;

        public DriveLocalizer() {
            frontLeft = MecanumDrive.this.frontLeft;
            frontRight = MecanumDrive.this.frontRight;
            backLeft = MecanumDrive.this.backLeft;
            backRight = MecanumDrive.this.backRight;

            txWorldRobot = new Pose2d(0, 0, 0);
        }

        @Override
        public void setPose(Pose2d pose) {
            txWorldRobot = pose;
        }

        @Override
        public Pose2d getPose() {
            return txWorldRobot;
        }

        @Override
        public PoseVelocity2d update() {
            double frontLeftPos = frontLeft.getCurrentPosition();
            double frontRightPos = frontRight.getCurrentPosition();
            double backLeftPos = backLeft.getCurrentPosition();
            double backRightPos = backRight.getCurrentPosition();
            double headingRad = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            if (!initialized) {
                initialized = true;

                lastFrontLeftPos = frontLeftPos;
                lastFrontRightPos = frontRightPos;
                lastBackLeftPos = backLeftPos;
                lastBackRightPos = backRightPos;
                lastHeadingRad = headingRad;

                return new PoseVelocity2d(new Vector2d(0, 0), 0);
            }

            double frontLeftDelta = frontLeftPos - lastFrontLeftPos;
            double frontRightDelta = frontRightPos - lastFrontRightPos;
            double backLeftDelta = backLeftPos - lastBackLeftPos;
            double backRightDelta = backRightPos - lastBackRightPos;
            double headingDelta = headingRad - lastHeadingRad;

            Twist2dDual<Time> twist = kinematics.forward(new MecanumKinematics.WheelIncrements<Time>(
                    new DualNum<Time>(new double[]{frontLeftDelta * PARAMS.inPerTick, 0}).times(PARAMS.lateralInPerTick / PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{backLeftDelta * PARAMS.inPerTick, 0}).times(PARAMS.lateralInPerTick / PARAMS.inPerTick),
                    new DualNum<Time>(new double[]{backRightDelta * PARAMS.inPerTick, 0}),
                    new DualNum<Time>(new double[]{frontRightDelta * PARAMS.inPerTick, 0})
            ));

            twist = new Twist2dDual<>(
                    twist.line,
                    DualNum.constant(headingDelta, 2)
            );

            txWorldRobot = txWorldRobot.plus(twist.value());

            lastFrontLeftPos = frontLeftPos;
            lastFrontRightPos = frontRightPos;
            lastBackLeftPos = backLeftPos;
            lastBackRightPos = backRightPos;
            lastHeadingRad = headingRad;

            return twist.velocity().value();
        }
    }

    public MecanumDrive(HardwareMap hardwareMap, Pose2d pose) {
        LynxFirmware.throwIfModulesAreOutdated(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Motor names matching your hardware config
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRight = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        // Motor directions matching your config
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        // Brake mode for all motors
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // IMU setup matching your config
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParams = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        PARAMS.logoFacingDirection,
                        PARAMS.usbFacingDirection
                )
        );
        imu.initialize(imuParams);
        imu.resetYaw();

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Use PinpointLocalizer
        localizer = new PinpointLocalizer(hardwareMap, PARAMS.inPerTick, pose);

        controller = new HolonomicController(
                PARAMS.axialGain, PARAMS.lateralGain, PARAMS.headingGain,
                PARAMS.axialVelGain, PARAMS.lateralVelGain, PARAMS.headingVelGain
        );

        FlightRecorder.write("MECANUM_PARAMS", PARAMS);
    }

    public void setDrivePowers(PoseVelocity2d powers) {
        MecanumKinematics.WheelVelocities<Time> wheelVels = new MecanumKinematics(1).inverse(
                PoseVelocity2dDual.constant(powers, 1));

        double maxPowerMag = 1;
        for (DualNum<Time> power : Arrays.asList(
                wheelVels.leftFront,
                wheelVels.leftBack,
                wheelVels.rightBack,
                wheelVels.rightFront
        )) {
            maxPowerMag = Math.max(maxPowerMag, power.value());
        }

        frontLeft.setPower(wheelVels.leftFront.get(0) / maxPowerMag);
        backLeft.setPower(wheelVels.leftBack.get(0) / maxPowerMag);
        backRight.setPower(wheelVels.rightBack.get(0) / maxPowerMag);
        frontRight.setPower(wheelVels.rightFront.get(0) / maxPowerMag);
    }

    public void setDrivePowersFF(PoseVelocity2dDual<Time> poseVelocity2dDual) {
        MecanumKinematics.WheelVelocities<Time> wheelVels =
                kinematics.inverse(poseVelocity2dDual);
        double voltage = voltageSensor.getVoltage();

        final MotorFeedforward feedforward = new MotorFeedforward(
                PARAMS.kS,
                PARAMS.kV / PARAMS.inPerTick,
                PARAMS.kA / PARAMS.inPerTick
        );
        double frontLeftPower = feedforward.compute(wheelVels.leftFront) / voltage;
        double backLeftPower = feedforward.compute(wheelVels.leftBack) / voltage;
        double backRightPower = feedforward.compute(wheelVels.rightBack) / voltage;
        double frontRightPower = feedforward.compute(wheelVels.rightFront) / voltage;
        mecanumCommandWriter.write(new MecanumCommandMessage(
                voltage, frontLeftPower, backLeftPower, backRightPower, frontRightPower
        ));

        frontLeft.setPower(frontLeftPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);
        frontRight.setPower(frontRightPower);
    }

    public final class FollowTrajectoryAction implements Action {
        public final TimeTrajectory timeTrajectory;
        private double beginTs = -1;

        private final double[] xPoints, yPoints;

        public FollowTrajectoryAction(TimeTrajectory t) {
            timeTrajectory = t;

            List<Double> disps = com.acmerobotics.roadrunner.Math.range(
                    0, t.path.length(),
                    Math.max(2, (int) Math.ceil(t.path.length() / 2)));
            xPoints = new double[disps.size()];
            yPoints = new double[disps.size()];
            for (int i = 0; i < disps.size(); i++) {
                Pose2d p = t.path.get(disps.get(i), 1).value();
                xPoints[i] = p.position.x;
                yPoints[i] = p.position.y;
            }
        }

        @Override
        public boolean run(@NotNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = System.nanoTime() * 1e-9;
                t = 0;
            } else {
                t = System.nanoTime() * 1e-9 - beginTs;
            }

            if (t >= timeTrajectory.duration) {
                frontLeft.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                frontRight.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = timeTrajectory.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = controller
                    .compute(txWorldTarget, localizer.getPose(), robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();

            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double frontLeftPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double backLeftPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double backRightPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double frontRightPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, frontLeftPower, backLeftPower, backRightPower, frontRightPower
            ));

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);
            frontRight.setPower(frontRightPower);

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
        public void preview(@NotNull Canvas c) {
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
        public boolean run(@NotNull TelemetryPacket p) {
            double t;
            if (beginTs < 0) {
                beginTs = System.nanoTime() * 1e-9;
                t = 0;
            } else {
                t = System.nanoTime() * 1e-9 - beginTs;
            }

            if (t >= turn.duration) {
                frontLeft.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
                frontRight.setPower(0);

                return false;
            }

            Pose2dDual<Time> txWorldTarget = turn.get(t);
            targetPoseWriter.write(new PoseMessage(txWorldTarget.value()));

            PoseVelocity2d robotVelRobot = updatePoseEstimate();

            PoseVelocity2dDual<Time> command = controller
                    .compute(txWorldTarget, localizer.getPose(), robotVelRobot);
            driveCommandWriter.write(new DriveCommandMessage(command));

            MecanumKinematics.WheelVelocities<Time> wheelVels = kinematics.inverse(command);
            double voltage = voltageSensor.getVoltage();
            final MotorFeedforward feedforward = new MotorFeedforward(PARAMS.kS,
                    PARAMS.kV / PARAMS.inPerTick, PARAMS.kA / PARAMS.inPerTick);
            double frontLeftPower = feedforward.compute(wheelVels.leftFront) / voltage;
            double backLeftPower = feedforward.compute(wheelVels.leftBack) / voltage;
            double backRightPower = feedforward.compute(wheelVels.rightBack) / voltage;
            double frontRightPower = feedforward.compute(wheelVels.rightFront) / voltage;
            mecanumCommandWriter.write(new MecanumCommandMessage(
                    voltage, frontLeftPower, backLeftPower, backRightPower, frontRightPower
            ));

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);
            frontRight.setPower(frontRightPower);

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
        public void preview(@NotNull Canvas c) {
            c.setStroke("#7C4DFF7A");
            c.fillCircle(turn.beginPose.position.x, turn.beginPose.position.y, 2);
        }
    }

    public PoseVelocity2d updatePoseEstimate() {
        PoseVelocity2d vel = localizer.update();
        Pose2d pose = localizer.getPose();

        poseHistory.add(pose);
        while (poseHistory.size() > 100) {
            poseHistory.removeFirst();
        }

        estimatedPoseWriter.write(new PoseMessage(pose));
        poseHistoryWriter.write(new PoseHistoryMessage(poseHistory));

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
                        new ProfileParams(
                                0.25, 0.1, 1e-2
                        )
                ),
                beginPose, 0.0,
                defaultTurnConstraints,
                defaultVelConstraint, defaultAccelConstraint
        );
    }
}

