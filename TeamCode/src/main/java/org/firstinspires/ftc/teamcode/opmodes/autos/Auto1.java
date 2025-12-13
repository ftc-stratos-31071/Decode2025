package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.commands.IntakeSeqCmd;
import org.firstinspires.ftc.teamcode.commands.ShootBallCmd;
import org.firstinspires.ftc.teamcode.commands.RobotResetCmd;
import org.firstinspires.ftc.teamcode.roadrunner.AutoMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import java.util.List;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name = "Auto1")
public class Auto1 extends NextFTCOpMode {

    /* ==================== TUNABLES ==================== */

    public static double SHOOT_RPM = 3500.0;  // Shooter RPM
    public static double PICKUP_TIME = 1.0;   // Pickup time
    public static double SPINUP_TIMEOUT = 3.0; // Shooter spin-up timeout

    // Turret auto-align constants
    public static double TRACKING_GAIN = 0.08;
    public static double SMOOTHING = 0.7;
    public static double DEADBAND = 3.0;
    public static double TURRET_LIMIT_DEG = 90.0;
    public static double NO_TARGET_TIMEOUT_SEC = 2.0;

    /* ==================== POSES ==================== */

    private final Pose2d startPose = new Pose2d(-52.5, -51.5, Math.toRadians(-124));

    // Score position
    private final Pose2d scorePose = new Pose2d(-14.0, -13.0, Math.toRadians(225));
    private final Vector2d scorePoint = scorePose.position;

    // Pickup positions
    private final Vector2d pickup1 = new Vector2d(-12.0, -54.0);
    private final Vector2d pickup2 = new Vector2d(12.0, -48.0);

    /* ==================== STATE ==================== */

    private AutoMecanumDrive drive;
    private Command autoCommand;

    // Limelight state
    private Limelight3A limelight;
    private double motorTargetX = 0.0;
    private double smoothedTx = 0.0;
    private long lastTargetSeenTime = 0;

    public Auto1() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, Turret.INSTANCE),
                BulkReadComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {
        // Initialize the drivetrain and components
        drive = new AutoMecanumDrive(hardwareMap, startPose);

        // Initialize Limelight
        try {
            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);
            limelight.start();
            limelight.pipelineSwitch(0);
        } catch (Exception e) {
            limelight = null;
        }

        // Initialize the autonomous sequence commands
        Command driveToScore = drive.commandBuilder(startPose)
                .afterTime(0.25, () -> Shooter.INSTANCE.setTargetRPM(SHOOT_RPM))
                .setReversed(true)
                .strafeTo(scorePoint)
                .build();

        Command driveToPickup1 = drive.commandBuilder(scorePose)
                .setReversed(false)
                .splineTo(pickup1, Math.toRadians(270))
                .build();

        Command pickup1Seq = new SequentialGroup(
                IntakeSeqCmd.create(),
                new Delay(PICKUP_TIME),
                Intake.INSTANCE.zeroPower
        );

        Command backToScore1 = drive.commandBuilder(
                        new Pose2d(pickup1.x, pickup1.y, Math.toRadians(270)))
                .setReversed(true)
                .splineToLinearHeading(scorePose, Math.toRadians(90))
                .build();

        Command driveToPickup2 = drive.commandBuilder(scorePose)
                .setReversed(false)
                .splineTo(pickup2, Math.toRadians(270))
                .build();

        Command pickup2Seq = new SequentialGroup(
                IntakeSeqCmd.create(),
                new Delay(PICKUP_TIME),
                Intake.INSTANCE.zeroPower
        );

        Command backToScore2 = drive.commandBuilder(
                        new Pose2d(pickup2.x, pickup2.y, Math.toRadians(270)))
                .setReversed(true)
                .splineToLinearHeading(scorePose, Math.toRadians(90))
                .build();

        // Full Auto Sequence
        autoCommand = new SequentialGroup(
                driveToScore,
                stopDrive(),
                spinUpAndWaitReady(),
                ShootBallCmd.create()

                // Pickup line 1
//                driveToPickup1
//                stopDrive(),
//                pickup1Seq,
//                backToScore1,
//                stopDrive(),
//                spinUpAndWaitReady(),
//                ShootBallCmd.create(),

                // Pickup line 2 (optional)
                // driveToPickup2,
                // stopDrive(),
                // pickup2Seq,
                // backToScore2,
                // stopDrive(),
                // spinUpAndWaitReady(),
                // ShootBallCmd.create()
        ).named("Auto1Sequence");
    }

    @Override
    public void onStartButtonPressed() {
        // Reset robot subsystems when the start button is pressed
        RobotResetCmd.create().schedule();

        // Start the autonomous sequence
        autoCommand.schedule();
    }

    /* ==================== HELPERS ==================== */

    // Hard-stop the drivetrain output
    private Command stopDrive() {
        return new Command() {
            @Override
            public void start() {
                drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
            }

            @Override
            public boolean isDone() {
                return true;
            }
        }.requires(drive);
    }

    // Start the shooter at the target RPM and wait for it to be ready
    private Command spinUpAndWaitReady() {
        Command startSpin = new Command() {
            @Override
            public void start() {
                Shooter.INSTANCE.setTargetRPM(SHOOT_RPM);
            }
            @Override
            public boolean isDone() { return true; }
        }.requires(Shooter.INSTANCE);

        Command waitReady = new Command() {
            @Override
            public boolean isDone() {
                return Shooter.INSTANCE.getRPM() >= SHOOT_RPM * 0.95;
            }
        };

        // Wait until ready or timeout occurs
        Command waitWithTimeout = new ParallelRaceGroup(
                waitReady,
                new Delay(SPINUP_TIMEOUT)
        );

        return new SequentialGroup(
                startSpin,
                waitWithTimeout
        ).named("SpinUpAndWait");
    }

    /* ==================== TURRET AUTO ALIGN ==================== */

    @Override
    public void onUpdate() {
        if (limelight == null) return;

        LLResult result;
        try {
            result = limelight.getLatestResult();
        } catch (Exception e) {
            return;
        }

        if (result != null && result.isValid()) {
            List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();

            if (tags != null && !tags.isEmpty()) {
                lastTargetSeenTime = System.currentTimeMillis();

                double tx = result.getTx();
                smoothedTx = SMOOTHING * smoothedTx + (1 - SMOOTHING) * tx;

                if (Math.abs(smoothedTx) > DEADBAND) {
                    motorTargetX += smoothedTx * TRACKING_GAIN;
                    motorTargetX = Math.max(-TURRET_LIMIT_DEG, Math.min(TURRET_LIMIT_DEG, motorTargetX));
                }
            }
        }

        if (System.currentTimeMillis() - lastTargetSeenTime > NO_TARGET_TIMEOUT_SEC * 1000) {
            motorTargetX = 0;
            smoothedTx = 0;
        }

        Turret.INSTANCE.setTargetDegrees(motorTargetX);
    }
}
