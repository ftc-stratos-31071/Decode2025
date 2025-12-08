package org.firstinspires.ftc.teamcode.opmodes.teleops;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.PedroConstants;

/**
 * ZeroPowerAccel_DPad
 *
 * D-Pad based manual measurement for theoretical forward & lateral zero-power acceleration.
 *
 * Controls:
 *   - DPad Up:    drive forward
 *   - DPad Down:  drive backward
 *   - DPad Left:  strafe left
 *   - DPad Right: strafe right
 *
 *   - A: CUT POWER and record v_cut (velocity at cut) + pose at cut
 *   - B: record final pose after robot has fully stopped
 *   - X: reset the measurement so you can do another run
 *
 * Workflow:
 *   1) Choose direction:
 *      - Forward test: use DPad Up (or Down).
 *      - Lateral test: use DPad Left / Right.
 *
 *   2) Drive until you reach the speed you want to measure.
 *   3) Press A:
 *        - OpMode records forward & lateral velocity (v_cut) and pose at that instant.
 *        - Sets drive power to zero (robot coasts).
 *   4) Let the robot fully stop.
 *   5) Press B:
 *        - OpMode records final pose.
 *        - Shows Δx (forward) and Δy (lateral) coast distance from localizer.
 *   6) On the field, mark the cut point and stop point and measure true distance with a ruler.
 *   7) Use:
 *        a = -v^2 / (2 * s)
 *      to get forwardZeroPowerAcceleration and lateralZeroPowerAcceleration.
 */
@TeleOp(name = "ZeroPowerAccel_DPad", group = "Pedro Pathing")
public class ZeroPowerAccel_DPad extends OpMode {

    private enum Phase { DRIVE, COASTING, REVIEW }
    private Phase phase = Phase.DRIVE;

    private Follower follower;

    // Data captured at power cut
    private Pose cutPose = null;
    private double vCutForward = 0.0; // in/s
    private double vCutLateral = 0.0; // in/s

    // Data captured when stopped
    private Pose stopPose = null;

    @Override
    public void init() {
        follower = PedroConstants.createFollower(hardwareMap);

        // Set any reasonable starting pose (adjust if you like)
        follower.setStartingPose(new Pose(72, 72, 0));
        follower.update();

        // TeleOp-style drive control
        follower.startTeleopDrive(true);
    }

    @Override
    public void loop() {
        // Always update follower first
        follower.update();

        // Current pose & velocity from localizer
        Pose pose = follower.getPose();
        Pose vel = follower.poseTracker.getLocalizer().getVelocity();

        double forwardVel = vel.getX(); // +X = forward
        double lateralVel = vel.getY(); // +Y = left

        switch (phase) {
            case DRIVE: {
                // Manual drive using D-Pad only
                double driveF = 0.0;
                double driveL = 0.0;

                if (gamepad1.dpad_up)    driveF = 1.0;   // forward
                if (gamepad1.dpad_down)  driveF = -1.0;  // backward
                if (gamepad1.dpad_left)  driveL = 1.0;   // left
                if (gamepad1.dpad_right) driveL = -1.0;  // right

                follower.setTeleOpDrive(driveF, driveL, 0.0, true);

                // Start measurement: press A to cut power & record v_cut
                if (gamepad1.a) {
                    cutPose = new Pose(pose.getX(), pose.getY(), pose.getHeading());
                    vCutForward = forwardVel;
                    vCutLateral = lateralVel;

                    // Cut power
                    follower.setTeleOpDrive(0.0, 0.0, 0.0, true);

                    phase = Phase.COASTING;
                }

                telemetry.addLine("PHASE: DRIVE");
                telemetry.addLine("Use D-Pad to move the robot.");
                telemetry.addLine("Forward test: DPad Up/Down.");
                telemetry.addLine("Lateral test: DPad Left/Right.");
                telemetry.addLine("Press A at your target speed to CUT power and record v_cut.");
                break;
            }

            case COASTING: {
                // Keep power at zero, just let robot coast
                follower.setTeleOpDrive(0.0, 0.0, 0.0, true);

                // When robot is fully stopped, press B
                if (gamepad1.b) {
                    stopPose = new Pose(pose.getX(), pose.getY(), pose.getHeading());
                    phase = Phase.REVIEW;
                }

                telemetry.addLine("PHASE: COASTING");
                telemetry.addLine("Robot is coasting with zero power.");
                telemetry.addLine("Wait until it fully stops, then press B.");
                break;
            }

            case REVIEW: {
                // Compute coast distances from localizer
                double dX = stopPose.getX() - cutPose.getX(); // forward distance
                double dY = stopPose.getY() - cutPose.getY(); // lateral distance

                telemetry.addLine("PHASE: REVIEW (last measurement)");
                telemetry.addLine("---------------------------------");

                telemetry.addLine(String.format("Cut Pose:  (x=%.2f, y=%.2f, h=%.2f rad)",
                        cutPose.getX(), cutPose.getY(), cutPose.getHeading()));
                telemetry.addLine(String.format("Stop Pose: (x=%.2f, y=%.2f, h=%.2f rad)",
                        stopPose.getX(), stopPose.getY(), stopPose.getHeading()));

                telemetry.addLine("");
                telemetry.addLine("Velocity at cut (from localizer):");
                telemetry.addLine(String.format("  Forward v_cut = %.3f in/s", vCutForward));
                telemetry.addLine(String.format("  Lateral v_cut = %.3f in/s", vCutLateral));

                telemetry.addLine("");
                telemetry.addLine("Localizer-measured coast distance:");
                telemetry.addLine(String.format("  Forward Δx ≈ %.3f in", dX));
                telemetry.addLine(String.format("  Lateral Δy ≈ %.3f in", dY));

                telemetry.addLine("");
                telemetry.addLine("On-field: mark CUT and STOP and measure actual distance with a ruler.");
                telemetry.addLine("Then use:  a = -v^2 / (2 * s)");
                telemetry.addLine("  Forward: a_f = -v_f^2 / (2 * s_forward)");
                telemetry.addLine("  Lateral: a_l = -v_l^2 / (2 * s_lateral)");
                telemetry.addLine("");
                telemetry.addLine("Press X to reset and run another measurement.");

                // Reset for another run
                if (gamepad1.x) {
                    cutPose = null;
                    stopPose = null;
                    vCutForward = 0.0;
                    vCutLateral = 0.0;
                    phase = Phase.DRIVE;
                }

                break;
            }
        }

        // Always show live pose & velocity
        telemetry.addLine();
        telemetry.addLine("Live Pose & Velocity:");
        telemetry.addData("Pose X (in)", "%.3f", pose.getX());
        telemetry.addData("Pose Y (in)", "%.3f", pose.getY());
        telemetry.addData("Heading (rad)", "%.3f", pose.getHeading());
        telemetry.addData("Forward Vel (X, in/s)", "%.3f", forwardVel);
        telemetry.addData("Lateral Vel (Y, in/s)", "%.3f", lateralVel);

        telemetry.update();
    }
}