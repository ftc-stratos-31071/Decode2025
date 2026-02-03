package org.firstinspires.ftc.teamcode.opmodes.teleops;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
@TeleOp(name = "OV9281 AprilTag", group = "Vision")
public class OV9281AprilTag extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() {

        aprilTag = new AprilTagProcessor.Builder()
                .setLensIntrinsics(530.423, 530.423, 447.938, 335.553)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "arducam"))
                .setCameraResolution(new Size(800, 600))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .addProcessor(aprilTag)
                .build();

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(visionPortal, 30);

        telemetry.addLine("OV9281 AprilTag ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            List<AprilTagDetection> detections = aprilTag.getDetections();

            if (detections.isEmpty()) {
                telemetry.addLine("No tags");
            } else {
                for (AprilTagDetection tag : detections) {
                    telemetry.addData("ID", tag.id);
                    telemetry.addData("X (in)", tag.ftcPose.x);
                    telemetry.addData("Y (in)", tag.ftcPose.y);
                    telemetry.addData("Z (in)", tag.ftcPose.z);
                    telemetry.addData("Yaw (deg)", tag.ftcPose.yaw);
                    telemetry.addLine();
                }
            }

            telemetry.addData("FPS", visionPortal.getFps());
            telemetry.update();
        }

        visionPortal.close();
    }
}
