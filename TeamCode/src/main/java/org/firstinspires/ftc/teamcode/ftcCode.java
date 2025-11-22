package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import java.util.List;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
@TeleOp
public class ftcCode extends OpMode {
    private Limelight3A limelight; //limelight!
    private double distance;
    public IMU imu;
    @Override
    public void init(){
         limelight = hardwareMap.get(Limelight3A.class, "limelight");
         limelight.pipelineSwitch(1);
         imu = hardwareMap.get(IMU.class, "imu");
         RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                 RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
         imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        YawPitchRollAngles orientations = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientations.getYaw());
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()){
            Pose3D botpose = llResult.getBotpose_MT2();
            telemetry.addData("Tx",llResult.getTx());
            telemetry.addData("Ty",llResult.getTy());
            telemetry.addData("Ta",llResult.getTa());
            telemetry.addData("Yaw",botpose.getOrientation().getYaw());
            int tagId = -1;
            List<FiducialResult> fiducials = llResult.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                FiducialResult firstTag = fiducials.get(0);
                tagId = firstTag.getFiducialId();
                telemetry.addData("Tag ID", tagId);
            }
            String tag21 = "green purple purple";
            String tag22 = "purple green purple";
            String tag23 = "purple purple green";
            if (tagId == 21){
                telemetry.addData("Goal",tag21);
            }
            if (tagId == 22){
                telemetry.addData("Goal",tag22);
            }
            if (tagId == 23){
                telemetry.addData("Goal",tag23);
            }

            telemetry.update();
        }


    }

}