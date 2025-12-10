package org.firstinspires.ftc.teamcode.roadrunner;

import com.acmerobotics.roadrunner.Pose2d;

import java.util.List;

public final class PoseHistoryMessage {
    public long timestamp;
    public double[] xPoints;
    public double[] yPoints;

    public PoseHistoryMessage(List<Pose2d> poseHistory) {
        this.timestamp = System.nanoTime();
        this.xPoints = new double[poseHistory.size()];
        this.yPoints = new double[poseHistory.size()];
        for (int i = 0; i < poseHistory.size(); i++) {
            Pose2d pose = poseHistory.get(i);
            xPoints[i] = pose.position.x;
            yPoints[i] = pose.position.y;
        }
    }
}

