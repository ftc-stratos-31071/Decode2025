package org.firstinspires.ftc.teamcode.constants;

public final class AutoPoseMemory {
    private AutoPoseMemory() {}

    public static boolean hasPose = false;
    public static double ftcX = 0.0;
    public static double ftcY = 0.0;
    public static double headingDeg = 90.0;
    public static long updatedAtMs = 0L;

    public static void setFtcPose(double x, double y, double heading) {
        hasPose = true;
        ftcX = x;
        ftcY = y;
        headingDeg = heading;
        updatedAtMs = System.currentTimeMillis();
    }
}
