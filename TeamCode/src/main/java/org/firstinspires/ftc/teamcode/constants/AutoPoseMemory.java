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

    public static void clear() {
        hasPose = false;
        ftcX = 0.0;
        ftcY = 0.0;
        headingDeg = 90.0;
        updatedAtMs = 0L;
    }

    // Traditional FTC centered frame -> Pedro frame (0..144 with rotated axes).
    public static double traditionalToPedroX(double traditionalX, double traditionalY) {
        return traditionalY + 72.0;
    }

    public static double traditionalToPedroY(double traditionalX, double traditionalY) {
        return 72.0 - traditionalX;
    }

    // Pedro frame -> Traditional FTC centered frame.
    public static double pedroToTraditionalX(double pedroX, double pedroY) {
        return 72.0 - pedroY;
    }

    public static double pedroToTraditionalY(double pedroX, double pedroY) {
        return pedroX - 72.0;
    }
}
