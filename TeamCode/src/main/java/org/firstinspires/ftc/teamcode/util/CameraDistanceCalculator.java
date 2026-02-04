package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.LimelightConstants;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

/**
 * CameraDistanceCalculator - Convert camera angles to distance
 *
 * Supports both Limelight 3A and ArduCam with AprilTag processor.
 * Uses trigonometry and your robot's physical measurements to calculate
 * the distance to an AprilTag.
 */
public class CameraDistanceCalculator {

    /**
     * Calculate distance to target using vertical angle (ty)
     *
     * @param ty Vertical angle offset from camera (degrees)
     * @return Distance to target in inches
     */
    public static double calculateDistance(double ty) {
        // Calculate total angle to target
        double angleToTarget = LimelightConstants.CAM_PITCH_DEG + ty;

        // Height difference between camera and target
        double heightDiff = LimelightConstants.TAG_HEIGHT_M - LimelightConstants.CAM_HEIGHT_M;

        // Trigonometry: distance = height / tan(angle)
        double distanceMeters = heightDiff / Math.tan(Math.toRadians(angleToTarget));

        // Convert to inches
        double distanceInches = distanceMeters * 39.3701;

        // Apply distance scale factor (for calibration adjustments)
        distanceInches *= LimelightConstants.DISTANCE_SCALE;

        // Clamp to reasonable range
        distanceInches = Math.max(LimelightConstants.MIN_DISTANCE_M * 39.3701,
                                 Math.min(LimelightConstants.MAX_DISTANCE_M * 39.3701, distanceInches));

        return distanceInches;
    }

    /**
     * Calculate distance from Limelight LLResult object
     *
     * @param result LLResult from Limelight
     * @return Distance in inches, or -1 if invalid
     */
    public static double calculateDistanceFromLimelight(LLResult result) {
        if (result == null || !result.isValid()) {
            return -1.0;
        }

        double ty = result.getTy();
        return calculateDistance(ty);
    }

    /**
     * Calculate distance from ArduCam AprilTagDetection
     * Uses the built-in pose estimation from the AprilTag processor
     *
     * @param detection AprilTagDetection from ArduCam/AprilTagProcessor
     * @return Distance to target in inches, or -1 if invalid
     */
    public static double calculateDistanceFromAprilTag(AprilTagDetection detection) {
        if (detection == null || detection.ftcPose == null) {
            return -1.0;
        }

        // The AprilTag processor already calculates the distance for us!
        // ftcPose.range is the direct distance to the tag
        // Convert from the configured units (should be INCH based on BlueTeleop setup)
        return detection.ftcPose.range;
    }

    /**
     * Get horizontal offset angle from ArduCam AprilTagDetection
     * This can be used for turret alignment
     *
     * @param detection AprilTagDetection from ArduCam
     * @return Horizontal angle offset in degrees, or 0 if invalid
     */
    public static double getHorizontalAngleFromAprilTag(AprilTagDetection detection) {
        if (detection == null || detection.ftcPose == null) {
            return 0.0;
        }

        // ftcPose.bearing gives us the horizontal angle to the tag
        return detection.ftcPose.bearing;
    }

    /**
     * Get the X offset from center for turret control
     * For ArduCam, this uses the ftcPose.x value (horizontal displacement)
     *
     * @param detection AprilTagDetection from ArduCam
     * @return X offset in inches
     */
    public static double getXOffsetFromAprilTag(AprilTagDetection detection) {
        if (detection == null || detection.ftcPose == null) {
            return 0.0;
        }

        // ftcPose.x is the horizontal offset from camera center
        return detection.ftcPose.x;
    }
}

