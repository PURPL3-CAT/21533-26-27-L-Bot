package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

public class FieldNavigator {

    // --- HARDWARE ---
    private IMU imu;

    // --- OPTION A: FIELD COORDINATES (when using official game tags with robotPose) ---
    private final double TARGET_FIELD_X   = -48.0; // inches
    private final double TARGET_FIELD_Y   = 0.0;   // inches
    private final double TARGET_FIELD_YAW = 0.0;   // degrees

    // --- OPTION B: DIRECT TAG DISTANCE FALLBACK (when using test tags / ftcPose) ---
    private final double TARGET_TAG_RANGE   = 48.0; // inches away from the tag ("back" distance)
    private final double TARGET_TAG_BEARING = 0.0;  // degrees (0 = centered straight at tag)

    // Proportional speed gains
    private final double kP_DRIVE = 0.035;
    private final double kP_TURN  = 0.020;

    // Speed limits and arrival tolerance
    private final double MAX_AUTO_SPEED = 0.65;
    private final double POS_TOLERANCE  = 1.5; // inches
    private final double YAW_TOLERANCE  = 2.0; // degrees

    // Latest telemetry & state
    private double posX = 0.0;
    private double posY = 0.0;
    private double posYaw = 0.0;
    private int detectedTagId = -1;
    private boolean usingFieldPose = false;
    private boolean hasValidPose = false;

    // IMU Data Tracking
    private double imuYaw = 0.0;
    private double imuPitch = 0.0;
    private double imuRoll = 0.0;

    // Outputs for drivetrain (axial, lateral, yaw)
    public double driveAxial = 0.0;
    public double driveLateral = 0.0;
    public double driveYaw = 0.0;

    /**
     * Initializes the FieldNavigator and configures the internal IMU.
     */
    public void init(HardwareMap hardwareMap) {
        imu = hardwareMap.get(IMU.class, "imu");
        
        // Match the working orientation profile: Upright on the left wall, ports skyward
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Initialize with correct parameters to clear NullPointerException
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    /**
     * Overloaded method wrapper to support the OpMode calling .update()
     */
    public void update(List<AprilTagDetection> detections) {
        updatePose(detections);
    }

    /**
     * Updates both IMU state variables and official FTC AprilTag positioning tracks.
     */
    public void updatePose(List<AprilTagDetection> detections) {
        // 1. Refresh IMU values
        if (imu != null) {
            Orientation imuOrientation = imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            imuYaw = imuOrientation.firstAngle;
            imuPitch = imuOrientation.secondAngle;
            imuRoll = imuOrientation.thirdAngle;
        }

        hasValidPose = false;
        usingFieldPose = false;
        detectedTagId = -1;

        if (detections != null && !detections.isEmpty()) {
            for (AprilTagDetection detection : detections) {
                // Check if the detection is a standard single AprilTag
                if (detection instanceof AprilTagSingleDetection) {
                    AprilTagSingleDetection tag = (AprilTagSingleDetection) detection;
                    detectedTagId = tag.id;
                    // 1. Prefer full Field Pose if tag ID is in the game database
                    if (tag.robotPose != null) {
                        Pose3D pose = tag.robotPose;
                        posX = pose.getPosition().x;
                        posY = pose.getPosition().y;
                        posYaw = pose.getOrientation().getYaw(AngleUnit.DEGREES);
                        usingFieldPose = true;
                        hasValidPose = true;
                        break;
                    }
                    // 2. Fallback to Camera-to-Tag Pose (works with ANY printed AprilTag!)
                    else if (tag.ftcPose != null) {
                        posX = tag.ftcPose.y;         // forward distance to tag (inches)
                        posY = tag.ftcPose.x;         // left/right offset from tag (inches)
                        posYaw = tag.ftcPose.bearing; // angle to tag (degrees)
                        usingFieldPose = false;
                        hasValidPose = true;
                        break;
                    }
                }
            }
        }
    }

    public void calculateDriveToBack() {
        if (!hasValidPose) {
            driveAxial = 0.0;
            driveLateral = 0.0;
            driveYaw = 0.0;
            return;
        }
        if (usingFieldPose) {
            // Navigate using Field (X, Y, Yaw) coordinates
            double errorX = TARGET_FIELD_X - posX;
            double errorY = TARGET_FIELD_Y - posY;
            double errorYaw = normalizeAngle(TARGET_FIELD_YAW - posYaw);

            if (Math.abs(errorX) < POS_TOLERANCE &&
                Math.abs(errorY) < POS_TOLERANCE &&
                Math.abs(errorYaw) < YAW_TOLERANCE) {
                driveAxial = 0.0;
                driveLateral = 0.0;
                driveYaw = 0.0;
                return;
            }

            double headingRad = Math.toRadians(posYaw);
            double robotForwardError = errorX * Math.cos(headingRad) + errorY * Math.sin(headingRad);
            double robotStrafeError  = -errorX * Math.sin(headingRad) + errorY * Math.cos(headingRad);

            driveAxial   = Range.clip(robotForwardError * kP_DRIVE, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            driveLateral = Range.clip(robotStrafeError * kP_DRIVE, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            driveYaw     = Range.clip(-errorYaw * kP_TURN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        } else {
            // Fallback: Back up to TARGET_TAG_RANGE inches away from the tag & center on it
            double rangeError = posX - TARGET_TAG_RANGE;     // positive = too close -> drive backward (-)
            double strafeError = posY;                       // lateral offset from tag center
            double turnError = posYaw - TARGET_TAG_BEARING;  // angle error to face tag

            if (Math.abs(rangeError) < POS_TOLERANCE &&
                Math.abs(strafeError) < POS_TOLERANCE &&
                Math.abs(turnError) < YAW_TOLERANCE) {
                driveAxial = 0.0;
                driveLateral = 0.0;
                driveYaw = 0.0;
                return;
            }

            driveAxial   = Range.clip(rangeError * kP_DRIVE, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            driveLateral = Range.clip(strafeError * kP_DRIVE, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
            driveYaw     = Range.clip(-turnError * kP_TURN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        }
    }

    private double normalizeAngle(double angle) {
        while (angle > 180.0) angle -= 360.0;
        while (angle < -180.0) angle += 360.0;
        return angle;
    }

    // --- GETTERS ---
    public boolean hasValidPose() { return hasValidPose; }
    public boolean isUsingFieldPose() { return usingFieldPose; }
    public int getDetectedTagId() { return detectedTagId; }
    public double getPosX() { return posX; }
    public double getPosY() { return posY; }
    public double getPosYaw() { return posYaw; }
    public double getImuYaw() { return imuYaw; }
    public double getImuPitch() { return imuPitch; }
    public double getImuRoll() { return imuRoll; }
}
