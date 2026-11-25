package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Shared engine for every autonomous start spot.
 *
 * Each start-spot file only says WHERE the robot starts and which waypoints to pass through.
 * This class does the driving, and it picks the parking box from the alliance color, so a
 * RED start can only ever park in the RED box and a BLUE start only in the BLUE box.
 *
 * FIELD COORDINATES (inches) follow the field diagram (Figure 11-1):
 *   origin = bottom-left corner of the diagram, +X = toward the right wall, +Y = toward the top wall.
 * HEADING (degrees, counter-clockwise positive):
 *   0 = robot front faces the right wall, 90 = top wall, 180 = left wall, 270 = bottom wall.
 *
 * ODOMETRY uses the same motor encoders as the TeleOp "Odo X / Odo Y" readout:
 *   intake_motor = left-right pod, flipout_ramp = forward-back pod.
 * Heading comes from the Control Hub IMU (hardware name "imu").
 */
public abstract class AutoBase extends LinearOpMode {

    protected enum Alliance { RED, BLUE }

    // =====================================================================
    //  VALUES TO CHECK / TUNE ON THE ROBOT
    // =====================================================================

    // Robot footprint. Used only to decide how far from the wall the robot stops when parking.
    private static final double ROBOT_WIDTH_IN = 18.0;

    // Park with the robot's edge this far off the perimeter wall. That keeps it partly inside the
    // parking box while no longer touching the wall (the manual's LEAVE rule).
    // Set to 0 if you would rather park flush against the wall.
    private static final double PARK_WALL_CLEARANCE_IN = 5.5;

    // Odometry pods: 8192 ticks per 6.283 in of travel (same number as the TeleOp).
    private static final double TICKS_PER_INCH = 1303.8;

    // Pod signs. Check on the Init screen: push the robot FORWARD by hand -> "Forward pod" must go UP.
    // Push it to the RIGHT -> "Right pod" must go UP. If one goes down, change that sign to -1.0.
    private static final double X_POD_SIGN = 1.0;   // intake_motor (left-right)
    private static final double Y_POD_SIGN = 1.0;   // flipout_ramp (forward-back)

    // How the Control Hub is mounted on the robot (see the REV IMU orientation picture).
    private static final RevHubOrientationOnRobot.LogoFacingDirection HUB_LOGO =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    private static final RevHubOrientationOnRobot.UsbFacingDirection HUB_USB =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    // Driving
    private static final double MAX_DRIVE_POWER = 0.6;
    private static final double MIN_DRIVE_POWER = 0.15;     // lowest power that still moves the robot
    private static final double DRIVE_KP = 0.035;           // slow down as the target gets close
    private static final double STRAFE_COMP = 1.15;         // mecanum strafes slower than it drives
    private static final double RAMP_UP_S = 0.5;            // gentle start so the wheels do not slip
    private static final double HEADING_KP = 0.02;          // keeps the robot pointing the same way
    private static final double HEADING_MAX_POWER = 0.3;
    private static final double POSITION_TOLERANCE_IN = 1.5;   // final stop
    private static final double VIA_TOLERANCE_IN = 5.0;        // waypoints on the way
    private static final double MOVE_TIMEOUT_S = 6.0;          // give up on one leg after this long
    private static final double AUTO_LIMIT_S = 29.0;           // AUTO is 30 s; stop a little early

    // =====================================================================
    //  FIELD GEOMETRY (from the field diagram, inches)
    // =====================================================================
    private static final double FIELD_SIZE_IN = 144.0;
    private static final double RED_PARK_Y_IN = 107.7;    // red box is on the left wall
    private static final double BLUE_PARK_Y_IN = 36.0;    // blue box is on the right wall

    // =====================================================================

    private DcMotor frontLeftDrive;
    private DcMotor backLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backRightDrive;
    private DcMotor xPod;   // intake_motor: left-right odometry
    private DcMotor yPod;   // flipout_ramp: forward-back odometry
    private IMU imu = null;

    private final ElapsedTime autoTimer = new ElapsedTime();

    private Alliance alliance = null;
    private String spotName = "";
    private double startX;
    private double startY;
    private double startHeadingDeg;

    private double poseX;
    private double poseY;
    private double headingDeg;
    private double lastXPodIn;
    private double lastYPodIn;
    private boolean inMotion = false;

    /** Call setStart(...) in here. */
    protected abstract void configure();

    /** Drive the route in here using via(...) and parkInAllianceZone(). */
    protected abstract void route();

    protected final void setStart(Alliance a, String name, double x, double y, double headingDegrees) {
        alliance = a;
        spotName = name;
        startX = x;
        startY = y;
        startHeadingDeg = headingDegrees;
    }

    /** Pass through a waypoint without stopping (5 in tolerance). */
    protected final void via(double x, double y) {
        goTo(x, y, VIA_TOLERANCE_IN, false);
    }

    /** Drive to a point and stop precisely. */
    protected final void driveTo(double x, double y) {
        goTo(x, y, POSITION_TOLERANCE_IN, true);
    }

    /** Drive into the parking box that matches this OpMode's alliance color. */
    protected final void parkInAllianceZone() {
        driveTo(parkX(alliance), parkY(alliance));
    }

    @Override
    public final void runOpMode() {
        configure();
        if (alliance == null) {
            telemetry.addData("ERROR", "configure() must call setStart(...)");
            telemetry.update();
            waitForStart();
            return;
        }

        initHardware();

        while (!isStarted() && !isStopRequested()) {
            showInitTelemetry();
            sleep(50);
        }
        if (isStopRequested()) {
            return;
        }

        if (imu != null) {
            imu.resetYaw();
        }
        poseX = startX;
        poseY = startY;
        headingDeg = startHeadingDeg;
        lastXPodIn = xPodInches();
        lastYPodIn = yPodInches();
        autoTimer.reset();

        route();
        stopDrive();

        updatePose();
        telemetry.addData("Done", "%s", spotName);
        telemetry.addData("Final pose", "%.1f, %.1f @ %.0f deg", poseX, poseY, headingDeg);
        telemetry.update();
    }

    // ---------------------------------------------------------------------
    //  Setup
    // ---------------------------------------------------------------------

    private void initHardware() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        // Same encoders as the TeleOp: Odo X = intake, Odo Y = flipout ramp.
        xPod = hardwareMap.get(DcMotor.class, "intake_motor");
        yPod = hardwareMap.get(DcMotor.class, "flipout_ramp");

        // Same directions as the TeleOp so the pod encoder signs match what you see there.
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        xPod.setDirection(DcMotor.Direction.REVERSE);
        yPod.setDirection(DcMotor.Direction.REVERSE);

        DcMotor[] drive = {frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive};
        for (DcMotor m : drive) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // The pods are only read, never powered.
        xPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        xPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        yPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        try {
            imu = hardwareMap.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(HUB_LOGO, HUB_USB)));
        } catch (Exception e) {
            imu = null;   // no IMU in the robot config: drive without heading hold
        }
    }

    private void showInitTelemetry() {
        telemetry.addLine("Put the robot on its start spot, then press PLAY.");
        telemetry.addData("Spot", spotName);
        telemetry.addData("Start x, y (in)", "%.1f, %.1f", startX, startY);
        telemetry.addData("Robot FRONT must face (deg)", "%.1f   (0 = right wall, 90 = top, 180 = left wall, 270 = bottom)",
                startHeadingDeg);
        telemetry.addData("Will park in the", "%s box at %.1f, %.1f", alliance, parkX(alliance), parkY(alliance));
        telemetry.addData("Forward pod (in)", "%.1f   push robot forward: must go UP", yPodInches());
        telemetry.addData("Right pod (in)", "%.1f   push robot right: must go UP", xPodInches());
        if (imu == null) {
            telemetry.addData("IMU", "NOT FOUND - no heading hold");
        } else {
            double yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            telemetry.addData("IMU yaw (deg)", "%.1f   turn robot counter-clockwise by hand: must go UP", yaw);
        }
        telemetry.update();
    }

    // ---------------------------------------------------------------------
    //  Driving
    // ---------------------------------------------------------------------

    private void goTo(double targetX, double targetY, double tolerance, boolean finalStop) {
        ElapsedTime leg = new ElapsedTime();
        boolean ramp = !inMotion;
        inMotion = true;

        while (opModeIsActive() && leg.seconds() < MOVE_TIMEOUT_S && autoTimer.seconds() < AUTO_LIMIT_S) {
            updatePose();

            double errX = targetX - poseX;
            double errY = targetY - poseY;
            double dist = Math.hypot(errX, errY);
            if (dist < tolerance) {
                break;
            }

            // Split the field-frame error into "forward" and "right" relative to the robot.
            double th = Math.toRadians(headingDeg);
            double forward = errX * Math.cos(th) + errY * Math.sin(th);
            double right = errX * Math.sin(th) - errY * Math.cos(th);

            double speed = finalStop
                    ? Range.clip(dist * DRIVE_KP, MIN_DRIVE_POWER, MAX_DRIVE_POWER)
                    : MAX_DRIVE_POWER;
            if (ramp) {
                speed = Math.max(MIN_DRIVE_POWER, speed * Range.clip(0.3 + leg.seconds() / RAMP_UP_S, 0.0, 1.0));
            }

            double axial = forward / dist * speed;
            double lateral = right / dist * speed * STRAFE_COMP;

            // Hold the starting heading. Positive yaw command = clockwise, positive error = needs CCW.
            double headingError = angleDiff(startHeadingDeg, headingDeg);
            double yaw = -Range.clip(headingError * HEADING_KP, -HEADING_MAX_POWER, HEADING_MAX_POWER);

            setDrive(axial, lateral, yaw);

            telemetry.addData("Target", "%.1f, %.1f", targetX, targetY);
            telemetry.addData("Pose", "%.1f, %.1f @ %.0f deg", poseX, poseY, headingDeg);
            telemetry.addData("Time", "%.1f s", autoTimer.seconds());
            telemetry.update();
        }

        if (finalStop || autoTimer.seconds() >= AUTO_LIMIT_S) {
            stopDrive();
        }
    }

    // Same mecanum mixing as the TeleOp.
    private void setDrive(double axial, double lateral, double yaw) {
        double fl = axial + lateral + yaw;
        double fr = axial - lateral - yaw;
        double bl = axial - lateral + yaw;
        double br = axial + lateral - yaw;

        double max = Math.max(1.0, Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));

        frontLeftDrive.setPower(fl / max);
        frontRightDrive.setPower(fr / max);
        backLeftDrive.setPower(bl / max);
        backRightDrive.setPower(br / max);
    }

    private void stopDrive() {
        frontLeftDrive.setPower(0);
        frontRightDrive.setPower(0);
        backLeftDrive.setPower(0);
        backRightDrive.setPower(0);
        inMotion = false;
    }

    // ---------------------------------------------------------------------
    //  Position tracking
    // ---------------------------------------------------------------------

    private void updatePose() {
        double xi = xPodInches();
        double yi = yPodInches();
        double dRight = xi - lastXPodIn;
        double dForward = yi - lastYPodIn;
        lastXPodIn = xi;
        lastYPodIn = yi;

        if (imu != null) {
            headingDeg = startHeadingDeg + imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        }

        // forward = (cos, sin) of the heading, right = (sin, -cos)
        double th = Math.toRadians(headingDeg);
        poseX += dForward * Math.cos(th) + dRight * Math.sin(th);
        poseY += dForward * Math.sin(th) - dRight * Math.cos(th);
    }

    private double xPodInches() {
        return X_POD_SIGN * xPod.getCurrentPosition() / TICKS_PER_INCH;
    }

    private double yPodInches() {
        return Y_POD_SIGN * yPod.getCurrentPosition() / TICKS_PER_INCH;
    }

    // ---------------------------------------------------------------------
    //  Helpers
    // ---------------------------------------------------------------------

    private static double parkX(Alliance a) {
        double inset = ROBOT_WIDTH_IN / 2.0 + PARK_WALL_CLEARANCE_IN;
        return a == Alliance.RED ? inset : FIELD_SIZE_IN - inset;
    }

    private static double parkY(Alliance a) {
        return a == Alliance.RED ? RED_PARK_Y_IN : BLUE_PARK_Y_IN;
    }

    /** Smallest signed angle from current to target, in (-180, 180]. Positive = counter-clockwise. */
    private static double angleDiff(double target, double current) {
        double d = (target - current) % 360.0;
        if (d > 180.0) {
            d -= 360.0;
        } else if (d <= -180.0) {
            d += 360.0;
        }
        return d;
    }
}
