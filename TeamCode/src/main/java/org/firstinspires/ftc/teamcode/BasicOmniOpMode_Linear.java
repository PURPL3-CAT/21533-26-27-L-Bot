package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="Basic: Omni Linear OpMode", group="Linear OpMode")
public class BasicOmniOpMode_Linear extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotor flipoutRamp = null;

    private FlywheelController flywheels = new FlywheelController();
    private StopperController stoppers = new StopperController();
    private FieldNavigator navigator = new FieldNavigator();

    private final double TAG_40_SIZE_INCHES = 6.0;

    @Override
    public void runOpMode() {

        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        flipoutRamp = hardwareMap.get(DcMotor.class, "flipout_ramp");
        flipoutRamp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flipoutRamp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flipoutRamp.setDirection(DcMotor.Direction.REVERSE);

        flywheels.init(hardwareMap);
        stoppers.init(hardwareMap);
        navigator.init(hardwareMap); 

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized (Start both servos UP)");
        telemetry.update();

        AprilTagLibrary customTagLibrary = new AprilTagLibrary.Builder()
            .addTags(AprilTagGameDatabase.getCurrentGameTagLibrary())
            .addTag(40, "Custom Tag 40", TAG_40_SIZE_INCHES, DistanceUnit.INCH)
            .build();

        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
            .setTagLibrary(customTagLibrary)
            .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
            .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
            .addProcessor(tagProcessor)
            .build();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            List<AprilTagDetection> detections = tagProcessor.getDetections();
            navigator.update(detections);

            double axial = gamepad1.left_stick_y;
            double lateral = -gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            boolean autoDrivingBack = gamepad1.right_trigger > 0.1;
            if (autoDrivingBack) {
                navigator.calculateDriveToBack();
                axial = navigator.driveAxial;
                lateral = navigator.driveLateral;
                yaw = navigator.driveYaw;
            }

            double frontLeftPower = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower = axial - lateral + yaw;
            double backRightPower = axial + lateral - yaw;

            double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);

            float y_in = (float) (flywheels.getOdoYEncoderTicks() / 1303.8);
            telemetry.addData("Odo Y", y_in);
            
            telemetry.addData("Nectar Speed", "%.0f RPM", flywheels.getNectarRpm());
            telemetry.addData("Pollen Speed", "%.0f RPM", flywheels.getPollenRpm());

            telemetry.addData("IMU Heading (Yaw)", "%.2f°", navigator.getImuYaw());
            telemetry.addData("IMU Pitch / Roll", "%.1f° / %.1f°", navigator.getImuPitch(), navigator.getImuRoll());

            flywheels.update(
                gamepad1.left_bumper, gamepad1.right_bumper, gamepad1.dpad_down, gamepad1.left_trigger,
                gamepad2.dpad_up, gamepad2.dpad_down, gamepad2.y, gamepad2.a, gamepad2.b
            );

            double rampPower = 0.0;
            if (gamepad1.y || gamepad2.left_bumper) {
                rampPower = 0.4;
            } else if (gamepad1.a || gamepad2.right_bumper) {
                rampPower = -0.4;
            }
            flipoutRamp.setPower(rampPower);

            stoppers.update(
                gamepad1.dpad_left, gamepad1.dpad_right, gamepad1.dpad_up,
                gamepad2.left_stick_y, gamepad2.right_stick_y,
                flywheels
            );

            if (navigator.hasValidPose()) {
                telemetry.addData("Tag ID", navigator.getDetectedTagId());
                telemetry.addData("Pose Mode", navigator.isUsingFieldPose() ? "FIELD (robotPose)" : "TAG RELATIVE (ftcPose)");
                telemetry.addData("X / Dist (in)", "%.1f", navigator.getPosX());
                telemetry.addData("Y / Strafe (in)", "%.1f", navigator.getPosY());
                telemetry.addData("Yaw / Bearing (deg)", "%.1f", navigator.getPosYaw());
            } else if (navigator.getDetectedTagId() != -1) {
                telemetry.addData("Tag Seen (No Pose)", navigator.getDetectedTagId());
            } else {
                telemetry.addData("Field Pose", "No Tag Visible");
            }

            if (autoDrivingBack) {
                telemetry.addData("Auto Drive", navigator.hasValidPose() ? "DRIVING TO BACK" : "WAITING FOR TAG");
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            
            telemetry.addData("Nectar Flywheel (LB)", flywheels.getNectarFlywheelStatus());
            telemetry.addData("Pollen Flywheel (RB)", flywheels.getPollenFlywheelStatus());
            
            telemetry.addData("Stopper 1 (Nectar)", stoppers.getNectarState());
            telemetry.addData("Stopper 2 (Pollen)", stoppers.getPollenState());
            telemetry.update();
        }
        visionPortal.close();
    }
}
