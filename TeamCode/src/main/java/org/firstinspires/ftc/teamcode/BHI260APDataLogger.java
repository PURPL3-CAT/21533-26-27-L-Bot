package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "BHI260AP Data Logger", group = "Logging")
public class BHI260APDataLogger extends LinearOpMode {
    private IMU imu;
    private final ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        // Retrieve the built-in BHI260AP IMU from the FTC hardware map
        imu = hardwareMap.get(IMU.class, "imu");

        telemetry.addData("Status", "Initializing IMU...");
        telemetry.update();

        // Configure orientation: Mounted upright on the left side wall with ports pointing skyward
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Initialize with the correct orientation parameters to avoid NullPointerException
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.addData("Status", "Initialized. Ready.");
        telemetry.update();

        waitForStart();
        timer.reset();

        // Main logging loop running at ~5Hz (200ms interval) to match your design
        while (opModeIsActive()) {
            Orientation orientation = imu.getRobotOrientation(
                    AxesReference.INTRINSIC, 
                    AxesOrder.ZYX, 
                    AngleUnit.DEGREES
            );
            
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            // Log data cleanly to the FTC Telemetry stream
            telemetry.addData("Time", "%.2f s", timer.seconds());
            telemetry.addLine("--- Orientation (Deg) ---");
            telemetry.addData("  Yaw (Z)", "%.2f", orientation.firstAngle);
            telemetry.addData("  Pitch (Y)", "%.2f", orientation.secondAngle);
            telemetry.addData("  Roll (X)", "%.2f", orientation.thirdAngle);
            
            telemetry.addLine("--- Angular Velocity (Deg/s) ---");
            telemetry.addData("  X Speed", "%.2f", angularVelocity.xRotationRate);
            telemetry.addData("  Y Speed", "%.2f", angularVelocity.yRotationRate);
            telemetry.addData("  Z Speed", "%.2f", angularVelocity.zRotationRate);
            
            telemetry.update();

            // 200ms delay as requested
            sleep(200);
        }
    }
}
