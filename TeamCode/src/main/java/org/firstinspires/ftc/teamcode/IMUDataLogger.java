package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "IMU Data Logger", group = "Logging")
public class IMUDataLogger extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Retrieve the Universal/Built-in IMU from the hardware map
        // (The BHI260AP is the standard internal IMU on newer Control Hubs)
        IMU imu = hardwareMap.get(IMU.class, "imu");

        telemetry.addData("Status", "Initializing IMU...");
        telemetry.update();

        // Initialize IMU with default parameters (assuming standard flat hub mounting)
        // Modify parameters here if your hub is mounted vertically or sideways
        imu.initialize(new IMU.Parameters(null));

        telemetry.addData("Status", "Initialized. Ready to start.");
        telemetry.update();

        waitForStart();

        // Main logging loop running while the OpMode is active
        while (opModeIsActive()) {
            // Get orientation data (Roll, Pitch, Yaw)
            Orientation orientation = imu.getRobotOrientation(
                    AxesReference.INTRINSIC, 
                    AxesOrder.ZYX, 
                    AngleUnit.DEGREES
            );
            
            // Get angular velocity data (Rotation speed)
            AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

            // Stream everything clearly to the Driver Station screen
            telemetry.addData("Status", "Running");
            telemetry.addLine("--- Orientation (Deg) ---");
            telemetry.addData("  Yaw (Z)", "%.2f", orientation.firstAngle);
            telemetry.addData("  Pitch (Y)", "%.2f", orientation.secondAngle);
            telemetry.addData("  Roll (X)", "%.2f", orientation.thirdAngle);
            
            telemetry.addLine("--- Angular Velocity (Deg/s) ---");
            telemetry.addData("  X Speed", "%.2f", angularVelocity.xRotationRate);
            telemetry.addData("  Y Speed", "%.2f", angularVelocity.yRotationRate);
            telemetry.addData("  Z Speed", "%.2f", angularVelocity.zRotationRate);
            
            telemetry.update();

            // Small sleep to keep the telemetry loop lightweight (~20Hz refresh rate)
            sleep(50);
        }
    }
}
