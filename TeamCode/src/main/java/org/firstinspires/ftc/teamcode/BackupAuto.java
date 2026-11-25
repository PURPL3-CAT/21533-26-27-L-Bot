/*
Copyright 2026 FIRST Tech Challenge Team 21533

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "BackupAuto", group = "Autonomous")
public class BackupAuto extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    // --- TUNING CONSTANTS (Adjust power and durations on the field) ---
    private final double FORWARD_POWER       = 0.5;
    private final double FORWARD_SECONDS     = 1.2;

    private final double STRAFE_POWER        = 0.5;
    private final double STRAFE_SECONDS      = 2.5;
    private final double STRAFE_BACK_SECONDS = 0.1;

    private final double PAUSE_SECONDS       = 0.15; // Brief stop between moves to prevent wheel slip

    @Override
    public void runOpMode() {
        // Initialize drive motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        // Set motor directions to match your TeleOp
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Brake when stopped so movements don't coast into each other
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Step 1: Drive Forward
        driveForTime(FORWARD_POWER, 0.0, 0.0, FORWARD_SECONDS, "Step 1: Driving Forward");
        driveForTime(0.0, 0.0, 0.0, PAUSE_SECONDS, "Pausing...");

        // Step 2: Strafe Right
        driveForTime(0.0, STRAFE_POWER, 0.0, STRAFE_SECONDS, "Step 2: Strafing Right");
        driveForTime(0.0, 0.0, 0.0, PAUSE_SECONDS, "Pausing...");

        // Step 3: Strafe Left (opposite direction) for 0.1s
        driveForTime(0.0, -STRAFE_POWER, 0.0, STRAFE_BACK_SECONDS, "Step 3: Strafing Left (0.1s)");

        // Stop all motors
        setDriveVectors(0.0, 0.0, 0.0);

        // Hold still until Autonomous ends
        while (opModeIsActive()) {
            telemetry.addData("Status", "Sequence Complete - Stopped");
            telemetry.update();
        }
    }

    private void driveForTime(double axial, double lateral, double yaw, double seconds, String statusMessage) {
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < seconds) {
            setDriveVectors(axial, lateral, yaw);
            telemetry.addData("Status", statusMessage);
            telemetry.addData("Time", "%.2f / %.2f s", runtime.seconds(), seconds);
            telemetry.update();
        }
    }

    private void setDriveVectors(double axial, double lateral, double yaw) {
        // Matches the exact kinematics from BasicOmniOpMode_Linear
        double frontLeftPower  = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower   = axial - lateral + yaw;
        double backRightPower  = axial + lateral - yaw;

        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower  /= max;
            frontRightPower /= max;
            backLeftPower   /= max;
            backRightPower  /= max;
        }

        frontLeftDrive.setPower(frontLeftPower);
        frontRightDrive.setPower(frontRightPower);
        backLeftDrive.setPower(backLeftPower);
        backRightDrive.setPower(backRightPower);
    }
}
