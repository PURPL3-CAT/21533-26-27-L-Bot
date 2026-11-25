package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Flywheel 1 test (standalone, does not touch the drivetrain or any other motor).
 *
 * Right bumper : toggle flywheel_1 on/off. Each time it turns on, it starts at 100%.
 * A button     : lower flywheel_1 power by 0.01 per press (works while running).
 */
@TeleOp(name = "Test: Flywheel 1", group = "Test")
public class Flywheel1Test extends LinearOpMode {

    private DcMotor flywheel1 = null;

    private double flywheelPower = 1.0;
    private final double POWER_INCREMENT = 0.01;

    private boolean flywheel1Enabled = false;
    private boolean previousRB = false;
    private boolean previousA = false;

    @Override
    public void runOpMode() {

        flywheel1 = hardwareMap.get(DcMotor.class, "flywheel_1");
        flywheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel1.setDirection(DcMotor.Direction.FORWARD);   // switch to REVERSE to spin the other way

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // right bumper: toggle flywheel_1 on/off; each time it turns on, start at 100%
            if (gamepad1.right_bumper && !previousRB) {
                flywheel1Enabled = !flywheel1Enabled;
                if (flywheel1Enabled) {
                    flywheelPower = 1.0;
                }
            }
            previousRB = gamepad1.right_bumper;

            // A: lower flywheel_1 power by 0.01 per press
            if (gamepad1.a && !previousA) {
                flywheelPower -= POWER_INCREMENT;
            }
            previousA = gamepad1.a;
            flywheelPower = Math.max(0.0, Math.min(1.0, flywheelPower));

            flywheel1.setPower(flywheel1Enabled ? flywheelPower : 0);

            telemetry.addData("Flywheel1 Enabled", flywheel1Enabled);
            telemetry.addData("Flywheel1 Power", "%.2f", flywheelPower);
            telemetry.addData("Flywheel1 Percent", "%.0f%%", flywheelPower * 100);
            telemetry.addData("Controls", "RB = on/off (starts at 100%), A = -0.01");
            telemetry.update();
        }
    }
}