package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Flywheel 2 test (standalone, does not touch the drivetrain or any other motor).
 *
 * Left bumper  : toggle flywheel_2 on/off. Each time it turns on, it starts at 100%.
 * A button     : lower flywheel_2 power by 0.01 per press (works while running).
 */
@TeleOp(name = "Test: Flywheel 2", group = "Test")
public class Flywheel2Test extends LinearOpMode {

    private DcMotor flywheel2 = null;

    private double flywheelPower = 1.0;
    private final double POWER_INCREMENT = 0.01;

    private boolean flywheel2Enabled = false;
    private boolean previousLB = false;
    private boolean previousA = false;

    @Override
    public void runOpMode() {

        flywheel2 = hardwareMap.get(DcMotor.class, "flywheel_2");
        flywheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel2.setDirection(DcMotor.Direction.FORWARD);   // switch to REVERSE to spin the other way

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // left bumper: toggle flywheel_2 on/off; each time it turns on, start at 100%
            if (gamepad1.left_bumper && !previousLB) {
                flywheel2Enabled = !flywheel2Enabled;
                if (flywheel2Enabled) {
                    flywheelPower = 1.0;
                }
            }
            previousLB = gamepad1.left_bumper;

            // A: lower flywheel_2 power by 0.01 per press
            if (gamepad1.a && !previousA) {
                flywheelPower -= POWER_INCREMENT;
            }
            previousA = gamepad1.a;
            flywheelPower = Math.max(0.0, Math.min(1.0, flywheelPower));

            flywheel2.setPower(flywheel2Enabled ? flywheelPower : 0);

            telemetry.addData("Flywheel2 Enabled", flywheel2Enabled);
            telemetry.addData("Flywheel2 Power", "%.2f", flywheelPower);
            telemetry.addData("Flywheel2 Percent", "%.0f%%", flywheelPower * 100);
            telemetry.addData("Controls", "LB = on/off (starts at 100%), A = -0.01");
            telemetry.update();
        }
    }
}