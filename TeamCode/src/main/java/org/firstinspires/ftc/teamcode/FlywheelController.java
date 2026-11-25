package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class FlywheelController {

    private DcMotorEx flywheelNectar = null; // bottom flywheel
    private DcMotorEx flywheelPollen = null; // top flywheel
    private DcMotor intakeMotor = null;

    // RPM Calculation Trackers
    private ElapsedTime velTimer = new ElapsedTime();
    private int lastNectarPos = 0;
    private int lastPollenPos = 0;
    private double currentNectarRpm = 0.0;
    private double currentPollenRpm = 0.0;

    // goBILDA 5203 Bare Motor Shaft = 28 Ticks Per Revolution.
    private static final double MOTOR_CPR = 28.0; 

    // TARGET RPM CONTROLS (Defaults to 2500 RPM)
    private double nectarTargetRpm = 2500.0;
    private double pollenTargetRpm = 2500.0;
    private final double RPM_STEP = 50.0; 

    // Gamepad button state tracking
    private boolean previousLB = false;
    private boolean previousRB = false;
    private boolean prevGp2DpadUp = false;
    private boolean prevGp2DpadDown = false;
    private boolean prevGp2Y = false;
    private boolean prevGp2A = false;
    private boolean prevGp2B = false;

    private boolean nectarEnabled = false;
    private boolean pollenEnabled = false;

    // Proportional gain for software loop
    private static final double Kp = 0.0003; 

    public void init(HardwareMap hardwareMap) {
        flywheelNectar = hardwareMap.get(DcMotorEx.class, "flywheel_1");
        flywheelPollen = hardwareMap.get(DcMotorEx.class, "flywheel_2");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");

        flywheelNectar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelPollen.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Keep internal firmware PID disabled
        flywheelNectar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelPollen.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Preserving physical directions that you confirmed are correct
        flywheelNectar.setDirection(DcMotor.Direction.FORWARD);
        flywheelPollen.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        nectarEnabled = false;
        pollenEnabled = false;
        nectarTargetRpm = 2500.0;
        pollenTargetRpm = 2500.0;

        lastNectarPos = 0;
        lastPollenPos = 0;
        currentNectarRpm = 0.0;
        currentPollenRpm = 0.0;
        velTimer.reset();
    }

    public void update(
        boolean gp1LB, boolean gp1RB, boolean gp1DpadDown, float gp1LeftTrigger,
        boolean gp2DpadUp, boolean gp2DpadDown, boolean gp2Y, boolean gp2A, boolean gp2B
    ) {
        double seconds = velTimer.seconds();
        if (seconds >= 0.02) { 
            // FIXED: Inverting the raw position reading for nectar so ticks match physical motor velocity direction
            int currentNectarPos = -flywheelNectar.getCurrentPosition(); 
            int currentPollenPos = flywheelPollen.getCurrentPosition();
            
            double deltaNectarTicks = currentNectarPos - lastNectarPos;
            double deltaPollenTicks = currentPollenPos - lastPollenPos;
            
            currentNectarRpm = (deltaNectarTicks / MOTOR_CPR) / seconds * 60.0;
            currentPollenRpm = (deltaPollenTicks / MOTOR_CPR) / seconds * 60.0;

            lastNectarPos = currentNectarPos;
            lastPollenPos = currentPollenPos;
            velTimer.reset();
        }

        if (gp1LB && !previousLB) { nectarEnabled = !nectarEnabled; }
        previousLB = gp1LB;

        if (gp1RB && !previousRB) { pollenEnabled = !pollenEnabled; }
        previousRB = gp1RB;

        if (gp2DpadUp && !prevGp2DpadUp) { nectarTargetRpm = Range.clip(nectarTargetRpm + RPM_STEP, 0.0, 6000.0); }
        if (gp2DpadDown && !prevGp2DpadDown) { nectarTargetRpm = Range.clip(nectarTargetRpm - RPM_STEP, 0.0, 6000.0); }
        prevGp2DpadUp = gp2DpadUp; prevGp2DpadDown = gp2DpadDown;

        if (gp2Y && !prevGp2Y) { pollenTargetRpm = Range.clip(pollenTargetRpm + RPM_STEP, 0.0, 6000.0); }
        if (gp2A && !prevGp2A) { pollenTargetRpm = Range.clip(pollenTargetRpm - RPM_STEP, 0.0, 6000.0); }
        prevGp2Y = gp2Y; prevGp2A = gp2A;

        if (gp2B && !prevGp2B) { nectarTargetRpm = 2500.0; pollenTargetRpm = 2500.0; }
        prevGp2B = gp2B;

        // Software power calculations based on target vs. actual error
        double activeNectarTarget = gp1DpadDown ? -nectarTargetRpm : (nectarEnabled ? nectarTargetRpm : 0.0);
        double activePollenTarget = gp1DpadDown ? -pollenTargetRpm : (pollenEnabled ? pollenTargetRpm : 0.0);

        double nectarError = activeNectarTarget - currentNectarRpm;
        double pollenError = activePollenTarget - currentPollenRpm;

        // FIXED: Hard stop power to 0.0 when flywheels are disabled to ensure they turn off cleanly
        double nectarPower = 0.0;
        if (activeNectarTarget != 0.0) {
            nectarPower = (activeNectarTarget / 6000.0) + (nectarError * Kp);
        }

        double pollenPower = 0.0;
        if (activePollenTarget != 0.0) {
            pollenPower = (activePollenTarget / 6000.0) + (pollenError * Kp);
        }

        // Apply clipped powers directly to the hardware safely
        flywheelNectar.setPower(Range.clip(nectarPower, -1.0, 1.0));
        flywheelPollen.setPower(Range.clip(pollenPower, -1.0, 1.0));

        if (gp1DpadDown) {
            intakeMotor.setPower(-1.0);
        } else {
            intakeMotor.setPower(gp1LeftTrigger);
        }
    }

    public double getNectarRpm() { return currentNectarRpm; }
    public double getPollenRpm() { return currentPollenRpm; }
    public double getNectarTargetRpm() { return nectarTargetRpm; }
    public double getPollenTargetRpm() { return pollenTargetRpm; }
    public boolean isNectarEnabled() { return nectarEnabled; }
    public boolean isPollenEnabled() { return pollenEnabled; }

    public int getOdoYEncoderTicks() { return intakeMotor.getCurrentPosition(); }

    public String getNectarFlywheelStatus() {
        return String.format("%s (%.0f / %.0f RPM)", nectarEnabled ? "ON" : "OFF", currentNectarRpm, nectarTargetRpm);
    }

    public String getPollenFlywheelStatus() {
        return String.format("%s (%.0f / %.0f RPM)", pollenEnabled ? "ON" : "OFF", currentPollenRpm, pollenTargetRpm);
    }
}
