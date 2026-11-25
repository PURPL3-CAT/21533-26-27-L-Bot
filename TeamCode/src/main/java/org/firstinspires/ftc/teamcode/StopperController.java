package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class StopperController {

    private static final double NECTAR_UP   = 0.50;
    private static final double NECTAR_SIDE = 0.20;
    
    private static final double POLLEN_UP   = 0.00;
    private static final double POLLEN_SIDE = 0.10;

    private static final boolean DEPLOY_ON_INIT = true;
    private static final boolean PULSE_MODE = true; 
    private static final long PULSE_DURATION_MS = 500; 

    // AUTO-PULSE THRESHOLD TUNING CONFIGURATION
    private static final double RPM_TOLERANCE = 150.0;      
    private static final double TIME_THRESHOLD_MS = 300.0;  

    private static class Stopper {
        final ServoImplEx servo;
        final double positionUp;
        final double positionSide;
        boolean deployed = true;   
        
        final ElapsedTime pulseTimer = new ElapsedTime();
        boolean inPulseCycle = false;

        final ElapsedTime thresholdTimer = new ElapsedTime();
        boolean trackingThreshold = false;
        boolean autoPulseTriggered = false; 

        Stopper(ServoImplEx servo, double positionUp, double positionSide) {
            this.servo = servo;
            this.positionUp = positionUp;
            this.positionSide = positionSide;
        }

        void setTarget(boolean toSide) {
            this.deployed = toSide;
            this.servo.setPosition(toSide ? positionSide : positionUp);
        }

        void startPulse() {
            inPulseCycle = true;
            pulseTimer.reset();
            setTarget(false); 
        }

        void handleTimeouts() {
            if (inPulseCycle && pulseTimer.milliseconds() >= PULSE_DURATION_MS) {
                setTarget(true); 
                inPulseCycle = false;
            }
        }

        void evaluateAutoPulse(boolean systemEnabled, double currentRpm, double targetRpm) {
            if (!systemEnabled) {
                trackingThreshold = false;
                autoPulseTriggered = false;
                return;
            }

            boolean atSpeed = Math.abs(currentRpm - targetRpm) <= RPM_TOLERANCE;

            if (atSpeed && !autoPulseTriggered) {
                if (!trackingThreshold) {
                    trackingThreshold = true;
                    thresholdTimer.reset();
                } else if (thresholdTimer.milliseconds() >= TIME_THRESHOLD_MS) {
                    startPulse();
                    autoPulseTriggered = true; 
                }
            } else {
                if (!atSpeed) {
                    trackingThreshold = false;
                }
            }
        }
    }

    private Stopper nectar;   
    private Stopper pollen;   

    private boolean previousDpadUp = false;
    private boolean previousDpadLeft = false;
    private boolean previousDpadRight = false;

    public void init(HardwareMap hardwareMap) {
        ServoImplEx servoNectar = hardwareMap.get(ServoImplEx.class, "stopper_1");
        ServoImplEx servoPollen = hardwareMap.get(ServoImplEx.class, "stopper_2");

        PwmControl.PwmRange goBildaRange = new PwmControl.PwmRange(500, 2500);
        servoNectar.setPwmRange(goBildaRange);
        servoPollen.setPwmRange(goBildaRange);

        nectar = new Stopper(servoNectar, NECTAR_UP, NECTAR_SIDE);
        pollen = new Stopper(servoPollen, POLLEN_UP, POLLEN_SIDE);

        previousDpadUp = false;
        previousDpadLeft = false;
        previousDpadRight = false;

        if (DEPLOY_ON_INIT) {
            nectar.setTarget(true);
            pollen.setTarget(true);
        } else {
            nectar.setTarget(false);
            pollen.setTarget(false);
        }
    }

    public void update(boolean dpadLeft, boolean dpadRight, boolean dpadUp,
                       double manualStick1, double manualStick2,
                       FlywheelController wheels) {

        boolean stick1Active = Math.abs(manualStick1) > 0.1;
        boolean stick2Active = Math.abs(manualStick2) > 0.1;

        if (stick1Active || stick2Active) {
            nectar.inPulseCycle = false;
            pollen.inPulseCycle = false;

            if (stick1Active) {
                double target = NECTAR_SIDE + (manualStick1 * (NECTAR_SIDE - NECTAR_UP));
                nectar.servo.setPosition(target);
                nectar.deployed = (Math.abs(target - NECTAR_SIDE) < Math.abs(target - NECTAR_UP));
            }
            if (stick2Active) {
                double target = POLLEN_SIDE + (manualStick2 * (POLLEN_SIDE - POLLEN_UP));
                pollen.servo.setPosition(target);
                pollen.deployed = (Math.abs(target - POLLEN_SIDE) < Math.abs(target - POLLEN_UP));
            }

            saveButtons(dpadLeft, dpadRight, dpadUp);
            return;
        }

        if (PULSE_MODE) {
            if (dpadLeft && !previousDpadLeft) { nectar.startPulse(); }
            if (dpadRight && !previousDpadRight) { pollen.startPulse(); }

            nectar.evaluateAutoPulse(wheels.isNectarEnabled(), wheels.getNectarRpm(), wheels.getNectarTargetRpm());
            pollen.evaluateAutoPulse(wheels.isPollenEnabled(), wheels.getPollenRpm(), wheels.getPollenTargetRpm());

            nectar.handleTimeouts();
            pollen.handleTimeouts();

            if (dpadUp && !previousDpadUp) {
                nectar.inPulseCycle = false; pollen.inPulseCycle = false;
                nectar.setTarget(false); pollen.setTarget(false);
            }
        } else {
            if (dpadLeft && !previousDpadLeft) { toggle(nectar); }
            if (dpadRight && !previousDpadRight) { toggle(pollen); }
            if (dpadUp && !previousDpadUp) { nectar.setTarget(false); pollen.setTarget(false); }
        }

        saveButtons(dpadLeft, dpadRight, dpadUp);
    }

    private void toggle(Stopper s) { s.setTarget(!s.deployed); }
    private void saveButtons(boolean left, boolean right, boolean up) {
        previousDpadLeft = left; previousDpadRight = right; previousDpadUp = up;
    }

    public void stopAll() {
        nectar.setTarget(nectar.deployed);
        pollen.setTarget(pollen.deployed);
    }

    private String stateOf(Stopper s) {
        if (s.inPulseCycle) return "PULSING (UP)";
        if (s.trackingThreshold) return "LOCKING RPM";
        return s.deployed ? "SIDE" : "UP";
    }

    public String getNectarState() { return stateOf(nectar); }
    public String getPollenState() { return stateOf(pollen); }
}
