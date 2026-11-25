package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

/**
 * RED 2 - Top wall (triangle)
 * Start: x = 60.5 in, y = 134.3 in, robot front facing 270.0 deg
 *        (0 = right wall, 90 = top, 180 = left wall, 270 = bottom of the field diagram).
 * Route: Drops away from the top wall first, then slides left above the center structure.
 * Parks in the RED box only (the box is chosen from the alliance in setStart).
 */
@Autonomous(name = "RED 2 - Top wall (triangle)", group = "Red Auto", preselectTeleOp = "Basic: Omni Linear OpMode")
public class AutoRed2TopTriangle extends AutoBase {

    @Override
    protected void configure() {
        setStart(Alliance.RED, "RED 2 - Top wall (triangle)", 60.5, 134.3, 270.0);
    }

    @Override
    protected void route() {
        via(58.0, 116.0);
        via(30.0, 114.0);
        parkInAllianceZone();
    }
}
