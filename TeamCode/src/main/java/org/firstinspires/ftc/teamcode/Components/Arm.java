package org.firstinspires.ftc.teamcode.Components;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;

/**
 * Harry
 */
public class Arm extends RFServo {
    private final double LOWER_LIMIT = 0.0, UPPER_LIMIT=0.0;
    public Arm() {
        super("armServo", 0);
        this.scaleRange(LOWER_LIMIT,UPPER_LIMIT);
    }
    public void update() {

    }
}
