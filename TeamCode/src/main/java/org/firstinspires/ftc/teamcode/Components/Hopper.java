package org.firstinspires.ftc.teamcode.Components;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFLEDStrip;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;

/**
 * Harry
 */
public class Hopper extends RFServo {
    RFLEDStrip leds;
    public Hopper(){
        super("hopperServo", 0);
        leds = new RFLEDStrip();
    }
    public void update(){

    }
}
