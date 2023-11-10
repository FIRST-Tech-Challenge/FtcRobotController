package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.isTeleop;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;

public class Clamp extends RFServo {
    public static double CLAMP = 0.2, UNCLAMP = 0.4;
    boolean clamped;

    public Clamp() {
        super("clampServo", 1.0);
        clamped = false;
        super.setFlipTime(0.1);
        if(isTeleop)
            super.setPosition(UNCLAMP);
        else
            super.setPosition(CLAMP);
        super.setLastTime(-100);


    }

    public void clamp() {
        LOGGER.log("clamped");
        super.setPosition(CLAMP);
        if (super.getTarget()== CLAMP) {
      clamped = true;
        }
    }

    public void unclamp() {
        LOGGER.log("unclamped");
        super.setPosition(UNCLAMP);
        if (super.getTarget()!= CLAMP) {
            clamped = false;
        }
    }


    public boolean getClamped() {
        return clamped;
    }
}
