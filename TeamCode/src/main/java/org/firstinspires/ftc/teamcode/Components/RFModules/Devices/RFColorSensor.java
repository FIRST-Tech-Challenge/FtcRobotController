package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import com.qualcomm.hardware.rev.RevColorSensorV3;

/**
 * Harry
 */
public class RFColorSensor {
    private RevColorSensorV3 colorSensor;
    public RFColorSensor(String p_deviceName){
        colorSensor = op.hardwareMap.get(RevColorSensorV3.class, p_deviceName);
    }

    public boolean isWhite(){
        boolean white = false;
        return white;
    }

    public boolean isPurple(){
        boolean purple = false;
        return purple;
    }

    public boolean isYellow(){
        boolean yellow = false;
        return yellow;
    }

    public boolean isGreen(){
        boolean green = false;
        return green;
    }
}
