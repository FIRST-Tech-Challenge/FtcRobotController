package org.firstinspires.ftc.teamcode.zeroCode.utils;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.zeroCode.finalClassess.Constants;

public class Ramp {
    Servo ramp;
    public Ramp() {
        ramp = Constants.ramp.ramp;
    }

    public void turnToPollen() {
        ramp.setPosition(Constants.ramp.rampPollenAngle);
    }

    public void turnToNectar() {
        ramp.setPosition(Constants.ramp.rampNectarAngle);
    }

}
