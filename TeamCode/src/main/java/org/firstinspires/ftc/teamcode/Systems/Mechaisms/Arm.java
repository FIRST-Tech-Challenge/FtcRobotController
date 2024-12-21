package org.firstinspires.ftc.teamcode.Systems.Mechaisms;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Wrappers.AnalogServo;

public class Arm {
    private AnalogServo rightServo;
    private AnalogServo leftServo;

    public Arm(Hardware hardware) {
        rightServo = new AnalogServo(hardware.armRight, hardware.armRightEnc);
        leftServo = new AnalogServo(hardware.armLeft, hardware.armLeftEnc);
    }

    public void setPosition(double position) {
        rightServo.setPos(position);
        leftServo.setPos(1-position);
    }

    public double getPosition(){
        return rightServo.getPos();
    }

}
