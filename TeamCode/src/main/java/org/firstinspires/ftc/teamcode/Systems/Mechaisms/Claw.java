package org.firstinspires.ftc.teamcode.Systems.Mechaisms;

import org.firstinspires.ftc.teamcode.Hardware.Constants.DepositConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Wrappers.AnalogServo;

public class Claw {
    private AnalogServo servo;

    public Claw(Hardware hardware) {
        servo = new AnalogServo(hardware.claw, hardware.clawEnc);
    }

    public void setPosition(double position) {
        servo.setPos(position);
    }

    public double getPosition() {
        return servo.getPos();
    }

}
