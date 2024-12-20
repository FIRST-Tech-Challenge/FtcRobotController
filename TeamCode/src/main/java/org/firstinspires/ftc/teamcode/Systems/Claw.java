package org.firstinspires.ftc.teamcode.Systems;

import org.firstinspires.ftc.teamcode.Hardware.Constants.DepositConstants;
import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Hardware.Wrappers.AnalogServo;

public class Claw {
    public AnalogServo servo;

    public double openPos = DepositConstants.clawOpenPos;
    public double closedPos = DepositConstants.clawClosedPos;

    public Claw(Hardware hardware) {
        servo = new AnalogServo(hardware.claw, hardware.clawEnc);
    }

}
