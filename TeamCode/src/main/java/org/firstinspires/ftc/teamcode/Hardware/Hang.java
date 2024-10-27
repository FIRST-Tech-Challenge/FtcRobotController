package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hang {
    private final CRServo hangL;
    private final CRServo hangR;

    public Hang(HardwareMap hardwareMap) {
        this.hangL = hardwareMap.get(CRServo.class, "hangl");
        this.hangR = hardwareMap.get(CRServo.class, "hangr");

    }

    public void move(double power) {
        hangL.setPower(-power);
        hangR.setPower(power);
    }
}
