package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hang {
    private final String hangLName;
    private final String hangRName;
    private final CRServo hangL;
    private final CRServo hangR;

    public Hang(HardwareMap hardwareMap) {
        this.hangLName = "hangl";
        this.hangRName = "hangr";
        this.hangL = hardwareMap.get(CRServo.class, hangLName);
        this.hangR = hardwareMap.get(CRServo.class, hangRName);

    }

    public Hang(HardwareMap hardwareMap, String hangLName, String hangRName) {
        this.hangLName = hangLName;
        this.hangRName = hangRName;
        this.hangL = hardwareMap.get(CRServo.class, hangLName);
        this.hangR = hardwareMap.get(CRServo.class, hangRName);
    }

    public void move(double power) {
        hangL.setPower(-power);
        hangR.setPower(power);
    }
}
