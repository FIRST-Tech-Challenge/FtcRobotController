package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Devices {
        DcMotorEx motorTest;
    public void init(HardwareMap hwmap) {

        

        motorTest = hwmap.get(DcMotorEx.class, "motorTest");


    }
}
