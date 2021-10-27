package org.firstinspires.ftc.teamcode.Test;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Outtake {
    public Outtake (HardwareMap hardwareMap) {
        motor1 = hardwareMap.dcMotor.get("motorOuttake")


    }

    private DcMotor motor1;
    private Servo servo1;


}
