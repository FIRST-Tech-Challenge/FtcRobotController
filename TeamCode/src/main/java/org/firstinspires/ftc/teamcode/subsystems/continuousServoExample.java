package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.teamUtil.robotConfig;

public class continuousServoExample {
    robotConfig r;

    public CRServo continuousServo;

    continuousServoExample(robotConfig r){
        this.r = r;
        continuousServo = r.hardwareMap.get(CRServo.class, "CR SERVO");
    }
}
