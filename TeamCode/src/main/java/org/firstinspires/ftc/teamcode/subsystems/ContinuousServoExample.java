package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.teamUtil.RobotConfig;

public class ContinuousServoExample {
    RobotConfig r;

    public CRServo continuousServo;

    ContinuousServoExample(RobotConfig r){
        this.r = r;
        continuousServo = r.opMode.hardwareMap.get(CRServo.class, "CR SERVO");
    }
}
