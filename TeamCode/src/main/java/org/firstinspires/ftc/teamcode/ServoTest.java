package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
//testing push directly from git (broswer)
@Autonomous
public class ServoTest extends LinearOpMode {
    private Servo servoTest;

    @Override

    public void runOpMode() {
        servoTest = hardwareMap.get(Servo.class, "ServoTest");

    }

}
