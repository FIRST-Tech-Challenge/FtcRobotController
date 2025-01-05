package org.firstinspires.ftc.teamcode.Opmode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class servoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo test = hardwareMap.get(Servo.class, "t");
        while(opModeInInit()){
            test.setPosition(.6);
        }
        waitForStart();
        while(opModeIsActive()){
            test.setPosition(.5);
        }
    }
}
