package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Test_ResetZero extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo armMotor = hardwareMap.get(Servo.class,"armMotor");

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.x){
                armMotor.setPosition(0.97);
            }
            if (gamepad1.y){
                armMotor.setPosition(0.92); //second level
            }
            if (gamepad1.b){
                armMotor.setPosition(0.95); //second level
            }

           /* if (gamepad1.b){
                armMotor.setPosition(0.75);
                //0.75 Base
            }*/
        }
    }


}
