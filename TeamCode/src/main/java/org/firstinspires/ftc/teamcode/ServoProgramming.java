package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Programming")
public class ServoProgramming extends LinearOpMode {
    @Override
    public void runOpMode(){
        Servo ServoTester = hardwareMap.servo.get("ServoTester");
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.a){
                ServoTester.setPosition(0);
            }
            else if (gamepad1.b){
                ServoTester.setPosition(.5);
            }
            else if (gamepad1.y){
                ServoTester.setPosition(1);
            }
            telemetry.addData("Servo Position", ServoTester.getPosition());
            telemetry.update(); // 0 is intake position for the intake four bar // 1 is transfer position for the intake four bar
        }
    }
}
