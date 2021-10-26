package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Test_ArmMotor extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo armMotor = hardwareMap.get(Servo.class,"armMotor");

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.a){
                armMotor.setPosition(1);
            }
            if (gamepad2.b){
                armMotor.setPosition(-1);
            }


        }
        armMotor.setPosition(0.0);
    }


}
