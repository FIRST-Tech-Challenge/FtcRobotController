package org.firstinspires.ftc.teamcode.gen1_Qualifying;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Test_ArmMotor extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo armMotor = hardwareMap.get(Servo.class,"armMotor");

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.x){
                armMotor.setPosition(0.5);
            }
            if (gamepad2.y){
                armMotor.setPosition(0.2);
            }


        }
        armMotor.setPosition(0.0);
    }


}
