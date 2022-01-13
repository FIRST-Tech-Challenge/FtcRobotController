package org.firstinspires.ftc.teamcode.gen1_Qualifying;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Test_ArmMotor2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo armMotor = hardwareMap.get(Servo.class,"armMotor");

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.right_stick_y >= 0) {
                armMotor.setPosition(-gamepad2.right_stick_y * 0.8);

            } else if (gamepad2.right_stick_y < 0) {
                armMotor.setPosition(-gamepad2.right_stick_y * 0.2);
            }


        }
        armMotor.setPosition(0.0);
    }


}
