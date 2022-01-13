package org.firstinspires.ftc.teamcode.gen1_Qualifying;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Test_CapArm extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Servo gripServo = hardwareMap.get(Servo.class,"gripServo");
        Servo capArm = hardwareMap.get(Servo.class,"capArm");

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad2.dpad_up) {
                capArm.setPosition(0.2);
            }
            if (gamepad2.dpad_down) {
                capArm.setPosition(0.77);
            }
            if (gamepad2.dpad_left) {
                gripServo.setPosition(0.5);
            }
            if (gamepad2.dpad_right) {
                gripServo.setPosition(0.0);
            }
        }
    }


}