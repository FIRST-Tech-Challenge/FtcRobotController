package org.firstinspires.ftc.masters.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Diffy Test ")
public class diffyTest extends LinearOpMode {

    public void runOpMode() throws InterruptedException {

        Servo servo1 = hardwareMap.servo.get("elbow1");
        Servo servo2 = hardwareMap.servo.get("elbow2");



        double servo1pos = 0.5;
        double servo2pos = 0.5;

        servo1.setPosition(servo1pos);
        servo2.setPosition(servo2pos);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                servo1pos += 0.01;
                servo2pos += 0.01;
            }

            if (gamepad1.dpad_down) {
                servo1pos -= 0.01;
                servo2pos -= 0.01;
            }

            if (gamepad1.dpad_left) {
                servo1pos += 0.01;
                servo2pos -= 0.01;
            }

            if (gamepad1.dpad_right) {
                servo1pos -= 0.01;
                servo2pos += 0.01;
            }

            servo1.setPosition(servo1pos);
            servo2.setPosition(servo2pos);

        }
    }
}

