package org.firstinspires.ftc.teamcode.team10515;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.team10515.auto.UGBase;

@TeleOp
public class TeleOp1 extends UGBase {
    @Override
    public void runOpMode() throws InterruptedException {

        checkForButtons1();
        checkForButtons2();

        double robotAngle;

        if (gamepad1.right_stick_y > 0) {
            robotAngle = (gamepad1.left_stick_x + 1) * 90;
        } else if (gamepad1.right_stick_y < 0) {
            robotAngle = ((gamepad1.left_stick_x + 1) * 90) + 180;
        } else {
            if (gamepad1.left_stick_x > 0) {
                robotAngle = 90;
            } else {
                robotAngle = 270;
            }
        }

    }

    void checkForButtons1() {
        if (gamepad1.left_bumper) {
            // forklift down position (0 or 90 degrees)
        }
        if (gamepad1.right_bumper) {
            // forklift up position (20 or 110 degrees)
        }
        if (gamepad1.left_trigger > 0) {
            // decrease robot speed
        }
        if (gamepad1.right_trigger > 0) {
            // increase robot speed
        }
        if (gamepad1.left_stick_button) {
            // set robot speed to default value
        }
        if (gamepad1.right_stick_button) {
            // change boolean flag so that robot cannot move
        }
    }

    void checkForButtons2() {
        if (gamepad2.a) {
            //Hit ring into launch position
        }
        if (gamepad2.y) {
            //Start and Stop the shooter
        }
        if (gamepad2.left_trigger > 0) {
            //Stop Shooter
        }
        if (gamepad2.right_trigger > 0) {
            //Start Shooter
        }
        if (gamepad2.right_bumper) {
            //Start Intake
        }
        if (gamepad2.left_bumper) {
            //Stop intake
        }
        if (gamepad2.dpad_up) {
            //Set Shooter Speed to 4
        }
        if (gamepad2.dpad_right) {
            //Set Shooter Speed to 1
        }
        if (gamepad2.dpad_down) {
            //Set Shooter Speed to 2
        }
        if (gamepad2.dpad_left) {
            //Set Shooter Speed to 3
        }
    }
}
