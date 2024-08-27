package org.firstinspires.ftc.teamcode.teamprograms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Double Servo", group = "Z")
public class DoubleServo extends LinearOpMode {

    Servo servoRight, servoLeft;


// TODO: CURRENTLY NOT SET UP FOR DOUBLE SERVO LIFT

    public void runOpMode() {

        servoRight = hardwareMap.get(Servo.class, "servoRight");
        servoLeft = hardwareMap.get(Servo.class, "servoLeft");

        servoLeft.setDirection(Servo.Direction.REVERSE);

        telemetry.addLine("Press Start");
        telemetry.update();


        waitForStart();




        while (opModeIsActive()) {


            // TODO: INDIVIDUAL SERVOS (comment out if needed)
            // right servo movement
//            if (gamepad1.x) {
//                servoRight.setPosition(0);
//            } else if (gamepad1.a) {
//                servoRight.setPosition(0.5);
//            } else if (gamepad1.b) {
//                servoRight.setPosition(1);
//            }
//            // left servo movement
//            if (gamepad1.dpad_left) {
//                servoLeft.setPosition(0);
//            } else if (gamepad1.dpad_down) {
//                servoLeft.setPosition(0.5);
//            } else if (gamepad1.dpad_right) {
//                servoLeft.setPosition(1);
//            }

            // TODO: DOUBLE SERVO (comment out if needed)
            if (gamepad1.x) {
                servoRight.setPosition(0);
                servoLeft.setPosition(0);
            } else if (gamepad1.a) {
                servoRight.setPosition(0.5);
                servoLeft.setPosition(0.5);
            } else if (gamepad1.b) {
                servoRight.setPosition(1);
                servoLeft.setPosition(1);
            }



            telemetry.addData("SERVO R", servoRight.getPosition());
            telemetry.addData("SERVO L", servoLeft.getPosition());
            telemetry.update();

        }

    }

}
