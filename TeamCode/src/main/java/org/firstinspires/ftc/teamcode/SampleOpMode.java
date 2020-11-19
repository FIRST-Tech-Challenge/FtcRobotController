package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="Testing Teleop", group="Linear Opmode")

public class SampleOpMode extends LinearOpMode {

    private DcMotor testMotor;
//    private DcMotor testMotor2;
//    private Servo testServo;
//    private double max = 0.00; // Maximum rotational position
//    private double min = 1.00; // Minimum rotational position
//    private double currentPos = 0.00;

    @Override
    public void runOpMode() {

        //initializing every motor, servo, and sensor
        //these names all need to match the names in the config

        testMotor = hardwareMap.get(DcMotor.class, "testmotor");
//        testMotor2 = hardwareMap.get(DcMotor.class, "testmotor2");
//        testServo = hardwareMap.get(Servo.class, "testservo");


        waitForStart();
        while (opModeIsActive()) {
            //gamepad 1
            //motor..........left stick up and down
            //servo..........right stick up

//            // gamepad 1
//            testMotor.setPower( gamepad1.left_stick_y );
            if(gamepad2.right_bumper) {
                testMotor.setPower(1);
//                testMotor2.setPower(.35);
            } else {
                testMotor.setPower(0);
//                testMotor2.setPower(0);
            }
            /*
            when stick is not at zero, it moves the servo. pressing A will set the current pos
            to the new max and pressing B will set the min. when stick is at zero, then pressing A
            or B will set the servo to the max or min position
            */

//            if (gamepad1.right_stick_y != 0) { //if stick is being moved
//                currentPos = Math.abs(gamepad1.right_stick_y);
//                if (gamepad1.a) { max = currentPos; }
//                if (gamepad1.b) { min = currentPos; }
//            } else { //if stick is untouched
//                if (gamepad1.a) { currentPos = max; }
//                if (gamepad1.b) { currentPos = min; }
//            }
//
//            testServo.setPosition(currentPos);

            telemetry.addData("Motor Power", gamepad1.left_stick_y);
            telemetry.addData("Right Stick Pos", gamepad1.right_stick_y);
//            telemetry.addData("Servo Position", currentPos);
//            telemetry.addData("Servo Max", max);
//            telemetry.addData("Servo Min", min);
            telemetry.update();

        }
    }
}
