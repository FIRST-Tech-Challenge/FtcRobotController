package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


// This defines the program as a TeleOp program instead of an Autonomous one.

@TeleOp

/*
*  This is making the class Tutorial (this entire file) as a public class, which means it is visible
*  to any other file in the entire program.
*
*  It makes the class inherit characteristics from the LinearOpMode class that was written by FTC.
 */
public class Tutorial extends LinearOpMode {

    //These two lines define what type of object we're creating, as well as what we call it for the rest of the file.

    private DcMotor TestMotor;
    private Servo TestServo;

    /*
    * "runOpMode" here is a Method. It's essentially another program written somewhere else in the files.
    *
    * Every time you call the "LinearOpMode" method, it requires you to have the "runOpMode" method.
    *
    * It needs to be overridden as the premade FTC code creates it as a method, but you need your code to run
    * instead of what is in the method already.
    */

    @Override
    public void runOpMode() throws InterruptedException {
        TestMotor = hardwareMap.get(DcMotor.class, "TestMotor");
        TestServo = hardwareMap.get(Servo.class, "TestServo");

        TestServo.setPosition(0.5);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        double tgtPower = 0;

        while (opModeIsActive()){

            tgtPower = this.gamepad1.left_stick_x;

            TestMotor.setPower(tgtPower);



            if (gamepad1.x){
                TestServo.setPosition(0);
            } else if (gamepad1.x) {
                TestServo.setPosition(0.33);
            } else if (gamepad1.b) {
                TestServo.setPosition((0.66));
            } else if (gamepad1.y) {
                TestServo.setPosition(1);
            }



            telemetry.addData("Status","Running");
            telemetry.addData("Target Power", tgtPower);
            telemetry.addData("Motor Power", TestMotor.getPower());
            telemetry.update();

        }

    }
}
