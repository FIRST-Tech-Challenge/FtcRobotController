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
    private Servo testServo;
    private double max = 0.01; // Maximum rotational position
    private double min = 0.99; // Minimum rotational position
    private String currentPos = "min";
    private boolean servoButtonIsDown = false;

    @Override
    public void runOpMode() {

        //initializing every motor, servo, and sensor
        //these names all need to match the names in the config

        hardwareMap.get(DcMotor.class, "testmotor");
        hardwareMap.get(Servo.class, "testservo");


        waitForStart();
        while (opModeIsActive()) {
            //gamepad 1
            //motor..........left stick up and down
            //servo..........a button

            // gamepad 1
            testMotor.setPower( gamepad1.left_stick_y );

            if (gamepad1.a && !servoButtonIsDown) { //make sure the switch triggers only once per button press
                servoButtonIsDown = true;
                if(currentPos == "min") { //toggle between the two positions of the servo
                    currentPos = "max";
                } else if(currentPos == "max") {
                    currentPos = "min";
                }
            } else if (!gamepad1.a) {
                servoButtonIsDown = false;
            }

            if(currentPos == "min") {
                testServo.setPosition(min);
            } else if(currentPos == "max") {
                testServo.setPosition(max);
            }

            telemetry.addData("Status", "Run Time: ");
            telemetry.addData("Motor Power", gamepad1.left_stick_y);
            telemetry.addData("Servo Position", currentPos);
            telemetry.update();

        }
    }
}
