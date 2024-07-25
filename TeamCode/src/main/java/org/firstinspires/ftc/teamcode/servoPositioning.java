package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 */

@TeleOp(name="servoPositioning", group="Linear OpMode")
//@Disabled
public class servoPositioning extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        Servo leftIntakeServo = hardwareMap.get(Servo.class, "leftIntakeServo");
        Servo rightIntakeServo = hardwareMap.get(Servo.class, "rightIntakeServo");

        double leftIntakePos = 0;
        double rightIntakePos = 0;
        boolean rightSide = false;
        boolean leftSide = false;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        telemetry.addData("Controls: ", "dpad to increase/decrease servo positions, x for left, y for right, a/b for both");

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            leftIntakeServo.setPosition(leftIntakePos);
            rightIntakeServo.setPosition(rightIntakePos);
            if (gamepad1.x){
                leftSide = true;
                rightSide = false;
            }
            if (gamepad1.y){
                leftSide = false;
                rightSide = true;
            }
            if (gamepad1.a || gamepad1.b){
                leftSide = true;
                rightSide = true;
            }
            if (gamepad1.dpad_up){
                if (leftSide)
                    leftIntakePos += 0.01;
                if (rightSide)
                    rightIntakePos += 0.01;
            }
            if (gamepad1.dpad_down){
                if (leftSide)
                    leftIntakePos -= 0.01;
                if (rightSide)
                    rightIntakePos -= 0.01;
            }
            telemetry.addData("Servos", "leftPos (%.2f), rightPos (%.2f)", leftIntakePos, rightIntakePos);
            telemetry.update();
        }
    }
}
