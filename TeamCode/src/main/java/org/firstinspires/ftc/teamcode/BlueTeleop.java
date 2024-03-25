/*
Copyright 2023 FIRST Tech Challenge Team 23246

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and
associated documentation files (the "Software"), to deal in the Software without restriction,
including without limitation the rights to use, copy, modify, merge, publish, distribute,
sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial
portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a PushBot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Remove a @Disabled the on the next line or two (if present) to add this OpMode to the Driver Station OpMode list,
 * or add a @Disabled annotation to prevent this OpMode from being added to the Driver Station
 */
@TeleOp(name="BlueTeleop")

public class BlueTeleop extends LinearOpMode {


    @Override
    public void runOpMode() {
        double ArmPower;
        DcMotor BackLeft = hardwareMap.dcMotor.get("BackLeft");
        DcMotor BackRight = hardwareMap.dcMotor.get("BackRight");
        DcMotor FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        DcMotor FrontRight = hardwareMap.dcMotor.get("FrontRight");

        DcMotorSimple ArmMotor  = hardwareMap.get(DcMotorSimple.class, "ArmMotor");
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        CRServo turn = hardwareMap.crservo.get("turn");
        Servo claw = hardwareMap.servo.get("claw");

        telemetry.addData("Value Of RightJoystickY: ", String.valueOf(gamepad2.right_stick_y));

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            telemetry.addData("Status", "Running");
            telemetry.update();
            double forward=-gamepad1.left_stick_x;
            double strafe=-gamepad1.left_stick_y;
            boolean turnRight=gamepad1.right_bumper;
            boolean turnLeft=gamepad1.left_bumper;
            double turnAmount =gamepad1.right_stick_x;


            double arm = gamepad2.left_stick_y;
            ArmPower    = Range.clip(arm, -1.0, 1.0) ;
            if (gamepad2.b){//claw
                claw.setPosition(.88);
            } else if (gamepad2.a) {
                claw.setPosition(.12);
            }
            if(gamepad1.b){
                forward=-gamepad1.left_stick_y;
                strafe=gamepad1.left_stick_x;
            }
            turn.setPower(gamepad2.right_stick_y/4);


            if(gamepad1.a){
                strafe/=2;
                forward/=2;
            }
                        

            turnAmount /=2;
            double denominator = Math.max(Math.abs(forward)+Math.abs(strafe)+Math.abs(turnAmount), 1);
            FrontRight.setPower((forward-strafe+turnAmount)/denominator);
            FrontLeft.setPower((forward+strafe+turnAmount)/denominator);
            BackLeft.setPower((forward-strafe-turnAmount)/denominator);
            BackRight.setPower((forward+strafe-turnAmount)/denominator);
            ArmMotor.setPower(ArmPower/3);


        }
    }
}
