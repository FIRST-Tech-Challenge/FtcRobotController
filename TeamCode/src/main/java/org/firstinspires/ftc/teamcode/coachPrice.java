/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Mr. Price's teleOp for test and explanation
 *
 * Need to confirm the drive system and add the drone servo to have a full test case
 * Also need the telemetry to read all sensor values
 *
 * I2C Bus 1: Port 1: FDS Rev 2m Distance sensor:  distFront
 * Port2 RDS: Rev 2m Distance sensor: distRear
 * See the sensor's product page: https://www.revrobotics.com/rev-31-1505/
 */

@TeleOp(name="coachTeleOp", group="TeleOp")

public class coachPrice extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor leftFront;
    public DcMotor rightFront;
    public DcMotor rightRear;
    public DcMotor leftRear;
    public DcMotor leftArm;
    public DcMotor rightArm;
    public CRServo rightWheel;
    public CRServo leftWheel;
    public Servo drone;

    public ColorSensor colorSensor;
    public DistanceSensor distFront;
    public DistanceSensor distRear;

    double clawOffset = 0;

    /*public static final double MID_SERVO   =  0.5 ;
    public static final double CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    */

    public void runOpMode() throws InterruptedException {
        double driveY;
        double strafe;
        double turn;
        double RWPower = 0;
        double LWPower;

    // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F,0F,0F};
    // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // Define and Initialize Motors
        leftFront = hardwareMap.get(DcMotor.class, "FLM");
        rightFront = hardwareMap.get(DcMotor.class, "FRM");
        rightRear = hardwareMap.get(DcMotor.class, "RRM");
        leftRear = hardwareMap.get(DcMotor.class, "RLM");
        rightWheel = hardwareMap.crservo.get("RSW");
        leftWheel = hardwareMap.crservo.get("LSW");

        // bPrevState and bCurrState represent the previous and current state of the button.
        boolean bPrevState = false;
        boolean bCurrState = false;

        // bLedOn represents the state of the LED.
        boolean bLedOn = true;

        // get a reference to our ColorSensor object.
        colorSensor = hardwareMap.get(ColorSensor.class, "CLR");

        // Set the LED in the beginning
        colorSensor.enableLed(bLedOn);

        distFront = hardwareMap.get(DistanceSensor.class, "FDS");
        distRear = hardwareMap.get(DistanceSensor.class, "RDS");

        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) distFront;

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        // If there are encoders connected, switch to RUN_USING_ENCODER mode for greater accuracy
        // leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Define and initialize ALL installed servos.
       /* leftClaw  = hardwareMap.get(Servo.class, "left_hand");
        rightClaw = hardwareMap.get(Servo.class, "right_hand");
        leftClaw.setPosition(MID_SERVO);
        rightClaw.setPosition(MID_SERVO);
        */

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        if (isStopRequested()) return;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            while (opModeIsActive()) {
                driveY = -gamepad1.left_stick_y;
                strafe = gamepad1.left_stick_x * 1.1;
                turn = gamepad1.right_stick_x;

                // Denominator is the largest motor power (absolute value) or 1
                // This ensures all the powers maintain the same ratio,
                // but only if at least one is out of the range [-1, 1]
                double denominator = Math.max(Math.abs(driveY) + Math.abs(strafe) + Math.abs(turn), 1);
                double frontLeftPower = (driveY + strafe + turn) / denominator;
                double backLeftPower = (driveY - strafe + turn) / denominator;
                double frontRightPower = (driveY - strafe - turn) / denominator;
                double backRightPower = (driveY + strafe - turn) / denominator;
                // Remember that the right is opposite of the left
                if (distFront.getDistance(DistanceUnit.CM)<10) {
                    leftFront.setPower(0.2 * frontLeftPower);
                    leftRear.setPower(0.2 * backLeftPower);
                    rightFront.setPower(0.2 * frontRightPower);
                    rightRear.setPower(0.2 * backRightPower);
                }
                else {
                    leftFront.setPower(frontLeftPower);
                    leftRear.setPower(backLeftPower);
                    rightFront.setPower(frontRightPower);
                    rightRear.setPower(backRightPower);
                }

                // Controlling the pixel pick-up with the dpad
                if (gamepad2.dpad_left) {
                    LWPower = 0.2;
                    RWPower = -0.2;
                }
                else if (gamepad2.dpad_right) {
                    LWPower = -0.2;
                    RWPower = 0.2;
                }
                else if (gamepad2.y)
                    LWPower = -0.2;
                else if (gamepad2.x)
                    LWPower = 0.2;
                else {
                    LWPower = 0;
                    RWPower = 0;
                }

                leftWheel.setPower(LWPower);
                rightWheel.setPower(RWPower);
                
// Adding telemetry readouts
            telemetry.addData(">", "Robot Running");
            telemetry.addData("Y", driveY);
            telemetry.addData("strafe", strafe);
            telemetry.addData("turn", turn);
// check the status of the x button on either gamepad.
            bCurrState = gamepad1.x;

            // check for button state transitions.
            if (bCurrState && (bCurrState != bPrevState))  {

                // button is transitioning to a pressed state. So Toggle LED
                bLedOn = !bLedOn;
                colorSensor.enableLed(bLedOn);
            }

            // update previous state variable.
            bPrevState = bCurrState;

            // convert the RGB values to HSV values.
            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

            // send the info back to driver station using telemetry function.
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.addData("Hue", hsvValues[0]);

            telemetry.addData("FDS", distFront.getDeviceName() );
            telemetry.addData("range", String.format("%.01f cm", distFront.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f in", distFront.getDistance(DistanceUnit.INCH)));

            telemetry.addData("RDS", distRear.getDeviceName() );
            telemetry.addData("range", String.format("%.01f cm", distRear.getDistance(DistanceUnit.CM)));
            telemetry.addData("range", String.format("%.01f in", distRear.getDistance(DistanceUnit.INCH)));

            // Rev2mDistanceSensor specific methods.
            telemetry.addData("ID", String.format("%x", sensorTimeOfFlight.getModelID()));
            telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));

            telemetry.update();

                // Use gamepad buttons to move arm up (Y) and down (A)
            /*if (gamepad1.y)
                leftArm.setPower(ARM_UP_POWER);
            else if (gamepad1.a)
                leftArm.setPower(ARM_DOWN_POWER);
            else
                leftArm.setPower(0.0);
*/
                // Send telemetry message to signify robot running;
               /* telemetry.addData("claw", "Offset = %.2f", clawOffset);
                telemetry.addData("left", "%.2f", left);
                telemetry.addData("right", "%.2f", right);
                telemetry.update();

                // Pace this loop so jaw action is reasonable speed.
                sleep(50);
                */

            }
        }
    }
}