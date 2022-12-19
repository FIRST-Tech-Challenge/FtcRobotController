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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="LiftyMcLifterFace", group="SCC")
public class LiftyMcLifterFace extends LinearOpMode {

    // Create motor and servo objects
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftRearMotor;
    private DcMotor rightRearMotor;
    private DcMotor liftMotor;
    private ElapsedTime runtime = new ElapsedTime();
    private DigitalChannel liftLimitLowSensor;
    private DigitalChannel liftLimitMidSensor;
    private DigitalChannel liftLimitHighSensor;
    private Servo clampServo;
    private boolean clampOpen = true;

    @Override
    public void runOpMode() {
        // Setup and zero robot
        setupMotorsAndServos();
        //zeroRobot();

        // Wait for the captain to press start on the cell phone
        waitForStart();

        // Wait for the game to start (driver presses PLAY)
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Get the state of the lift limit sensor
            boolean liftLimitLowState = !liftLimitLowSensor.getState();
            boolean liftLimitMidState = !liftLimitMidSensor.getState();
            boolean liftLimitHighState = !liftLimitHighSensor.getState();

            // Setup a variable for each drive wheel to save power level for telemetry
            double liftPower;
            double driveFactor  = 1;

            // Service the driver operator's inputs
            double motorFR = 0;
            double motorFL = 0;
            double motorBR = 0;
            double motorBL = 0;
            final float threshold = 0.2f;

            if(gamepad1.left_bumper)
                driveFactor = 0.5;

            if (Math.abs(gamepad1.left_stick_y) > threshold || Math.abs(gamepad1.left_stick_x) > threshold) {
                motorFR = (gamepad1.left_stick_y - gamepad1.left_stick_x) / 2;
                motorFL = (-gamepad1.left_stick_y - gamepad1.left_stick_x) / 2;
                motorBR = (-gamepad1.left_stick_y - gamepad1.left_stick_x) / 2;
                motorBL = (gamepad1.left_stick_y - gamepad1.left_stick_x) / 2;
            }
            if (Math.abs(gamepad1.left_stick_x) > threshold) {
                motorFR *= -1;
                motorFL *= -1;
                motorBR *= -1;
                motorBL *= -1;
            }
            if (Math.abs(gamepad1.right_stick_x) > threshold) {
                //rotate
                motorFR = (gamepad1.right_stick_x) / 2;
                motorFL = (gamepad1.right_stick_x) / 2;
                motorBR = (-gamepad1.right_stick_x) / 2;
                motorBL = (-gamepad1.right_stick_x) / 2;
            }
            leftFrontMotor.setPower(motorFL * driveFactor);
            rightFrontMotor.setPower(motorFR * driveFactor);
            leftRearMotor.setPower(motorBL * driveFactor);
            rightRearMotor.setPower(motorBR * driveFactor);

            double liftFactor = 0.6;

            if(gamepad2.left_bumper)
                liftFactor = 0.9;

            //if (gamepad2.left_stick_y < 0 && liftMotor.getCurrentPosition() < 900 && !liftLimitHighState && !liftLimitMidState) {
            if (gamepad2.left_stick_y < 0
                    && ((!liftLimitHighState && liftLimitMidState)
                    || (liftLimitHighState && !liftLimitMidState)
                    || (!liftLimitHighState && !liftLimitMidState))) {
                // Up
                liftMotor.setPower(1.0 * liftFactor);
            /*} else if (gamepad2.left_stick_y > 0 && liftMotor.getCurrentPosition() < 20
                    && liftMotor.getCurrentPosition() > 0) {
                // Down slow

                liftMotor.setPower(-0.1 * liftFactor);*/
            //} else if (gamepad2.left_stick_y > 0 && liftMotor.getCurrentPosition() > 0 && !liftLimitLowState) {
            } else if (gamepad2.left_stick_y > 0 && !liftLimitLowState) {
                // Down
                liftMotor.setPower(-1.0 * liftFactor);
            } else {
                // Hold power
                liftMotor.setPower(0.0);
            }
            if (gamepad2.a) {
                openClamp();
            } else if (gamepad2.b) {
                closeClamp();
            }

            updateTelemetry();
            idle();
        }
    }

    public void updateTelemetry()
    {
        telemetry.addData("currentliftMotorPosition", liftMotor.getCurrentPosition());
        telemetry.update();
    }

    public void openClamp()
    {
        clampServo.setPosition(0.83); // MAX: 0.84
        clampOpen = true;
    }

    public void closeClamp()
    {
        clampServo.setPosition(0.49);
        clampOpen = false;
    }

    public void setupMotorsAndServos()
    {
        // Setup motors and servos
        leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFrontMotor");
        leftRearMotor = hardwareMap.dcMotor.get("leftRearMotor");
        rightRearMotor = hardwareMap.dcMotor.get("rightRearMotor");

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");

        liftLimitLowSensor = hardwareMap.get(DigitalChannel.class, "liftLimitLowSensor");
        liftLimitLowSensor.setMode(DigitalChannel.Mode.INPUT);
        liftLimitMidSensor = hardwareMap.get(DigitalChannel.class, "liftLimitMidSensor");
        liftLimitMidSensor.setMode(DigitalChannel.Mode.INPUT);
        liftLimitHighSensor = hardwareMap.get(DigitalChannel.class, "liftLimitHighSensor");
        liftLimitHighSensor.setMode(DigitalChannel.Mode.INPUT);

        clampServo = hardwareMap.servo.get("clampServo");
    }
    public void zeroRobot()
    {
        //liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Lower the motor until we see the sensor
        while (liftLimitLowSensor.getState()) {
            liftMotor.setPower(-0.1);
        }
        liftMotor.setPower(0.5);
        sleep(50);
        liftMotor.setPower(0.0);
        // Set the zero location of the lift motor
        //liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}



