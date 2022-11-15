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
import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Robot: Auto Drive By Time with Color Sensing capabilities", group="Robot")
//@Disabled
public class RobotAutoDriveByTime_ColorSensing extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor         leftDrive   = null;
    private DcMotor         rightDrive  = null;

    private ElapsedTime     runtime = new ElapsedTime();

    ColorSensor colorSensor;    // Hardware Device Object


    static final double FAST_SPEED = 0.6;
    static final double SLOW_SPEED = 0.3;
    static final double MOVE_TIME = 3.0;
    static final double SHORT_TIME = 1.0;

    static final double ROTATION_SPEED = 0.4;
    static final double QUARTER_ROTATION = 1.0;
    static final double HALF_ROTATION = 2.0 * QUARTER_ROTATION;
    static final double FULL_ROTATION = 2.0 * HALF_ROTATION;

    boolean LedOn = false;

    // The specifics of these values depend on hueScalingFactor
    // and the general lighting conditions.
    // Brighter light = higher values needed
    static final double greenHue = 154.0;
    static final double purpleHue = 215.0;
    static final double orangeHue = 45.0;
    static final double hueRange = 15.0;

    float hsvValues[] = {0F,0F,0F};
    final float values[] = hsvValues;
    static final int hueScalingFactor = 8;

    enum ColorSensed {
        GREEN,
        PURPLE,
        ORANGE,
        NONE;
    }

    ColorSensed colorSensed = ColorSensed.NONE;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        leftDrive  = hardwareMap.get(DcMotor.class, "left");
        rightDrive = hardwareMap.get(DcMotor.class, "right");

        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        colorSensor.enableLed(LedOn);

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        loop();

        while (opModeIsActive()) {

//            moveForward(SHORT_TIME, SLOW_SPEED);
//            moveBackward(SHORT_TIME, SLOW_SPEED);

            stallRobot(MOVE_TIME);
            senseColor();
        }
    }

    private void moveForward(double time, double speed) {
        leftDrive.setPower(speed);
        rightDrive.setPower(-speed);

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < time){
            telemetry.addData("Path", "Moving forward with %4.1f speed, %4.1f / %4.1f seconds", speed, runtime.seconds(), time);
            telemetry.update();
        }

        resetPower();
    }

    private void moveBackward(double time, double speed) {
        leftDrive.setPower(-speed);
        rightDrive.setPower(speed);

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < time) {
            telemetry.addData("Path", "Moving backward with %4.1f speed, %4.1f / %4.1f seconds", speed, runtime.seconds(), time);
            telemetry.update();
        }

        resetPower();
    }

    private void spinRight(double time, double speed) {
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < time) {
            telemetry.addData("Path", "Spinning right with %4.1f speed, %4.1f / %4.1f seconds", speed, runtime.seconds(), time);
            telemetry.update();
        }

        resetPower();
    }

    private void spinLeft(double time, double speed) {
        leftDrive.setPower(-speed);
        rightDrive.setPower(-speed);

        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < time) {
            telemetry.addData("Path", "Spinning left with %4.1f speed, %4.1f / %4.1f seconds", speed, runtime.seconds(), time);
            telemetry.update();
        }

        resetPower();
    }

    private void resetPower() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }

    private void senseColor() {

        Color.RGBToHSV(colorSensor.red() * hueScalingFactor, colorSensor.green() * hueScalingFactor,
                colorSensor.blue() * hueScalingFactor, hsvValues);
        if (hsvValues[0] >= greenHue - hueRange && hsvValues[0] <= greenHue + hueRange) {
            colorSensed = ColorSensed.GREEN;
        } else if (hsvValues[0] >= purpleHue - hueRange && hsvValues[0] <= purpleHue + hueRange) {
            colorSensed = ColorSensed.PURPLE;
        } else if (hsvValues[0] >= orangeHue - hueRange && hsvValues[0] <= orangeHue + hueRange) {
            colorSensed = ColorSensed.ORANGE;
        } else {
            colorSensed = ColorSensed.NONE;
        }

        switch (colorSensed) {
            case PURPLE:
                moveForward(MOVE_TIME, FAST_SPEED);
                break;
            case ORANGE:
                moveBackward(MOVE_TIME, FAST_SPEED);
                break;
            case GREEN:
                spinLeft(QUARTER_ROTATION, ROTATION_SPEED);
                spinRight(QUARTER_ROTATION, ROTATION_SPEED);
                break;
            case NONE:
                stallRobot(MOVE_TIME);
        }
    }

    private void stallRobot(double time) {
        runtime.reset();

        while (opModeIsActive() && runtime.seconds() <= time) {
            telemetry.addData("Waiting", "for %4.1f / %4.1f seconds", runtime.seconds(), time);
            telemetry.update();
        }
    }
}
