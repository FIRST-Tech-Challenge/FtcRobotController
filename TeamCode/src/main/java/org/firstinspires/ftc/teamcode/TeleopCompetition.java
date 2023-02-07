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

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Disabled

@TeleOp(name="Teleop Competition", group="Pushbot")
public class TeleopCompetition extends OpMode {

    /* Declare OpMode members. */
    HardwarePushbotBurg2022 robot = new HardwarePushbotBurg2022(); // use the class created to define a Pushbot's hardware
    final double SLOW_SPEED = 0.4;
    boolean isDrivingFast = true;
    double debounceTimeIsDrivingFast;
    Orientation angles;
    private boolean isHolding = false;
    private boolean isOpening = false;
    double firstPressedCarousel;
    boolean isPressingCarousel;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.initIMU();
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        final double MAX_POWER = .8;
        double rightTrigger;
        double leftTrigger;
        boolean dpadUp;
        boolean dpadDown;
        boolean a_button;
        boolean b_button;
        double i;
        double l = 0;
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        double forward = -gamepad1.left_stick_y;
        double turn = -gamepad1.right_stick_x;
        dpadDown = gamepad1.dpad_down;
        dpadUp = gamepad1.dpad_up;

        //boolean isIntaking = false;
        //boolean bumperWasClicked = false;

//        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;

        angles = robot.checkOrientation();

        double leftPower = forward - turn;
        double rightPower = forward + turn;

        //When isDrivingFast is true fullPower is MAX_POWER, when it's false, fullPower is SLOW_SPEED
        //() ? __ : __ is a ternary operator, where it checks the boolean inside (), if true does first statement, if false does second
        double fullPower = (isDrivingFast) ? MAX_POWER : SLOW_SPEED;

        //checks the absolute value of leftPower and rightPower, then sets maxInput to the greatest value
        double maxInput = Math.max(Math.abs(leftPower), Math.abs(rightPower));

        if (maxInput > fullPower) {
            double scale = fullPower / maxInput;
            leftPower = scalePower(scale, leftPower, true);
            rightPower = scalePower(scale, rightPower, true);
        }
        telemetry.addData("LeftPower:", leftPower);
        telemetry.addData("RightPower:", rightPower);

        robot.frontLeft.setPower(leftPower);
        robot.backLeft.setPower(leftPower);
        robot.frontRight.setPower(rightPower);
        robot.backRight.setPower(rightPower);
        /*
        if (dpadDown){
            robot.lift.setPower(-1);
        }
        else if (dpadUp){
            robot.lift.setPower(1);
        }
        else{
            robot.lift.setPower(0);
        }
        */

        /*
        if (gamepad1.right_bumper == true) {
            if (bumperWasClicked == false) {
                isIntaking = true;
                bumperWasClicked = true;
            } else if (bumperWasClicked)
                isIntaking = false;
                bumperWasClicked = false;
        }

        if(bumperWasClicked){
            robot.closeGrip();
        }else if(bumperWasClicked == false){
            robot.stopGrip();
        }
        */

        if(gamepad1.y && getRuntime()>debounceTimeIsDrivingFast + 0.2){
            isDrivingFast = !isDrivingFast;
            debounceTimeIsDrivingFast = getRuntime();
        }

        telemetry.addData("z angle", angles.firstAngle);
        telemetry.addData("Is Driving Fast:", isDrivingFast);
    }

    private double scalePower(double scale, double input, boolean curveThePower) {
        // Design a power curve so that full speed (input: 1) is still 1, but
        // small inputs are made even smaller
        if (curveThePower) {
            telemetry.addData("curving power!", "");
            double curvedPower = (scale * input) * (scale * input);
            if (input < 0) curvedPower = -curvedPower;
            return curvedPower;
        }
        return scale * input;
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
