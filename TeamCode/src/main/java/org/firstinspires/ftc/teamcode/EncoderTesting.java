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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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

@TeleOp(name="Encoder Testing", group="Pushbot")
public class EncoderTesting extends OpMode {

    private static final int LOWER_LIFT_LIMIT = 50;
    /* Declare OpMode members. */
    HardwarePushbotBurg robot = new HardwarePushbotBurg(); // use the class created to define a Pushbot's hardware
    final double BALANCE_CORRECTION_POWER = 1;
    final double LIFT_HOLD_POWER = 0.1;
    final double LIFT_SPEED = 0.6;
    final int MAX_LIFT_HEIGHT = 2989;

    boolean isMovingLift = false;
    int liftHoldPosition = 0;


    int bottomLift = 1345;
    int middleLift = 2200;
    int topLift = 3000;
    int groundLift = 0;

    int liftPositionCalibration = 0;
    private boolean isHolding = false;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        robot.initIMU(hardwareMap);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");
        robot.gripper.setPosition(0);
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
        robot.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)

        final double MAX_POWER = .8;

        double ly = gamepad1.left_stick_y; // Forward and backward control
        double lx = gamepad1.left_stick_x; // Strafe Control
        double rx = gamepad1.right_stick_x; // Yaw control (Rotation)

        boolean dpadUp; //Raise Lift
        boolean dpadDown; // Lower Lift
        boolean dpadUp2; //Raise Lift on Controller 2
        boolean dpadDown2; // Lower Lift on Controller 2
        boolean a_button; // Set Lift Position to Ground
        boolean b_button; // Set Lift Position to Middle
        boolean x_button; // Set Lift Position to Low
        boolean y_button; // Set Lift Position to High
        boolean a_button2; // Set Lift Position to Ground on Controller 2
        boolean b_button2; // Set Lift Position to Middle on Controller 2
        boolean x_button2; // Set Lift Position to Low on Controller 2
        boolean y_button2; // Set Lift Position to High on Controller 2
        boolean right_bumper; // Open Gripper
        boolean left_bumper; // Close Gripper
        boolean right_bumper2; // Open Gripper Controller 2
        boolean left_bumper2; // Close Gripper Controller 2
        double right_trigger; // Toggle Slowmode
        double left_trigger; // Reset encoders


        double open = 0.1; // Position that the gripper is at when open

        dpadDown = gamepad1.dpad_down;
        dpadUp = gamepad1.dpad_up;
        a_button = gamepad1.a;
        b_button = gamepad1.b;
        x_button = gamepad1.x;
        y_button = gamepad1.y;
        right_bumper = gamepad1.right_bumper;
        left_bumper = gamepad1.left_bumper;
        a_button2 = gamepad2.a;
        b_button2 = gamepad2.b;
        x_button2 = gamepad2.x;
        y_button2 = gamepad2.y;
        dpadDown2 = gamepad2.dpad_down;
        dpadUp2 = gamepad2.dpad_up;
        right_bumper2 = gamepad2.right_bumper;
        left_bumper2 = gamepad2.left_bumper;
        right_trigger = gamepad1.right_trigger;
        left_trigger = gamepad1.left_trigger;


        double factor = 0.75;
        double slowFactor = 0.5;

        if(right_trigger > 0.5){
            robot.runMecanum(scalePower(slowFactor,ly,true)
                    ,scalePower(slowFactor,lx,true),rx);
        }
        else{
            robot.runMecanum(scalePower(factor,ly,true)
                    ,scalePower(factor,lx,true),rx);
        }

        if(a_button){
            robot.resetEncoders();
        }


        robot.displayPositions(telemetry);
        displayValues();
        telemetry.addData("ly: ", ly);
        telemetry.addData("lx: ", lx);
        telemetry.addData("rx: ", rx);
        telemetry.addData("back encoder position", robot.backEncoder.getCurrentPosition());
        telemetry.addData("left encoder position", robot.leftEncoder.getCurrentPosition());
        telemetry.addData("right encoder position", robot.rightEncoder.getCurrentPosition());
       // telemetry.addData("range", String.format("%.01f in", robot.sensorRange.getDistance(DistanceUnit.INCH)));
//        telemetry.addData ("rangeSensor", robot.getRangeDistance());
    }


    /*
     * Code to run ONCE after the driver hits STOP
     */

    private double scalePower(double scale,double input,boolean curveThePower){
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

    private void moveLift(double power){
        robot.lift.setPower(power);
        isMovingLift = true;
        robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void holdLift(){
        if(isMovingLift){
            isMovingLift = false;
            liftHoldPosition = robot.lift.getCurrentPosition() - liftPositionCalibration;
            robot.lift.setPower(LIFT_HOLD_POWER);
            robot.lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.lift.setTargetPosition(liftHoldPosition);
            isHolding = true;
        }
        if (isHolding && robot.lift.getCurrentPosition() < LOWER_LIFT_LIMIT) {
            isHolding = false;
            robot.lift.setPower(0);
            robot.lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void setLiftPosition(int position){
        robot.lift.setTargetPosition(position);
    }

    private void newHoldLift(){
        if(isMovingLift){
            robot.lift.setPower(0);

        }
    }

    public void displayValues(){
        int[] values = robot.showEncoderValues();

        telemetry.addData("Back encoder", values[0]);
        telemetry.addData("Left encoder", values[1]);
        telemetry.addData("Right encoder", values[2]);
    }


    @Override
    public void stop() {
    }
}
