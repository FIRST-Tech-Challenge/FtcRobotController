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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "arcadeDrive", group = "Iterative Opmode")

public class arcadeDrive extends OpMode {
    // Declare OpMode members.

    public ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftFoward = null;
    public DcMotor rightReverse = null;
    public DcMotor leftReverse = null;
    public DcMotor rightFoward = null;
    public DcMotor intake = null;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        //Name motors

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFoward = hardwareMap.get(DcMotor.class, "left_foward_drive");
        rightReverse = hardwareMap.get(DcMotor.class, "right_reverse_drive");
        leftReverse = hardwareMap.get(DcMotor.class, "left_reverse_drive");
        rightFoward = hardwareMap.get(DcMotor.class, "right_foward_drive");
        intake = hardwareMap.get(DcMotor.class, "intake_intial");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFoward.setDirection(DcMotor.Direction.FORWARD);
        rightReverse.setDirection(DcMotor.Direction.REVERSE);
        leftReverse.setDirection(DcMotor.Direction.FORWARD);
        rightFoward.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        //Reset encoders
        leftFoward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightReverse.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftReverse.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFoward.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Reset power
        leftFoward.setPower(0);
        rightReverse.setPower(0);
        leftReverse.setPower(0);
        rightFoward.setPower(0);
        intake.setPower(0);

        //use encoders
        leftFoward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightReverse.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftReverse.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFoward.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
        runtime.reset();
    }

    @Override
    public void loop() {

        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        //hypotenuse: distance of stick from center
        double robotAngle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) - Math.PI / 4;
        //Angle of stick: shifted -45 degrees
        double rightX = gamepad1.right_stick_x;
        //Speed control

        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;
        /* Finding power
        Front motors: positive speed
        Back motors: negative speed

         */

        leftFoward.setPower(v1);
        rightFoward.setPower(v2);
        leftReverse.setPower(v3);
        rightReverse.setPower(v4);
        //set power

    }


}

