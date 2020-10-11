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

package org.firstinspires.ftc.teamcode.robots.minimech;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Disabled
@TeleOp(name="MiniMech", group="Linear Opmode")
public class MiniMech extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront  = null;
    private DcMotor leftBack   = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack  = null;
    private DcMotor elbow = null;
    private Crane pos = null;
    private DcMotor extender = null;
    private Servo gate = null;
    private Servo hook = null;

    private boolean[] buttonSavedStates = new boolean[16];
    private int a = 0; //lower glyph lift
    private int b = 1; //toggle grip/release on glyph
    private int x = 2; //no function
    private int y = 3; //raise glyph lift
    private int dpad_down = 4; //enable/disable ftcdash telemetry
    private int dpad_up = 5; //vision init/de-init
    private int dpad_left = 6; //vision provider switch
    private int dpad_right = 7; //switch viewpoint
    private int left_bumper = 8; //increment state down (always)
    private int right_bumper = 9; //increment state up (always)
    private int startBtn = 10; //toggle active (always)
    private int left_trigger = 11; //vision detection
    private int right_trigger = 12;
    private int back_button = 13;
    private int left_stick_button = 14;
    private int right_stick_button = 15; //sound player




    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront = hardwareMap.get(DcMotor.class, "motorFrontLeft");
        leftBack = hardwareMap.get(DcMotor.class, "motorBackLeft");
        rightFront = hardwareMap.get(DcMotor.class, "motorFrontRight");
        rightBack = hardwareMap.get(DcMotor.class, "motorBackRight");
        elbow = hardwareMap.get(DcMotor.class, "elbow");
        extender = hardwareMap.get(DcMotor.class, "extender");
        hook = hardwareMap.get(Servo.class, "hook");
        gate = hardwareMap.get(Servo.class, "gate");
        pos = new Crane(elbow, extender, hook, gate);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        elbow.setDirection(DcMotor.Direction.REVERSE);
        extender.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftFrontPower;
            double leftBackPower;
            double rightFrontPower;
            double rightBackPower;
            double elbowSetPose;


            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight

            double drive = -gamepad1.left_stick_y *.3;
            double turn = gamepad1.right_stick_x *.3;
            double strafe = gamepad1.left_stick_x;

            leftFrontPower = Range.clip(drive + turn + strafe, -1.0, 1.0);
            leftBackPower = Range.clip(drive + turn - strafe, -1.0, 1.0);
            rightFrontPower = Range.clip(drive - turn - strafe, -1.0, 1.0);
            rightBackPower = Range.clip(drive - turn + strafe, -1.0, 1.0);



            // Send calculated power to wheels
            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightFront.setPower(rightFrontPower);
            rightBack.setPower(rightBackPower);

            //elbow code
            if (gamepad1.dpad_right) {
                //pos.articulate(PoseBigWheel.Articulation.manual);
                pos.increaseElbowAngle();
            }
            if (gamepad1.dpad_left) {
                //robot.articulate(PoseBigWheel.Articulation.manual);
                pos.decreaseElbowAngle();
            }
            if (gamepad1.dpad_up) {
                //robot.articulate(PoseBigWheel.Articulation.manual);
                pos.extendBelt();
            }
            if (gamepad1.dpad_down) {
                //robot.articulate(PoseBigWheel.Articulation.manual);
                pos.retractBelt();
            }
//            if (gamepad1.right_stick_y < -.1){
//                //robot.articulate(PoseBigWheel.Articulation.manual);
//                pos.extendBelt();
//            }
//            if (gamepad1.right_stick_y > .1) {
//                //robot.articulate(PoseBigWheel.Articulation.manual);
//                pos.retractBelt();
//            }
            if(toggleAllowed(gamepad1.a,a)){
                pos.hookToggle();
            }
            if(toggleAllowed(gamepad1.y,y)){
                pos.gateToggle();
            }
            //call the update method in crane
            pos.update();
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f) (%.2f), right (%.2f) (%.2f)",
                    leftFrontPower, leftBackPower, rightFrontPower, rightBackPower);
            telemetry.update();
        }
    }
    //checks to see if a specific button should allow a toggle at any given time; needs a rework
    private boolean toggleAllowed(boolean button, int buttonIndex) {
        if (button) {
            if (!buttonSavedStates[buttonIndex]) { //we just pushed the button, and when we last looked at it, it was not pressed
                buttonSavedStates[buttonIndex] = true;
                return true;
            } else { //the button is pressed, but it was last time too - so ignore

                return false;
            }
        }

        buttonSavedStates[buttonIndex] = false; //not pressed, so remember that it is not
        return false; //not pressed

    }
}
