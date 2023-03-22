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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/** BSci!TechA
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Bigger Neater", group="Iterative Opmode")
public class BasicOpMode_Iterative extends OpMode
{
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor FrontLeft = null;
    private DcMotor FrontRight = null;
    private DcMotor BackLeft = null;
    private DcMotor BackRight = null;
    private double firstTime = 0;
    private Servo claw = null;
    private DcMotor liftLeft   = null;
    private DcMotor liftRight = null;
    //private Servo claw = null;
    //private DcMotor  liftLeft   = null;
    //private DcMotor liftRight = null;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override

    public void init() {
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        FrontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "frontRight");
        BackLeft = hardwareMap.get(DcMotor.class, "backLeft");
        BackRight = hardwareMap.get(DcMotor.class, "backRight");
        claw = hardwareMap.get(Servo.class, "claw");
        liftLeft = hardwareMap.get(DcMotor.class, "liftleft");
        liftRight = hardwareMap.get(DcMotor.class, "liftright");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackRight.setDirection(DcMotor.Direction.REVERSE);


        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
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
        //runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        // Setup a variable for each drive wheel to save power level for telemetry
        double FrontleftPower;
        double FrontrightPower;
        double BackleftPower;
        double BackrightPower;
        double liftPower;
        double buttGex = 0;
        //double claw;

        // Choose to drive using either Tank Mode, or POV Mode
        // Comment out the method that's not used.  The default below is POV.

        // POV Mode uses left stick to go forward, and right stick to turn.
        // - This uses basic math to combine motions and is easier to drive straight.
        //double drive = -gamepad1.left_stick_y;
        //double turn  =  gamepad1.right_stick_x;
        //leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
        //rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;


            FrontleftPower = -gamepad1.left_stick_y + gamepad1.left_stick_x;
            FrontrightPower = -gamepad1.right_stick_y - gamepad1.right_stick_x;
            BackleftPower = gamepad1.left_stick_y + gamepad1.left_stick_x;
            BackrightPower = gamepad1.right_stick_y - gamepad1.right_stick_x;

        liftPower=.5;

        if(gamepad1.dpad_up){
            FrontleftPower = +0.25;
            FrontrightPower =+0.25;
            BackleftPower =  -0.25;
            BackrightPower = -0.25;
        }

        if(gamepad1.dpad_right){
            FrontleftPower = +0.25;
            FrontrightPower =-0.25;
            BackleftPower =  +0.25;
            BackrightPower = -0.25;
        }

        if(gamepad1.dpad_left){
            FrontleftPower = -0.25;
            FrontrightPower =+0.25;
            BackleftPower =  -0.25;
            BackrightPower = +0.25;
        }

        if(gamepad1.dpad_down){
            FrontleftPower = -0.25;
            FrontrightPower =-0.25;
            BackleftPower =  +0.25;
            BackrightPower = +0.205;
        }

        if(gamepad1.right_bumper){
            claw.setPosition(0.0);
            //buttGex = buttGex-0.1;
        }else if (gamepad1.left_bumper){
            claw.setPosition(0.45);
            //buttGex=0.0;
        }

        if(gamepad1.left_trigger >= .3){
            liftLeft.setPower(-liftPower*gamepad1.left_trigger*2);
            liftRight.setPower(-liftPower*gamepad1.left_trigger*2);
        } else if (gamepad1.right_trigger >= .3) {
            liftLeft.setPower(liftPower * gamepad1.right_trigger*2);
            liftRight.setPower(liftPower * gamepad1.right_trigger*2);
        } else if(gamepad1.x){
            liftLeft.setPower(-liftPower*0.2);
            liftRight.setPower(-liftPower*0.2);
        }else if(gamepad1.y){
            liftLeft.setPower(liftPower * 0.4);
            liftRight.setPower(liftPower * 0.4);
        }else if (gamepad1.right_bumper || gamepad1.left_bumper ) {
            liftLeft.setPower(liftPower * 0.35);
            liftRight.setPower(liftPower * 0.35);


        }else{
            liftLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            liftLeft.setPower(0);
            liftRight.setPower(0);
        }




/*
        if(gamepad1.right_bumper) {
            FrontleftPower *= 0.8;
            FrontrightPower *= 0.4;
            BackleftPower *= 0.4;
            BackrightPower *= 0.8;
        }

        if(gamepad1.left_bumper) {
            FrontleftPower *= 0.4;
            FrontrightPower *= 0.8;
            BackleftPower *= 0.8;
            BackrightPower *= 0.4;
        }
*/



        /*if(gamepad2.left_trigger>0){
            liftLeft.setPower(gamepad2.left_trigger*0.25);
            liftRight.setPower(gamepad2.left_trigger*0.25);
        } else if(gamepad2.right_trigger>0){
            liftRight.setPower(-gamepad2.right_trigger*0.25);
            liftLeft.setPower(-gamepad2.right_trigger*0.25);
        } else {
            liftRight.setPower(0);
            liftLeft.setPower(0);
        }*/

        // Send calculated power to wheels
        //leftDrive.setPower(leftPower);
        //rightDrive.setPower(rightPower);

        FrontLeft.setPower(FrontleftPower*0.85);
        BackLeft.setPower(BackleftPower*0.85);
        FrontRight.setPower(FrontrightPower*0.85);
        BackRight.setPower(BackrightPower*0.85);



        // Show the elapsed game time and wheel power.
        telemetry.addData("Stat2us", "Run Time: " + runtime.toString());
        //telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);




    /*
     * Code to run ONCE after the driver hits STOP
     */


}}


