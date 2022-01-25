package org.firstinspires.ftc.teamcode;
/** Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/** This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list. */


@TeleOp(name="Starter TeleOp", group="Iterative Opmode")

// @Disabled
public class TeleOp_StarterTeleOp extends OpMode
{
    /** Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor duckWheel = null;
    private DcMotor frontL = null;
    private DcMotor frontR = null;
    private DcMotor backL = null;
    private DcMotor backR = null;
    private CRServo intakeL = null;
    private CRServo intakeR = null;
    private Servo intakeYL= null;
    private Servo intakeYR = null;
    private DcMotor extender = null;
    private DcMotor arm = null;
    private Servo liftclaw = null;
    private Servo closeClaw = null;

    /** Code to run ONCE when the driver hits INIT. */
    @Override
    public void init() {
        telemetry.addData("Status", "Initializing");

        /** Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone). */
        duckWheel = hardwareMap.get(DcMotor.class, "duckWheel");
        frontL  = hardwareMap.get(DcMotor.class, "leftFront");
        frontR = hardwareMap.get(DcMotor.class, "rightFront");
        backL  = hardwareMap.get(DcMotor.class, "leftRear");
        backR = hardwareMap.get(DcMotor.class, "rightRear");
        intakeL = hardwareMap.get(CRServo.class, "intakeL");
        intakeR = hardwareMap.get(CRServo.class, "intakeR");
        intakeYL = hardwareMap.get(Servo.class, "intakeLiftL");
        intakeYR = hardwareMap.get(Servo.class, "intakeLiftR");
        extender = hardwareMap.get(DcMotor.class, "extender");
        arm = hardwareMap.get(DcMotor.class, "arm");

        liftclaw = hardwareMap.get(Servo.class, "liftCLaw");
        closeClaw = hardwareMap.get(Servo.class, "clawServo");

        /* Sets the motors to run using encoders. */
        frontL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /* makes the motors break on zero power */
        frontR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        /* Most robots need the motor on one side to be reversed to drive forward.
         * Reverse the motor that runs backwards when connected directly to the battery. */
        frontL.setDirection(DcMotor.Direction.FORWARD);
        backL.setDirection(DcMotor.Direction.FORWARD);
        frontR.setDirection(DcMotor.Direction.REVERSE);
        backR.setDirection(DcMotor.Direction.REVERSE);
        intakeL.setDirection(CRServo.Direction.REVERSE);
        intakeR.setDirection(CRServo.Direction.FORWARD);
        intakeYL.setDirection(Servo.Direction.REVERSE);
        intakeYR.setDirection(Servo.Direction.FORWARD);
        duckWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        liftclaw.setDirection(Servo.Direction.FORWARD);



        /* Tell the driver that initialization is complete. */
        telemetry.addData("Status", "Initialized");
    }


    /** Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY. */
    @Override
    public void init_loop() {
        intakeYL.setPosition(0.5);
        intakeYR.setPosition(0);
        closeClaw.setPosition(0);
        liftclaw.setPosition(0.3);

    }


    /** Code to run ONCE when the driver hits PLAY. */
    @Override
    public void start() {
        runtime.reset();
    }
    boolean clawClosed = false;
    boolean intakeDown =false;
    boolean duckOn = false;
    //boolean intakeOn = false;
    //double intakePow =0;
    double armPow = 0;
    double clawPos = 0;
    double clawClose = 0;

    /** Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP. */
    @Override
    public void loop() {
        /* Setup a variable for each drive wheel to save power level for telemetry. */
        double leftFPower ;
        double rightFPower;
        double leftBPower ;
        double rightBPower;

        double intakeLowerSpeed;




        /* More variable setup*/
        double drive = -gamepad1.right_stick_y;
        double turn  =  gamepad1.left_stick_x ;
        double strafe = gamepad1.right_stick_x;
        boolean isIntake = gamepad1.a;
        boolean intakeChangePosA = gamepad1.a;
        boolean intakeChangePosB = gamepad1.b;
        boolean isClawA = gamepad2.a;
        boolean isClawB = gamepad2.b;
        boolean clawLift = gamepad2.x;
        boolean clawLower = gamepad2.y;
        boolean isDuckR = gamepad1.right_bumper;
        boolean isDuckL = gamepad1.left_bumper;
        double duckPower= 0;
        double extension = gamepad2.right_stick_y;
        double armMove = gamepad2.left_stick_y;


        if (armMove == 0){
            armPow=0.001;


        }
        else armPow= armMove * 0.5;

        if (isDuckR) {
            if (!duckOn){
                duckPower = 0.69;
                duckOn = true;

            }
            else if (duckOn) {
                duckPower = 0;
                duckOn = false;
            }
        }

        if (isDuckL) {
            if (!duckOn){
                duckPower = -0.69;
                duckOn = true;

            }
            else if (duckOn) {
                duckPower = 0;
                duckOn = false;
            }
        }

        if (strafe != 0 ) {
            /* Strafing */
            leftFPower = -strafe;
            rightFPower = strafe;
            leftBPower =  strafe;
            rightBPower = -strafe;
        }

        else if (drive != 0 || turn != 0) {
            leftFPower = Range.clip(drive + turn, -1.0, 1.0);
            rightFPower = Range.clip(drive - turn, -1.0, 1.0);
            leftBPower = Range.clip(drive + turn, -1.0, 1.0);
            rightBPower = Range.clip(drive - turn, -1.0, 1.0);
        } else {
            leftFPower = 0;
            rightFPower = 0;
            leftBPower = 0;
            rightBPower = 0;
        }

       /* if (intakeChangePosA) {

                intakeYL.setPosition(0.4);
                intakeYR.setPosition(0.4);


            }
            if (intakeChangePosB) {
                intakeYL.setPosition(0);
                intakeYR.setPosition(0);
            }



       /*if (isIntake){
           if (!intakeOn)
           intakePow = 1;
           intakeR.setPower(intakePow);
           intakeL.setPower(intakePow);
           intakeOn = true;


        /*} else if (!isIntake){
            intakePow = 0;
           intakeR.setPower(intakePow);
           intakeL.setPower(intakePow);
            intakeOn = false;
        }
*/

        if (isClawA) {

            clawClose=0.8;

        }
        if (isClawB) {
            clawClose = 0;
        }

        if (clawLift) {
            clawPos = 0.7;
        }
        if (clawLower) {
            clawPos = 0.3;
        }


        frontL.setPower(leftFPower);
        backL.setPower(leftBPower);
        frontR.setPower(rightFPower);
        backR.setPower(rightBPower);
        // intakeR.setPower(intakePow);
        // intakeL.setPower(intakePow);
        duckWheel.setPower(duckPower);
        extender.setPower(extension * 0.5);
        arm.setPower(armPow);
        closeClaw.setPosition(clawClose);
        liftclaw.setPosition(clawPos);

        /**  Show the elapsed game time and wheel power. */
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "front left (%.2f), front right (%.2f), back left (%.2f), back right (%.2f)", leftFPower, rightFPower,leftBPower, rightBPower);
        // telemetry.addData("Intake Servos",intakePow);
        telemetry.addData("Claw Positions" + clawClose, clawPos);
    }

    /** Code to run ONCE after the driver hits STOP. */
    @Override
    public void stop() {

    }
}
