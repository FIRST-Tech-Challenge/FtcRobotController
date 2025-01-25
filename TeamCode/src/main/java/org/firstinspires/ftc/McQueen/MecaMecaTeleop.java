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

package org.firstinspires.ftc.McQueen;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp

/*
    MJ here...
    It's very empty here right now, but it will fill up once
    the robot gets completed...
*/

public class MecaMecaTeleop extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor  frontRight   = null;
    public DcMotor  frontLeft  = null;
    public DcMotor backRight = null;
    public DcMotor backLeft = null;
    public DcMotor armLift = null;
    public DcMotor armExtend = null;
    public Servo claw = null;
    public Servo rotate = null;
    public Servo wrist = null;
    public Servo elbow = null;
    public Servo upperWrist = null;

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    static final double     STRAFE_SPEED            = 0.5;

    static final double     INCHES_PER_NINETY_DEGREES = 14.0; //TODO calibrate later

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontRight  = hardwareMap.get(DcMotor.class, "frontRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        armLift = hardwareMap.get(DcMotor.class, "armLift"); // Arm joint?
        armExtend = hardwareMap.get(DcMotor.class, "armExtend");

        claw = hardwareMap.get(Servo.class, "claw");
        rotate = hardwareMap.get(Servo.class, "rotate");
        wrist = hardwareMap.get(Servo.class, "wrist");
        elbow = hardwareMap.get(Servo.class, "elbow");
        upperWrist = hardwareMap.get(Servo.class, "upperWrist");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // Set the motors to RUN_USING_ENCODER ofc
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armExtend.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // todo: change this to 2 motors

        armExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        runtime.reset();

        // end of setup
        
        double minClaw = 0.25; // open
        double maxClaw = 0.5; // closed

        double maxArm = 5000;
        double armSpeed = 75;


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // DPAD UP [2] - extend arm
            if (gamepad2.dpad_up && armExtend.getCurrentPosition() < maxArm) {
                armExtend.setPower(armSpeed);
            }
            else { // NOTE: This is only just if the viper slide violates size restraints.
                armExtend.setPower(0);
            }

            // DPAD DOWN [2] - roll back arm
            if (gamepad2.dpad_down && armExtend.getCurrentPosition() < maxArm) { // lower arm
                armExtend.setPower(-armSpeed);
            }
            else {
                armExtend.setPower(0);
            }

            // A [2]
            boolean wasAPressed;
            wasAPressed = gamepad2.a;
            if (gamepad2.a && !wasAPressed) {
                claw.setPosition(minClaw);
            }
            else {
                claw.setPosition(maxClaw);
            }
        } // End of opModeIsActive()


    } // End of runOpMode()

    // Auto functions!
    public void encoderDrive(double speed,
                             double frontLeftInches, double frontRightInches,
                             double backLeftInches, double backRightInches)
    {



        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackRightTarget;
        int newBackLeftTarget;


        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = frontLeft.getCurrentPosition() + (int)(frontLeftInches * COUNTS_PER_INCH);
            newFrontRightTarget = frontRight.getCurrentPosition() + (int)(frontRightInches * COUNTS_PER_INCH);
            newBackLeftTarget = backLeft.getCurrentPosition() + (int)(backLeftInches * COUNTS_PER_INCH);
            newBackRightTarget = backRight.getCurrentPosition() + (int)(backRightInches * COUNTS_PER_INCH);

            frontLeft.setTargetPosition(newFrontLeftTarget);
            frontRight.setTargetPosition(newFrontRightTarget);
            backLeft.setTargetPosition(newBackLeftTarget);
            backRight.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            frontLeft.setPower(Math.abs(speed));
            frontRight.setPower(Math.abs(speed));
            backLeft.setPower(Math.abs(speed));
            backRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d", newFrontLeftTarget,  newFrontRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                        frontLeft.getCurrentPosition(), frontRight.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);

            // Turn off RUN_TO_POSITION
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

    public void driveForward(double distance, double speed){

        encoderDrive(speed, distance, distance, distance, distance);
    }

    // Purely a convenience function - with this, we can just call driveForward(12)
    // without bothering to state a speed
    public void driveForward(double distance) {
        driveForward(distance, DRIVE_SPEED);
    }

    public void strafeLeft(double distance, double speed){
        encoderDrive(speed, -distance, distance,
                distance, -distance);
    }

    public void strafeLeft(double distance){
        strafeLeft(distance, STRAFE_SPEED); //
    }

    public void turnRight(double degrees, double speed){
        double inches = (degrees / 90.0) * INCHES_PER_NINETY_DEGREES;
        encoderDrive (speed, inches, -inches, inches, -inches);
    }

    public void turnRight(double degrees){
        turnRight (degrees, TURN_SPEED);
    }

    public void driveForwardDiagonalRight(double inches, double speed){
        encoderDrive (speed, inches, 0,
                0, inches);
    }
    public void driveForwardDiagonalRight(double inches){
        driveForwardDiagonalRight(inches, DRIVE_SPEED);
    }

    public void driveForwardDiagonalLeft(double inches, double speed){
        encoderDrive (speed, 0, inches,
                inches, 0);
    }

    public void driveForwardDiagonalLeft(double inches){
        driveForwardDiagonalLeft(inches, DRIVE_SPEED);
    }
} // End of the OpMode class
