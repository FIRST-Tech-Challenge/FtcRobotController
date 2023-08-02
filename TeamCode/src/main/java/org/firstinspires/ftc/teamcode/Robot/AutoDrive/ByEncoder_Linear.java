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

package org.firstinspires.ftc.teamcode.Robot.AutoDrive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 *  Adapted from SDK 8.2 example, RobotAutoDriveByEncoderLinear
 *  2023-08-01 0.1 armw initial foray using prior season's Mecanum drivetrain parameters
 */

@Autonomous(name="Auto: Drive By Encoder", group="Robot")
//@Disabled
public class ByEncoder_Linear extends LinearOpMode {

    // Declare OpMode members
    /*
    private DcMotor         leftDrive   = null;
    private DcMotor         rightDrive  = null;
     */
    private final DcMotorEx[] motor = new DcMotorEx[]{null, null, null, null};
    String[] motorLabels = {
            "motorLeftFront",           // port 0 Control Hub
            "motorLeftBack",            // port 1 Control Hub
            "motorRightFront",          // port 2 Control Hub
            "motorRightBack"            // port 3 Control Hub
    };

    private final ElapsedTime     runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 537.6898396 ; // 5203-2402-0019  goBILDA 5203 series 19.2:1 ratio
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;         // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.779528 ;    // 96 mm Mecanum wheel
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables
        /*
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
         */
        // Initialize the hardware variables
        // The strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        for (int i = 0; i < motor.length; i++) {
            motor[i] = hardwareMap.get(DcMotorEx.class, motorLabels[i]);
            // motor stops and then brakes actively resisting any external force which attempts to turn the motor
            motor[i].setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            motor[i].setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        }
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        /*
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
         */

        // Send telemetry message to indicate successful Encoder reset
        /*
        telemetry.addData("Starting at",  "%7d :%7d",
                          leftDrive.getCurrentPosition(),
                          rightDrive.getCurrentPosition());
         */

        motor[0].setDirection(DcMotorEx.Direction.REVERSE); // motorLeftFront
        motor[1].setDirection(DcMotorEx.Direction.REVERSE); // motorLeftBack
        motor[2].setDirection(DcMotorEx.Direction.FORWARD); // motorRightFront
        motor[3].setDirection(DcMotorEx.Direction.FORWARD); // motorRightBack

        telemetry.addData("Starting at",  "%7d :%7d",
            motor[0].getCurrentPosition(), motor[2].getCurrentPosition());
        telemetry.addLine("Start: press PLAY");
        telemetry.update();

        //waitForStart(); // Wait for the game to start (driver presses PLAY)

        while (!opModeIsActive() && !isStopRequested()) {
            sleep(50);
            idle();
        }

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED,  24,  24, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
        encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_SPEED, -12, -12, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(3000);  // pause to let driver absorb the message
    }

    /*
     * helper method to set a common speed for all motors in the drivetrain
     * @param speed setting applied to all motors
     */
    private void setMotorSpeed(double speed) {
        for (DcMotorEx dcMotorEx : motor) {
            dcMotorEx.setPower(speed);
        }
    }

    /*
     * helper method to set a common speed for all motors in the drivetrain
     * @param mode setting applied to all motors
     */
    private void setMotorMode(DcMotorEx.RunMode mode) {
        for (DcMotorEx dcMotorEx : motor) {
            //dcMotorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            dcMotorEx.setMode(mode);
        }
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        if (opModeIsActive()) { // Ensure that the opmode is still active

            // Determine new target position, and pass to motor controller
            newLeftTarget = motor[0].getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = motor[2].getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            motor[0].setTargetPosition(newLeftTarget);
            motor[1].setTargetPosition(newRightTarget);
            motor[2].setTargetPosition(newRightTarget);
            motor[3].setTargetPosition(newLeftTarget);

            setMotorMode(DcMotor.RunMode.RUN_TO_POSITION); // Turn On RUN_TO_POSITION

            runtime.reset(); // reset the timeout time and start motion
            setMotorSpeed(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (motor[0].isBusy() && motor[2].isBusy())) {

                // Display current parameters to the driver
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                                            motor[0].getCurrentPosition(), motor[1].getCurrentPosition());
                telemetry.update();
            }

            setMotorSpeed(0.0); // Stop all motion
            setMotorMode(DcMotorEx.RunMode.RUN_USING_ENCODER); // Turn off RUN_TO_POSITION

            sleep(250);   // optional pause after each move.
        }
    }
}
