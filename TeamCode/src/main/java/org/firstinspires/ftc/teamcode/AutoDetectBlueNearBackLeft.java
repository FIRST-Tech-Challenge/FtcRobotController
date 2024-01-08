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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
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
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Auto Detect Blue Near Back Left", group="")
public class AutoDetectBlueNearBackLeft extends LinearOpMode {

    /* Declare OpMode members. */
    private RobotHardware robot = new RobotHardware(this);
    private ElapsedTime     runtime = new ElapsedTime();
    private FirstVisionProcessor visionProcessor;
    private VisionPortal visionPortal;

    /************Encoder parameters*****************/
    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.78 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                                                      (WHEEL_DIAMETER_INCHES * 3.1415);
    /************Encoder params end****************/
    static final double     DRIVE_SPEED             = 0.3;
    static final double     TURN_SPEED              = 0.2;

    @Override
    public void runOpMode() {
        robot.init();

        visionProcessor = new FirstVisionProcessor();
        visionProcessor.colorToCheck = "blue";

        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "webcam1"), visionProcessor);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        if (opModeIsActive()) {
            FirstVisionProcessor.Selected centerSelectedDirection = visionProcessor.getSelection();

            double[] centerColorValues = visionProcessor.colorValues;
            telemetry.addData("CenterSelectionIdentified", centerSelectedDirection);
            telemetry.update();

            while (opModeIsActive() && !isStopRequested()) {

                if (centerSelectedDirection == FirstVisionProcessor.Selected.LEFT) {
                    telemetry.addData("Travelling Left", "");
                    telemetry.update();
                    travelLeft();
                }
                else if (centerSelectedDirection == FirstVisionProcessor.Selected.RIGHT) {
                    telemetry.addData("Travelling Right", "");
                    telemetry.update();
                    travelRight();
                }
                else {
                    telemetry.addData("Travelling Straight", "");
                    telemetry.update();
                    travelStraight();
                }
                break;
            }
        }
    }

    public void travelStraight(){
        robot.setMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                robot.getLeftMotorCurrentPosition(),
                robot.getRightMotorCurrentPosition());
        telemetry.update();

        // Step through each leg of the path,
        telemetry.addData("go forward", "");
        telemetry.update();
        encoderDrive(DRIVE_SPEED, 0, 29,  29, 10);  // S1: Forward 47 Inches with 5 Sec timeout
        //reverse
        telemetry.addData("Reverse", "");
        telemetry.update();
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED, 0,   -3, -3, 10);  // S2: Turn Right 12 Inches with 4 Sec timeout
        telemetry.addData("Turning left", "");
        telemetry.update();

        //turn left
        robot.setMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.driveRobot(0, -TURN_SPEED);
        sleep(3700);

        //go straight
        robot.setMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("go to back stage", "");
        telemetry.update();
        encoderDrive(DRIVE_SPEED, 0,    -50, -50, 10);
        //encoderDrive(DRIVE_SPEED, 24, 24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        telemetry.addData("Path", "Complete");
        telemetry.update();
        robot.moveGrabberToPosition(RobotHardware.GRABBER_MIN);
        telemetry.addData("Grabber", "released");
        telemetry.update();
        sleep(100);  // pause to display final telemetry message.
    }

    public void travelLeft(){
        robot.setMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                robot.getLeftMotorCurrentPosition(),
                robot.getRightMotorCurrentPosition());
        telemetry.update();

        // Step through each leg of the path,
        telemetry.addData("go forward", "");
        telemetry.update();
        encoderDrive(DRIVE_SPEED, 0, 12,  12, 10);  // S1: Forward 47 Inches with 5 Sec timeout

        //turn left
        robot.setMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.driveRobot(0.1, TURN_SPEED);
        sleep(1300);

        //forward
        robot.setMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("go forward", "");
        telemetry.update();
        encoderDrive(DRIVE_SPEED, 0, 6,  6, 10);  // S1: Forward 47 Inches with 5 Sec timeout

        //reverse
        telemetry.addData("Reverse", "");
        telemetry.update();
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED, 0,   -5, -5, 10);  // S2: Turn Right 12 Inches with 4 Sec timeout
        telemetry.addData("Turning left", "");
        telemetry.update();

        //turn left
        robot.setMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.driveRobot(0, TURN_SPEED);
        sleep(2000);

        //go straight
        robot.setMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("go to back stage", "");
        telemetry.update();
        encoderDrive(DRIVE_SPEED, 0,    40, 40, 10);
        //encoderDrive(DRIVE_SPEED, 24, 24, 4.0);  // S3: Reverse 24 I  nches with 4 Sec timeout

        robot.setMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.driveRobot(0, -TURN_SPEED);
        sleep(7000);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        robot.moveGrabberToPosition(RobotHardware.GRABBER_MIN);
        telemetry.addData("Grabber", "released");
        telemetry.update();
        sleep(100);  // pause to display final telemetry message.
    }

    public void travelRight(){
        robot.setMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d",
                robot.getLeftMotorCurrentPosition(),
                robot.getRightMotorCurrentPosition());
        telemetry.update();

        // Step through each leg of the path,
        telemetry.addData("go forward", "");
        telemetry.update();
        encoderDrive(DRIVE_SPEED, 0, 17,  17, 10);  // S1: Forward 47 Inches with 5 Sec timeout

        //turn right
        robot.setMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.driveRobot(0, -TURN_SPEED);
        sleep(2000);

        //forward
        robot.setMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("go forward", "");
        telemetry.update();
        encoderDrive(DRIVE_SPEED, 0, 7,  7, 10);  // S1: Forward 47 Inches with 5 Sec timeout

        //reverse
        telemetry.addData("Reverse", "");
        telemetry.update();
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        encoderDrive(DRIVE_SPEED, 0,   -15, -15, 10);  // S2: Turn Right 12 Inches with 4 Sec timeout
        telemetry.addData("Turning left", "");
        telemetry.update();

        //turn left
        robot.setMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.driveRobot(0, -TURN_SPEED);
        sleep(2700);

        //go straight
        robot.setMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
        telemetry.addData("go to back stage", "");
        telemetry.update();
        encoderDrive(DRIVE_SPEED, 0,    -47, -47, 10);
        //encoderDrive(DRIVE_SPEED, 24, 24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout

        telemetry.addData("Path", "Complete");
        telemetry.update();
        robot.moveGrabberToPosition(RobotHardware.GRABBER_MIN);
        telemetry.addData("Grabber", "released");
        telemetry.update();
        sleep(100);  // pause to display final telemetry message.
    }
    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void encoderDrive(double straightSpeed, double turnSpeed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.getLeftMotorCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.getRightMotorCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.setLeftTargetPosition(newLeftTarget);
            robot.setRightTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.setMotorsMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            //robot.setDrivePower(Math.abs(speed), Math.abs(speed));
            robot.driveRobot(straightSpeed, turnSpeed);
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                   (runtime.seconds() < timeoutS) &&
                   (robot.isLeftMotorBusy() && robot.isRightMotorBusy())) {

                // Display it for the drivAutoDriveByEncoder.
                telemetry.addData("Running to",  " %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Currently at",  " at %7d :%7d",
                                            robot.getLeftMotorCurrentPosition(), robot.getRightMotorCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.driveRobot(0, 0);

            // Turn off RUN_TO_POSITION
            robot.setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }
}
