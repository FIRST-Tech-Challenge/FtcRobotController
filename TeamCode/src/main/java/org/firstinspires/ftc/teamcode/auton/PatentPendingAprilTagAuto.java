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

package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.PPMyRobot;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


import java.util.ArrayList;


/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */


@Autonomous(name="Autonomous: PPTest", group="Autonomous")
//@Disabled
public class PatentPendingAprilTagAuto extends LinearOpMode {

    public enum Direction {
        FORWARD,
        BACKWARD,
        RIGHT,
        LEFT,
        F_RIGHT_DIG,
        F_LEFT_DIG,
        B_RIGHT_DIG,
        B_LEFT_DIG,
        CLOCK_WISE_TURN,
        ANTI_CLOCK_WISE_TURN
    }
    public enum TEST_MODE {
        WORKING1,
        WORKING2,
        WORKING3,
        WORKING4,
        TEST1,
        TEST2,
        TEST3,
        TEST4
    }

    /* Declare OpMode members. */
    PPMyRobot robot   = new PPMyRobot();   // Use NF my Robot h/w

    private final ElapsedTime runtime = new ElapsedTime();

    //Encoder produced TICK COUNTS per revolution
    static final double COUNTS_PER_MOTOR_REV = 537.7;    // eg: TETRIX Motor Encoder - 1440, REV Hex Motors: 2240
    static final double DRIVE_GEAR_REDUCTION = 1; //2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 3.7;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1428);
    static final double COUNTS_FULL_TURN = 72;
    static final int ENCODER_COUNT_BEFORE_STOP = 140; //slow down before 3"

    static TEST_MODE TEST_RUN_TYPE = TEST_MODE.WORKING1;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    //static final double     DRIVE_SPEED             = 1;
    //static final double     TURN_SPEED              = 0.5;
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    //Tag 1,2,3 from 36h11
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    static final double FEET_PER_METER = 3.28084;

    AprilTagDetection tagOfInterest = null;

    double distance;

    @Override
    public void runOpMode() /*throws InterruptedException*/ {
        Direction dir;
        distance = 0;
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        runtime.reset();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Starting Autorun");    //Auto run
        telemetry.update();

        // Reset Encoder
        robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set Encoder
        robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Position: Front L:%7d R:%7d, Back L:%7d R:%7d",
                robot.motorFrontLeft.getCurrentPosition(),
                robot.motorFrontRight.getCurrentPosition(),
                robot.motorBackLeft.getCurrentPosition(),
                robot.motorBackRight.getCurrentPosition());
        telemetry.update();

        // Reverse the right side motors
        robot.motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        boolean tagFound = false;

        while (!isStopRequested() && !tagFound ) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                telemetry.addLine("\n Detected something");
                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
                //move to the desired location
                if (tagFound ) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);

                   if (tagOfInterest.id == MIDDLE) {
                       myEncoderTurn(0.50, 360);
                        telemetry.addLine("\ntag 2");
                    } else if (tagOfInterest.id == LEFT) {
                       myEncoderTurn(0.50, 360);
                        telemetry.addLine("\ntag 1");
                    } else if (tagOfInterest.id == RIGHT) {
                        myEncoderTurn(0.50, 360);
                        telemetry.addLine("\ntag 3");
                    }
                    // once detected break
                } else {
                    telemetry.addLine("Don't see tag of interest :(");

                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            telemetry.update();
            sleep(1000);
        }
    }

    public void myEncoderTurn(double speed, double degree)
    {
        double inch_cnt;

        if (Math.abs(degree) < 1) return;

        // Adjust to turn less than 180
        if (degree < -180) {
            degree = degree + 360;
        }
        if (degree > 180) {
            degree = degree - 360;
        }

        inch_cnt = degree*(COUNTS_FULL_TURN/360);
        RobotLog.ii("Input:myEncoderTurn", "Speed/Degree/inch_cnt, %f, %f %f",
                speed, degree, inch_cnt);

        if (degree >= 0) {
            myEncoderDrive(Direction.ANTI_CLOCK_WISE_TURN, speed, Math.abs(inch_cnt), 3000);
        } else {
            myEncoderDrive(Direction.CLOCK_WISE_TURN, speed, Math.abs(inch_cnt), 3000);
        }
    }

    public void myEncoderDrive(Direction direction, double speed, double Inches,
                               double timeoutS) {   //SensorsToUse sensors_2_use)
        int newFrontLeftTarget = 0;
        int newFrontRightTarget = 0;
        int newBackLeftTarget = 0;
        int newBackRightTarget = 0;
        int remainingDistance;
        int cnt = 0;
        double new_speed = 0;


        RobotLog.ii("Input", "Enter - myEncoderDrive -  speed=%f, Inches=%f, timeout=%f",
                speed, Inches, timeoutS);
        telemetry.addData("Path1", "Enter - myEncoderDrive -  speed=%f," +
                " Inches=%f, timeout=%f", speed, Inches, timeoutS);
        telemetry.update();

        // Turn off ENCODER
        // Reset Encoder beginning to see it gets better.
        robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RobotLog.ii("Path0", "Starting Position: Front L:%7d R:%7d, Back L:%7d R:%7d",
                robot.motorFrontLeft.getCurrentPosition(),
                robot.motorFrontRight.getCurrentPosition(),
                robot.motorBackLeft.getCurrentPosition(),
                robot.motorBackRight.getCurrentPosition());

        // Ensure that the op mode is still active
        if (opModeIsActive() && !isStopRequested()) {

            // Determine new target position, and pass to motor controller
            if (direction == Direction.BACKWARD) {
                //Go forward
                RobotLog.ii("NFusion", "Moving BACKWARD.... [%f inches]..", Inches);
                newFrontLeftTarget = robot.motorFrontLeft.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newFrontRightTarget = robot.motorFrontRight.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newBackLeftTarget = robot.motorBackLeft.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newBackRightTarget = robot.motorBackRight.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            } else if (direction == Direction.FORWARD) {
                //Go backward
                RobotLog.ii("NFusion", "Moving FORWARD..... [%f inches]..", Inches);
                newFrontLeftTarget = robot.motorFrontLeft.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newFrontRightTarget = robot.motorFrontRight.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newBackLeftTarget = robot.motorBackLeft.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newBackRightTarget = robot.motorBackRight.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
            } else if (direction == Direction.LEFT) {
                //Move Right
                RobotLog.ii("NFusion", "Moving LEFT..... [%f inches]..", Inches);
                newFrontLeftTarget = robot.motorFrontLeft.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newFrontRightTarget = robot.motorFrontRight.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newBackLeftTarget = robot.motorBackLeft.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newBackRightTarget = robot.motorBackRight.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            } else if (direction == Direction.RIGHT) {
                //Move Left
                RobotLog.ii("NFusion", "Moving RIGHT..... [%f inches]..", Inches);
                newFrontLeftTarget = robot.motorFrontLeft.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newFrontRightTarget = robot.motorFrontRight.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newBackLeftTarget = robot.motorBackLeft.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newBackRightTarget = robot.motorBackRight.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
            } else if (direction == Direction.F_RIGHT_DIG) {
                //Forward Left Diagonal
                RobotLog.ii("NFusion", "Moving F_RIGHT_DIG ..... [%f inches]..", Inches);
                newFrontRightTarget = robot.motorFrontRight.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newBackLeftTarget = robot.motorBackLeft.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            } else if (direction == Direction.F_LEFT_DIG) {
                //Move Forward Right Diagonal
                RobotLog.ii("NFusion", "Moving F_LEFT_DIG..... [%f inches]..", Inches);
                newFrontLeftTarget = robot.motorFrontLeft.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newBackRightTarget = robot.motorBackRight.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            } else if (direction == Direction.B_RIGHT_DIG){
                //Move Backward Left Diagonal
                RobotLog.ii("NFusion", "Moving B_RIGHT_DIG..... [%f inches]..", Inches);
                newFrontLeftTarget = robot.motorFrontLeft.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newBackRightTarget = robot.motorBackRight.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
            } else if (direction == Direction.B_LEFT_DIG) {
                //Backward Right Diagonal
                RobotLog.ii("NFusion", "Moving B_LEFT_DIG ..... [%f inches]..", Inches);
                newFrontRightTarget = robot.motorFrontRight.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newBackLeftTarget = robot.motorBackLeft.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
            } else if (direction == Direction.ANTI_CLOCK_WISE_TURN) {
                // Turn Clock Wise
                RobotLog.ii("NFusion", "Turn Anti Clockwise ..... [%f ms]..", timeoutS);
                newFrontLeftTarget = robot.motorFrontLeft.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newFrontRightTarget = robot.motorFrontRight.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newBackLeftTarget = robot.motorBackLeft.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newBackRightTarget = robot.motorBackRight.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
            } else if (direction == Direction.CLOCK_WISE_TURN) {
                // Turn Clock Wise
                RobotLog.ii("NFusion", "Turn Clockwise ..... [%f ms]..", timeoutS);
                newFrontLeftTarget = robot.motorFrontLeft.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newFrontRightTarget = robot.motorFrontRight.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
                newBackLeftTarget = robot.motorBackLeft.getCurrentPosition() + (int) (-1 * Inches * COUNTS_PER_INCH);
                newBackRightTarget = robot.motorBackRight.getCurrentPosition() + (int) (Inches * COUNTS_PER_INCH);
            } else {
                // Do Not move
                speed = 0;
            }

            // Set the Encoder to the target position.
            robot.motorFrontLeft.setTargetPosition(newFrontLeftTarget);
            robot.motorFrontRight.setTargetPosition(newFrontRightTarget);
            robot.motorBackLeft.setTargetPosition(newBackLeftTarget);
            robot.motorBackRight.setTargetPosition(newBackRightTarget);

            // Turn On RUN_TO_POSITION
            robot.motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set power for the motors.
            robot.motorFrontLeft.setPower(Math.abs(speed));
            robot.motorFrontRight.setPower(Math.abs(speed));
            robot.motorBackLeft.setPower(Math.abs(speed));
            robot.motorBackRight.setPower(Math.abs(speed));

            // reset the timeout time and start motion.
            runtime.reset();

            RobotLog.ii("NFusion", "Final Target   FL: %7d, FR: %7d, BL: %7d, BR: %7d",
                    newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);

            while (opModeIsActive() &&
                    (runtime.milliseconds() < timeoutS) &&
                    ((robot.motorFrontLeft.isBusy() || robot.motorFrontRight.isBusy()) &&
                            (robot.motorBackLeft.isBusy() || robot.motorBackRight.isBusy()))) {
                // Display it for the driver every 10 ms
                if(runtime.milliseconds() > cnt * 30 ) { // Print every 30 ms
                    RobotLog.ii("NFusion", "Current Target FL: %7d, FR: %7d, BL: %7d, BR: %7d",
                            robot.motorFrontLeft.getCurrentPosition(),
                            robot.motorFrontRight.getCurrentPosition(),
                            robot.motorBackLeft.getCurrentPosition(),
                            robot.motorBackRight.getCurrentPosition());
                    telemetry.addData("Path1", "Running to: FL: %7d FR: %7d BL: %7d BR: %7d",
                            newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                    telemetry.addData("Path2", "Running at: FL: %7d FR: %7d BL: %7d BR: %7d",
                            robot.motorFrontLeft.getCurrentPosition(),
                            robot.motorFrontRight.getCurrentPosition(),
                            robot.motorBackLeft.getCurrentPosition(),
                            robot.motorBackRight.getCurrentPosition());
                    telemetry.update();
                    cnt++;
                }
                // Reduce the Speed before stopping
                remainingDistance = Math.abs(newFrontLeftTarget-robot.motorFrontLeft.getCurrentPosition());
                if (remainingDistance < 10)
                {
                    remainingDistance = Math.abs(newFrontRightTarget-robot.motorFrontRight.getCurrentPosition());
                }
                if ((remainingDistance < ENCODER_COUNT_BEFORE_STOP) && (remainingDistance >= 10)){
                    new_speed = Math.abs(speed)*(remainingDistance/(float)ENCODER_COUNT_BEFORE_STOP);
                    robot.motorFrontLeft.setPower(new_speed);
                    robot.motorFrontRight.setPower(new_speed);
                    robot.motorBackLeft.setPower(new_speed);
                    robot.motorBackRight.setPower(new_speed);
                    if((cnt % 5) == 0) { // skip 5 cnt and print
                        RobotLog.ii("NFusion", "Remaining Dist: %7d, Speed %f, new Speed %f",
                                remainingDistance, speed, new_speed);
                    }
                }
            }

            //RobotLog.ii("NFusion", "Motor Encoder Status FL: %d, FR: %d, BL: %d, BR: %d",
            //      motorFrontLeft.isBusy(),
            //    motorFrontRight.isBusy(),
            //  motorBackLeft.isBusy(),
            //motorBackRight.isBusy());

            // Stop all motion;
            robot.motorFrontLeft.setPower(0);
            robot.motorFrontRight.setPower(0);
            robot.motorBackLeft.setPower(0);
            robot.motorBackRight.setPower(0);

            //sleep(500);   // optional pause after each move

            // Turn off ENCODER
            // Reset Encoder
            //robot.motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //robot.motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //robot.motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //robot.motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            RobotLog.ii("Path0", "Exit - myEncoderDrive Last Position: Front L:%7d R:%7d, Back L:%7d R:%7d",
                    robot.motorFrontLeft.getCurrentPosition(),
                    robot.motorFrontRight.getCurrentPosition(),
                    robot.motorBackLeft.getCurrentPosition(),
                    robot.motorBackRight.getCurrentPosition());

            //telemetry.addData("Status", "Movement Distance: %7d",
            //        distance_traveled);
            //telemetry.update();

        }
    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}



