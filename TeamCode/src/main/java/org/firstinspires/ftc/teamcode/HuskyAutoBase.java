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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class HuskyAutoBase extends LinearOpMode {
    /* Declare OpMode members. */
    HuskyBot huskyBot = new HuskyBot();
    ElapsedTime runtime = new ElapsedTime();

    // autonomous mode constants
    static final double COUNTS_PER_MOTOR_REV = 537.6;
    // see: https://docs.revrobotics.com/rev-control-system/programming/hello-robot-autonomous-robot/robot-navigation-blocks/autonomous-navigation-blocks#total-gear-reduction
    static final double DRIVE_GEAR_REDUCTION = 1;

    // 96 mm mecanum wheels
    static final double WHEEL_DIAMETER_INCHES = 96 / 25.4;
    static final double COUNTS_PER_WHEEL_REV = COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION;
    static final double COUNTS_PER_INCH = COUNTS_PER_WHEEL_REV / (WHEEL_DIAMETER_INCHES * Math.PI);

    public static final double AUTO_DRIVE_SPEED = 0.5;
    public static final double AUTO_STRAFE_SPEED = 0.5;
    public static final int TURN_TRAVEL_INCHES = 19;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // The intrinsics only matter if we are utilizing the translation and rotation values of the April tag, which we don't need in this game.
    // And because we need to utilize the translation, it's preferred to leave these variables as they are.
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double aprilTagSize = 0.02794;

    // Apriltag ID 1, 2, 3 from the 36h11 family
    final int LOCATION_1_TAG_ID = 1;
    final int LOCATION_2_TAG_ID = 2;
    final int LOCATION_3_TAG_ID = 3;

    public static double FORWARD_DISTANCE = 29;
    public static double STRAFE_DISTANCE = 27;
    public static double BACKUP_STRAFE_DISTANCE = 8;

    OpenCVPipeline pipeline;

    public enum Location {
        LOCATION_0,
        LOCATION_1,
        LOCATION_2,
        LOCATION_3
    }

    Location parkLocation = Location.LOCATION_0;

    @Override
    public void runOpMode() throws InterruptedException {
        huskyBot.init(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        huskyBot.webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new OpenCVPipeline(aprilTagSize, fx, fy, cx, cy);

        huskyBot.webcam.setPipeline(pipeline);

        huskyBot.webcam.setMillisecondsPermissionTimeout(2500); // Timeout for obtaining permission is configurable. Set before opening.
        huskyBot.webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() { huskyBot.webcam.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT); }

            @Override
            public void onError(int errorCode) { }
        });
    }

    /*
     *  Method to perform a straight line forward/backward move, based on encoder counts.
     *  forward is +ve target, backward is -ve
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed, double distanceInches, double timeoutSecs) {
        // Determine new target position, and pass to motor controller
        // target is same for all motors
        int target = (int) (distanceInches * COUNTS_PER_INCH);
        driveToTarget(speed, target, target, target, target, timeoutSecs);

        // Wait after move is complete
        sleep(500);
    }

    public void encoderTurn(double speed, double angleDegrees, double timeoutSecs) {
        // convert angles to inches
        // based on field tests, 90º turn is TURN_TRAVEL_INCHES distance
        // obviously, this is not going to be accurate. future improvements will be using
        // roadrunner (which uses imu) to turn accurately
        double targetAngle = AngleUnit.normalizeDegrees(angleDegrees);
        int target = (int) ((targetAngle / 90) * TURN_TRAVEL_INCHES * COUNTS_PER_INCH);
        driveToTarget(speed, target, -target, target, -target, timeoutSecs);
    }

    /*
     *  Method to perform a strafe move, based on encoder counts.
     *  right (looking from behind of robot) is +ve, left is -ve
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderStrafe(double speed, double distanceInches, double timeoutSecs) {
        // Determine new target position, and pass to motor controller
        int target = (int) (distanceInches * COUNTS_PER_INCH);
        driveToTarget(speed, (int) (target * 1.2), (int) (-target * 1.1), -target, target, timeoutSecs);

        // Wait after move is complete
        sleep(500);
    }

    private void resetDriveEncoders() {
        huskyBot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        huskyBot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        huskyBot.rearLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        huskyBot.rearRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void driveToTarget(double speed, int frontLeftTarget, int frontRightTarget,
                              int rearLeftTarget, int rearRightTarget, double timeoutSecs) {
        resetDriveEncoders();
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            huskyBot.frontLeftDrive.setTargetPosition(frontLeftTarget);
            huskyBot.frontRightDrive.setTargetPosition(frontRightTarget);
            huskyBot.rearLeftDrive.setTargetPosition(rearLeftTarget);
            huskyBot.rearRightDrive.setTargetPosition(rearRightTarget);

            // Turn On RUN_TO_POSITION
            huskyBot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            huskyBot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            huskyBot.rearRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            huskyBot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            huskyBot.frontLeftDrive.setPower(speed);
            huskyBot.frontRightDrive.setPower(speed);
            huskyBot.rearLeftDrive.setPower(speed);
            huskyBot.rearRightDrive.setPower(speed);

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutSecs) &&
                    (huskyBot.frontLeftDrive.isBusy() && huskyBot.frontRightDrive.isBusy())) {
                telemetry.addData("Runtime", runtime.seconds());
                telemetry.addData("Target Position", "front left: %7d, front right: %7d",
                        frontLeftTarget, frontRightTarget);
                telemetry.addData("Target Position", "rear left: %7d, rear right: %7d",
                        rearLeftTarget, rearRightTarget);
                displayTelemetry();
            }

            // Stop all motion;
            huskyBot.frontLeftDrive.setPower(0);
            huskyBot.frontRightDrive.setPower(0);
            huskyBot.rearLeftDrive.setPower(0);
            huskyBot.rearRightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            huskyBot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            huskyBot.rearLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            huskyBot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            huskyBot.rearRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    public Location getParkLocation(){
        boolean tagFound = false;
        Location parkLocation = Location.LOCATION_0;

        // get the park location with timeout of 3 seconds.
        while (opModeIsActive() && (runtime.seconds() < 3)) {
            ArrayList<AprilTagDetection> currentDetections = pipeline.getLatestDetections();

            if(currentDetections.size() != 0) {
                for(AprilTagDetection tag : currentDetections) {
                    switch (tag.id){
                        case LOCATION_1_TAG_ID:
                            tagFound = true;
                            parkLocation = Location.LOCATION_1;
                            break;
                        case LOCATION_2_TAG_ID:
                            tagFound = true;
                            parkLocation = Location.LOCATION_2;
                            break;
                        case LOCATION_3_TAG_ID:
                            tagFound = true;
                            parkLocation = Location.LOCATION_3;
                            break;
                    }
                }
            }

            if(tagFound)
                break;

            sleep(50);
        }

        return parkLocation;
    }

    private void displayTelemetry() {
        telemetry.addData("Position", "front left: %7d, front right: %7d",
                huskyBot.frontLeftDrive.getCurrentPosition(),
                huskyBot.frontRightDrive.getCurrentPosition());
        telemetry.addData("Position", "rear left: %7d, rear right: %7d",
                huskyBot.rearLeftDrive.getCurrentPosition(),
                huskyBot.rearRightDrive.getCurrentPosition());
        telemetry.addData("Velocity", "front left (%.2f), front right (%.2f)",
                huskyBot.frontLeftDrive.getVelocity(),
                huskyBot.frontRightDrive.getVelocity());
        telemetry.addData("Velocity", "rear left (%.2f), rear right (%.2f)",
                huskyBot.rearLeftDrive.getVelocity(),
                huskyBot.rearRightDrive.getVelocity());
        telemetry.addData("Power", "front left (%.2f), front right (%.2f)",
                huskyBot.frontLeftDrive.getPower(),
                huskyBot.frontRightDrive.getPower());
        telemetry.addData("Power", "rear left (%.2f), rear right (%.2f)",
                huskyBot.rearLeftDrive.getPower(),
                huskyBot.rearRightDrive.getPower());

        telemetry.update();
    }
}