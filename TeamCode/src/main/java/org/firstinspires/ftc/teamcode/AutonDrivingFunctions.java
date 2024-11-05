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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

// Above this line in importing packages. Do not change anything above this line
//
//
//
//
//
//
// The line below is what our auton in named.

@Autonomous(name="autonDrivingFunctions", group="Robot")
//@Disabled
public class AutonDrivingFunctions extends LinearOpMode {

    IMU imu;

    /* Declare OpMode members. */
    // The lines below set up the new motors that we use.
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private ElapsedTime     runtime = new ElapsedTime();
    //
    //
    //
    //
    //
    //
    //
    //
    // Below this line is where we define our variables that we use for the program

    static final double     COUNTS_PER_INCH         = 42.5;
    double                  CURRENT_YAW             = 0.0;
    double                  robotDesiredDirection   = 0.0;
    double                  directionError          = 0.0;
    double                  directionCorrectionModifier = 0.0;
    double                  countsPerDegree         = 9.44;

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        imu.resetYaw();

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        //Set the motors to brake when there is no power
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Reset the encoders
        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set the motors to run using encoder mode
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d : %7d : %7d : %7d" ,
                          leftFrontDrive.getCurrentPosition(),
                          leftBackDrive.getCurrentPosition(),
                          rightFrontDrive.getCurrentPosition(),
                          rightBackDrive.getCurrentPosition());
        telemetry.update();

        //
        //
        //
        //
        //
        //
        //
        //
        // Wait for the game to start (driver presses START)
        waitForStart();

        // Step through each leg of the path, this is what our robot is actually doing
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        turnLeft(0.5, 90);

        //        encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
//        encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
        while (opModeIsActive()){
            driveForward(0.5,20.0,30.0);
            turnLeft(0.5, -90.0);
            getYaw();
            telemetry.addData("Path", "Complete");
            telemetry.update();
        }
        sleep(100000);  // pause to display final telemetry message.
    }

    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    //
    // Everything below this line is functions that we use in our main program.

    public void driveForward(double speed,
                             double forwardInches,
                             double timeoutS) {
        int newFrontLeftTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position
            newFrontLeftTarget = leftFrontDrive.getCurrentPosition() + (int)(forwardInches * COUNTS_PER_INCH);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));

            //If we are running forward
            if(forwardInches>0.0) {
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (newFrontLeftTarget > leftFrontDrive.getCurrentPosition())) {
                    getYaw();
                    // Below: Ensuring that that the robot corrects itself correctly if the
                    // target angle is close to 180 or -180, and the robot crosses that threshold
                    if (robotDesiredDirection > 130){
                        if (CURRENT_YAW < 0) {
                            directionError = CURRENT_YAW +360 - robotDesiredDirection;
                        }
                        else {
                            directionError = CURRENT_YAW - robotDesiredDirection;
                        }
                    }
                    else if (robotDesiredDirection < -130){
                        if (CURRENT_YAW > 0){
                            directionError = CURRENT_YAW - 360 - robotDesiredDirection;
                        }
                        else {
                            directionError = CURRENT_YAW - robotDesiredDirection;
                        }
                    }
                    else {
                        directionError = CURRENT_YAW - robotDesiredDirection;
                    }

                    directionCorrectionModifier = directionError * 0.01;
                    telemetry.addData("modifier", "%.2f Deg. (Heading)", directionCorrectionModifier);
                    telemetry.update();
                    leftFrontDrive.setPower(speed + directionCorrectionModifier);
                    leftBackDrive.setPower(speed + directionCorrectionModifier);
                    rightFrontDrive.setPower(speed - directionCorrectionModifier);
                    rightBackDrive.setPower(speed - directionCorrectionModifier);

                }
            }
            else{
                while (opModeIsActive() &&
                        (runtime.seconds() < timeoutS) &&
                        (newFrontLeftTarget < leftFrontDrive.getCurrentPosition())){
                    getYaw();
                    directionError = CURRENT_YAW - robotDesiredDirection;
                    directionCorrectionModifier = directionError * 0.01;
                    telemetry.addData("modifier", "%.2f Deg. (Heading)", directionCorrectionModifier);
                    telemetry.update();
                    leftFrontDrive.setPower(-speed + directionCorrectionModifier);
                    leftBackDrive.setPower(-speed + directionCorrectionModifier);
                    rightFrontDrive.setPower(-speed - directionCorrectionModifier);
                    rightBackDrive.setPower(-speed - directionCorrectionModifier);

                }
            }

            // Stop all motion;
            leftFrontDrive.setPower(0);
            leftBackDrive.setPower(0);
            rightFrontDrive.setPower(0);
            rightBackDrive.setPower(0);

            sleep(250);   // optional pause after each move.
        }
    }

    // Above is the DriveForward Function
    //
    //
    //
    //
    //
    // Below is the TurnLeft Function

    public void turnLeft(double turnSpeed,
                        double leftAngle){
        if(robotDesiredDirection + leftAngle > 180){
            robotDesiredDirection = robotDesiredDirection + leftAngle - 360;
        }
        else if (robotDesiredDirection + leftAngle < -180) {
            robotDesiredDirection = robotDesiredDirection + leftAngle + 360;
        }
        else {
            robotDesiredDirection = robotDesiredDirection + leftAngle;
        }

        double              turnTargetPosition      = leftFrontDrive.getCurrentPosition() - (int)leftAngle * countsPerDegree;


        if (leftAngle > 0.0) {
            leftFrontDrive.setPower(-turnSpeed);
            leftBackDrive.setPower(-turnSpeed);
            rightFrontDrive.setPower(turnSpeed);
            rightBackDrive.setPower(turnSpeed);
            while (opModeIsActive() && turnTargetPosition < (leftFrontDrive.getCurrentPosition()-5)) {
            }

        }
        else {
                leftFrontDrive.setPower(turnSpeed);
                leftBackDrive.setPower(turnSpeed);
                rightFrontDrive.setPower(-turnSpeed);
                rightBackDrive.setPower(-turnSpeed);
                while (opModeIsActive() && turnTargetPosition > (leftFrontDrive.getCurrentPosition()+5)){
                };
        }
        sleep(250);

        while(opModeIsActive() && (Math.abs(CURRENT_YAW - robotDesiredDirection) > 1.0)) {
            getYaw();
            // modify direction error if we are close to 180 or -180
            if (robotDesiredDirection > 90) {
                if (CURRENT_YAW < 0){
                    directionError = CURRENT_YAW + 360 - robotDesiredDirection;
                }
                else{
                    directionError = CURRENT_YAW - robotDesiredDirection;
                }
            }
            else if (robotDesiredDirection < -90){
                if (CURRENT_YAW > 0){
                    directionError = CURRENT_YAW - 360 - robotDesiredDirection;
                }
                else {
                    directionError = CURRENT_YAW - robotDesiredDirection;
                }
            }
            else {
                directionError = CURRENT_YAW - robotDesiredDirection;
            }
            // directionError = CURRENT_YAW - robotDesiredDirection;
            directionCorrectionModifier = directionError * 0.02;
            telemetry.addData("modifier", "%.2f Deg. (Heading)", directionCorrectionModifier);
            telemetry.update();
            leftFrontDrive.setPower(directionCorrectionModifier);
            leftBackDrive.setPower(directionCorrectionModifier);
            rightFrontDrive.setPower(-directionCorrectionModifier);
            rightBackDrive.setPower(-directionCorrectionModifier);
        }
        leftFrontDrive.setPower(0.0);
        leftBackDrive.setPower(0.0);
        rightFrontDrive.setPower(0.0);
        rightBackDrive.setPower(0.0);

    }

    // Above is the TurnLeft Function
    //
    //
    //
    //
    //
    //
    // Below is the getYaw Function

    public void getYaw(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        CURRENT_YAW = orientation.getYaw(AngleUnit.DEGREES);
        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", CURRENT_YAW);
        telemetry.update();
    }
}
