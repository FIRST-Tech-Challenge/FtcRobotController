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
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

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

@Autonomous(name="LeftCloseAuton", group="Robot")
//@Disabled
public class LeftCloseAuton extends LinearOpMode {

    IMU imu;

    /* Declare OpMode members. */
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private ElapsedTime     runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = 42.5;
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
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

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

        // Wait for the game to start (driver presses START)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        turnLeft(0.5, 90);

        //        encoderDrive(TURN_SPEED,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
//        encoderDrive(DRIVE_SPEED, -24, -24, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout
        while (opModeIsActive()){
            driveForward(0.5,25,30);
            //sampleTower place preloaded specimen
            sleep(3000);
            turnLeft(0.5,90);
            driveForward(0.5,48,30);
            turnLeft(0.5,-90);
            //samplePickup pick up neutral sample
            sleep(3000);
            turnLeft(0.5,180.0);
            driveForward(0.5,10,30);
            //sampleTower up, sample into high basket
            sleep(2000);
            turnLeft(0.5,180);
            driveForward(0.5,12.0, 3.0);
            turnLeft(0.5,180.0);

            sleep(30000);

            getYaw();
            telemetry.addData("Path", "Complete");
            telemetry.update();
        }
        sleep(100000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void driveForward(double speed,
                             double forwardInches,
                             double timeoutS) {
        int newFrontLeftTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = leftFrontDrive.getCurrentPosition() + (int)(forwardInches * COUNTS_PER_INCH);

            // Turn On RUN_TO_POSITION
            //leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFrontDrive.setPower(Math.abs(speed));
            leftBackDrive.setPower(Math.abs(speed));
            rightFrontDrive.setPower(Math.abs(speed));
            rightBackDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
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
                    //directionError = CURRENT_YAW - robotDesiredDirection;

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

            // Turn off RUN_TO_POSITION
            leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move.
        }
    }

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
//        leftFrontDrive.setPower(0.0);
//        leftBackDrive.setPower(0.0);
//        rightFrontDrive.setPower(0.0);
//        rightBackDrive.setPower(0.0);
//
//        if(CURRENT_YAW > robotDesiredDirection){
//            while (opModeIsActive() && CURRENT_YAW > robotDesiredDirection + 1.0){
//                getYaw();
//                leftFrontDrive.setPower(0.1);
//                leftBackDrive.setPower(0.1);
//                rightFrontDrive.setPower(-0.1);
//                rightBackDrive.setPower(-0.1);
//            }
//        }
//        else{
//            while (opModeIsActive() && CURRENT_YAW < robotDesiredDirection - 1.0){
//                getYaw();
//                leftFrontDrive.setPower(-0.1);
//                leftBackDrive.setPower(-0.1);
//                rightFrontDrive.setPower(0.1);
//                rightBackDrive.setPower(0.1);
//            }
//        }
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

    public void getYaw(){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        CURRENT_YAW = orientation.getYaw(AngleUnit.DEGREES);
        telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", CURRENT_YAW);
        telemetry.update();
    }
}
