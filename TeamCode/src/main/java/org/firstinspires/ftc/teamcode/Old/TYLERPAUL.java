///* Copyright (c) 2022 FIRST. All rights reserved.
// *
// * Redistribution and use in source and binary forms, with or without modification,
// * are permitted (subject to the limitations in the disclaimer below) provided that
// * the following conditions are met:
// *
// * Redistributions of source code must retain the above copyright notice, this list
// * of conditions and the following disclaimer.
// *
// * Redistributions in binary form must reproduce the above copyright notice, this
// * list of conditions and the following disclaimer in the documentation and/or
// * other materials provided with the distribution.
// *
// * Neither the name of FIRST nor the names of its contributors may be used to endorse or
// * promote products derived from this software without specific prior written permission.
// *
// * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
// * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// */
//
//package org.firstinspires.ftc.teamcode.Old;
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//
///**
// *  This file illustrates the concept of driving an autonomous path based on Gyro heading and encoder counts.
// *  The code is structured as a LinearOpMode
// *
// *  The path to be followed by the robot is built from a series of drive, turn or pause steps.
// *  Each step on the path is defined by a single function call, and these can be strung together in any order.
// *
// *  The code REQUIRES that you have encoders on the drive motors, otherwise you should use: RobotAutoDriveByTime;
// *
// *  This code ALSO requires that you have a BOSCH BNO055 IMU, otherwise you would use: RobotAutoDriveByEncoder;
// *  This IMU is found in REV Control/Expansion Hubs shipped prior to July 2022, and possibly also on later models.
// *  To run as written, the Control/Expansion hub should be mounted horizontally on a flat part of the robot chassis.
// *
// *  This sample requires that the drive Motors have been configured with names : left_drive and right_drive.
// *  It also requires that a positive power command moves both motors forward, and causes the encoders to count UP.
// *  So please verify that both of your motors move the robot forward on the first move.  If not, make the required correction.
// *  See the beginning of runOpMode() to set the FORWARD/REVERSE option for each motor.
// *
// *  This code uses RUN_TO_POSITION mode for driving straight, and RUN_USING_ENCODER mode for turning and holding.
// *  Note: You must call setTargetPosition() at least once before switching to RUN_TO_POSITION mode.
// *
// *  Notes:
// *
// *  All angles are referenced to the coordinate-frame that is set whenever resetHeading() is called.
// *  In this sample, the heading is reset when the Start button is touched on the Driver station.
// *  Note: It would be possible to reset the heading after each move, but this would accumulate steering errors.
// *
// *  The angle of movement/rotation is assumed to be a standardized rotation around the robot Z axis,
// *  which means that a Positive rotation is Counter Clockwise, looking down on the field.
// *  This is consistent with the FTC field coordinate conventions set out in the document:
// *  ftc_app\doc\tutorial\FTC_FieldCoordinateSystemDefinition.pdf
// *
// *  Control Approach.
// *
// *  To reach, or maintain a required heading, this code implements a basic Proportional Controller where:
// *
// *      Steering power = Heading Error * Proportional Gain.
// *
// *      "Heading Error" is calculated by taking the difference between the desired heading and the actual heading,
// *      and then "normalizing" it by converting it to a value in the +/- 180 degree range.
// *
// *      "Proportional Gain" is a constant that YOU choose to set the "strength" of the steering response.
// *
// *  Use Android Studio to Copy this Class, and Paste it into your "TeamCode" folder with a new name.
// *  Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
// */
//
//@Autonomous(name="TYLERPAUL", group="Robot")
//@Disabled
//public class TYLERPAUL extends LinearOpMode {
//
//    /* Declare OpMode members. */
//    private DcMotor         backright   = null;
//    private DcMotor         backleft  = null;
//    private DcMotor         frontleft   = null;
//    private DcMotor         frontright  = null;
//
//    private ModernRoboticsI2cGyro imu         = null;      // Control/Expansion Hub IMU
//
//    private double          robotHeading  = 0;
//    private double          headingOffset = 0;
//    private double          headingError  = 0;
//
//    // These variable are declared here (as class members) so they can be updated in various methods,
//    // but still be displayed by sendTelemetry()
//    private double  targetHeading = 0;
//    private double  driveSpeed    = 0;
//    private double  turnSpeed     = 0;
//    private double  leftSpeed     = 0;
//    private double  rightSpeed    = 0;
//    private int     leftTarget    = 0;
//    private int     rightTarget   = 0;
//
//    // Calculate the COUNTS_PER_INCH for your specific drive train.
//    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
//    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
//    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
//    // This is gearing DOWN for less speed and more torque.
//    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
//    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;   // eg: GoBILDA 312 RPM Yellow Jacket
//    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
//    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
//    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
//            (WHEEL_DIAMETER_INCHES * 3.1415);
//
//    // These constants define the desired driving/control characteristics
//    // They can/should be tweaked to suit the specific robot drive train.
//    static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
//    static final double     TURN_SPEED              = 0.2;     // Max Turn speed to limit turn rate
//    static final double     HEADING_THRESHOLD       = 1.0 ;    // How close must the heading get to the target before moving to next step.
//    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
//    // Define the Proportional control coefficient (or GAIN) for "heading control".
//    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
//    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
//    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
//    static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
//    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable
//
//
//    @Override
//    public void runOpMode() {
//
//        // Initialize the drive system variables.
//        frontleft = hardwareMap.dcMotor.get("frontleft");
//        frontright = hardwareMap.dcMotor.get("frontright");
//        backleft = hardwareMap.dcMotor.get("backleft");
//        backright = hardwareMap.dcMotor.get("backright");
//
//        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
//        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
//        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
//        backleft.setDirection(DcMotor.Direction.FORWARD);
//        frontleft.setDirection(DcMotor.Direction.FORWARD);
//        backright.setDirection(DcMotor.Direction.REVERSE);
//        frontright.setDirection(DcMotor.Direction.REVERSE);
//
//        // define initialization values for IMU, and then initialize it.
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
//        imu = hardwareMap.get(ModernRoboticsI2cGyro.class, "Gyro");
//
//
//        // Ensure the robot is stationary.  Reset the encoders and set the motors to BRAKE mode
//        backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        backleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontleft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        backright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        frontright.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//
//        // Wait for the game to start (Display Gyro value while waiting)
//        while (opModeInInit()) {
//            telemetry.addData(">", "Robot Heading = %4.0f", getRawHeading());
//            telemetry.update();
//        }
//
//        // Set the encoders for closed loop speed control, and reset the heading.
//        backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        resetHeading();
//
//        driveStraight(.7, -40, 0);
//
//    }
//
//
//    public void driveStraight(double maxDriveSpeed,
//                              double distance,
//                              double heading) {
//
//        // Ensure that the opmode is still active
//        if (opModeIsActive()) {
//
//            // Determine new target position, and pass to motor controller
//            int moveCounts = (int)(distance * COUNTS_PER_INCH);
//            leftTarget = frontleft.getCurrentPosition() + moveCounts;
//            rightTarget = frontright.getCurrentPosition() + moveCounts;
//            leftTarget = backleft.getCurrentPosition() + moveCounts;
//            rightTarget = backright.getCurrentPosition() + moveCounts;
//
//            // Set Target FIRST, then turn on RUN_TO_POSITION
//            frontleft.setTargetPosition(leftTarget);
//            frontright.setTargetPosition(rightTarget);
//            backleft.setTargetPosition(leftTarget);
//            backright.setTargetPosition(rightTarget);
//
//            frontleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            frontright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            backleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            backright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//            // Set the required driving speed  (must be positive for RUN_TO_POSITION)
//            // Start driving straight, and then enter the control loop
//            maxDriveSpeed = Math.abs(maxDriveSpeed);
//            moveRobot(maxDriveSpeed, 0);
//
//            // keep looping while we are still active, and BOTH motors are running.
//            while (opModeIsActive() &&
//                    (frontleft.isBusy() && frontright.isBusy() && backleft.isBusy() && backright.isBusy())) {
//
//                // Determine required steering to keep on heading
//                turnSpeed = getSteeringCorrection(heading, P_DRIVE_GAIN);
//
//                // if driving in reverse, the motor correction also needs to be reversed
//                if (distance < 0)
//                    turnSpeed *= -1.0;
//
//                // Apply the turning correction to the current driving speed.
//                moveRobot(driveSpeed, turnSpeed);
//
//                // Display drive status for the driver.
////                sendTelemetry(true);
//            }
//
//            // Stop all motion & Turn off RUN_TO_POSITION
//            moveRobot(0, 0);
//            frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//    }
//
//    /**
//     *  Method to spin on central axis to point in a new direction.
//     *  Move will stop if either of these conditions occur:
//     *  1) Move gets to the heading (angle)
//     *  2) Driver stops the opmode running.
//     *
//     * @param maxTurnSpeed Desired MAX speed of turn. (range 0 to +1.0)
//     * @param heading Absolute Heading Angle (in Degrees) relative to last gyro reset.
//     *              0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
//     *              If a relative angle is required, add/subtract from current heading.
//     */
//    public void turnToHeading(double maxTurnSpeed, double heading) {
//
//        // Run getSteeringCorrection() once to pre-calculate the current error
//        getSteeringCorrection(heading, P_DRIVE_GAIN);
//
//        // keep looping while we are still active, and not on heading.
//        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {
//
//            // Determine required steering to keep on heading
//            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
//
//            // Clip the speed to the maximum permitted value.
//            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
//
//            // Pivot in place by applying the turning correction
//            moveRobot(0, turnSpeed);
//
//            // Display drive status for the driver.
////            sendTelemetry(false);
//        }
//
//        // Stop all motion;
//        moveRobot(0, 0);
//    }
//
//    /**
//     *  Method to obtain & hold a heading for a finite amount of time
//     *  Move will stop once the requested time has elapsed
//     *  This function is useful for giving the robot a moment to stabilize it's heading between movements.
//     *
//     * @param maxTurnSpeed      Maximum differential turn speed (range 0 to +1.0)
//     * @param heading    Absolute Heading Angle (in Degrees) relative to last gyro reset.
//     *                   0 = fwd. +ve is CCW from fwd. -ve is CW from forward.
//     *                   If a relative angle is required, add/subtract from current heading.
//     * @param holdTime   Length of time (in seconds) to hold the specified heading.
//     */
//    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {
//
//        ElapsedTime holdTimer = new ElapsedTime();
//        holdTimer.reset();
//
//        // keep looping while we have time remaining.
//        while (opModeIsActive() && (holdTimer.time() < holdTime)) {
//            // Determine required steering to keep on heading
//            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);
//
//            // Clip the speed to the maximum permitted value.
//            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
//
//            // Pivot in place by applying the turning correction
//            moveRobot(0, turnSpeed);
//
//            // Display drive status for the driver.
////            sendTelemetry(false);
//        }
//
//        // Stop all motion;
//        moveRobot(0, 0);
//    }
//
//    // **********  LOW Level driving functions.  ********************
//
//    /**
//     * This method uses a Proportional Controller to determine how much steering correction is required.
//     *
//     * @param desiredHeading        The desired absolute heading (relative to last heading reset)
//     * @param proportionalGain      Gain factor applied to heading error to obtain turning power.
//     * @return                      Turning power needed to get to required heading.
//     */
//    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
//        targetHeading = desiredHeading;  // Save for telemetry
//
//        // Get the robot heading by applying an offset to the IMU heading
//        robotHeading = getRawHeading() - headingOffset;
//
//        // Determine the heading current error
//        headingError = targetHeading - robotHeading;
//
//        // Normalize the error to be within +/- 180 degrees
//        while (headingError > 180)  headingError -= 360;
//        while (headingError <= -180) headingError += 360;
//
//        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
//        return Range.clip(headingError * proportionalGain, -1, 1);
//    }
//
//    /**
//     * This method takes separate drive (fwd/rev) and turn (right/left) requests,
//     * combines them, and applies the appropriate speed commands to the left and right wheel motors.
//     * @param drive forward motor speed
//     * @param turn  clockwise turning motor speed.
//     */
//    public void moveRobot(double drive, double turn) {
//        driveSpeed = drive;     // save this value as a class member so it can be used by telemetry.
//        turnSpeed  = turn;      // save this value as a class member so it can be used by telemetry.
//
//        leftSpeed  = drive - turn;
//        rightSpeed = drive + turn;
//
//        // Scale speeds down if either one exceeds +/- 1.0;
//        double max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
//        if (max > 1.0)
//        {
//            leftSpeed /= max;
//            rightSpeed /= max;
//        }
//
//        frontleft.setPower(leftSpeed);
//        frontright.setPower(rightSpeed);
//        backleft.setPower(leftSpeed);
//        backright.setPower(rightSpeed);
//    }
//
//    /**
//     *  Display the various control parameters while driving
//     *
//     * @param straight  Set to true if we are driving straight, and the encoder positions should be included in the telemetry.
//     */
////    private void sendTelemetry(boolean straight) {
////
////        if (straight) {
////            telemetry.addData("Motion", "Drive Straight");
////            telemetry.addData("Target Pos L:R",  "%7d:%7d",      leftTarget,  rightTarget);
////            telemetry.addData("Actual Pos L:R",  "%7d:%7d",      leftDrive.getCurrentPosition(),
////                    rightDrive.getCurrentPosition());
////        } else {
////            telemetry.addData("Motion", "Turning");
////        }
////
////        telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", targetHeading, robotHeading);
////        telemetry.addData("Error:Steer",  "%5.1f:%5.1f", headingError, turnSpeed);
////        telemetry.addData("Wheel Speeds L:R.", "%5.2f : %5.2f", leftSpeed, rightSpeed);
////        telemetry.update();
////    }
//
//    /**
//     * read the raw (un-offset Gyro heading) directly from the IMU
//     */
//    public double getRawHeading() {
//        Orientation angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//        return angles.firstAngle;
//    }
//
//    /**
//     * Reset the "offset" heading back to zero
//     */
//    public void resetHeading() {
//        // Save a new heading offset equal to the current raw heading.
//        headingOffset = getRawHeading();
//        robotHeading = 0;
//    }
//}
