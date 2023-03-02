/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name = "Left Side April Tag Auto", group = "Linear Opmode")
public class LeftSideAprilTagAutonomous extends LinearOpMode
{
    public DcMotorEx leftDrive;
    public DcMotorEx rightDrive;
    public DcMotorEx leftBackDrive;
    public DcMotorEx rightBackDrive;

    public DcMotorEx liftMotor;
    public Servo servoGrabber1;
    public Servo servoGrabber2;

    // Sensors
    BNO055IMU imu;
    ColorSensor color;

    public ElapsedTime runtime = new ElapsedTime();

    public boolean hasMoved = false;

    // Linear slide information
    public double liftCorrection = 1;

    public double tickPerInchForLift = 173 * liftCorrection;

    final double MIN_LIFT_POS = 0;
    final double MAX_LIFT_POS = tickPerInchForLift * 34.5;

    // Servo information
    static final double MAX_POS     =    .52;
    static final double MAX_POS2    =    .48;
    static final double MIN_POS     =     1;
    static final double MIN_POS2    =     0;

    public double position = 1;
    public double position2 = 0;

    // Wheel constants
    public double ticksPerRotation = 537.6; // For AndyMark NeveRest 20
    public double rpm = 340;
    public double diameter = 10; //cm

    public double angleCorrectionCW = 8.17;
    public double angleCorrectionCCW = 11.26;

    // Cone information
    public int coneCount = 1;
    public double firstConePos = 5.775;
    public double coneSeparationDistance = firstConePos / 5;

    // General constants
    double oneFootCm = 30.48;

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

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

    int ID_TAG_OF_INTEREST_1 = 7; // Tags from the 36h11 family
    int ID_TAG_OF_INTEREST_2 = 9;
    int ID_TAG_OF_INTEREST_3 = 12;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        setUpHardware();

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

        telemetry.setMsTransmissionInterval(50);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        turnNinety(true);
        telemetry.addData("angle:",getAngle());
        telemetry.update();
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == ID_TAG_OF_INTEREST_1 || tag.id == ID_TAG_OF_INTEREST_3 || tag.id == ID_TAG_OF_INTEREST_2)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /*moveLift(19);
        waitTime(1);
        turnNinety(false);
        while(opModeIsActive()) {
            telemetry.addData("angle: ", getAngle());
            telemetry.update();
        }*/
        if(tagOfInterest == null){
            moveGrabber(true);
            waitTime(.5);
            moveLiftAndDrive(true,18.25,17);
            turnNinety(true);
            moveInchAmount(true,.1);
            sleep(500);
            moveGrabber(false);
            moveInchAmount(false,.1);
            turnNinety(false);
            moveLiftAndDrive(true,39.25,0);
            sleep(100);
            turnNinety(true);
            /*for(int i = 0; i < 3; i++){
                moveToTape();
                pickupCone();
                moveLiftAndDrive(false,45,MAX_LIFT_POS);
                sleep(100);
                turnNinety(false);
                moveLift(30);
                moveGrabber(false);
                moveLift(32);
                sleep(100);
                turnNinety(true);
                moveLiftAndDrive(true,28,8);
            }*/
        }else{
            moveGrabber(true);
            waitTime(.5);
            moveLiftAndDrive(true,16.25,17);
            turnNinety(true);
            moveInchAmount(true,2);
            sleep(500);
            moveGrabber(false);
            moveInchAmount(false,2);
            turnNinety(false);
            moveLiftAndDrive(true,11.75,0);
            if(tagOfInterest.id == ID_TAG_OF_INTEREST_1){
                sleep(100);
                turnNinety(false);
                moveInchAmount(true,24);
            }else if(tagOfInterest.id == ID_TAG_OF_INTEREST_3){
                sleep(100);
                turnNinety(true);
                moveInchAmount(true,24);
            }
        }
    }

    public void setUpHardware() { // Assigns motor names in phone to the objects in code
        leftDrive  = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        rightDrive = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back_drive");

        liftMotor  = hardwareMap.get(DcMotorEx.class, "lift_motor");
        servoGrabber1 = hardwareMap.get(Servo.class, "servo_grabber_one");
        servoGrabber2 = hardwareMap.get(Servo.class, "servo_grabber_two");

        color = hardwareMap.get(ColorSensor.class, "sensor_color");

        // Makes all motors go forward, if they don't, switch the direction
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftMotor.setDirection(DcMotor.Direction.FORWARD);

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        servoGrabber1.setPosition(position);
        servoGrabber2.setPosition(position2);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        // Technically this is the default, however specifying it is clearer
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        // Without this, data retrieving from the IMU throws an exception
        imu.initialize(parameters);

        liftMotor.setMode(STOP_AND_RESET_ENCODER); // Set to 0 just in case
    }

    public void strafeVelo(boolean isLeft, double maxPercent, double time){
        //Strafe left or right
        int direction = -1;
        if(isLeft){
            direction = 1;
        }
        runtime.reset();
        while(opModeIsActive() && runtime.seconds() < time){
            leftWheel(maxPercent * direction);
            rightWheel(maxPercent * direction * -1);
            leftBackWheel(maxPercent * direction * -1);
            rightBackWheel(maxPercent * direction);
        }
        if(runtime.seconds() >= time){
            motorsOff();
        }
    }

    /**
     * Turns 90 degrees either clockwise or counter clockwise, depending on value of CW
     * @param CW True or false
     */
    public void turnNinety(boolean CW){
        double originalAngle = getAngle();

        if(CW) {
            if(originalAngle - 90 < 0 && !(originalAngle < 0)) {
                while (opModeIsActive() && (getAngle() < originalAngle + 5 || getAngle() > originalAngle - 90 + angleCorrectionCW + 360)) {
                    leftVelo(.75);
                    rightVelo(-.75);
                }
            }else{
                while (opModeIsActive() && getAngle() > originalAngle - 90 + angleCorrectionCW && getAngle() < originalAngle + 5) {
                    leftVelo(.75);
                    rightVelo(-.75);
                }
            }
        }else{
            if(originalAngle + 90 > 360) {
                while (opModeIsActive() && (getAngle() > originalAngle - 5 || getAngle() < originalAngle + 90 - angleCorrectionCCW - 360)) {
                    leftVelo(-.75);
                    rightVelo(.75);
                }
            }else{
                while (opModeIsActive() && getAngle() < originalAngle + 90 - angleCorrectionCCW && getAngle() > originalAngle - 5) {
                    leftVelo(-.75);
                    rightVelo(.75);
                }
            }
        }
        motorsOff();
    }

    /**
     * Turns to the desired angle as specified in targetAngle, assuming starting position is 0 degrees
     * @param targetAngle Target angle to rotate to
     */
    public void turnToAngle(double targetAngle){
        double originalAngle = getAngle();
        double changeInAngle = optimalAngleChange(targetAngle);
        double turningCorrection = (angleCorrectionCW / 90) * changeInAngle;

        boolean CW = optimalDirection(targetAngle);

        if(CW) {
            if(originalAngle - changeInAngle < 0 && !(originalAngle < 0)) {
                while (opModeIsActive() && (getAngle() < originalAngle + 5 || getAngle() > originalAngle - changeInAngle + turningCorrection + 360)) {
                    leftVelo(.75);
                    rightVelo(-.75);
                }
            }else{
                while (opModeIsActive() && getAngle() > originalAngle - changeInAngle  + turningCorrection && getAngle() < originalAngle + 5) {
                    leftVelo(.75);
                    rightVelo(-.75);
                }
            }
        }else{
            if(originalAngle + changeInAngle > 360) {
                while (opModeIsActive() && (getAngle() > originalAngle - 5 || getAngle() < originalAngle + changeInAngle - turningCorrection - 360)) {
                    leftVelo(-.75);
                    rightVelo(.75);
                }
            }else{
                while (opModeIsActive() && getAngle() < originalAngle + changeInAngle - turningCorrection && getAngle() > originalAngle - 5) {
                    leftVelo(-.75);
                    rightVelo(.75);
                }
            }
        }
        motorsOff();
    }

    /**
     * Moves forward or backward depending on the amount specified in inches
     * @param forward True or false
     * @param inches Amount to move forward
     */
    public void moveInchAmount(boolean forward, double inches){
        //using motor encoders
        double driveTrainCorrection = 1;

        double oneRotationDistance = diameter * Math.PI; // In cm
        double rotationAmount = (oneFootCm / 12) / oneRotationDistance;
        double totalTicks = rotationAmount * ticksPerRotation * inches * 2 * driveTrainCorrection; // *2 is to account for gear ratio

        resetEncoders();

        if(forward){
            motorsOn(.75);
            while(opModeIsActive() && leftBackDrive.getCurrentPosition() < totalTicks){
                telemetry.addData("Encoder Value:", leftBackDrive.getCurrentPosition());
                telemetry.update();
            }
        }else{
            totalTicks = -totalTicks;
            motorsOn(-.75);
            while(opModeIsActive() && leftBackDrive.getCurrentPosition() > totalTicks){
                telemetry.addData("Encoder Value:", leftBackDrive.getCurrentPosition());
                telemetry.update();
            }
        }
        motorsOff();
        resetEncoders();
    }

    /**
     * Picks up one of the cones from the stacked pile depending on how many cones there are on it
     */
    public void pickupCone(){
        double conePosMod = coneCount * coneSeparationDistance;
        moveLift(firstConePos - conePosMod);
        moveGrabber(true);
        waitTime(1);
        moveLift(firstConePos - conePosMod + 5);
        coneCount++;
    }

    /**
     * Drives forward until it reads red or blue tape below it
     */
    public void moveToTape(){
        while(opModeIsActive() && (color.red() < 1200 || color.blue() < 1200)){
            motorsOn(.5);
        }
        motorsOff();
        moveLift(12);
        moveInchAmount(true, 10.5);
    }

    /**
     * Moves the linear slide up to a specified height in inches
     * @param toHeight Height to move to (inches)
     */
    public void moveLift(double toHeight){
        if(opModeIsActive()){
            int targetTick = (int) (tickPerInchForLift * toHeight);

            // Initial movement to get as close as possible
            liftMotor.setTargetPosition(targetTick);
            liftMotor.setMode(RUN_TO_POSITION);
            liftMotor.setPower(1);

            while(opModeIsActive() && liftMotor.isBusy()){
                telemetry.addData("Lift Encoder Value:",liftMotor.getCurrentPosition());
                telemetry.update();
            }
            liftMotor.setPower(0);

            // Secondary movement to get to correct value using lower power
            liftMotor.setTargetPosition(targetTick);
            liftMotor.setMode(RUN_TO_POSITION);
            liftMotor.setPower(.25);

            while(opModeIsActive() && liftMotor.isBusy()){
                telemetry.addData("Lift Encoder Value:",liftMotor.getCurrentPosition());
                telemetry.update();
            }
            liftMotor.setPower(0);
        }
    }

    /**
     * Combines moveLift and moveInchAmount to be faster and more fluid. Moves forward and raises lift at the same time,
     * stopping individually when it reaches the desired height or distance.
     * @param forward True or false
     * @param distance How many inches to move forward
     * @param height Distance to raise linear slide by
     */
    public void moveLiftAndDrive(boolean forward, double distance, double height){
        // Linear slide calculations
        int targetTick = (int) (tickPerInchForLift * height);

        boolean correctionsDone = false;
        boolean motorsOff = false;
        boolean liftOff = false;

        // Drive train calculations
        double driveTrainCorrection = 1;

        double oneRotationDistance = diameter * Math.PI; // In cm
        double rotationAmount = (oneFootCm / 12) / oneRotationDistance;
        double totalTicks = rotationAmount * ticksPerRotation * distance * 2 * driveTrainCorrection; // *2 is to account for gear ratio

        resetEncoders();

        liftMotor.setTargetPosition(targetTick);
        liftMotor.setMode(RUN_TO_POSITION);

        liftMotor.setPower(1);

        runtime.reset();
        if(forward){
            motorsOn(.75);
            while(opModeIsActive() && (!liftOff || !motorsOff)){
                if(leftBackDrive.getCurrentPosition() >= totalTicks){
                    motorsOff();
                    motorsOff = true;
                }
                if(liftMotor.getCurrentPosition() > targetTick - 173 && liftMotor.getCurrentPosition() < targetTick + 173 && !correctionsDone){
                    liftMotor.setTargetPosition(targetTick);
                    liftMotor.setMode(RUN_TO_POSITION);
                    liftMotor.setPower(.25);
                    correctionsDone = true;
                }else if(liftMotor.getCurrentPosition() > targetTick - 17.3 && liftMotor.getCurrentPosition() < targetTick + 17.3){
                    liftMotor.setPower(0);
                    liftOff = true;
                }
                if(runtime.seconds() > 8){
                    break;
                }
                telemetry.addData("Lift is busy:", liftMotor.isBusy());
                telemetry.update();
            }
        }else{
            totalTicks = -totalTicks;
            motorsOn(-.75);
            while(opModeIsActive() && (!liftOff || !motorsOff)){
                if(leftBackDrive.getCurrentPosition() <= totalTicks){
                    motorsOff();
                    motorsOff = true;
                }
                if(!liftMotor.isBusy() && !correctionsDone){
                    liftMotor.setTargetPosition(targetTick);
                    liftMotor.setMode(RUN_TO_POSITION);
                    liftMotor.setPower(.25);
                    correctionsDone = true;
                }else{
                    liftMotor.setPower(0);
                    liftOff = true;
                }
                if(runtime.seconds() > 6){
                    break;
                }
                telemetry.addData("Lift is busy:", liftMotor.isBusy());
                telemetry.update();
            }
        }
        resetEncoders();
    }

    /**
     * Opens and closes the grabber
     * @param closed True or false
     */
    public void moveGrabber(boolean closed){
        if(closed){
            servoGrabber1.setPosition(MAX_POS);
            servoGrabber2.setPosition(MAX_POS2);
        }else{
            servoGrabber1.setPosition(MIN_POS);
            servoGrabber2.setPosition(MIN_POS2);
        }
    }

    public void leftVelo(double maxPercent){ //sets power for left wheels
        leftWheel(maxPercent);
        leftBackWheel(maxPercent);
    }

    public void rightVelo(double maxPercent){ //sets power for right wheels
        rightWheel(maxPercent);
        rightBackWheel(maxPercent);
    }

    public void leftWheel(double percent){
        leftDrive.setPower(percent);
    }
    public void rightWheel(double percent){
        rightDrive.setPower(percent);
    }
    public void leftBackWheel(double percent){
        leftBackDrive.setPower(percent);
    }
    public void rightBackWheel(double percent){
        rightBackDrive.setPower(percent);
    }

    //Primitive functions

    /**
     * Pauses all movement for the specified time in seconds
     * @param time How many seconds
     */
    @SuppressWarnings("StatementWithEmptyBody")
    public void waitTime(double time){ // Waits for time (seconds)
        runtime.reset();
        while(opModeIsActive() && runtime.seconds()<time){
        }
    }

    /**
     * Turns all wheels off
     */
    public void motorsOff(){
        leftDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    /**
     * Turns all wheels on at specified power
     * @param powerPercent Percent of power to run at
     */
    public void motorsOn(double powerPercent){
        leftDrive.setPower(powerPercent);
        leftBackDrive.setPower(powerPercent);
        rightDrive.setPower(powerPercent);
        rightBackDrive.setPower(powerPercent);
    }

    /**
     * Resets all wheel encoders to be at 0 and to run without encoders
     */
    public void resetEncoders() { // Reset all encoder positions
        leftDrive.setMode(STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(STOP_AND_RESET_ENCODER);
        rightDrive.setMode(STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(STOP_AND_RESET_ENCODER);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void encoderTarget(int target){
        leftDrive.setTargetPosition(target);
        leftBackDrive.setTargetPosition(target);
        rightDrive.setTargetPosition(target);
        rightBackDrive.setTargetPosition(target);
    }

    /**
     * Calculates the angle the robot is at and returns the orientation
     * @return Current Orientation
     */
    public double getAngle(){
        if(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < 0){
            return 360 + imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle; // Addition since the value is negative
        }
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }

    /**
     * Finds whether turning clockwise or counterclockwise reaches the target angle faster.
     * Returns true for clockwise and false for counterclockwise
     * @param target Target angle to rotate to
     * @return true (CW) or false (CCW)
     */
    public boolean optimalDirection(double target){
        if(target > getAngle()){
            double x1 = target - getAngle();
            double x2 = 360 - x1;
            return x1 < x2;
        }else{
            double x1 = getAngle() - target;
            double x2 = 360 - x1;
            return x1 > x2;
        }
    }

    /**
     * Finds the smallest angle change needed to reach the target angle from current angle
     * @param target Target angle to reach
     * @return Shortest angle to target
     */
    public double optimalAngleChange(double target) {
        double x1;
        double x2;
        if(target > getAngle()){
            x1 = target - getAngle();
        }else{
            x1 = getAngle() - target;
        }
        x2 = 360 - x1;
        return Math.min(x1, x2);
    }

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection)
    {
        if(detection.id == 7){
            telemetry.addLine("\nDetected tag location = 1");
        }else if(detection.id == 9){
            telemetry.addLine("\nDetected tag location = 2");
        }else if(detection.id == 12){
            telemetry.addLine("\nDetected tag location = 3");
        }
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
}