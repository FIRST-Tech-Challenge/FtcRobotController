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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwarePushbotBurg
{

    // Constants for autonomous tower heights:
    public double TIER_1_HEIGHT = 5.4;
    public double TIER_2_HEIGHT = 6.2;
    public double TIER_3_HEIGHT = 7.2;
    public double SAFE_HEIGHT = 5;
    public double MAX_HEIGHT = 6.9;
    public double MIN_HEIGHT = 4.1;

    public double SLOW_TURN_SPEED = 0.6;

    /* Public OpMode members. */
    public DcMotor  frontLeft   = null;
    public DcMotor  backLeft  = null;
    public DcMotor  frontRight   = null;
    public DcMotor  backRight  = null;
    public DcMotor  lift  =  null;
    public Servo gripper = null;
    public Rev2mDistanceSensor rangeSensor=null;
    public DcMotor rightEncoder = null;
    public DcMotor leftEncoder = null;
    public DcMotor backEncoder = null;
    //public CRServo  gripper  =  null;
    BNO055IMU imu;
  //  public DistanceSensor sensorRange;
  //  public DistanceSensor backRangeSensor;

    public double runPower = 1;
    private boolean positionIsValid = false;
    //public DcMotor  leftArm     = null;
    //public Servo    leftClaw    = null;
   // public Servo    rightClaw   = null;

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwarePushbotBurg(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        frontLeft  = hwMap.get(DcMotor.class, "FL");
        backLeft = hwMap.get(DcMotor.class, "BL");
        frontRight  = hwMap.get(DcMotor.class, "FR");
        backRight = hwMap.get(DcMotor.class, "BR");
        lift = hwMap.get(DcMotor.class,"lift");
       // sensorRange = hwMap.get(DistanceSensor.class, "distance");
     //   rangeSensor = hwMap.get(Rev2mDistanceSensor.class, "range");
     //   backRangeSensor = hwMap.get(DistanceSensor.class, "backrange");
        gripper = hwMap.get(Servo.class,"gripper");
        leftEncoder = hwMap.get(DcMotor.class, "leftEncoder");
        rightEncoder = hwMap.get(DcMotor.class, "rightEncoder");
        backEncoder = hwMap.get(DcMotor.class, "backEncoder");
        //leftArm    = hwMap.get(DcMotor.class, "left_arm");
        frontLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        backLeft.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        frontRight.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        backRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        lift.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        // Set all motors to zero power
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        //leftArm.setPower(0);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Define and initialize ALL installed servos.
        //leftClaw  = hwMap.get(Servo.class, "left_hand");
        //rightClaw = hwMap.get(Servo.class, "right_hand");
        //leftClaw.setPosition(MID_SERVO);
        //rightClaw.setPosition(MID_SERVO);

    }

    protected void initIMU(HardwareMap hardwareMap) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    public Orientation checkOrientation() {
        Orientation angles = this.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        this.imu.getPosition();
        return angles;
    }

    int getPosition(){
        return frontLeft.getCurrentPosition();
    }
    void displayPositions(Telemetry telemetry){
        telemetry.addData("fL encoder", frontLeft.getCurrentPosition());
        telemetry.addData("fR encoder", frontRight.getCurrentPosition());
        telemetry.addData("bL encoder", backLeft.getCurrentPosition());
        telemetry.addData("bR encoder", backRight.getCurrentPosition());

    }
    void runOneMotor(DcMotor motor, int position){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setTargetPosition(position);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(runPower);
    }
    void runToPosition(int positionFL, int positionFR, int positionBL, int positionBR){
        if(!positionIsValid){
            runOneMotor(frontLeft, positionFL);
            runOneMotor(frontRight,positionFR);
            runOneMotor(backLeft,positionBL);
            runOneMotor(backRight,positionBR);
            positionIsValid=true;
        }
    }
    void resetPosition(){
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        positionIsValid=false;
    }

    public void runNormal(){
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        positionIsValid=false;
    }

    public void stopmotor() {
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
    }


    public void power(double powerLeft, double powerRight) {
        frontLeft.setPower(powerLeft);
        backLeft.setPower(powerLeft);
        frontRight.setPower(powerRight);
        backRight.setPower(powerRight);
    }

    public boolean BaseClawHeight() {
        return AdjustClawHeight(4.2);
    }

    public void resetEncoders (){
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public int[] showEncoderValues (){
       int[] returnValues = {backEncoder.getCurrentPosition(), leftEncoder.getCurrentPosition(), rightEncoder.getCurrentPosition()};
       return returnValues;
    }


    public void runMecanum(double ly, double lx, double rx){

        double fl = ly - lx - rx;
        double fr = ly + lx + rx;
        double bl = ly + lx - rx;
        double br = ly - lx + rx;

        frontLeft.setPower(-fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(-br);
    }

    public boolean AdjustClawHeight(double desiredHeight) {
    /*    if (desiredHeight > MAX_HEIGHT) {
            desiredHeight = MAX_HEIGHT;
        }
        if (desiredHeight < MIN_HEIGHT) {
            desiredHeight = MIN_HEIGHT;
        }
        double clawHeight = sensorRange.getDistance(DistanceUnit.INCH);
        final double tolerance = .2;
        final double powerScale = .5;
        if(clawHeight < desiredHeight + tolerance && clawHeight > desiredHeight - tolerance){
            lift.setPower(0);
            return true;
        }
        // distance we want to move the lift.  If this is small, use less power.
        double delta = desiredHeight - clawHeight;
        if (delta > 0) {
            delta = Math.min(delta, .8);
        } else {
            delta = Math.max(delta, -.8);
        }
        double power = Math.min(Math.max(delta, -1), 1); // Clamp value between -1, 1
        lift.setPower(power);*/
        return false;

    }


    boolean isBusy(){
        return frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy();
    }

    public void openGrip(){  gripper.setPosition(1); }

    public void openGrip(double power){ gripper.setPosition(power); }

    public void closeGrip(){
        gripper.setPosition(0);
    }

    public void closeGrip(double position){
        gripper.setPosition(position);
    }

    public void stopGrip(){
        gripper.setPosition(0);
    }

   // public double getRangeDistance(){return rangeSensor.getDistance(DistanceUnit.INCH);}
 }

