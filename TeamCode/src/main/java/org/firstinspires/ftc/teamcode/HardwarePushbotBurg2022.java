package org.firstinspires.ftc.teamcode;/* Copyright (c) 2017 FIRST. All rights reserved.
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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
public class HardwarePushbotBurg2022
{

    public double SLOW_TURN_SPEED = 0.6;

    /* Public OpMode members. */
    public DcMotor  frontLeft   = null;
    public DcMotor  backLeft  = null;
    public DcMotor  frontRight   = null;
    public DcMotor  backRight  = null;
    public DcMotor  lift  = null;
    BNO055IMU imu;

    public DistanceSensor sensorRange;


    public double runPower = 1;
    private boolean positionIsValid = false;

    /* local OpMode members. */
    HardwareMap hardwareMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwarePushbotBurg2022(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hardwareMap = ahwMap;
        // Define and Initialize Motors
        frontLeft  = hardwareMap.get(DcMotor.class, "FL");
        backLeft = hardwareMap.get(DcMotor.class, "BL");
        frontRight  = hardwareMap.get(DcMotor.class, "FR");
        backRight = hardwareMap.get(DcMotor.class, "BR");
        lift = hardwareMap.get(DcMotor.class, "lift");
        sensorRange = hardwareMap.get(DistanceSensor.class, "distance");

        frontLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        backLeft.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        frontRight.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        backRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        lift.setDirection(DcMotor.Direction.FORWARD);
        // Set all motors to zero power
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        lift.setPower(0);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODERS);

        // Define and initialize ALL installed servos.
    }

    protected void initIMU() {
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

    public boolean powerToDegrees(double desiredAngle, double deadband, double speed, Telemetry telemetry){
        double startingAngle = checkOrientation().firstAngle;
        double upperDeadband = desiredAngle + deadband;
        double lowerDeadband = desiredAngle - deadband;
        boolean isAtPosition = false;

        if (desiredAngle > 0){
            if(lowerDeadband < checkOrientation().firstAngle && checkOrientation().firstAngle < upperDeadband){
                power(0, 0);
                isAtPosition = true;
            }
            else{
                power(speed, -speed);
            }

        }
        else if(desiredAngle < 0){
            if(lowerDeadband < checkOrientation().firstAngle && checkOrientation().firstAngle < upperDeadband){
                power(0, 0);
                isAtPosition = true;
            }
            else{
                power(-speed,speed);
            }

        }
        telemetry.addData("Lower: ", lowerDeadband);
        telemetry.addData("Desired: ", desiredAngle);
        telemetry.addData("Upper: ", upperDeadband);

        return isAtPosition;
    }

    public void runMecanum(double ly, double lx, double rx){

        double fl = ly - lx - rx;
        double fr = ly + lx + rx;
        double bl = ly + lx - rx;
        double br = ly - lx + rx;

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }


    boolean isBusy(){
        return frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy();
    }
 }

