/*
    TODO: Add movement methods
    TODO: Add camera
    TODO: Add sensors (gyro, distance)
 */

package org.firstinspires.ftc.teamcode;

/*
    Imports
 */

import static java.lang.Thread.sleep;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class RobotClass {

    /*
        initializing variables
     */
    private LinearOpMode myOpMode = null;

    //defining motor variables
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;
    public BNO055IMU imu = null;
    public Orientation angles = null;

    public RobotClass(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void init(HardwareMap ahsMap) throws InterruptedException {
        //assigning motor variables to configuration name
        frontLeft = ahsMap.get(DcMotor.class, "frontLeft");
        frontRight = ahsMap.get(DcMotor.class, "frontRight");
        backLeft = ahsMap.get(DcMotor.class, "backLeft");
        backRight = ahsMap.get(DcMotor.class, "backRight");

        //setting direction of motors
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        //setting zero power behavior to brake
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //initializing imu
            // Set up the parameters with which we will use our IMU.
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample OpMode
            parameters.loggingEnabled      = true;
            parameters.loggingTag          = "IMU";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        //hardware mapping and initializing imu
        imu = ahsMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        while(!imu.isGyroCalibrated()){
            sleep(10);
        }
        myOpMode.telemetry.addData("Status", imu.isGyroCalibrated());
        myOpMode.telemetry.update();
    }

    public void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //stops all motors
    public void stopMotors() {
        //setting mode of motors
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //setting power to 0
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void moveNoEncoders(double powerLeft, double powerRight, int timeInMs) throws InterruptedException {
        //setting mode of motors
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //setting power of motors
        frontLeft.setPower(powerLeft);
        frontRight.setPower(powerRight);
        backLeft.setPower(powerLeft);
        backRight.setPower(powerRight);
        //waiting while running motors
        sleep(timeInMs);
        //stops motors
        stopMotors();
    }

    //Moving using encoders
    public void move(double power, double cm) throws InterruptedException{
        //setting number of ticks per 10 cm to get number of ticks per cm
        int ticksPer10cm = 0;
        int ticksPerCm = ticksPer10cm / 10;
        int target = (int) Math.round(cm * ticksPerCm);

        //resetting
        resetEncoders();

        //setting motor mode
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //setting target position of motors
        frontLeft.setTargetPosition(target);
        frontRight.setTargetPosition(target);
        backLeft.setTargetPosition(target);
        backRight.setTargetPosition(target);

        //waiting while the motors are busy
        while (frontLeft.isBusy()){
            sleep(50);
        }

        //stopping all motors
        stopMotors();
    }

    //turning with gyro code
    public void gyroTurning(double targetAngle) throws InterruptedException{
        angles = imu.getAngularOrientation();
        //using gyro
        if (angles.firstAngle >= targetAngle-0.5 && angles.firstAngle <= targetAngle+0.5){
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            sleep(500);
            if (angles.firstAngle >= targetAngle-0.5 && angles.firstAngle <= targetAngle + 0.5) {
                return;
            }
        }else if (angles.firstAngle >= targetAngle){
            if (angles.firstAngle <= targetAngle+10){
                frontLeft.setPower(0.15);
                frontRight.setPower(-0.15);
                backLeft.setPower(0.15);
                backRight.setPower(-0.15);
            }else {
                frontLeft.setPower(0.25);
                frontRight.setPower(-0.25);
                backLeft.setPower(0.25);
                backRight.setPower(-0.25);
            }
        }else if (angles.firstAngle <= targetAngle) {
            if (angles.firstAngle >= targetAngle - 10) {
                frontLeft.setPower(-0.15);
                frontRight.setPower(0.15);
                backLeft.setPower(-0.15);
                backRight.setPower(0.15);

            } else {
                frontLeft.setPower(-0.25);
                frontRight.setPower(0.25);
                backLeft.setPower(-0.25);
                backRight.setPower(0.25);
            }
        }
        stopMotors();
    }

    //strafing class with power and direction as parameters
    public void strafing (String direction, double power){
        if (direction == "left") {
            frontLeft.setPower(-power);
            frontRight.setPower(power);
            backLeft.setPower(power);
            backRight.setPower(-power);
        } else if (direction == "right"){
            frontLeft.setPower(power);
            frontRight.setPower(-power);
            backLeft.setPower(-power);
            backRight.setPower(power);
        } else {
            myOpMode.telemetry.addData("Error", "Invalid direction");
            myOpMode.telemetry.update();
        }
    }
}
