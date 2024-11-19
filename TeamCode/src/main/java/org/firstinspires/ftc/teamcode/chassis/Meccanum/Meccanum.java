package org.firstinspires.ftc.teamcode.chassis.Meccanum;


import static java.lang.Math.abs;
import static java.lang.Math.pow;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.chassis.Chassis;

// robot driving and motion class

public class Meccanum implements Chassis {
    public double offset = 0;

    protected double[] left = {
            1,  -1,
            -1,  1
    };
    protected double[] back = {
            -1, -1,
            -1, -1
    };
    protected double[] clock = {
            -1,  1,
            -1,  1
    };

    public BNO055IMU imu = null;
    protected DistanceSensor distanceBack = null;
    protected DistanceSensor distanceFront = null;
    protected DistanceSensor distanceLeft = null;
    protected DistanceSensor distanceRight = null;

    protected DcMotorEx motorFrontRight = null;
    protected DcMotorEx motorBackRight = null;
    protected DcMotorEx motorFrontLeft = null;
    protected DcMotorEx motorBackLeft = null;

    HardwareMap hw = null;
    public void init(HardwareMap hardwareMap){
        hw = hardwareMap;
        // init the class, declare all the sensors and motors and stuff
        // should be called before using class ALWAYS

        // internal IMU setup (copied and pasted, idk what it really does, but it works)
        //BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        //parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        //parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        //parameters.loggingEnabled      = true;
        //parameters.loggingTag          = "IMU";
        //parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

       // imu = hardwareMap.get(BNO055IMU.class, "imu");
       // imu.initialize(parameters);
        // angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        //distace sensors (unused for now)


        distanceBack = tryDeclareDistanceSensor("distanceBack", hw);
        distanceRight = tryDeclareDistanceSensor("distanceRight", hw);
        distanceLeft = tryDeclareDistanceSensor("distanceLeft", hw);
        distanceFront = tryDeclareDistanceSensor("distanceFront", hw);

        // Meccanum Motors Definition and setting prefs

        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("motorBackRight");

        // Reverse the left side motors and set behaviors to stop instead of coast
        //motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // define hw as the hardware map for possible access later in this class
        hw = hardwareMap;
    }

    // SUPPORT METHODS
    public void motorDrive(double motorFrontLeftPower, double motorBackLeftPower, double motorFrontRightPower, double motorBackRightPower){
        // drive the motors at custom powers for each
        // used for every other drive class

        motorBackLeft.setPower(motorBackLeftPower);
        motorFrontLeft.setPower(motorFrontLeftPower);
        motorBackRight.setPower(motorBackRightPower);
        motorFrontRight.setPower(motorFrontRightPower);
    }

    public void motorDriveXYVectors(double xvec, double yvec, double spinvec){
        // this class drives the robot in the direction of vectors from a joystick and a spin value
        // used for teleop mode driving wheels with joysticks


        double y = pow(yvec,1); // Remember, this is reversed!
        double x = pow(xvec * 1.1,1); // Counteract imperfect strafing
        double rx = pow(spinvec,1);


        //denominator is the largest motor power (absolute value) or 1
        //this ensures all the powers maintain the same ratio, but only when
        //at least one is out of the range [-1, 1]
        double denominator = Math.max(abs(y) + abs(x) + abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        motorDrive(frontLeftPower,backLeftPower, frontRightPower, backRightPower);

    }

    protected void driveVector(double[] arr){
        motorFrontLeft.setPower(arr[0]);
        motorFrontRight.setPower(arr[1]);
        motorBackLeft.setPower(arr[2]);
        motorBackRight.setPower(arr[3]);
    }
    /**
     * Takes an array and returns the value furthest from 0.
     * @param arr the array to act upon.
     * @return The absolute maximum of the array.
     */
    protected double absmac(double[] arr){

        int outi = 0; // index of max
        for (int i = 0; i<arr.length; i++){
            if( abs(arr[i]) > abs(arr[outi])){
                outi = i;
            }
        }
        return arr[outi];

    }
    /**
     * Stops all robot motors
     */
    public void motorStop(){
        // stop all the motors
        // used at the end of all movement functions
        motorBackLeft.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontRight.setPower(0);
    }

    /**
     * Gets an orientation which reports all robot rotation
     * @return Orientation object with angle unit degrees
     */
    public Orientation getAnglesDeg() {
        // gets current angle on field [-180, 180]
        // useful for angle methods
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }
    public void fieldCentricDrive(double x, double y, double rx){
        // Read inverse IMU heading, as the IMU heading is CW positive
        double botHeading = -imu.getAngularOrientation().firstAngle - offset;

        double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
        double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        motorFrontLeft.setPower(frontLeftPower);
        motorBackLeft.setPower(backLeftPower);
        motorFrontRight.setPower(frontRightPower);
        motorBackRight.setPower(backRightPower);
    }

}
