package org.wheelerschool.robotics.comp.auto;

import static com.sun.tools.doclint.Entity.and;
import static com.sun.tools.doclint.Entity.ge;
import static com.sun.tools.doclint.Entity.pi;
import static com.sun.tools.doclint.Entity.tau;
import static java.lang.Math.floor;
import static java.lang.Math.round;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.comp.Todo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous
public class AutoNav5 extends LinearOpMode {


    private ElapsedTime runtime = new ElapsedTime();

    private Servo servo0;
    private DcMotor arm;


    private final double SERVO_FULLY_CLOSED = 0;
    private final double SERVO_FULLY_OPENED = 0;
    private final double ARM_MAX_SPEED = 0;
    private final double HIGH_SPINNER_POWER = 0;
    private final double OPTIMAL_SPINNER_POWER = 0;
    private final double MOTOR_STOP = 0;

    private final double NORMAL_SPEED = 0.5;



    private DcMotor spinner;

    private BNO055IMU imu;

    private DcMotor motorFrontRight;
    private DcMotor motorBackRight;
    private DcMotor motorFrontLeft;
    private DcMotor motorBackLeft;

    private Orientation angles;




    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // internal IMU setup

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        angles   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        // Meccanum Motors Definition and setting prefs

        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        // Reverse the left side motors and set behaviors to stop instead of coast
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //define arm and servo objects and also spinner
        servo0 = hardwareMap.get(Servo.class, "servo-0");
        arm = hardwareMap.get(DcMotor.class, "arm");
        spinner = hardwareMap.get(DcMotor.class, "spinner");

        //set prefs for arm and servo
        servo0.setDirection(Servo.Direction.FORWARD);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spinner = hardwareMap.get(DcMotor.class, "spinner");
        telemetry.addData("Motor Names & Numbers: ", hardwareMap.allDeviceMappings);

        waitForStart();



        runtime.reset();
        while(opModeIsActive()){
            angles   = getAngles();
            telemetry.addData("Elapsed Time  ", floor(runtime.seconds() / 60) + ":" + runtime.seconds() % 60 + ":" + round(runtime.milliseconds() % 1000) );

            // 685mm  0.5power 1sec

            telemetry.update();
            executeAutomaticSequence2();


        }
    }
    private void executeAutomaticSequence2(){
        // auto for near carousel

        motorDriveBack(NORMAL_SPEED);
        driveEncodedDistance();

        spinnySpinForward(OPTIMAL_SPINNER_POWER);

        spinnySpinForward();





    }
    private void motorDrive(double motorFrontLeftPower, double motorBackLeftPower, double motorFrontRightPower, double motorBackRightPower){
        motorBackLeft.setPower(motorBackLeftPower);
        motorFrontLeft.setPower(motorFrontLeftPower);
        motorBackRight.setPower(motorBackRightPower);
        motorFrontRight.setPower(motorFrontRightPower);
    }
    private void motorStop(){
        motorBackLeft.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackRight.setPower(0);
        motorFrontRight.setPower(0);
    }
    private void motorDriveForward(double speed){
        motorDrive(speed, speed, speed, speed);
    }
    private void motorDriveLeft(double speed){
        motorDrive(speed, -speed, speed, -speed);
    }
    private void motorDriveRight(double speed){
        motorDrive(-speed, speed, -speed, speed);
    }
    private void motorDriveBack(double speed){
        motorDrive(-speed, -speed, -speed, -speed);
    }
    private void motorSpinLeft(double speed){
        motorDrive(-speed, -speed, speed, speed);
    }
    private void motorSpinRight(double speed){
        motorDrive(speed, speed, -speed, -speed);
    }
    private void spinnySpin(double speed){
        arm.setPower(speed);
    }
    private void spinnySpinForward(double speed){
        spinnySpin(speed);
    }
    private void spinnySpinBackward(double speed){
        spinnySpin(-speed);
    }
    private void turnRadians(double radians, double speed) {
        turnRadians(radians, speed, angles.firstAngle);
    }
    private double turnRadians(double radians, double speed, double startRadians) {
        double target = startRadians + radians;
        double minSpeed = 0.1;
        while(angles.firstAngle< target && opModeIsActive()){

            if(target-angles.firstAngle>minSpeed) {
                motorSpinRight(target - angles.firstAngle);
            }else{
                motorSpinRight(minSpeed);
            }

            telemetry.addData("Angles: ", angles.firstAngle);
            telemetry.addData("Not Angles: ", angles.firstAngle - target);
            angles = getAngles();
            telemetry.update();
        }


        motorStop();
        return target-startRadians;

    }
    private Orientation getAngles() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
    }
    private void spinRotations(double rotation){
        double encodedSpins = 0;
        while (opModeIsActive() && encodedSpins < rotation){
            double encodedSpins = 0;
        }
        motorStop();
    }
    private void driveEncodedDistance(double distance){
        double encodedDistance = 0;
        while (opModeIsActive() && encodedDistance < distance){


        }
        motorStop();
        double pos = motorBackLeft.getCurrentPosition();
        telemetry.addData("Motor Get Position Return: ", pos);
        telemetry.update();
    }

}
