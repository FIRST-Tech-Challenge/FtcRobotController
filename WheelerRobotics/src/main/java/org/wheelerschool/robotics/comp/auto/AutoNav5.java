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
import com.qualcomm.robotcore.util.ElapsedTime;
import com.sun.tools.javac.comp.Todo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous
public class AutoNav5 extends LinearOpMode {

    BNO055IMU imu;
    DcMotor motorFrontLeft = null;
    DcMotor motorBackLeft = null;
    DcMotor motorFrontRight = null;
    DcMotor motorBackRight = null;
    private ElapsedTime runtime = new ElapsedTime();
    private double seconds = runtime.seconds();

    private Orientation angles;




    @Override
    public void runOpMode() throws InterruptedException {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        angles = getAngles();


        waitForStart();
        turnRadians(Math.PI/2, 1);
        while (runtime.seconds()<15) { // turn 90 then go straight
            motorDriveForward(1.0);
        }
        runtime.reset();
        while(opModeIsActive()){
            angles   = getAngles();
            telemetry.addData("Elapsed Time  ", floor(runtime.seconds() / 60) + ":" + runtime.seconds() % 60 + ":" + round(runtime.milliseconds() % 1000) );

            // 685mm  0.5power 1sec

            telemetry.update();


        }
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

}
