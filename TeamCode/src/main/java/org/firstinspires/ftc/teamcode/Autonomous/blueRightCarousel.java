//TODO: Test Code

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous

public class blueRightCarousel extends LinearOpMode {
    
    BNO055IMU imu;
    Orientation angles;
    
    //Init motors.
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    DcMotor carousel;
    DcMotor crane;

    public void runOpMode()
    {
        initDriveMotors();
        initMiscMotors();
        initGyro();

        waitForStart();
        telemetry.addLine("Y'all ready for this?");
        telemetry.update();
        
        if (opModeIsActive()){
            //move forwards a few inches
            move(-0.25, 500);
//             //turn 90 degrees counter clockwise

            while (!gyroTurning(90)) {
            }
//             //reverse back into carousel
            move(0.5, 500);
//             //turn on carousel motor
//             carouselMotor(0.25, 2500);
        }
    }

    //Init methods
    public void initDriveMotors()
    {
        //Setting variables in code to a motor in the configuration.
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        //Setting direction of motors.
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void initMiscMotors()
    {
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        crane = hardwareMap.get(DcMotor.class, "crane");
    }

    public void initGyro()
    {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile  = "BNO055IMUCalibration.json";
        parameters.loggingEnabled       = true;
        parameters.loggingTag           = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    //Movement methods
    public boolean gyroTurning(int target)
    {
        angles                       = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentAngle = angles.firstAngle;
        boolean foundTarget = false;
        if (angles.firstAngle >= target-0.5 && angles.firstAngle <= target+0.5) {
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            foundTarget = true;
        } else if (angles.firstAngle >= target+0.5) {
            if (angles.firstAngle <= target+1) {
                frontLeft.setPower(-0.15);
                frontRight.setPower(0.15);
                backLeft.setPower(-0.15);
                backRight.setPower(0.15);
                foundTarget = false;
            } else {
                frontLeft.setPower(-0.5);
                frontRight.setPower(0.5);
                backLeft.setPower(-0.5);
                backRight.setPower(0.5);
                foundTarget = false;
            }
        } else if (angles.firstAngle > target-0.5) {
            if (angles.firstAngle >= 80) {
                frontLeft.setPower(0.15);
                frontRight.setPower(-0.15);
                backLeft.setPower(0.15);
                backRight.setPower(-0.15);
                foundTarget = false;
            } else {
                frontLeft.setPower(0.5);
                frontRight.setPower(-0.5);
                backLeft.setPower(0.5);
                backRight.setPower(-0.5);
                foundTarget = false;
            }
        }
        return foundTarget;
    }

    public void stopMotors(){
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void move(double power, int time){
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
        sleep(time);
        stopMotors();
    }

    public void strafeLeft(double power, int time){
        frontLeft.setPower(-power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(-power);
        sleep(time);
        stopMotors();
    }

    public void strafeRight(double power, int time){
        frontLeft.setPower(power);
        frontRight.setPower(-power);
        backLeft.setPower(-power);
        backRight.setPower(power);
        sleep(time);
        stopMotors();
    }

    //Other methods
    public void carouselMotor(double power, int time){
        carousel.setPower(power);
        sleep(time);
        carousel.setPower(0);
    }
}
