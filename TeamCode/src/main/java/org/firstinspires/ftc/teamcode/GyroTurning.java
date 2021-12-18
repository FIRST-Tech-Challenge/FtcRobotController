/*
This program is testing to make sure the gyroscopic sensor works and we are able to turn to a specific angle. This will change due to us not testing it yet.
 */
package org.firstinspires.ftc.teamcode;

//program required imports
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//gyro specific imports
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Disabled
@TeleOp

public class GyroTurning extends LinearOpMode {
    //declaring gyro and motors
    private BNO055IMU imu;

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    //initializing gyro
    Orientation angles;
    @Override public void runOpMode(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile  = "BNO055IMUCalibration.json";
        parameters.loggingEnabled       = true;
        parameters.loggingTag           = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    //initializing motor
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

    //starting opMode code.
        waitForStart();
        while (opModeIsActive()){
            angles                       = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double targetAngle = 90;
            double currentAngle = angles.firstAngle;
            if (angles.firstAngle >= targetAngle-0.5 && angles.firstAngle <= targetAngle+0.5){
                frontLeft.setPower(0);
                frontRight.setPower(0);
                backLeft.setPower(0);
                backRight.setPower(0);
            }else if (angles.firstAngle >= targetAngle+0.5){
                if (angles.firstAngle <= targetAngle+10){
                    frontLeft.setPower(-0.15);
                    frontRight.setPower(0.15);
                    backLeft.setPower(-0.15);
                    backRight.setPower(0.15);
                }else {
                    frontLeft.setPower(-0.5);
                    frontRight.setPower(0.5);
                    backLeft.setPower(-0.5);
                    backRight.setPower(0.5);
                }
            }else if (angles.firstAngle <= targetAngle-0.5){
                if (angles.firstAngle >= targetAngle-10){
                    frontLeft.setPower(0.15);
                    frontRight.setPower(-0.15);
                    backLeft.setPower(0.15);
                    backRight.setPower(-0.15);

                }else{
                    frontLeft.setPower(0.5);
                    frontRight.setPower(-0.5);
                    backLeft.setPower(0.5);
                    backRight.setPower(-0.5);
                }
            }
            System.out.println(angles.firstAngle);
        }
    }
}
