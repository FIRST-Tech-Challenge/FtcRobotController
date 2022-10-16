package org.firstinspires.ftc.team6220_PowerPlay;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

// used for reading IMU
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

abstract public class BaseOpMode extends LinearOpMode {

    //Declared drive motors
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;

    // declare imu sensor
    BNO055IMU imu;
    // original angle reading from imu that will
    // be used to find unwanted angle offset during drive
    Orientation IMUOriginalAngles;
    // constant for converting angle error to motor speed
    double correctionConstant = 1/45.0;

    public void initHardware() {
        //init drive motors
        motorFL = hardwareMap.dcMotor.get("motorFL");
        motorFR = hardwareMap.dcMotor.get("motorFR");
        motorBL = hardwareMap.dcMotor.get("motorBL");
        motorBR = hardwareMap.dcMotor.get("motorBR");

        //reset encoder
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFL.setDirection(DcMotor.Direction.FORWARD);
        motorFR.setDirection(DcMotor.Direction.REVERSE);
        motorBL.setDirection(DcMotor.Direction.FORWARD);
        motorBR.setDirection(DcMotor.Direction.REVERSE);

        //create parameters for IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        // init IMU sensor
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        // preset the IMUAngles so it doesn't start on null
        // since it will only later be read when turning
        IMUOriginalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);;
    }


    public void driveRobot(double x, double y, double t)  {

        telemetry.addData("org: ", IMUOriginalAngles.firstAngle);
        telemetry.addData("t1=", t);
        telemetry.addData("error=", 0);
        // read imu when turning (when t != 0)
        if (t != 0) {
            IMUOriginalAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        // otherwise read imu for correction
        } else {
            Orientation currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            double angleError = IMUOriginalAngles.firstAngle - currentAngle.firstAngle;
            telemetry.addData("error=", angleError);
            t = -correctionConstant * angleError;
        }

        telemetry.addData("t2=", t);
        telemetry.update();

        double speedFL = (-y+x+t);
        double speedFR = (-y-x-t);
        double speedBL = (-y-x+t);
        double speedBR = (-y+x-t);

        motorFL.setPower(speedFL);
        motorFR.setPower(speedFR);
        motorBL.setPower(speedBL);
        motorBR.setPower(speedBR);

    }
}
