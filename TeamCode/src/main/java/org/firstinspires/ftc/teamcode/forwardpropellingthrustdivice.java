package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
@TeleOp(name="forwardpropellingthrustdivice")
public class forwardpropellingthrustdivice extends LinearOpMode {


    DcMotor backLeft;
    DcMotor backRight;
    DcMotor frontLeft;
    DcMotor frontRight;
//        ColorSensor color1;
//        DistanceSensor distance1;
//        BNO055IMU imu;

    @Override
    public void runOpMode() {
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        //           color1 = hardwareMap.get(ColorSensor.class, "color1");
//            distance1 = hardwareMap.get(DistanceSensor.class, "distance1");
//            imu = hardwareMap.get(BNO055IMU.class, "imu");
        backLeft.setDirection(DcMotor.Direction.REVERSE);//-
        backRight.setDirection(DcMotor.Direction.FORWARD);//+
        frontLeft.setDirection(DcMotor.Direction.REVERSE);//-
        frontRight.setDirection(DcMotor.Direction.REVERSE);//+
        waitForStart();
        while (opModeIsActive()) {
            double leftdrive = -gamepad1.left_stick_y;
            double rightdrive = -gamepad1.left_stick_y;
            double driveright = -gamepad1.left_stick_x;
            double driveleft = -gamepad1.left_stick_x;
            double fl = leftdrive + driveleft;  //front left=fl
            double bl = leftdrive - driveleft;//back left=bl
            double fr = rightdrive - driveright;//front right=fr
            double br = rightdrive + driveright;//back right=br

            backLeft.setPower(bl*0.5);
            backRight.setPower(br*0.5);
            frontLeft.setPower(fl*0.5);
            frontRight.setPower(fr*0.5);

//            while (gamepad1.circle) {
//                double driveright = -gamepad1.left_stick_x;
//                double driveleft = -gamepad1.left_stick_x;
//                backLeft.setDirection(DcMotor.Direction.FORWARD);//+
//                backRight.setDirection(DcMotor.Direction.REVERSE);//-
//                frontLeft.setDirection(DcMotor.Direction.FORWARD);//+
//                frontRight.setDirection(DcMotor.Direction.REVERSE);//-
//                backLeft.setPower(driveright);
//                frontLeft.setPower(driveleft);
//                backRight.setPower(driveright);
//                frontRight.setPower(driveleft);
            }
        }
    }