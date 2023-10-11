package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="forwardpropellingthrustdiviceblurb")
public class forwardpropellingthrustdiviceblurb extends LinearOpMode {


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
        waitForStart();
        while (opModeIsActive()) {
            double leftdrive = -gamepad1.left_stick_y;
            double rightdrive = -gamepad1.left_stick_y;
            backLeft.setDirection(DcMotor.Direction.FORWARD);
            backRight.setDirection(DcMotor.Direction.REVERSE);
            frontLeft.setDirection(DcMotor.Direction.FORWARD);
            frontRight.setDirection(DcMotor.Direction.REVERSE);
            backLeft.setPower(leftdrive);
            backRight.setPower(rightdrive);
            frontLeft.setPower(leftdrive);
            frontRight.setPower(rightdrive);

        }
    };
}
