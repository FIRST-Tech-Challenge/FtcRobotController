package org.firstinspires.ftc.team6220_2021;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name = "Turn Test", group = "TeleOp")
public class TurnTest extends LinearOpMode {

    public static DcMotor motorBackLeft;
    public static DcMotor motorBackRight;
    public static DcMotor motorFrontLeft;
    public static DcMotor motorFrontRight;
    public BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");

        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        double startAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
        double angleLeft;
        double angleTraveled;
        double targetAngle = 90;
        double currentAngle;

        boolean angleReached = false;

        PIDFilter translationPID;
        translationPID = new PIDFilter(0.005, 0.0, 0.0);

        waitForStart();

        while (opModeIsActive()) {

            /*if (gamepad1.dpad_up) {
                targetAngle = 360;
            }
            else if (gamepad1.dpad_right) {
                targetAngle = 90;
            }
            else if (gamepad1.dpad_down) {
                targetAngle = 180;
            }
            else if (gamepad1.dpad_left) {
                targetAngle = 270;
            }*/

            double startTime = System.currentTimeMillis();
            while (System.currentTimeMillis() - startTime < 3000 && opModeIsActive()) {
                idle();
            }

            while (!angleReached && opModeIsActive()) {
                currentAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

                angleTraveled = currentAngle - startAngle;
                //angleLeft = targetAngle - angleTraveled;

                motorFrontRight.setPower(-0.5);
                motorFrontLeft.setPower(0.5);
                motorBackRight.setPower(-0.5);
                motorBackLeft.setPower(0.5);

                if (Math.abs(targetAngle - angleTraveled) < 1) {
                    motorFrontRight.setPower(0.0);
                    motorFrontLeft.setPower(0.0);
                    motorBackRight.setPower(0.0);
                    motorBackLeft.setPower(0.0);
                    angleReached = true;
                }
            }
        }
    }
}