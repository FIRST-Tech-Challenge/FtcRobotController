//  This is a class for TeleOp tank drive.
//  You can install this program to the rev control hub without any edits.
//  Using the logitech controller, you can move the robot in tank drive.

package org.firstinspires.ftc.team6220_2021.TestClasses;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TankDriveTest01", group = "TeleOp")
public class TankDriveTest01 extends LinearOpMode {

    //Motors
    DcMotor motorBackLeft;
    DcMotor motorBackRight;
    DcMotor motorFrontLeft;
    DcMotor motorFrontRight;
    DcMotor motorDuck;
    DcMotor motorArm;
    Servo servoGrabber;

    //Other Devices
    BNO055IMU imu;

    @Override
    public void runOpMode() {

        //Initialize
        motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        motorBackRight = hardwareMap.dcMotor.get("motorBackRight");
        motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        motorDuck = hardwareMap.dcMotor.get("motorDuck");

        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);

         /* DISABLE ENCODER
         motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/

        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu". Certain parameters must be specified before using the imu.
         /*     I AINT USING IMU
         BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
         parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
         parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
         parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
         parameters.loggingEnabled = true;
         parameters.loggingTag = "IMU";
         imu = hardwareMap.get(BNO055IMU.class, "imu");
         imu.initialize(parameters);*/

        waitForStart();

        while (opModeIsActive()) {
            motorBackLeft.setPower((gamepad1.left_stick_y));
            motorFrontLeft.setPower((gamepad1.left_stick_y));
            motorBackRight.setPower((gamepad1.left_stick_y));
            motorFrontRight.setPower((gamepad1.left_stick_y));

            motorBackRight.setPower(-(gamepad1.right_stick_x));
            motorFrontRight.setPower(-(gamepad1.right_stick_x));
            motorBackLeft.setPower((gamepad1.right_stick_x));
            motorFrontLeft.setPower((gamepad1.right_stick_x));

            if(gamepad1.right_trigger > 0.75) {
                motorArm.setPower(1);
            }

            if (gamepad1.left_trigger > 0.75) {
                motorArm.setPower(-1);
            }

            if(gamepad1.x){
                servoGrabber.setPosition(0.5);
            }

            if(gamepad1.a){
                servoGrabber.setPosition(0);
            }

        }
    }
}