package org.firstinspires.ftc.team6220_2020.TestClasses;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "DBTest", group = "TeleOp")
public class DBTest extends LinearOpMode {

    //Motors
    DcMotor motorLeft;
    DcMotor motorRight;

    //Other Devices
    BNO055IMU imu;

    @Override
    public void runOpMode() {

        //Initialize
        motorLeft = hardwareMap.dcMotor.get("motorLeft");
        motorRight = hardwareMap.dcMotor.get("motorRight");
        motorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        motorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu". Certain parameters must be specified before using the imu.
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "AdafruitIMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        telemetry.addData("Init", "Done");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            motorLeft.setPower(-(2 * gamepad1.left_stick_y));
            motorRight.setPower(-(2 * gamepad1.right_stick_y));

            //Telemetry
            telemetry.addData("MotorLeftPower: ", -motorLeft.getPower());
            telemetry.addData("MotorRightPower: ", -motorRight.getPower());
            telemetry.addData("EncoderLeft: ", motorLeft.getCurrentPosition());
            telemetry.addData("EncoderRight: ", motorRight.getCurrentPosition());

            telemetry.addData("IMU Angle: ", imu.getAngularOrientation().firstAngle);
            telemetry.addData("IMU x Acceleration: ", imu.getLinearAcceleration().xAccel);
            telemetry.addData("IMU y Acceleration: ", imu.getLinearAcceleration().yAccel);
            telemetry.update();

            idle();
        }
    }
}
