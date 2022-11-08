package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.config.Hardware2;

import com.qualcomm.robotcore.hardware.TouchSensor;


//@TeleOp(name = "Drivercentric", group = "Taus")
public class teleop extends LinearOpMode {

    private Hardware2 robot = null;
    public BNO055IMU imu;


    float rotate_angle = 0;
    double reset_angle = 0;


    private double liftPower = 0;
    private double extensionPower = 0;


    @Override
    public void runOpMode() {
        Hardware2 robot = new Hardware2();
        robot.initTeleOpIMU();

        robot.frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        robot.backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        robot.frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);


        while (!opModeIsActive()) {
        }

        while (opModeIsActive()) {
            drive();

            resetAngle();

            //Pressing gamepad2.x is for closing claw and gamepad2.A is for opening claw
            claw();

            //gamepad2 trigger is for lift going up and down.
            lift();

            telemetry.update();

        }
    }



    public void drive() {
        //This condition is used to correct simple touches. if the condition it is true then value would be set it to zero
        double Protate = Math.abs(gamepad1.right_stick_x) <= 0.07 ? 0 :  gamepad1.right_stick_x/ 4;
        double stick_x = Math.abs(gamepad1.left_stick_x) <= 0.07 ? 0 : gamepad1.left_stick_x * Math.sqrt(Math.pow(1 - Math.abs(Protate), 2) / 2); //Accounts for Protate when limiting magnitude to be less than 1
        double stick_y = Math.abs( gamepad1.left_stick_y) <= 0.07 ? 0 : gamepad1.left_stick_y * Math.sqrt(Math.pow(1 - Math.abs(Protate), 2) / 2);
        double theta = 0;
        double Px = 0;
        double Py = 0;

        double gyroAngle = getHeading() * Math.PI / 180; //Converts gyroAngle into radians
        if (gyroAngle <= 0) {
            gyroAngle = gyroAngle + (Math.PI / 2);
        } else if (0 < gyroAngle && gyroAngle < Math.PI / 2) {
            gyroAngle = gyroAngle + (Math.PI / 2);
        } else if (Math.PI / 2 <= gyroAngle) {
            gyroAngle = gyroAngle - (3 * Math.PI / 2);
        }
        gyroAngle = -1 * gyroAngle;

        if (gamepad1.right_bumper) {
            //Disables gyro, sets to -Math.PI/2 so front is defined correctly.
            gyroAngle = -Math.PI / 2;
        }

        //Linear directions in case you want to do straight lines.
        if (gamepad1.dpad_right) {
            stick_x = 0.5;
        } else if (gamepad1.dpad_left) {
            stick_x = -0.5;
        }
        if (gamepad1.dpad_up) {
            stick_y = -0.5;
        } else if (gamepad1.dpad_down) {
            stick_y = 0.5;
        }


        //MOVEMENT for rotation
        theta = Math.atan2(stick_y, stick_x) - gyroAngle - (Math.PI / 2);
        Px = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta + Math.PI / 4));
        Py = Math.sqrt(Math.pow(stick_x, 2) + Math.pow(stick_y, 2)) * (Math.sin(theta - Math.PI / 4));


        robot.frontLeftMotor.setPower(Py - Protate);
        robot.backLeftMotor.setPower(Px - Protate);
        robot.backRightMotor.setPower(Py + Protate);
        robot.frontRightMotor.setPower(Px + Protate);
    }


    public void resetAngle() {
        if (gamepad1.a) {
            reset_angle = getHeading() + reset_angle;
        }
    }

    public double getHeading() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double heading = angles.firstAngle;
        if (heading < -180) {
            heading = heading + 360;
        } else if (heading > 180) {
            heading = heading - 360;
        }
        heading = heading - reset_angle;
        return heading;
    }


    public void claw() {
        if (gamepad2.right_bumper) {
            robot.intakeServo.setPower(0.5);
        } else if (gamepad2.left_bumper) {
            robot.intakeServo.setPower(-0.5);
        } else if (!gamepad2.right_bumper && !gamepad2.left_bumper)
            robot.intakeServo.setPower(0);

    }

    public void lift(){
        if (gamepad1.right_trigger != 0) {
            robot.verticalLiftMotor.setPower(1.0);
        } else {
            for (int i = 5; i>0; i--){
                robot.verticalLiftMotor.setPower(0.01 * i);
                sleep(100);
            }
        }
    }
}