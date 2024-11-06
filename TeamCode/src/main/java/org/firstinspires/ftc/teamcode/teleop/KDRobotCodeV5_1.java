package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.kdrobot.DriveTrainDirection;
import org.firstinspires.ftc.teamcode.kdrobot.DriveTrainSpeed;
import org.firstinspires.ftc.teamcode.kdrobot.KDRobot;

@TeleOp
public class KDRobotCodeV5_1 extends LinearOpMode {
    @Override
    public void runOpMode() {
        KDRobot robot = new KDRobot();
        robot.init(hardwareMap, telemetry);
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeft");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeft");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRight");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRight");
        DcMotor armMotor = hardwareMap.get(DcMotor.class,"armMotor");
        DcMotor andyMark = hardwareMap.get(DcMotor.class, "andymark");
        Servo droneServo = hardwareMap.get(Servo.class, "droneServo");
        DcMotor hangMotor = hardwareMap.get(DcMotor.class,"hangMotor");
        waitForStart();
        double leftTriggerPressed;
        double rightTriggerPressed;
        double armControlSwitch = 1;
        boolean leftTriggerAtRest = true;
        boolean rightTriggerAtRest = true;
        double adjustment = 0;
        boolean setMotorPower = false;
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setPower(0.35);
        sleep(250);
        armMotor.setPower(0);
        while (opModeIsActive()) {
            robot.setArmMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
            double y = gamepad1.left_stick_x; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_y * 1.1; // Counteract imperfect strafing
            double rx = 0;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            if (gamepad1.left_trigger == 0) {
                leftTriggerAtRest = true;
            } else if (gamepad1.left_trigger == 1) {
                leftTriggerAtRest = false;
            }
            if (gamepad1.right_trigger == 0) {
                rightTriggerAtRest = true;
            } else if (gamepad1.right_trigger == 1) {
                rightTriggerAtRest = false;
            }
            if (leftTriggerAtRest && rightTriggerAtRest) {
                adjustment = 2;
                setMotorPower = true;
            }
            if (!leftTriggerAtRest && !rightTriggerAtRest) {
                adjustment = 2;
                setMotorPower = true;
            }
            else if (!leftTriggerAtRest) {
                adjustment = 4;
                setMotorPower = true;
            }
            else if (!rightTriggerAtRest) {
                adjustment = 0.5;
                setMotorPower = true;
            }
            if (gamepad1.right_stick_x == 1) {
                frontLeftMotor.setPower(1/adjustment);
                frontRightMotor.setPower(1/adjustment);
                backLeftMotor.setPower(-1/adjustment);
                backRightMotor.setPower(-1/adjustment);
                setMotorPower = true;
            }
            if (gamepad1.right_stick_x == -1) {
                frontLeftMotor.setPower(-1/adjustment);
                frontRightMotor.setPower(-1/adjustment);
                backLeftMotor.setPower(1/adjustment);
                backRightMotor.setPower(1/adjustment);
                setMotorPower = true;
            }
            if (setMotorPower) {
                frontLeftMotor.setPower(frontLeftPower / adjustment);
                frontRightMotor.setPower(frontRightPower / adjustment);
                backLeftMotor.setPower(backLeftPower / adjustment);
                backRightMotor.setPower(backRightPower / adjustment);
                setMotorPower = false;
            }



            if (gamepad2.left_stick_y == -1) {
                armMotor.setPower(0.35);
                leftTriggerPressed = 1;
            }
            else {
                leftTriggerPressed = 0;
            }
            if (gamepad2.left_stick_y == 1) {
                armMotor.setPower(-0.1);
                rightTriggerPressed = 1;
            }
            else {
                rightTriggerPressed = 0;
            }
            if (gamepad2.y) {
                armControlSwitch = armControlSwitch * -1;
                sleep(250);
            }
            if (gamepad2.a) {
                armMotor.setPower(-1);
            }
            if (rightTriggerPressed + leftTriggerPressed == 0){
                if (armControlSwitch == 1) {
                    armMotor.setPower(0.075);
                }
                else if (armControlSwitch == -1)  {
                    armMotor.setPower(0);
                }
            }
            if (gamepad2.right_bumper) {
                andyMark.setPower(0.25);
            }
            else if (gamepad2.left_bumper) {
                andyMark.setPower(-0.25);
            }
            else {
                andyMark.setPower(0);
            }
            if (gamepad2.b) {
                andyMark.setPower(0);
                armMotor.setPower(0);
            } if (gamepad1.x && gamepad2.x) {
                //drone code here
                //requires both driver's input to launch drone
            }
            if (gamepad2.right_stick_y == -1) {
                hangMotor.setPower(1);
            }
            else if (gamepad2.right_stick_y == 1) {
               hangMotor.setPower(-1);
            }
            else {
                hangMotor.setPower(0);
            }

        }
    }
}