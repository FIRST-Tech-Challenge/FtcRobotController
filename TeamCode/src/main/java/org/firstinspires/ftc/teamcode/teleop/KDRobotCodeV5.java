package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.kdrobot.DriveTrainDirection;
import org.firstinspires.ftc.teamcode.kdrobot.DriveTrainSpeed;
import org.firstinspires.ftc.teamcode.kdrobot.KDRobot;

@TeleOp
public class KDRobotCodeV5 extends LinearOpMode {
    @Override
    public void runOpMode() {
        KDRobot robot = new KDRobot();
        robot.init(hardwareMap, telemetry);
        DcMotor armMotor = hardwareMap.get(DcMotor.class,"armMotor");
        DcMotor andymark = hardwareMap.get(DcMotor.class, "andymark");
        Servo droneServo = hardwareMap.get(Servo.class, "droneServo");
        waitForStart();
        double reverseRight = 0;
        double forwardLeft = 0;
        double reverseLeft = 0;
        double forwardRight = 0;
        int armMotorVelocity = 250;
        double leftTriggerPressed;
        double rightTriggerPressed;
        double armControlSwitch = 1;
        armMotor.setPower(0.35);
        sleep(250);
        armMotor.setPower(0);
        while (opModeIsActive()) {
            robot.setArmMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
            double y = gamepad1.left_stick_x; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_y * 1.1; // Counteract imperfect strafing
            double rx = 0;

            if (gamepad1.b) {
                robot.stopDriveTrain();
            }
            if (gamepad1.left_stick_y == -1) {
                robot.moveDriveTrain(DriveTrainDirection.FORWARD);
            }
            if (gamepad1.left_stick_y == 1) {
                robot.moveDriveTrain(DriveTrainDirection.BACKWARD);
            }
            if (gamepad1.left_stick_x == -1) {
                robot.moveDriveTrain(DriveTrainDirection.STRAFE_LEFT);
            }
            if (gamepad1.left_stick_x == 1) {
                robot.moveDriveTrain(DriveTrainDirection.STRAFE_RIGHT);
            }
            if (gamepad1.right_stick_x == -1) {
                robot.moveDriveTrain(DriveTrainDirection.TURN_LEFT);
            }
            if (gamepad1.right_stick_x == 1) {
                robot.moveDriveTrain(DriveTrainDirection.TURN_RIGHT);
            }
            if (forwardLeft > 1.25) {
                robot.moveDriveTrain(DriveTrainDirection.STRAFE_DIAGONAL_FORWARD_LEFT);
            }
            if (forwardRight > 1.25) {
                robot.moveDriveTrain(DriveTrainDirection.STRAFE_DIAGONAL_FORWARD_RIGHT);
            }
            if (reverseLeft > 1.25) {
                robot.moveDriveTrain(DriveTrainDirection.STRAFE_DIAGONAL_BACKWARD_LEFT);
            }
            if (reverseRight > 1.25) {
                robot.moveDriveTrain(DriveTrainDirection.STRAFE_DIAGONAL_BACKWARD_RIGHT);

                robot.moveDriveTrain(DriveTrainDirection.FORWARD);

            }
            if (gamepad1.right_bumper) {
                sleep(250);
                robot.changeDriveTrainSpeed(DriveTrainSpeed.INCREASE);
            }
            if (gamepad1.left_bumper) {
                sleep(250);
                robot.changeDriveTrainSpeed(DriveTrainSpeed.DECREASE);
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
                andymark.setPower(0.25);
            }
            else if (gamepad2.left_bumper) {
                andymark.setPower(-0.25);
            }
            else {
                andymark.setPower(0);
            }
            if (gamepad2.b) {
                andymark.setPower(0);
                armMotor.setPower(0);
            } if (gamepad1.x && gamepad2.x) {
               //drone code here
               //requires both driver's input to launch drone
            }
            if (gamepad2.right_stick_y == 1) {
                //hang code here
            }
            if (gamepad1.guide) {

            }
            reverseLeft = -gamepad1.left_stick_x + gamepad1.left_stick_y;
            forwardRight = gamepad1.left_stick_x + -gamepad1.left_stick_y;
            forwardLeft = -gamepad1.left_stick_x + -gamepad1.left_stick_y;
            reverseRight = gamepad1.left_stick_x + gamepad1.left_stick_y;
        }
    }
}