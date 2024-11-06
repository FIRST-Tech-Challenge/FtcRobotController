package org.firstinspires.ftc.teamcode.teleop;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.kdrobot.DriveTrainDirection;
import org.firstinspires.ftc.teamcode.kdrobot.DriveTrainSpeed;
import org.firstinspires.ftc.teamcode.kdrobot.KDRobot;

@TeleOp
public class KDRobotCodeV3 extends LinearOpMode {
    @Override
    public void runOpMode() {
        KDRobot robot = new KDRobot();
        robot.init(hardwareMap, telemetry);
        DcMotor armMotor = hardwareMap.get(DcMotor.class,"armMotor");
        DcMotor andymark = hardwareMap.get(DcMotor.class, "andymark");
        CRServo droneServo = hardwareMap.get(CRServo.class, "droneServo");

        telemetry.addData("Hello",", Team KryptoDragons");
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        double reverseRight = 0;
        double forwardLeft = 0;
        double reverseLeft = 0;
        double forwardRight = 0;
        int armMotorVelocity = 250;
        double leftTriggerPressed;
        double rightTriggerPressed;
        double armControlSwitch = 1;
        robot.setArmMotorVelocity(armMotorVelocity);
        armMotor.setPower(0.35);
        sleep(250);
        armMotor.setPower(0);
        while (opModeIsActive()) {
            robot.setArmMotorVelocity(armMotorVelocity);
            robot.setWheelPower(0, 0, 0, 0);
            robot.setArmMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

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
            if (reverseRight >= 1.25) {
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
            /*
            if (gamepad2.left_bumper) {
                sleep(5);
                robot.moveArmPosition(ArmDirection.UP);
            }
            if (gamepad2.right_bumper) {
                sleep(5);
                robot.moveArmPosition(ArmDirection.DOWN);
            }
           if (gamepad2.left_trigger == 1.0) {
                robot.setServoPower(ServoDirection.INTAKE);
            }
            if (gamepad2.right_trigger == 1.0) {
                robot.setServoPower(ServoDirection.OUTTAKE);
            }
            if (gamepad2.b) {
                robot.setServoPower(ServoDirection.STOP);
            }
            */
            if (gamepad2.left_trigger == 1) {
                armMotor.setPower(0.35);
                leftTriggerPressed = 1;
            }
            else {
                leftTriggerPressed = 0;
            }
            if (gamepad2.right_trigger == 1) {
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
            if (rightTriggerPressed + leftTriggerPressed == 0){
                if (armControlSwitch == 1) {
                    armMotor.setPower(0.075);
                }
                else if (armControlSwitch == -1)  {
                    armMotor.setPower(0);
                }
            } if (gamepad2.right_bumper) {
                andymark.setPower(0.25);
            } if (gamepad2.left_bumper) {
                andymark.setPower(-0.25);
            } if (gamepad2.b) {
                andymark.setPower(0);
                armMotor.setPower(0);
            } if (gamepad2.x) {
                droneServo.setPower(0.5); // Rotate the servo a half rotation (0.5)
                sleep(750);
                droneServo.setPower(0); // Set servo position back to 0
            } if (gamepad2.a) {
                armMotor.setPower(-1);
            }
            reverseLeft = -gamepad1.left_stick_x + gamepad1.left_stick_y;
            forwardRight = gamepad1.left_stick_x + -gamepad1.left_stick_y;
            forwardLeft = -gamepad1.left_stick_x + -gamepad1.left_stick_y;
            reverseRight = gamepad1.left_stick_x + gamepad1.left_stick_y;
        }
    }
}