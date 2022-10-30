package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.config.Hardware2;
@TeleOp(name="Main OpMode", group="Linear Opmode")
public class teleop extends LinearOpMode {
    Hardware2 robot = new Hardware2();
     @Override
        public void runOpMode() throws InterruptedException {
            robot.initTeleOpIMU(hardwareMap);
            while (opModeIsActive())
            {

                boolean isBoostActive = false;
                double drive = -gamepad1.left_stick_y;
                double turn  = Range.clip(-gamepad1.right_stick_x, -0.4, 0.4);
                double leftPower = 0;
                double rightPower = 0;
                robot.backLeftMotor.setPower(leftPower);
                robot.frontLeftMotor.setPower(leftPower);
                robot.frontRightMotor.setPower(rightPower);
                robot.backRightMotor.setPower(rightPower);


                if (gamepad2.right_bumper) {
                    robot.intakeServo.setPower(0.5);
                } else if (gamepad2.left_bumper) {
                    robot.intakeServo.setPower(-0.5);
                }

                if (gamepad1.right_trigger != 0) {
                    robot.verticalLiftMotor.setPower(0.5);
                } else {
                    robot.verticalLiftMotor.setPower(0);
                }
            }
        }
    }

