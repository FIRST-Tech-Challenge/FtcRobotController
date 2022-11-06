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
        waitForStart();
        while (opModeIsActive())
        {

            boolean isBoostActive = false;
            double drive = -gamepad1.left_stick_y;
            double turn  = Range.clip(-gamepad1.right_stick_x, -0.5, 0.5);
            double leftPower = Range.clip(drive + turn, -0.8, 0.8) ;
            double rightPower = Range.clip(drive - turn, -0.8, 0.8) ;
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
                for (int i = 5; i>0; i--){
                    robot.verticalLiftMotor.setPower(0.01 * i);
                    sleep(100);
                }
            }
        }
    }
}